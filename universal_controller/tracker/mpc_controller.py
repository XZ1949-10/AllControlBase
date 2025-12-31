"""MPC 轨迹跟踪控制器"""
from typing import Dict, Any, Optional
import numpy as np
import time
import logging
import gc

from ..core.interfaces import ITrajectoryTracker
from ..core.data_types import Trajectory, ControlOutput, ConsistencyResult
from ..core.enums import PlatformType
from ..core.ros_compat import normalize_angle, angle_difference, get_monotonic_time
from ..core.velocity_smoother import VelocitySmoother
from ..core.constants import EPSILON
from ..config.default_config import PLATFORM_CONFIG

logger = logging.getLogger(__name__)

# 尝试导入 ACADOS
try:
    from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel
    import casadi as ca
    ACADOS_AVAILABLE = True
except ImportError:
    ACADOS_AVAILABLE = False

# ACADOS 约束边界常量
# JSON 标准不支持 Infinity，使用足够大的有限值代替无约束
# 1e9 对于位置/角度约束来说实际等同于无约束
ACADOS_INF = 1e9


class MPCController(ITrajectoryTracker):
    """MPC 轨迹跟踪控制器
    
    资源管理说明:
    - ACADOS 求解器在 __init__ 中初始化
    - 调用 shutdown() 或对象销毁时释放资源
    - set_horizon() 会重新初始化求解器，有节流机制防止频繁调用
    
    线程安全性:
    - compute() 方法不是线程安全的，应在单线程中调用
    - 如需多线程访问，调用者应在外部加锁
    """
    
    def __init__(self, config: Dict[str, Any], platform_config: Dict[str, Any]):
        mpc_config = config.get('mpc', config)
        
        self.horizon = mpc_config.get('horizon', 20)
        self.dt = mpc_config.get('dt', 0.1)  # 默认值与 mpc_config.py 一致
        
        constraints = config.get('constraints', platform_config.get('constraints', {}))
        self.v_max = constraints.get('v_max', 2.0)
        self.v_min = constraints.get('v_min', 0.0)
        self.omega_max = constraints.get('omega_max', 2.0)
        self.a_max = constraints.get('a_max', 1.5)
        self.vz_max = constraints.get('vz_max', 2.0)
        self.alpha_max = constraints.get('alpha_max', 3.0)  # 角加速度限制
        
        # 运行时约束验证：omega_max 必须为正值
        # 零或负值会导致 MPC 约束设置错误和求解失败
        if self.omega_max <= 0:
            raise ValueError(
                f"omega_max must be > 0, got {self.omega_max}. "
                f"Check constraints.omega_max in config."
            )
        
        self.output_frame = platform_config.get('output_frame', 'base_link')
        self.platform_type = platform_config.get('type', PlatformType.DIFFERENTIAL)
        self.is_3d = self.platform_type == PlatformType.QUADROTOR
        self.is_omni = self.platform_type == PlatformType.OMNI
        
        # MPC 权重
        mpc_weights = mpc_config.get('weights', {})
        self.Q_pos = mpc_weights.get('position', 10.0)
        self.Q_vel = mpc_weights.get('velocity', 1.0)
        self.Q_heading = mpc_weights.get('heading', 5.0)
        
        # 权重验证：确保权重为正值，避免 ACADOS 数值问题
        # 零权重会导致 Hessian 矩阵奇异，求解器可能失败
        MIN_WEIGHT = EPSILON  # 最小权重值
        if self.Q_pos < MIN_WEIGHT:
            logger.warning(f"MPC position weight {self.Q_pos} too small, using {MIN_WEIGHT}")
            self.Q_pos = MIN_WEIGHT
        if self.Q_vel < MIN_WEIGHT:
            logger.warning(f"MPC velocity weight {self.Q_vel} too small, using {MIN_WEIGHT}")
            self.Q_vel = MIN_WEIGHT
        if self.Q_heading < MIN_WEIGHT:
            logger.warning(f"MPC heading weight {self.Q_heading} too small, using {MIN_WEIGHT}")
            self.Q_heading = MIN_WEIGHT
        
        # 控制输入权重 (R 矩阵)
        # control_accel: 加速度控制权重 (用于 ax, ay, az)
        # control_alpha: 角加速度控制权重 (用于 alpha)
        self.R_accel = mpc_weights.get('control_accel', 0.1)
        self.R_alpha = mpc_weights.get('control_alpha', 0.1)
        
        # 控制权重验证
        if self.R_accel < MIN_WEIGHT:
            logger.warning(f"MPC control_accel weight {self.R_accel} too small, using {MIN_WEIGHT}")
            self.R_accel = MIN_WEIGHT
        if self.R_alpha < MIN_WEIGHT:
            logger.warning(f"MPC control_alpha weight {self.R_alpha} too small, using {MIN_WEIGHT}")
            self.R_alpha = MIN_WEIGHT
        
        # Fallback 求解器参数
        # MPC 特有参数从 mpc.fallback 读取
        fallback_config = mpc_config.get('fallback', {})
        self.fallback_lookahead_steps = fallback_config.get('lookahead_steps', 3)
        
        # 共享参数从 backup 配置读取，确保与 Pure Pursuit 备份控制器一致
        # 包括: kp_heading, max_curvature, min_distance_thresh, min_turn_speed, default_speed_ratio
        backup_config = config.get('backup', {})
        self.fallback_heading_kp = backup_config.get('kp_heading', 1.5)
        self.fallback_max_curvature = backup_config.get('max_curvature', 5.0)
        self.fallback_min_distance_thresh = backup_config.get('min_distance_thresh', 0.1)
        self.min_turn_speed = backup_config.get('min_turn_speed', 0.1)
        self.default_speed_ratio = backup_config.get('default_speed_ratio', 0.5)
        
        # ACADOS 求解器参数
        solver_config = mpc_config.get('solver', {})
        self.nlp_max_iter = solver_config.get('nlp_max_iter', 50)
        self.qp_solver = solver_config.get('qp_solver', 'PARTIAL_CONDENSING_HPIPM')
        self.integrator_type = solver_config.get('integrator_type', 'ERK')
        self.nlp_solver_type = solver_config.get('nlp_solver_type', 'SQP_RTI')
        
        # ACADOS 求解器
        self._solver = None
        self._ocp = None
        self._is_initialized = False
        self._initialize_solver()
        
        # Horizon 调整节流 - 防止频繁重新初始化求解器导致资源泄漏
        self._last_horizon_change_time: Optional[float] = None
        self._horizon_change_min_interval = mpc_config.get('horizon_change_min_interval', 1.0)  # 秒
        
        self._last_solve_time_ms = 0.0
        self._last_kkt_residual = 0.0
        self._last_condition_number = 1.0
        self._last_cmd: Optional[ControlOutput] = None
        self._last_predicted_next_state: Optional[np.ndarray] = None  # 预测的下一步状态
        
        # 速度平滑器
        # 注意: VelocitySmoother 的 dt 应该使用实际控制周期，而不是 MPC 预测步长
        # 实际控制周期 = 1/ctrl_freq，从 system 配置读取
        # 如果未配置，使用 backup.dt 作为回退（向后兼容）
        ctrl_freq = config.get('system', {}).get('ctrl_freq', 50)
        smoother_dt = 1.0 / ctrl_freq
        az_max = constraints.get('az_max', 1.0)
        self._velocity_smoother = VelocitySmoother(
            a_max=self.a_max, az_max=az_max,
            alpha_max=self.alpha_max, dt=smoother_dt
        )
    
    def _initialize_solver(self) -> None:
        """
        初始化 ACADOS 求解器
        
        如果初始化失败，会清理所有已创建的资源，
        并将 _is_initialized 设为 False，后续调用会使用 fallback 求解器。
        
        资源管理策略:
        - 初始化前先调用 _release_solver_resources() 确保旧资源完全释放
        - 使用局部变量 ocp 和 solver 进行初始化
        - 只有在完全成功后才赋值给实例变量
        - 失败时清理局部变量并强制 GC
        """
        if not ACADOS_AVAILABLE:
            logger.info("ACADOS not available, using fallback mode")
            self._is_initialized = False
            return
        
        # 确保之前的资源已完全清理（包括 GC）
        # 这对于 ACADOS C 资源的正确释放至关重要
        self._release_solver_resources()
        
        # 临时变量，用于在失败时清理
        ocp = None
        solver = None
        
        try:
            ocp = AcadosOcp()
            model = AcadosModel()
            model.name = 'trajectory_tracker'
            
            # 状态变量: [px, py, pz, vx, vy, vz, theta, omega]
            px = ca.SX.sym('px')
            py = ca.SX.sym('py')
            pz = ca.SX.sym('pz')
            vx = ca.SX.sym('vx')
            vy = ca.SX.sym('vy')
            vz = ca.SX.sym('vz')
            theta = ca.SX.sym('theta')
            omega = ca.SX.sym('omega')
            x = ca.vertcat(px, py, pz, vx, vy, vz, theta, omega)
            
            # 控制输入
            ax = ca.SX.sym('ax')
            ay = ca.SX.sym('ay')
            az = ca.SX.sym('az')
            alpha = ca.SX.sym('alpha')
            u = ca.vertcat(ax, ay, az, alpha)
            
            # 参考轨迹参数
            px_ref = ca.SX.sym('px_ref')
            py_ref = ca.SX.sym('py_ref')
            pz_ref = ca.SX.sym('pz_ref')
            vx_ref = ca.SX.sym('vx_ref')
            vy_ref = ca.SX.sym('vy_ref')
            vz_ref = ca.SX.sym('vz_ref')
            theta_ref = ca.SX.sym('theta_ref')
            p = ca.vertcat(px_ref, py_ref, pz_ref, vx_ref, vy_ref, vz_ref, theta_ref)
            
            # 动力学
            if self.is_omni or self.is_3d:
                # 全向移动或无人机：世界坐标系下的简单积分模型
                xdot = ca.vertcat(vx, vy, vz, ax, ay, az, omega, alpha)
            else:
                # 差速车动力学模型
                # 状态: [px, py, pz, vx_world, vy_world, vz, theta, omega]
                # 控制: [a_body, ay(unused), az, alpha]
                # 
                # 差速车非完整约束: 速度方向与航向一致（无侧向滑移）
                # vx_world = v_forward * cos(theta)
                # vy_world = v_forward * sin(theta)
                # 其中 v_forward 是沿车辆前进方向的速度（可正可负，支持倒车）
                # 
                # 从世界坐标系速度恢复带符号的前向速度:
                # v_forward = vx_world * cos(theta) + vy_world * sin(theta)
                # 这是速度向量在航向方向上的投影，保留了正负号
                # 
                # 动力学方程 (考虑向心加速度):
                # dpx/dt = v_forward * cos(theta)
                # dpy/dt = v_forward * sin(theta)
                # dvx_world/dt = a_forward * cos(theta) - v_forward * omega * sin(theta)
                # dvy_world/dt = a_forward * sin(theta) + v_forward * omega * cos(theta)
                # 
                # 其中 a_forward 是沿车辆前进方向的加速度 (控制输入 ax)
                # 第二项是由于旋转产生的向心加速度
                v_forward = vx * ca.cos(theta) + vy * ca.sin(theta)
                xdot = ca.vertcat(
                    v_forward * ca.cos(theta),                                    # dpx/dt
                    v_forward * ca.sin(theta),                                    # dpy/dt
                    vz,                                                           # dpz/dt
                    ax * ca.cos(theta) - v_forward * omega * ca.sin(theta),       # dvx_world/dt
                    ax * ca.sin(theta) + v_forward * omega * ca.cos(theta),       # dvy_world/dt
                    az,                                                           # dvz/dt
                    omega,                                                        # dtheta/dt
                    alpha                                                         # domega/dt
                )
            
            model.x = x
            model.u = u
            model.p = p
            model.f_expl_expr = xdot
            model.xdot = ca.SX.sym('xdot', 8)
            model.f_impl_expr = model.xdot - xdot
            
            ocp.model = model
            ocp.dims.N = self.horizon
            ocp.solver_options.tf = self.horizon * self.dt
            
            # 代价函数
            # LINEAR_LS 代价形式: ||Vx*x + Vu*u - y_ref||_W
            # 需要正确设置 Vx 和 Vu 矩阵
            Q = np.diag([self.Q_pos, self.Q_pos, self.Q_pos,
                        self.Q_vel, self.Q_vel, self.Q_vel,
                        self.Q_heading, 0.1])
            # R 矩阵：控制输入权重 [ax, ay, az, alpha]
            R = np.diag([self.R_accel, self.R_accel, self.R_accel, self.R_alpha])
            
            ocp.cost.cost_type = 'LINEAR_LS'
            ocp.cost.cost_type_e = 'LINEAR_LS'
            
            # 状态输出矩阵 (8 状态 -> 8 输出)
            Vx = np.eye(8)
            # 控制输出矩阵 (4 控制 -> 4 输出)
            Vu = np.zeros((8, 4))
            
            # 组合权重矩阵: 状态权重 + 控制权重
            # y_ref 维度 = 8 (状态) + 4 (控制) = 12
            # 但 ACADOS LINEAR_LS 要求 Vx 和 Vu 输出维度相同
            # 因此使用扩展的输出向量
            ny = 8 + 4  # 输出维度
            Vx_ext = np.zeros((ny, 8))
            Vx_ext[:8, :8] = np.eye(8)
            Vu_ext = np.zeros((ny, 4))
            Vu_ext[8:12, :4] = np.eye(4)
            
            # 组合权重矩阵
            W = np.zeros((ny, ny))
            W[:8, :8] = Q
            W[8:12, 8:12] = R
            
            ocp.cost.Vx = Vx_ext
            ocp.cost.Vu = Vu_ext
            ocp.cost.W = W
            ocp.cost.W_e = Q  # 终端代价只有状态
            ocp.cost.Vx_e = np.eye(8)
            
            # 参考轨迹维度
            ocp.cost.yref = np.zeros(ny)
            ocp.cost.yref_e = np.zeros(8)
            
            # 约束
            ocp.constraints.lbu = np.array([-self.a_max, -self.a_max, -self.a_max, -self.alpha_max])
            ocp.constraints.ubu = np.array([self.a_max, self.a_max, self.a_max, self.alpha_max])
            ocp.constraints.idxbu = np.array([0, 1, 2, 3])
            
            # 状态约束 (x, y, z 位置无约束，速度和角速度有约束)
            ocp.constraints.lbx = np.array([-ACADOS_INF, -ACADOS_INF, -ACADOS_INF, 
                                           -self.v_max, -self.v_max, -self.vz_max,
                                           -ACADOS_INF, -self.omega_max])
            ocp.constraints.ubx = np.array([ACADOS_INF, ACADOS_INF, ACADOS_INF,
                                           self.v_max, self.v_max, self.vz_max,
                                           ACADOS_INF, self.omega_max])
            ocp.constraints.idxbx = np.array([0, 1, 2, 3, 4, 5, 6, 7])
            ocp.constraints.x0 = np.zeros(8)
            
            # 参数初始值 (p 有 7 个元素: px_ref, py_ref, pz_ref, vx_ref, vy_ref, vz_ref, theta_ref)
            ocp.parameter_values = np.zeros(7)
            
            ocp.solver_options.qp_solver = self.qp_solver
            ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
            ocp.solver_options.integrator_type = self.integrator_type
            ocp.solver_options.nlp_solver_type = self.nlp_solver_type
            ocp.solver_options.nlp_solver_max_iter = self.nlp_max_iter
            
            # 先赋值给局部变量，只有完全成功后才赋值给实例变量
            solver = AcadosOcpSolver(ocp, json_file='acados_ocp.json')
            
            # 初始化成功，赋值给实例变量
            self._solver = solver
            self._ocp = ocp
            self._is_initialized = True
            logger.info("ACADOS MPC solver initialized successfully")
            
        except Exception as e:
            logger.error(f"ACADOS initialization failed: {e}")
            # 清理局部变量（如果已创建）
            # 注意：Python 的 del 只是删除引用，实际释放由 GC 处理
            if solver is not None:
                del solver
            if ocp is not None:
                del ocp
            # 确保实例变量也被清理
            self._solver = None
            self._ocp = None
            self._is_initialized = False
            gc.collect()  # 强制垃圾回收，确保 ACADOS C 资源被释放

    
    def _solve_with_acados(self, state: np.ndarray, trajectory: Trajectory,
                          consistency: ConsistencyResult) -> ControlOutput:
        """使用 ACADOS 求解 MPC 问题"""
        self._solver.set(0, 'lbx', state)
        self._solver.set(0, 'ubx', state)
        
        theta_ref = state[6]
        alpha = consistency.alpha
        
        for i in range(self.horizon):
            traj_idx = min(i, len(trajectory.points) - 1)
            ref_point = trajectory.points[traj_idx]
            
            # 使用统一的混合速度接口
            blended_vel = trajectory.get_blended_velocity(traj_idx, alpha)
            vx_ref, vy_ref, vz_ref, wz_ref = blended_vel[0], blended_vel[1], blended_vel[2], blended_vel[3]
            
            if i == 0:
                theta_ref = state[6]
            elif abs(wz_ref) > 1e-3:
                theta_ref = theta_ref + wz_ref * self.dt
                theta_ref = normalize_angle(theta_ref)
            elif traj_idx < len(trajectory.points) - 1:
                next_point = trajectory.points[traj_idx + 1]
                dx = next_point.x - ref_point.x
                dy = next_point.y - ref_point.y
                if np.sqrt(dx**2 + dy**2) > self.fallback_min_distance_thresh:
                    theta_ref = np.arctan2(dy, dx)
            
            # y_ref 维度 = 8 (状态) + 4 (控制参考，通常为0)
            y_ref = np.array([ref_point.x, ref_point.y, ref_point.z,
                             vx_ref, vy_ref, vz_ref, theta_ref, 0.0,
                             0.0, 0.0, 0.0, 0.0])  # 控制参考为0
            self._solver.set(i, 'yref', y_ref)
            
            p = np.array([ref_point.x, ref_point.y, ref_point.z,
                         vx_ref, vy_ref, vz_ref, theta_ref])
            self._solver.set(i, 'p', p)
        
        final_idx = min(self.horizon, len(trajectory.points) - 1)
        final_point = trajectory.points[final_idx]
        y_ref_e = np.array([final_point.x, final_point.y, final_point.z,
                          0.0, 0.0, 0.0, state[6], 0.0])
        self._solver.set(self.horizon, 'yref', y_ref_e)
        
        status = self._solver.solve()
        
        self._last_kkt_residual = self._solver.get_residuals()[0] if hasattr(self._solver, 'get_residuals') else 0.0
        
        if status != 0:
            return self._solve_fallback(state, trajectory, consistency)
        
        u_opt = self._solver.get(0, 'u')
        x_next = self._solver.get(1, 'x')
        
        # 保存预测的下一步状态，用于计算预测误差
        self._last_predicted_next_state = x_next.copy()
        
        if self.is_omni or self.is_3d:
            vx = x_next[3]
            vy = x_next[4]
            vz = x_next[5] if self.is_3d else 0.0
            omega = x_next[7]
            frame_id = self.output_frame  # 使用配置的输出坐标系
        else:
            # 差速车：从世界坐标系速度恢复带符号的前向速度
            # v_forward = vx_world * cos(theta) + vy_world * sin(theta)
            # 这是速度向量在航向方向上的投影，保留了正负号
            theta_next = x_next[6]
            vx_world = x_next[3]
            vy_world = x_next[4]
            v_forward = vx_world * np.cos(theta_next) + vy_world * np.sin(theta_next)
            vx = v_forward  # 机体坐标系下的前向速度（带符号）
            vy = 0.0
            vz = 0.0
            omega = x_next[7]
            frame_id = self.output_frame  # 使用配置的输出坐标系
        
        # 构建原始控制输出
        raw_result = ControlOutput(
            vx=vx, vy=vy, vz=vz, omega=omega,
            frame_id=frame_id, success=True,
            health_metrics={
                'kkt_residual': self._last_kkt_residual,
                'condition_number': self._last_condition_number,
                'solver_status': status
            }
        )
        
        # 使用 VelocitySmoother 进行速度平滑
        # smooth() 会复制 health_metrics，无需再次赋值
        result = self._velocity_smoother.smooth(raw_result, self._last_cmd)
        
        self._last_cmd = result
        return result
    
    def compute(self, state: np.ndarray, trajectory: Trajectory, 
                consistency: ConsistencyResult) -> ControlOutput:
        """计算 MPC 控制输出"""
        start_time = time.time()
        
        if len(trajectory.points) < 2:
            return ControlOutput(vx=0, vy=0, vz=0, omega=0,
                               frame_id=self.output_frame, success=False)
        
        try:
            if self._is_initialized and self._solver is not None:
                result = self._solve_with_acados(state, trajectory, consistency)
            else:
                result = self._solve_fallback(state, trajectory, consistency)
            
            result.solve_time_ms = (time.time() - start_time) * 1000
            self._last_solve_time_ms = result.solve_time_ms  # 更新成员变量
            return result
        except ValueError as e:
            # 输入数据错误（如 NaN、维度不匹配等）
            # 这通常是可恢复的，下一次调用可能成功
            logger.warning(f"Input validation error: {e}")
            return ControlOutput(vx=0, vy=0, vz=0, omega=0,
                               frame_id=self.output_frame, success=False,
                               health_metrics={'error_type': 'input_validation'})
        except RuntimeError as e:
            # 求解器运行时错误（如数值问题、求解失败等）
            # 可能需要重新初始化求解器
            logger.warning(f"Solver runtime error: {e}")
            # 尝试使用 fallback 求解器
            try:
                result = self._solve_fallback(state, trajectory, consistency)
                result.solve_time_ms = (time.time() - start_time) * 1000
                self._last_solve_time_ms = result.solve_time_ms
                result.health_metrics['fallback_reason'] = 'solver_runtime_error'
                return result
            except (ValueError, RuntimeError):
                return ControlOutput(vx=0, vy=0, vz=0, omega=0,
                                   frame_id=self.output_frame, success=False,
                                   health_metrics={'error_type': 'solver_runtime'})
        except MemoryError as e:
            # 内存错误，这是严重问题
            logger.error(f"Memory error: {e}")
            # 尝试释放资源并重新初始化
            self._solver = None
            self._is_initialized = False
            return ControlOutput(vx=0, vy=0, vz=0, omega=0,
                               frame_id=self.output_frame, success=False,
                               health_metrics={'error_type': 'memory_error'})
        except (KeyboardInterrupt, SystemExit):
            # 不捕获这些异常，让它们正常传播
            raise
        except Exception as e:
            # 其他未预期的错误 - 记录详细信息以便调试
            logger.error(f"Unexpected error in MPC compute: {type(e).__name__}: {e}", exc_info=True)
            return ControlOutput(vx=0, vy=0, vz=0, omega=0,
                               frame_id=self.output_frame, success=False,
                               health_metrics={'error_type': 'unexpected', 'error_class': type(e).__name__})
    
    def _solve_fallback(self, state: np.ndarray, trajectory: Trajectory,
                       consistency: ConsistencyResult) -> ControlOutput:
        """简化的 fallback 求解器"""
        px, py, pz = state[0], state[1], state[2]
        theta = state[6]
        
        min_dist = float('inf')
        closest_idx = 0
        for i, p in enumerate(trajectory.points):
            dist = np.sqrt((p.x - px)**2 + (p.y - py)**2)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        
        lookahead_idx = min(closest_idx + self.fallback_lookahead_steps, len(trajectory.points) - 1)
        target = trajectory.points[lookahead_idx]
        
        dx = target.x - px
        dy = target.y - py
        dist = np.sqrt(dx**2 + dy**2)
        
        # 使用统一的混合速度接口计算目标速度
        alpha = consistency.alpha
        target_v = trajectory.get_blended_speed(lookahead_idx, alpha)
        
        # 如果轨迹点不足，使用默认速度
        if len(trajectory.points) < 2:
            target_v = self.v_max * self.default_speed_ratio
        
        target_v = min(target_v, self.v_max)
        
        if self.is_omni or self.is_3d:
            if dist > self.fallback_min_distance_thresh:
                vx = target_v * dx / dist
                vy = target_v * dy / dist
            else:
                vx = 0.0
                vy = 0.0
            
            target_heading = np.arctan2(dy, dx)
            heading_error = angle_difference(target_heading, theta)
            omega = self.fallback_heading_kp * heading_error
            omega = np.clip(omega, -self.omega_max, self.omega_max)
            
            vz = 0.0
            if self.is_3d and lookahead_idx < len(trajectory.points):
                dz = target.z - pz
                vz = np.clip(dz, -self.vz_max, self.vz_max)
            
            frame_id = self.output_frame  # 使用配置的输出坐标系
        else:
            local_x = dx * np.cos(theta) + dy * np.sin(theta)
            local_y = -dx * np.sin(theta) + dy * np.cos(theta)
            
            L_sq = local_x**2 + local_y**2
            
            # Pure Pursuit 曲率公式: κ = 2*y/L²
            # 当目标点在车辆后方 (local_x < 0) 时，应该先转向
            if L_sq > EPSILON:
                if local_x < 0:
                    # 目标点在后方，使用航向误差控制而非曲率
                    target_heading = np.arctan2(dy, dx)
                    heading_error = np.arctan2(np.sin(target_heading - theta), 
                                               np.cos(target_heading - theta))
                    omega = self.fallback_heading_kp * heading_error
                    
                    # 根据平台类型决定是否可以原地转向
                    if self.platform_type == PlatformType.ACKERMANN:
                        # 阿克曼车辆不能原地转向，需要以最小速度前进/后退
                        # 使用配置的最小转向速度
                        min_turn_speed = max(self.min_turn_speed, self.v_min) if self.v_min > 0 else self.min_turn_speed
                        vx = min_turn_speed
                    else:
                        # 差速车可以原地转向
                        vx = 0.0
                else:
                    curvature = 2.0 * local_y / L_sq
                    # 限制曲率范围，避免数值爆炸
                    curvature = np.clip(curvature, -self.fallback_max_curvature, self.fallback_max_curvature)
                    omega = target_v * curvature
                    vx = target_v
            else:
                # 目标点非常近，直接前进
                omega = 0.0
                vx = target_v
            
            omega = np.clip(omega, -self.omega_max, self.omega_max)
            
            vy = 0.0
            vz = 0.0
            frame_id = self.output_frame  # 使用配置的输出坐标系
        
        # 构建原始控制输出
        # fallback 求解器没有 KKT 残差等指标，使用 0 值表示
        raw_result = ControlOutput(
            vx=vx, vy=vy, vz=vz, omega=omega,
            frame_id=frame_id, success=True,
            health_metrics={
                'kkt_residual': 0.0,
                'condition_number': 1.0,
                'solver_type': 'fallback'
            }
        )
        
        # 使用 VelocitySmoother 进行速度平滑
        result = self._velocity_smoother.smooth(raw_result, self._last_cmd)
        
        # 更新内部状态
        self._last_kkt_residual = 0.0
        self._last_condition_number = 1.0
        self._last_cmd = result
        self._last_predicted_next_state = None  # fallback 求解器不提供预测状态
        
        return result
    
    def _release_solver_resources(self) -> None:
        """
        释放 ACADOS 求解器资源
        
        ACADOS Python 绑定在 __del__ 中会释放 C 资源，但 Python 的 GC 
        不保证立即调用 __del__。此方法通过以下策略确保资源及时释放：
        
        1. 显式删除引用并设为 None
        2. 调用 gc.collect() 触发垃圾回收
        3. 记录资源释放状态用于调试
        
        注意：此方法是幂等的，可以安全地多次调用
        """
        has_resources = self._solver is not None or self._ocp is not None
        
        if self._solver is not None:
            try:
                # 保存引用用于日志
                solver_id = id(self._solver)
                # 显式设为 None（不使用 del，避免 None 时报错）
                self._solver = None
                logger.debug(f"Released solver reference (id={solver_id})")
            except Exception as e:
                logger.warning(f"Error releasing solver: {e}")
                self._solver = None
        
        if self._ocp is not None:
            try:
                self._ocp = None
            except Exception as e:
                logger.warning(f"Error releasing OCP: {e}")
                self._ocp = None
        
        # 只有在确实有资源需要释放时才调用 gc.collect()
        # 避免不必要的性能开销
        if has_resources:
            gc.collect()
    
    def set_horizon(self, horizon: int) -> bool:
        """
        动态调整预测 horizon
        
        注意：频繁调整 horizon 会导致 ACADOS 求解器重新初始化，
        这是一个相对昂贵的操作。为防止资源泄漏和性能问题，
        此方法有最小调用间隔限制。
        
        Args:
            horizon: 新的预测时域长度
        
        Returns:
            bool: True 表示 horizon 已更新或无需更新，False 表示被节流
        
        Note:
            当返回 False 时，调用者应该继续使用当前的 horizon 值，
            而不是假设新的 horizon 已生效。可以通过 get_health_metrics()
            获取当前实际使用的 horizon 值。
        """
        if horizon == self.horizon:
            return True  # 无需更新，状态一致
        
        # 验证 horizon 值
        if horizon < 1:
            logger.warning(f"Invalid horizon value {horizon}, must be >= 1")
            return False
        
        # 节流检查：防止频繁重新初始化
        # 使用单调时钟避免系统时间调整的影响
        current_time = get_monotonic_time()
        if self._last_horizon_change_time is not None:
            elapsed = current_time - self._last_horizon_change_time
            if elapsed < self._horizon_change_min_interval:
                logger.debug(
                    f"Horizon change throttled: {elapsed:.2f}s < {self._horizon_change_min_interval}s, "
                    f"requested={horizon}, current={self.horizon}"
                )
                return False  # 明确返回 False 表示未更新
        
        old_horizon = self.horizon
        was_initialized = self._is_initialized
        
        self.horizon = horizon
        self._last_horizon_change_time = current_time
        logger.info(f"MPC horizon changed from {old_horizon} to {horizon}")
        
        # 重置速度历史，避免旧 horizon 下的速度影响新 horizon 的平滑计算
        # 这确保了 horizon 变化后的第一次计算不会受到旧状态的影响
        self._last_cmd = None
        
        if was_initialized:
            logger.info("Reinitializing ACADOS solver with new horizon...")
            # 使用统一的资源释放方法
            self._release_solver_resources()
            self._is_initialized = False
            self._initialize_solver()
            
            # 检查重新初始化是否成功
            if not self._is_initialized:
                # ACADOS 初始化失败，但 horizon 值已更新
                # 系统会使用 fallback 求解器，这是可接受的降级行为
                # 不恢复旧 horizon，因为：
                # 1. fallback 求解器也能使用新 horizon
                # 2. 调用者请求的 horizon 应该被尊重
                # 3. 恢复旧 horizon 可能导致状态不一致
                logger.warning(
                    f"ACADOS solver reinitialization failed. "
                    f"Horizon updated to {horizon}, using fallback solver. "
                    f"Check ACADOS installation and configuration."
                )
        
        # 返回 True 表示 horizon 已更新
        # 调用者可通过 get_health_metrics()['acados_available'] 检查 ACADOS 状态
        return True
    
    def get_health_metrics(self) -> Dict[str, Any]:
        return {
            'type': 'mpc',
            'horizon': self.horizon,
            'last_solve_time_ms': self._last_solve_time_ms,
            'kkt_residual': self._last_kkt_residual,
            'condition_number': self._last_condition_number,
            'acados_available': self._is_initialized
        }
    
    def get_predicted_next_state(self) -> Optional[np.ndarray]:
        """
        获取 MPC 预测的下一步状态
        
        Returns:
            Optional[np.ndarray]: 预测的下一步状态向量 [8]，
                                  如果 MPC 未成功求解则返回 None
        """
        return self._last_predicted_next_state
    
    def reset(self) -> None:
        """重置控制器内部状态（不释放求解器资源）"""
        self._last_cmd = None
        self._last_solve_time_ms = 0.0
        self._last_kkt_residual = 0.0
        self._last_condition_number = 1.0
        self._last_predicted_next_state = None
        # 注意：不重置 _solver 和 _is_initialized，保留求解器资源
    
    def shutdown(self) -> None:
        """释放资源并重置状态"""
        self._release_solver_resources()
        self._is_initialized = False
        self.reset()  # 重置内部状态
    
    def __del__(self) -> None:
        """析构函数，确保资源释放"""
        # 注意：__del__ 中不应该抛出异常
        # 也不应该依赖其他可能已被销毁的对象
        try:
            # 直接设为 None，不调用 _release_solver_resources 
            # 因为 __del__ 中其他对象可能已被销毁
            if hasattr(self, '_solver') and self._solver is not None:
                self._solver = None
            if hasattr(self, '_ocp') and self._ocp is not None:
                self._ocp = None
            # 不在 __del__ 中调用 gc.collect()，可能导致问题
        except Exception:
            pass  # 忽略析构时的异常
