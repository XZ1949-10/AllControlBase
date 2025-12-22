"""MPC 轨迹跟踪控制器"""
from typing import Dict, Any, Optional
import numpy as np
import time
import logging

from ..core.interfaces import ITrajectoryTracker
from ..core.data_types import Trajectory, ControlOutput, ConsistencyResult
from ..core.enums import PlatformType
from ..core.ros_compat import normalize_angle, angle_difference
from ..core.velocity_smoother import VelocitySmoother
from ..config.default_config import PLATFORM_CONFIG

logger = logging.getLogger(__name__)

# 尝试导入 ACADOS
try:
    from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel
    import casadi as ca
    ACADOS_AVAILABLE = True
except ImportError:
    ACADOS_AVAILABLE = False


class MPCController(ITrajectoryTracker):
    """MPC 轨迹跟踪控制器"""
    
    def __init__(self, config: Dict[str, Any], platform_config: Dict[str, Any]):
        mpc_config = config.get('mpc', config)
        
        self.horizon = mpc_config.get('horizon', 20)
        self.dt = mpc_config.get('dt', 0.02)
        
        constraints = config.get('constraints', platform_config.get('constraints', {}))
        self.v_max = constraints.get('v_max', 2.0)
        self.v_min = constraints.get('v_min', 0.0)
        self.omega_max = constraints.get('omega_max', 2.0)
        self.a_max = constraints.get('a_max', 1.5)
        self.vz_max = constraints.get('vz_max', 2.0)
        self.alpha_max = constraints.get('alpha_max', 3.0)  # 角加速度限制
        
        self.output_frame = platform_config.get('output_frame', 'base_link')
        self.platform_type = platform_config.get('type', PlatformType.DIFFERENTIAL)
        self.is_3d = self.platform_type == PlatformType.QUADROTOR
        self.is_omni = self.platform_type == PlatformType.OMNI
        
        # MPC 权重
        mpc_weights = mpc_config.get('weights', {})
        self.Q_pos = mpc_weights.get('position', 10.0)
        self.Q_vel = mpc_weights.get('velocity', 1.0)
        self.Q_heading = mpc_weights.get('heading', 5.0)
        self.R_v = mpc_weights.get('control_v', 0.1)
        self.R_omega = mpc_weights.get('control_omega', 0.1)
        
        # Fallback 求解器参数
        fallback_config = mpc_config.get('fallback', {})
        self.fallback_lookahead_steps = fallback_config.get('lookahead_steps', 3)
        self.fallback_heading_kp = fallback_config.get('heading_kp', 1.5)
        self.fallback_max_curvature = fallback_config.get('max_curvature', 5.0)
        
        # 阿克曼车辆最小转向速度 - 从 backup 配置读取以保持一致性
        backup_config = config.get('backup', {})
        self.min_turn_speed = backup_config.get('min_turn_speed', 0.1)
        
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
        
        self._last_solve_time_ms = 0.0
        self._last_kkt_residual = 0.0
        self._last_condition_number = 1.0
        self._last_cmd: Optional[ControlOutput] = None
        
        # 速度平滑器
        az_max = constraints.get('az_max', 1.0)
        self._velocity_smoother = VelocitySmoother(
            a_max=self.a_max, az_max=az_max,
            alpha_max=self.alpha_max, dt=self.dt
        )
    
    def _initialize_solver(self) -> None:
        """初始化 ACADOS 求解器"""
        if not ACADOS_AVAILABLE:
            logger.info("ACADOS not available, using fallback mode")
            self._is_initialized = False
            return
        
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
                xdot = ca.vertcat(vx, vy, vz, ax, ay, az, omega, alpha)
            else:
                v_body = ca.sqrt(vx**2 + vy**2)
                xdot = ca.vertcat(
                    v_body * ca.cos(theta),
                    v_body * ca.sin(theta),
                    vz,
                    ax * ca.cos(theta),
                    ax * ca.sin(theta),
                    az,
                    omega,
                    alpha
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
            R = np.diag([self.R_v, self.R_v, self.R_v, self.R_omega])
            
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
            
            ocp.constraints.lbx = np.array([-np.inf, -np.inf, -np.inf, 
                                           -self.v_max, -self.v_max, -self.vz_max,
                                           -np.inf, -self.omega_max])
            ocp.constraints.ubx = np.array([np.inf, np.inf, np.inf,
                                           self.v_max, self.v_max, self.vz_max,
                                           np.inf, self.omega_max])
            ocp.constraints.idxbx = np.array([0, 1, 2, 3, 4, 5, 6, 7])
            ocp.constraints.x0 = np.zeros(8)
            
            ocp.solver_options.qp_solver = self.qp_solver
            ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
            ocp.solver_options.integrator_type = self.integrator_type
            ocp.solver_options.nlp_solver_type = self.nlp_solver_type
            ocp.solver_options.nlp_solver_max_iter = self.nlp_max_iter
            
            self._solver = AcadosOcpSolver(ocp, json_file='acados_ocp.json')
            self._is_initialized = True
            self._ocp = ocp
            logger.info("ACADOS MPC solver initialized successfully")
            
        except Exception as e:
            logger.error(f"ACADOS initialization failed: {e}")
            self._is_initialized = False

    
    def _solve_with_acados(self, state: np.ndarray, trajectory: Trajectory,
                          consistency: ConsistencyResult) -> ControlOutput:
        """使用 ACADOS 求解 MPC 问题"""
        self._solver.set(0, 'lbx', state)
        self._solver.set(0, 'ubx', state)
        
        ref_velocities = trajectory.velocities if trajectory.soft_enabled else trajectory.get_hard_velocities()
        theta_ref = state[6]
        
        for i in range(self.horizon):
            traj_idx = min(i, len(trajectory.points) - 1)
            ref_point = trajectory.points[traj_idx]
            
            if ref_velocities is not None and traj_idx < len(ref_velocities):
                ref_vel = ref_velocities[traj_idx]
                vx_ref, vy_ref, vz_ref = ref_vel[0], ref_vel[1], ref_vel[2]
                wz_ref = ref_vel[3] if len(ref_vel) > 3 else 0.0
            else:
                vx_ref, vy_ref, vz_ref, wz_ref = 0.0, 0.0, 0.0, 0.0
            
            if i == 0:
                theta_ref = state[6]
            elif abs(wz_ref) > 1e-3:
                theta_ref = theta_ref + wz_ref * self.dt
                theta_ref = normalize_angle(theta_ref)
            elif traj_idx < len(trajectory.points) - 1:
                next_point = trajectory.points[traj_idx + 1]
                dx = next_point.x - ref_point.x
                dy = next_point.y - ref_point.y
                if np.sqrt(dx**2 + dy**2) > 0.01:
                    theta_ref = np.arctan2(dy, dx)
            
            alpha = consistency.alpha
            vx_ref *= alpha
            vy_ref *= alpha
            vz_ref *= alpha
            
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
        
        if self.is_omni or self.is_3d:
            vx = x_next[3]
            vy = x_next[4]
            vz = x_next[5] if self.is_3d else 0.0
            omega = x_next[7]
            frame_id = self.output_frame  # 使用配置的输出坐标系
        else:
            v_world = np.sqrt(x_next[3]**2 + x_next[4]**2)
            vx = v_world
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
        result = self._velocity_smoother.smooth(raw_result, self._last_cmd)
        result.health_metrics = raw_result.health_metrics
        
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
            except Exception:
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
        except Exception as e:
            # 其他未预期的错误
            logger.error(f"Unexpected error: {type(e).__name__}: {e}")
            return ControlOutput(vx=0, vy=0, vz=0, omega=0,
                               frame_id=self.output_frame, success=False,
                               health_metrics={'error_type': 'unexpected'})
    
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
        
        if trajectory.soft_enabled and trajectory.velocities is not None:
            if lookahead_idx < len(trajectory.velocities):
                v_ref = trajectory.velocities[lookahead_idx]
                target_v = np.sqrt(v_ref[0]**2 + v_ref[1]**2) * consistency.alpha
                target_v = min(target_v, self.v_max)
            else:
                target_v = self.v_max * 0.5
        else:
            target_v = self.v_max * 0.5
        
        if self.is_omni or self.is_3d:
            if dist > 0.1:
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
            if L_sq > 1e-6:
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
                curvature = 0.0
                omega = 0.0
                vx = target_v
            
            omega = np.clip(omega, -self.omega_max, self.omega_max)
            
            vy = 0.0
            vz = 0.0
            frame_id = self.output_frame  # 使用配置的输出坐标系
        
        # 构建原始控制输出
        raw_result = ControlOutput(vx=vx, vy=vy, vz=vz, omega=omega,
                                   frame_id=frame_id, success=True)
        
        # 使用 VelocitySmoother 进行速度平滑
        result = self._velocity_smoother.smooth(raw_result, self._last_cmd)
        
        self._last_kkt_residual = 0.0
        self._last_condition_number = 1.0
        self._last_cmd = result
        
        return result
    
    def set_horizon(self, horizon: int) -> None:
        """动态调整预测 horizon"""
        if horizon == self.horizon:
            return
        
        old_horizon = self.horizon
        self.horizon = horizon
        logger.info(f"MPC horizon changed from {old_horizon} to {horizon}")
        
        if self._is_initialized:
            logger.info("Reinitializing ACADOS solver with new horizon...")
            self._initialize_solver()
    
    def get_health_metrics(self) -> Dict[str, Any]:
        return {
            'type': 'mpc',
            'horizon': self.horizon,
            'last_solve_time_ms': self._last_solve_time_ms,
            'kkt_residual': self._last_kkt_residual,
            'condition_number': self._last_condition_number,
            'acados_available': self._is_initialized
        }
    
    def shutdown(self) -> None:
        if self._solver is not None:
            self._solver = None
        self._is_initialized = False
