"""MPC 轨迹跟踪控制器"""
from typing import Dict, Any, Optional
import numpy as np
import time
import logging
import weakref
import gc

from ..core.interfaces import ITrajectoryTracker
from ..core.data_types import Trajectory, ControlOutput, ConsistencyResult
from ..core.enums import PlatformType
from ..core.ros_compat import normalize_angle, angle_difference, get_monotonic_time
from ..core.velocity_smoother import VelocitySmoother
from ..core.constants import (
    EPSILON,
    MPC_QP_SOLVER,
    MPC_INTEGRATOR_TYPE,
    MPC_NLP_SOLVER_TYPE,
    MPC_NLP_MAX_ITER,
)
from ..config.default_config import PLATFORM_CONFIG

logger = logging.getLogger(__name__)

# 尝试导入 ACADOS
try:
    from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel
    import casadi as ca
    ACADOS_AVAILABLE = True
except ImportError:
    ACADOS_AVAILABLE = False
    AcadosOcp = None
    AcadosOcpSolver = None
    AcadosModel = None
    ca = None

# ACADOS 约束边界常量
ACADOS_INF = 1e9


class MPCController(ITrajectoryTracker):
    """MPC 轨迹跟踪控制器
    
    资源管理说明:
    - 使用 weakref.finalize 确保资源（尤其是 ACADOS C对象）在垃圾回收时被正确释放
    - 提供了显式的 shutdown() 方法供手动清理
    
    线程安全性:
    - compute() 方法不是线程安全的，应在单线程中调用
    """
    
    def __init__(self, config: Dict[str, Any], platform_config: Dict[str, Any]):
        mpc_config = config.get('mpc', config)
        
        self.horizon = mpc_config.get('horizon', 20)
        self.dt = mpc_config.get('dt', 0.1)
        
        constraints = config.get('constraints', platform_config.get('constraints', {}))
        self.v_max = constraints.get('v_max', 2.0)
        self.v_min = constraints.get('v_min', 0.0)
        self.omega_max = constraints.get('omega_max', 2.0)
        self.a_max = constraints.get('a_max', 1.5)
        self.vz_max = constraints.get('vz_max', 2.0)
        self.alpha_max = constraints.get('alpha_max', 3.0)
        
        if self.omega_max <= 0:
            raise ValueError(f"omega_max must be > 0, got {self.omega_max}")
        
        self.output_frame = platform_config.get('output_frame', 'base_link')
        self.platform_type = platform_config.get('type', PlatformType.DIFFERENTIAL)
        self.is_3d = self.platform_type == PlatformType.QUADROTOR
        self.is_omni = self.platform_type == PlatformType.OMNI
        self.can_rotate_in_place = platform_config.get(
            'can_rotate_in_place', 
            self.platform_type != PlatformType.ACKERMANN
        )
        
        # MPC 权重
        mpc_weights = mpc_config.get('weights', {})
        self.Q_pos = max(mpc_weights.get('position', 10.0), EPSILON)
        self.Q_vel = max(mpc_weights.get('velocity', 1.0), EPSILON)
        self.Q_heading = max(mpc_weights.get('heading', 5.0), EPSILON)
        
        self.R_accel = max(mpc_weights.get('control_accel', 0.1), EPSILON)
        self.R_alpha = max(mpc_weights.get('control_alpha', 0.1), EPSILON)
        
        # ACADOS 求解器参数
        self.nlp_max_iter = MPC_NLP_MAX_ITER
        self.qp_solver = MPC_QP_SOLVER
        self.integrator_type = MPC_INTEGRATOR_TYPE
        self.nlp_solver_type = MPC_NLP_SOLVER_TYPE
        
        # 求解器状态
        self._solver = None
        self._solver_cache = {}  # Cache: horizon -> solver
        self._is_initialized = False
        self._acados_creation_failed = False
        
        # 生成唯一模型名以支持多进程
        import os
        import uuid
        pid = os.getpid()
        uid = str(uuid.uuid4())[:8]
        self._model_name = f"mpc_model_{pid}_{uid}"
        
        # 注册资源清理回调
        self._finalizer = weakref.finalize(self, self._cleanup_resources, self._solver_cache)
        
        # 尝试初始化
        if ACADOS_AVAILABLE:
            self._initialize_solver(self.horizon)
        else:
            logger.warning("ACADOS library not installed. MPC Controller will not function.")
        
        # Horizon 调整节流
        self._last_horizon_change_time: Optional[float] = None
        self._horizon_change_min_interval = mpc_config.get('horizon_change_min_interval', 1.0)
        
        self._last_solve_time_ms = 0.0
        self._last_kkt_residual = 0.0
        self._last_condition_number = 1.0
        self._last_cmd: Optional[ControlOutput] = None
        self._last_predicted_next_state: Optional[np.ndarray] = None
        
        # 速度平滑器
        ctrl_freq = config.get('system', {}).get('ctrl_freq', 50)
        smoother_dt = 1.0 / ctrl_freq
        az_max = constraints.get('az_max', 1.0)
        self._velocity_smoother = VelocitySmoother(
            a_max=self.a_max, az_max=az_max,
            alpha_max=self.alpha_max, dt=smoother_dt
        )
    
    @staticmethod
    def _cleanup_resources(cache):
        """静态清理方法"""
        if cache:
            # 清理所有缓存的求解器
            cache.clear()
        gc.collect()

    def _get_solver(self, horizon: int):
        """获取或创建指定 horizon 的求解器 (带缓存)"""
        if horizon in self._solver_cache:
            return self._solver_cache[horizon]
        return self._create_solver(horizon)

    def _create_solver(self, horizon: int):
        """创建新的 ACADOS 求解器"""
        ocp = AcadosOcp()
        model = AcadosModel()
        # Ensure unique model name to avoid conflicts if multiple models are loaded
        model.name = f'{self._model_name}_{horizon}'
        
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
        
        # 控制输入: [ax, ay, az, alpha]
        ax = ca.SX.sym('ax')
        ay = ca.SX.sym('ay')
        az = ca.SX.sym('az')
        alpha = ca.SX.sym('alpha')
        u = ca.vertcat(ax, ay, az, alpha)
        
        # 参数: [px_ref, py_ref, pz_ref, vx_ref, vy_ref, vz_ref, theta_ref]
        p = ca.SX.sym('p', 7)
        
        # 动力学模型
        if self.is_omni or self.is_3d:
            xdot = ca.vertcat(vx, vy, vz, ax, ay, az, omega, alpha)
        else:
            v_forward = vx * ca.cos(theta) + vy * ca.sin(theta)
            xdot = ca.vertcat(
                v_forward * ca.cos(theta),
                v_forward * ca.sin(theta),
                vz,
                ax * ca.cos(theta) - v_forward * omega * ca.sin(theta),
                ax * ca.sin(theta) + v_forward * omega * ca.cos(theta),
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
        ocp.dims.N = horizon
        ocp.solver_options.tf = horizon * self.dt
        
        # 代价函数
        ny = 8 + 4
        ny_e = 8
        
        Q = np.diag([self.Q_pos, self.Q_pos, self.Q_pos,
                    self.Q_vel, self.Q_vel, self.Q_vel,
                    self.Q_heading, 0.1])
        R = np.diag([self.R_accel, self.R_accel, self.R_accel, self.R_alpha])
        
        W = np.zeros((ny, ny))
        W[:8, :8] = Q
        W[8:12, 8:12] = R
        
        Vx_ext = np.zeros((ny, 8))
        Vx_ext[:8, :8] = np.eye(8)
        Vu_ext = np.zeros((ny, 4))
        Vu_ext[8:12, :4] = np.eye(4)
        
        ocp.cost.cost_type = 'LINEAR_LS'
        ocp.cost.cost_type_e = 'LINEAR_LS'
        ocp.cost.W = W
        ocp.cost.W_e = Q
        ocp.cost.Vx = Vx_ext
        ocp.cost.Vu = Vu_ext
        ocp.cost.Vx_e = np.eye(8)
        
        ocp.cost.yref = np.zeros(ny)
        ocp.cost.yref_e = np.zeros(ny_e)
        
        # 约束
        ocp.constraints.lbu = np.array([-self.a_max, -self.a_max, -self.a_max, -self.alpha_max])
        ocp.constraints.ubu = np.array([self.a_max, self.a_max, self.a_max, self.alpha_max])
        ocp.constraints.idxbu = np.array([0, 1, 2, 3])
        
        ocp.constraints.lbx = np.array([-ACADOS_INF, -ACADOS_INF, -ACADOS_INF, 
                                       -self.v_max, -self.v_max, -self.vz_max,
                                       -ACADOS_INF, -self.omega_max])
        ocp.constraints.ubx = np.array([ACADOS_INF, ACADOS_INF, ACADOS_INF,
                                       self.v_max, self.v_max, self.vz_max,
                                       ACADOS_INF, self.omega_max])
        ocp.constraints.idxbx = np.array([0, 1, 2, 3, 4, 5, 6, 7])
        ocp.constraints.x0 = np.zeros(8)
        ocp.parameter_values = np.zeros(7)
        
        ocp.solver_options.qp_solver = self.qp_solver
        ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
        ocp.solver_options.integrator_type = self.integrator_type
        ocp.solver_options.nlp_solver_type = self.nlp_solver_type
        ocp.solver_options.nlp_solver_max_iter = self.nlp_max_iter
        
        # Use a unique json file name
        json_file = f'{self._model_name}_{horizon}.json'
        
        solver = AcadosOcpSolver(ocp, json_file=json_file)
        
        # Cache solver
        self._solver_cache[horizon] = solver
        return solver

    def _initialize_solver(self, horizon: int = None) -> None:
        """初始化 ACADOS 求解器"""
        if not ACADOS_AVAILABLE:
            self._is_initialized = False
            return
            
        target_horizon = horizon if horizon is not None else self.horizon
        
        try:
            solver = self._get_solver(target_horizon)
            self._solver = solver
            self.horizon = target_horizon
            self._is_initialized = True
            self._acados_creation_failed = False
            
            logger.info(f"ACADOS MPC solver initialized/switched to horizon {target_horizon}")
            
        except Exception as e:
            logger.error(f"ACADOS initialization failed: {e}")
            self._solver = None
            self._is_initialized = False
            self._acados_creation_failed = True
            # Check if this was a new solver creation failure
            if target_horizon not in self._solver_cache:
                 if target_horizon in self._solver_cache:
                      del self._solver_cache[target_horizon]
            gc.collect()

    def _solve_with_acados(self, state: np.ndarray, trajectory: Trajectory,
                          consistency: ConsistencyResult) -> ControlOutput:
        """使用 ACADOS 求解 MPC 问题"""
        if not self._is_initialized or self._solver is None:
            return ControlOutput(vx=0, vy=0, vz=0, omega=0, frame_id=self.output_frame,
                               success=False, health_metrics={'error_type': 'not_initialized'})

        self._solver.set(0, 'lbx', state)
        self._solver.set(0, 'ubx', state)
        
        theta_ref = state[6]
        alpha = consistency.alpha
        
        # 填充 Horizon 序列
        traj_points = trajectory.points
        traj_len = len(traj_points)
        
        # 批量获取混合速度
        blended_vels = trajectory.get_blended_velocities_slice(0, self.horizon, alpha)
        # 补全
        if len(blended_vels) < self.horizon:
             pad_len = self.horizon - len(blended_vels)
             if len(blended_vels) > 0:
                 blended_vels = np.pad(blended_vels, ((0, pad_len), (0, 0)), 'edge')
             else:
                 blended_vels = np.zeros((self.horizon, 4))
        
        for i in range(self.horizon):
            traj_idx = min(i, traj_len - 1)
            ref_point = traj_points[traj_idx]
            
            vx_ref, vy_ref, vz_ref, wz_ref = blended_vels[i]
            
            if i == 0:
                theta_ref = state[6]
            elif abs(wz_ref) > 1e-3:
                theta_ref = normalize_angle(theta_ref + wz_ref * self.dt)
            elif traj_idx < traj_len - 1:
                next_point = traj_points[traj_idx + 1]
                dx, dy = next_point.x - ref_point.x, next_point.y - ref_point.y
                if np.hypot(dx, dy) > 1e-3: 
                    theta_ref = np.arctan2(dy, dx)
            
            y_ref = np.array([ref_point.x, ref_point.y, ref_point.z,
                             vx_ref, vy_ref, vz_ref, theta_ref, 0.0,
                             0.0, 0.0, 0.0, 0.0])
            self._solver.set(i, 'yref', y_ref)
            
            p = np.array([ref_point.x, ref_point.y, ref_point.z,
                         vx_ref, vy_ref, vz_ref, theta_ref])
            self._solver.set(i, 'p', p)
        
        # 终端代价
        final_idx = min(self.horizon, traj_len - 1)
        final_point = traj_points[final_idx]
        y_ref_e = np.array([final_point.x, final_point.y, final_point.z,
                          0.0, 0.0, 0.0, state[6], 0.0])
        self._solver.set(self.horizon, 'yref', y_ref_e)
        
        status = self._solver.solve()
        
        # 获取结果
        self._last_kkt_residual = self._solver.get_residuals()[0] if hasattr(self._solver, 'get_residuals') else 0.0
        
        if status == 0:
            x1 = self._solver.get(1, "x")
            self._last_predicted_next_state = x1
            result = ControlOutput(
                vx=x1[3], vy=x1[4], vz=x1[5], omega=x1[7],
                frame_id=self.output_frame, success=True,
                health_metrics={'kkt_residual': self._last_kkt_residual}
            )
        else:
            logger.warning(f"ACADOS solve failed with status {status}")
            result = ControlOutput(
                vx=0, vy=0, vz=0, omega=0, frame_id=self.output_frame, success=False,
                health_metrics={'error_type': 'solver_failure', 'status': status}
            )
            
        return result
    
    def compute(self, state: np.ndarray, trajectory: Trajectory, 
                consistency: ConsistencyResult) -> ControlOutput:
        """计算 MPC 控制输出"""
        start_time = time.time()
        
        if not ACADOS_AVAILABLE:
            return ControlOutput(vx=0, vy=0, vz=0, omega=0, frame_id=self.output_frame, success=False,
                               health_metrics={'error_type': 'acados_not_installed'})
        
        if not self._is_initialized:
            # 尝试重新初始化（如果是之前的瞬时失败），或者直接返回失败
            if not self._acados_creation_failed:
                 # 尚未失败过但未初始化，可能是未调用或者第一次
                 self._initialize_solver()
            
            if not self._is_initialized:
                return ControlOutput(vx=0, vy=0, vz=0, omega=0, frame_id=self.output_frame, success=False,
                                   health_metrics={'error_type': 'solver_not_initialized'})

        if len(trajectory.points) < 2:
            return ControlOutput(vx=0, vy=0, vz=0, omega=0, frame_id=self.output_frame, success=False,
                               health_metrics={'error_type': 'traj_too_short'})
        
        try:
            result = self._solve_with_acados(state, trajectory, consistency)
            result.solve_time_ms = (time.time() - start_time) * 1000
            self._last_solve_time_ms = result.solve_time_ms
            self._last_cmd = result
            return result
                
        except Exception as e:
            logger.exception(f"MPC compute error: {e}")
            return ControlOutput(vx=0, vy=0, vz=0, omega=0, frame_id=self.output_frame, success=False,
                               health_metrics={'error_type': 'compute_exception', 'msg': str(e)})
    
    def _release_solver_resources(self) -> None:
        """释放 ACADOS 资源"""
        if self._solver is not None:
            self._solver = None
        if self._ocp is not None:
            self._ocp = None
        
        # 强制 GC 以清理 C 资源
        gc.collect()
    
    def set_horizon(self, horizon: int) -> bool:
        """动态调整预测 horizon"""
        if horizon == self.horizon and self._is_initialized:
            return True
        if horizon < 1:
            return False
        
        current_time = get_monotonic_time()
        if self._last_horizon_change_time is not None:
             # 如果已有缓存，允许快速切换；否则限制频率
            if (horizon not in self._solver_cache) and (current_time - self._last_horizon_change_time < self._horizon_change_min_interval):
                return False
        
        logger.info(f"Switching MPC horizon to {horizon}")
        self._initialize_solver(horizon)
        
        self._last_horizon_change_time = current_time
        self._last_cmd = None
            
        return self._is_initialized
    
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
        return self._last_predicted_next_state
    
    def reset(self) -> None:
        """重置状态"""
        self._last_cmd = None
        self._last_solve_time_ms = 0.0
        self._last_kkt_residual = 0.0
        self._last_condition_number = 1.0
        self._last_predicted_next_state = None
    
    def shutdown(self) -> None:
        """关闭并释放所有资源"""
        # 调用 finalizer 清理资源 (会调用 static _cleanup_resources)
        if hasattr(self, '_finalizer'):
            self._finalizer()
        
        self._solver = None
        self._is_initialized = False
        self.reset()

    def __enter__(self):
        return self
        
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.shutdown()
