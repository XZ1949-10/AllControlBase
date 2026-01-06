"""MPC 轨迹跟踪控制器"""
from typing import Dict, Any, Optional
import numpy as np
import time
import logging
import weakref
import gc

from ..core.interfaces import ITrajectoryTracker
from ..core.data_types import Trajectory, ControlOutput, ConsistencyResult
from ..core.indices import MPCStateIdx, MPCInputIdx, MPCSlices
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
        
        # 可视化配置
        self.visualize_prediction = mpc_config.get('visualize_prediction', False)
        
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
        self._solver_cache_order = []  # LRU order tracking: [oldest, ..., newest]
        self._solver_cache_max_size = mpc_config.get('solver_cache_max_size', 5)
        self._is_initialized = False
        self._acados_creation_failed = False
        
        # 生成模型名 (使用确定性命名以利用缓存)
        import os
        # 使用 PID 区分不同进程，但移除 UUID 以确保同一进程内 Horizon 切换时名称稳定
        # 注意: 如果不同 Horizon 共用同一个基础名，会导致文件冲突
        pid = os.getpid()
        # 加入 id(self) 以区分同一进程内的不同实例 (如多机器人仿真)
        self._model_name_base = f"mpc_model_pid{pid}_id{id(self)}"
        
        # 预先检查 ACADOS 库是否存在，避免频繁编译
        self._solver_lib_path = {}
        
        # 注册资源清理回调
        self._finalizer = weakref.finalize(self, self._cleanup_resources, self._solver_cache)
        
        # 尝试初始化
        if ACADOS_AVAILABLE:
            self._initialize_solver(self.horizon)
            
            # Pre-warm commonly used horizons to avoid runtime compile overhead
            # This is critical for performance when switching modes in the state machine
            prewarm_horizons = mpc_config.get('prewarm_horizons', [10, 20])
            for h in prewarm_horizons:
                if h != self.horizon:
                    try:
                        self._get_solver(h)
                        logger.info(f"Pre-warmed MPC solver for horizon {h}")
                    except Exception as e:
                        logger.warning(f"Failed to pre-warm solver for horizon {h}: {e}")
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
        
        # 性能优化: 预分配 Buffer
        self._yref_buffer: Optional[np.ndarray] = None
        
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
        """获取或创建指定 horizon 的求解器 (带 LRU 缓存)
        
        缓存策略:
        - 使用 LRU (Least Recently Used) 策略管理缓存
        - 缓存大小由 solver_cache_max_size 配置控制（默认 5）
        - 当缓存满时，移除最久未使用的求解器
        - 访问已缓存的求解器会更新其 LRU 顺序
        """
        if horizon in self._solver_cache:
            # 更新 LRU 顺序：移到末尾（最近使用）
            if horizon in self._solver_cache_order:
                self._solver_cache_order.remove(horizon)
            self._solver_cache_order.append(horizon)
            return self._solver_cache[horizon]
        
        # 缓存满时，移除最久未使用的求解器
        if len(self._solver_cache) >= self._solver_cache_max_size:
            if self._solver_cache_order:
                oldest_horizon = self._solver_cache_order.pop(0)
                if oldest_horizon in self._solver_cache:
                    del self._solver_cache[oldest_horizon]
                    logger.debug(f"Evicted MPC solver for horizon {oldest_horizon} from cache (LRU)")
        
        return self._create_solver(horizon)

    def _create_solver(self, horizon: int):
        """创建新的 ACADOS 求解器"""
        ocp = AcadosOcp()
        model = AcadosModel()
        
        # 使用确定性的模型名称 (base_name + horizon)
        # 这确保了同一进程中，相同 horizon 的求解器对应相同的 json/so 文件
        model_name = f'{self._model_name_base}_h{horizon}'
        model.name = model_name
        
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
            # 使用 StateIdx 保证顺序一致性
            theta_fn = theta
            xdot_expr = [0] * 8
            xdot_expr[MPCStateIdx.X] = vx
            xdot_expr[MPCStateIdx.Y] = vy
            xdot_expr[MPCStateIdx.Z] = vz
            xdot_expr[MPCStateIdx.VX] = ax
            xdot_expr[MPCStateIdx.VY] = ay
            xdot_expr[MPCStateIdx.VZ] = az
            xdot_expr[MPCStateIdx.THETA] = omega
            xdot_expr[MPCStateIdx.OMEGA] = alpha
            xdot = ca.vertcat(*xdot_expr)
        else:
            theta_fn = theta
            v_forward = vx * ca.cos(theta_fn) + vy * ca.sin(theta_fn)
            
            xdot_expr = [0] * 8
            xdot_expr[MPCStateIdx.X] = v_forward * ca.cos(theta_fn)
            xdot_expr[MPCStateIdx.Y] = v_forward * ca.sin(theta_fn)
            xdot_expr[MPCStateIdx.Z] = vz
            xdot_expr[MPCStateIdx.VX] = ax * ca.cos(theta_fn) - v_forward * omega * ca.sin(theta_fn)
            xdot_expr[MPCStateIdx.VY] = ax * ca.sin(theta_fn) + v_forward * omega * ca.cos(theta_fn)
            xdot_expr[MPCStateIdx.VZ] = az
            xdot_expr[MPCStateIdx.THETA] = omega
            xdot_expr[MPCStateIdx.OMEGA] = alpha
            
            xdot = ca.vertcat(*xdot_expr)
        
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
        
        # Use a unique json file name based on deterministic model name
        json_file = f'{model_name}.json'
        
        # 智能编译策略:
        # 如果对应的共享库已经存在，且模型参数未变（此处假设同一 horizon 配置不变），
        # 则跳过生成和编译步骤，极大地加速加载时间
        # ACADOS 生成的库通常在 c_generated_code 目录下
        generate_code = True
        build_code = True
        
        # 简单的存在性检查 (实际路径可能由 acados_template 内部处理，这里主要做优化提示)
        # 标记: 如果希望完全依赖 ACADOS 的 makefile 检查，可以保持 True
        # 但明确的 False 可以避免任何文件 IO 检查及 make 调用开销
        if horizon in self._solver_lib_path:
            generate_code = False
            build_code = False
            logger.debug(f"Reusing existing solver library for horizon {horizon}")

        solver = AcadosOcpSolver(ocp, json_file=json_file, generate=generate_code, build=build_code)
        
        # 记录库已准备好
        self._solver_lib_path[horizon] = True
        
        # Cache solver and update LRU order
        self._solver_cache[horizon] = solver
        self._solver_cache_order.append(horizon)
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
            # 强制垃圾回收以清理可能的部分初始化资源
            gc.collect()

    def _solve_with_acados(self, state: np.ndarray, trajectory: Trajectory,
                          consistency: ConsistencyResult) -> ControlOutput:
        """使用 ACADOS 求解 MPC 问题 (向量化优化版)"""
        if not self._is_initialized or self._solver is None:
            return ControlOutput(vx=0, vy=0, vz=0, omega=0, frame_id=self.output_frame,
                               success=False, health_metrics={'error_type': 'not_initialized'})

        # 1. 设置初始状态约束
        self._solver.set(0, 'lbx', state)
        self._solver.set(0, 'ubx', state)
        
        # 2. 准备轨迹参考数据 (向量化处理，避免 Python 循环计算)
        # traj_points = trajectory.points
        # traj_len unused
        alpha = consistency.alpha
        
        # 获取混合速度 [H, 4]
        # 注意: get_blended_velocities_slice 返回的是 copy，安全修改
        blended_vels = trajectory.get_blended_velocities_slice(0, self.horizon, alpha)
        
        # 填充不足的部分
        actual_len = len(blended_vels)
        if actual_len < self.horizon:
            # 需要填充的长度
            pad_len = self.horizon - actual_len
            if actual_len > 0:
                # 速度用 0 填充
                padding = np.zeros((pad_len, 4))
                blended_vels = np.vstack([blended_vels, padding])
            else:
                blended_vels = np.zeros((self.horizon, 4))
                
        # 提取点坐标 [N, 3] -> 截取前 Horizon 个点并填充
        # get_points_matrix() 有缓存，速度快
        points_all = trajectory.get_points_matrix()
        points_slice = points_all[:self.horizon]
        
        points_len = len(points_slice)
        if points_len < self.horizon:
            # 使用最后一个点填充剩余部分 (保持位置不变)
            if points_len > 0:
                last_pt = points_slice[-1]
                pad_pts = np.tile(last_pt, (self.horizon - points_len, 1))
                points_slice = np.vstack([points_slice, pad_pts])
            else:
                points_slice = np.zeros((self.horizon, 3))
                
        # ----------------------------------------------------------
        # 3. 计算参考航向角 theta_ref (重构: 优先使用几何一致性)
        # ----------------------------------------------------------
        # 旧逻辑存在 "积分回环" 风险 (Points -> Diff -> Wz -> Integrate -> Theta)
        # 新逻辑: 
        #   1. 计算路径几何切向 (Geometric Tangent) 作为基础航向
        #   2. 处理由静止或倒车引起的航向不确定性
        #   3. 使用 Soft Wz 对 Geometric Theta 进行平滑 (可选，目前保持几何优先)
        
        # 计算路径几何差分 [H-1, 3]
        diffs = points_slice[1:] - points_slice[:-1]
        
        # 填充最后一个差分
        last_diff = np.zeros(3)
        if len(diffs) > 0:
            last_diff = diffs[-1]
        diffs = np.vstack([diffs, last_diff]) # [H, 3]
        
        # 计算几何航向 (atan2)
        # 修正: 支持倒车逻辑
        # 检查参考速度 (blended_vels[:, 0] 即 vx)
        # 如果 vx < -0.01 (hysteresis), 则认为是在倒车，几何航向应翻转 180 度
        vx_refs = blended_vels[:, 0]
        geom_thetas = np.arctan2(diffs[:, 1], diffs[:, 0]) # [H]
        
        # 倒车检测与修正
        # 使用切片操作避免循环
        reversing_mask = vx_refs < -0.01
        if np.any(reversing_mask):
            # 只有当检测到确实在倒车时才修正
            # geom = atan2(dy, dx). Reversing means heading is opposite to velocity vec
            geom_thetas[reversing_mask] = normalize_angle(geom_thetas[reversing_mask] + np.pi)
        
        # 处理原地停滞 (Points 重叠): 此时 atan2 无意义 (0/0)
        # 使用上一个有效的航向进行填充
        dist_sq = diffs[:, 0]**2 + diffs[:, 1]**2
        valid_mask = dist_sq > 1e-6
        
        theta_refs = np.zeros(self.horizon)
        
        # 1. 初始点: 优先使用当前状态航向，以保证连续性
        theta_refs[0] = state[MPCStateIdx.THETA]
        
        # 2. 后续点: 使用几何航向
        # 为了防止角度跳变 (如 -pi -> pi)，需要进行 unwrapping
        # 但我们先填充数值
        
        # 简单的填充逻辑：无效点沿用上一个有效点的航向
        last_valid_theta = theta_refs[0]
        for i in range(1, self.horizon):
            if valid_mask[i-1]: # diff[i-1] 对应从 i-1 到 i 的方向
                # 使用几何方向
                # 检测是否需要翻转 (例如倒车路径):
                # 如果几何方向与上一时刻方向差接近 180 度，可能是倒车？
                # 暂时假设前向跟踪
                last_valid_theta = geom_thetas[i-1]
            
            theta_refs[i] = last_valid_theta
            
        # 3. 角度解缠 (Unwrapping) 及 归一化
        # 确保 theta_refs 连续，没有 2pi 跳变，这对于 QP 求解器非常重要
        # 首先计算相对于当前状态的相对角，然后归一化到 [-pi, pi]
        # 但 MPC 中经常使用无限角度或 unwrapped 角度。
        # ACADOS 取决于是否使用 Quaternions。这里是 Euler。
        # 简单策略: 保持 reference 接近当前状态
        
        # 使用累计差分的方法来重建连续角度，避免跳变
        # delta = normalize(theta[i] - theta[i-1])
        # theta[i] = theta[i-1] + delta
        for i in range(1, self.horizon):
            delta = normalize_angle(theta_refs[i] - theta_refs[i-1])
            theta_refs[i] = theta_refs[i-1] + delta
            
        # 4. Soft Wz 混合 (已移除复杂的积分逻辑，仅在完全静止时信任 Soft Wz? 不，几何优先)
        # 如果启用 soft mode 且 confidence 高，是否应该用 soft theta?
        # 原始设计意图不明，现在的几何优先更稳健。
        
        # 最终不进行归一化，保留累计角度以保证求解器连续性
        # theta_refs = normalize_angle(theta_refs)
        
        # 4. 构建 yref 矩阵 [H, 12]
        # yref: [px, py, pz, vx, vy, vz, theta, omega, ax, ay, az, alpha]
        
        # 性能优化: 使用预分配 Buffer 避免重复 malloc
        if self._yref_buffer is None or self._yref_buffer.shape[0] != self.horizon:
            self._yref_buffer = np.zeros((self.horizon, 12), dtype=np.float64)
            
        yrefs = self._yref_buffer
        
        # 填充数据
        # 填充数据
        yrefs[:, MPCSlices.REF_POS] = points_slice         # px, py, pz
        yrefs[:, MPCSlices.REF_VEL] = blended_vels[:, 0:3] # vx, vy, vz
        yrefs[:, MPCSlices.REF_THETA] = theta_refs         # theta
        yrefs[:, MPCSlices.REF_OMEGA] = blended_vels[:, 3] # omega
        yrefs[:, MPCSlices.REF_CONTROLS] = 0.0             # controls (ax, ay, az, alpha) 期望为 0
        
        # 5. 构建 parameters 矩阵 p [H, 7]
        # p: [px_ref, py_ref, pz_ref, vx_ref, vy_ref, vz_ref, theta_ref]
        # 这几乎是 yref 的前 7 列
        ps = yrefs[:, MPCSlices.POSE_VEL]
        
        # 6. 批量设置到求解器
        # ACADOS Python 接口目前的 set() 必须循环调用
        # 优化1: 缓存方法引用，减少属性查找开销
        # 优化2: 确保数组 C 连续且为 float64
        # yrefs 已经是连续的(来自 buffer)，ps 是 slice

        
        # 确保 yrefs 是连续的 (通常已经是)
        if not yrefs.flags['C_CONTIGUOUS']:
            yrefs = np.ascontiguousarray(yrefs, dtype=np.float64)
             
        # 优化: 尽量减少 Python 循环内的属性查找
        solver_set = self._solver.set
        horizon = self.horizon
        
        # 预先获取整个矩阵的 VIEW，避免在循环中重复切片
        # yrefs 已经确保是连续的 np.float64
        # p_params 是 yrefs 的前7列 (Pose + Vel)
        
        for i in range(horizon):
            # 直接传递行引用 (NumPy 切片 view)
            # Acados Python Interface 会处理 contiguous 检查，
            # 但我们在外部保证 yrefs 是 contiguous 的，所以 yrefs[i] 也是 contiguous
            solver_set(i, 'yref', yrefs[i])
            
            # 设置参数 p. 
            # 优化: 使用 ps[i] 直接获取参数，避免重复切片 yrefs[i, MPCSlices.POSE_VEL]
            solver_set(i, 'p', ps[i])
            
        # 7. 终端代价 (Terminal Cost)
        # 使用最后一个点的状态作为终端参考
        final_pt = points_slice[-1]
        final_theta = theta_refs[-1]
        
        # yref_e: [px, py, pz, vx, vy, vz, theta, omega] (8维)
        yref_e = np.array([
            final_pt[0], final_pt[1], final_pt[2],
            0.0, 0.0, 0.0, final_theta, 0.0
        ])
        solver_set(self.horizon, 'yref', yref_e)
        solver_set(self.horizon, 'p', ps[-1]) # 终端参数通常也需要设置
        
        # 8. 求解
        status = self._solver.solve()
        
        # 获取结果
        self._last_kkt_residual = self._solver.get_residuals()[0] if hasattr(self._solver, 'get_residuals') else 0.0
        
        if status == 0:
            x1 = self._solver.get(1, "x")
            self._last_predicted_next_state = x1
            
            # 使用索引常量提取结果
            result = ControlOutput(
                vx=x1[MPCStateIdx.VX], 
                vy=x1[MPCStateIdx.VY], 
                vz=x1[MPCStateIdx.VZ], 
                omega=x1[MPCStateIdx.OMEGA],
                frame_id=self.output_frame, 
                success=True,
                health_metrics={'kkt_residual': self._last_kkt_residual}
            )
            
            # 只有在启用可视化时才提取完整轨迹，节省开销
            if self.visualize_prediction:
                pred_states = []
                solver_get = self._solver.get
                for i in range(self.horizon + 1):
                    pred_states.append(solver_get(i, "x"))
                result.extras['predicted_trajectory'] = pred_states
            
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
            # 开发调试友好：重抛代码错误（如拼写错误、类型错误），不掩盖 Bug
            if isinstance(e, (AttributeError, NameError, TypeError, SyntaxError)):
                raise e
            
            # 运行时错误（如求解器失败、数学域错误）：记录日志并安全降级
            logger.exception(f"MPC compute error: {e}")
            return ControlOutput(vx=0, vy=0, vz=0, omega=0, frame_id=self.output_frame, success=False,
                               health_metrics={'error_type': 'compute_exception', 'msg': str(e)})
    
    def _release_solver_resources(self) -> None:
        """释放 ACADOS 资源"""
        if self._solver is not None:
            self._solver = None
        
        # 清理缓存的求解器和 LRU 顺序
        if hasattr(self, '_solver_cache') and self._solver_cache:
            self._solver_cache.clear()
        if hasattr(self, '_solver_cache_order'):
            self._solver_cache_order.clear()
        
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
            'acados_available': self._is_initialized,
            'solver_cache_size': len(self._solver_cache),
            'solver_cache_max_size': self._solver_cache_max_size,
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
