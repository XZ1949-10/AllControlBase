"""
数据源接口 - 统一数据管理

从 ControllerManager 获取数据并转换为统一的 DisplayData 模型。
所有数据处理逻辑集中在此，面板只负责显示。

注意：此模块仅用于直接访问 ControllerManager 的场景。
ROS 模式下请使用 ros_data_source.py 中的 ROSDashboardDataSource。
"""

import time
import math
from typing import Dict, Any, Optional
from collections import deque

from .models import (
    DisplayData, EnvironmentStatus, PlatformConfig, ControllerStatus,
    MPCHealthStatus, ConsistencyStatus, TimeoutStatus, TrackingStatus,
    EstimatorStatus, TransformStatus, ControlCommand, TrajectoryData,
    StatisticsData, SafetyStatus, ControllerStateEnum, DataAvailability
)

# 尝试导入 ROS 相关模块
try:
    from ..core.ros_compat import ROS_AVAILABLE, TF2_AVAILABLE
except ImportError:
    ROS_AVAILABLE = False
    TF2_AVAILABLE = False

# 尝试导入 mock 配置
try:
    from ..config.mock_config import is_mock_allowed
except ImportError:
    def is_mock_allowed(config, module=None, feature=None):
        return False

# 尝试检测 ACADOS
try:
    from acados_template import AcadosOcp
    ACADOS_AVAILABLE = True
except ImportError:
    ACADOS_AVAILABLE = False

# 平台名称映射
PLATFORM_NAMES = {
    'differential': '差速车',
    'ackermann': '阿克曼',
    'omni': '全向车',
    'quadrotor': '四旋翼',
}

# 状态名称映射
STATE_INFO = {
    0: ('INIT', '初始化'),
    1: ('NORMAL', '正常运行'),
    2: ('SOFT_DISABLED', 'Soft禁用'),
    3: ('MPC_DEGRADED', 'MPC降级'),
    4: ('BACKUP_ACTIVE', '备用激活'),
    5: ('STOPPING', '停车中'),
    6: ('STOPPED', '已停车'),
}


class DashboardDataSource:
    """
    Dashboard 数据源 - 统一数据管理
    
    职责：
    1. 从 ControllerManager 获取原始数据
    2. 转换为统一的 DisplayData 模型
    3. 处理所有数据逻辑
    4. 维护历史数据和统计信息
    
    使用场景：
    - 直接嵌入到使用 ControllerManager 的应用中
    - 需要传入有效的 controller_manager 实例
    
    ROS 模式：
    - 请使用 ROSDashboardDataSource (ros_data_source.py)
    """

    def __init__(self, controller_manager=None, config: Dict[str, Any] = None):
        """
        初始化数据源
        
        Args:
            controller_manager: ControllerManager 实例（必需）
            config: 配置字典
        """
        self.manager = controller_manager
        self.config = config or {}
        self._start_time = time.time()

        # 统计数据
        self._total_cycles = 0
        self._state_counts = {i: 0 for i in range(7)}
        self._mpc_success_count = 0
        self._backup_switch_count = 0
        self._safety_limit_count = 0
        self._tf2_fallback_count = 0
        self._soft_disable_count = 0

        # 历史数据
        self._history_size = 500
        self._solve_time_history = deque(maxlen=self._history_size)
        self._lateral_error_history = deque(maxlen=self._history_size)
        self._alpha_history = deque(maxlen=self._history_size)

        # 周期时间统计
        self._cycle_times = deque(maxlen=100)
        self._last_update_time = time.time()

        # 缓存的显示数据
        self._cached_data: Optional[DisplayData] = None

    def get_display_data(self) -> DisplayData:
        """
        获取统一的显示数据
        
        这是面板获取数据的唯一入口，所有数据处理逻辑集中在此。
        当没有真实数据时，返回带有 availability 标记的空数据，
        而不是生成模拟数据。
        """
        # 获取原始数据
        raw_diagnostics = self._get_raw_diagnostics()
        
        # 检查数据可用性
        has_diagnostics = bool(raw_diagnostics)
        
        # 更新统计
        self._update_statistics(raw_diagnostics)

        # 构建统一数据模型
        data = DisplayData()
        
        # 数据可用性
        data.availability = self._build_availability(raw_diagnostics)

        # 环境状态 (统一处理 mock 模式)
        data.environment = self._build_environment_status()

        # 平台配置
        data.platform = self._build_platform_config()

        # 控制器状态
        data.controller = self._build_controller_status(raw_diagnostics)

        # MPC 健康
        data.mpc_health = self._build_mpc_health(raw_diagnostics)

        # 一致性
        data.consistency = self._build_consistency(raw_diagnostics)

        # 超时状态
        data.timeout = self._build_timeout_status(raw_diagnostics)

        # 跟踪状态
        data.tracking = self._build_tracking_status(raw_diagnostics)

        # 状态估计器
        data.estimator = self._build_estimator_status(raw_diagnostics)

        # 坐标变换
        data.transform = self._build_transform_status(raw_diagnostics)

        # 安全状态
        data.safety = self._build_safety_status(raw_diagnostics)

        # 控制命令
        data.command = self._build_control_command(raw_diagnostics)

        # 轨迹数据
        data.trajectory = self._build_trajectory_data()

        # 统计数据
        data.statistics = self._build_statistics()

        # 元信息
        data.version = 'v3.17.12'
        data.transition_progress = raw_diagnostics.get('transition_progress', 1.0)

        self._cached_data = data
        return data

    def _build_availability(self, diagnostics: Dict[str, Any]) -> DataAvailability:
        """构建数据可用性状态"""
        has_diag = bool(diagnostics)
        
        # 检查各类数据是否存在且有效
        mpc_health = diagnostics.get('mpc_health', {})
        consistency = diagnostics.get('consistency', {})
        tracking = diagnostics.get('tracking', {})
        estimator = diagnostics.get('estimator_health', {})
        transform = diagnostics.get('transform', {})
        timeout = diagnostics.get('timeout', {})
        
        # 检查轨迹数据
        has_trajectory = False
        if self.manager:
            has_trajectory = (hasattr(self.manager, '_last_trajectory') and 
                            self.manager._last_trajectory is not None and
                            len(getattr(self.manager._last_trajectory, 'points', [])) > 0)
        
        # 检查位置数据
        has_position = False
        if self.manager and hasattr(self.manager, '_state_estimator') and self.manager._state_estimator:
            state = self.manager._state_estimator.get_state()
            if state and hasattr(state, 'state'):
                has_position = True
        
        return DataAvailability(
            diagnostics_available=has_diag,
            trajectory_available=has_trajectory,
            position_available=has_position,
            odom_available=has_diag and not timeout.get('odom_timeout', True),
            imu_data_available=has_diag and estimator.get('imu_available', False),
            mpc_data_available=has_diag and isinstance(mpc_health, dict) and bool(mpc_health),
            consistency_data_available=has_diag and isinstance(consistency, dict) and bool(consistency),
            tracking_data_available=has_diag and isinstance(tracking, dict) and bool(tracking),
            estimator_data_available=has_diag and isinstance(estimator, dict) and bool(estimator),
            transform_data_available=has_diag and isinstance(transform, dict) and bool(transform),
            last_update_time=time.time(),
            data_age_ms=0.0,
        )

    def _get_raw_diagnostics(self) -> Dict[str, Any]:
        """获取原始诊断数据"""
        if self.manager and hasattr(self.manager, '_last_published_diagnostics'):
            return self.manager._last_published_diagnostics or {}
        
        # 没有 manager 时返回空数据
        return {}

    def _update_statistics(self, diagnostics: Dict[str, Any]):
        """更新统计数据"""
        current_time = time.time()

        # 更新周期时间
        cycle_time = current_time - self._last_update_time
        self._cycle_times.append(cycle_time * 1000)
        self._last_update_time = current_time

        self._total_cycles += 1

        if diagnostics:
            # 状态计数
            state = diagnostics.get('state', 0)
            if 0 <= state < 7:
                self._state_counts[state] += 1

            # MPC 成功计数
            if diagnostics.get('mpc_success', False):
                self._mpc_success_count += 1

            # 历史数据
            self._solve_time_history.append(diagnostics.get('mpc_solve_time_ms', 0))
            
            tracking = diagnostics.get('tracking', {})
            if isinstance(tracking, dict):
                self._lateral_error_history.append(tracking.get('lateral_error', 0))

            consistency = diagnostics.get('consistency', {})
            if isinstance(consistency, dict):
                self._alpha_history.append(consistency.get('alpha_soft', 0))

    def _build_environment_status(self) -> EnvironmentStatus:
        """构建环境状态"""
        return EnvironmentStatus(
            ros_available=ROS_AVAILABLE,
            tf2_available=TF2_AVAILABLE,
            acados_available=ACADOS_AVAILABLE,
            imu_available=True,
            is_mock_mode=False,  # 此数据源不支持 mock 模式
        )

    def _build_platform_config(self) -> PlatformConfig:
        """构建平台配置"""
        platform = self.config.get('system', {}).get('platform', 'differential')
        return PlatformConfig(
            platform=platform,
            platform_display=PLATFORM_NAMES.get(platform, platform),
            ctrl_freq=self.config.get('system', {}).get('ctrl_freq', 50),
            mpc_horizon=self.config.get('mpc', {}).get('horizon', 20),
            mpc_horizon_degraded=self.config.get('mpc', {}).get('horizon_degraded', 10),
            mpc_dt=self.config.get('mpc', {}).get('dt', 0.02),
        )

    def _build_controller_status(self, diag: Dict) -> ControllerStatus:
        """构建控制器状态"""
        state = diag.get('state', 0)
        state_name, state_desc = STATE_INFO.get(state, ('UNKNOWN', '未知'))
        
        consistency = diag.get('consistency', {})
        alpha = consistency.get('alpha_soft', 0) if isinstance(consistency, dict) else 0

        return ControllerStatus(
            state=ControllerStateEnum(state) if 0 <= state <= 6 else ControllerStateEnum.INIT,
            state_name=state_name,
            state_desc=state_desc,
            mpc_success=diag.get('mpc_success', False),
            backup_active=diag.get('backup_active', False),
            current_controller='Backup' if diag.get('backup_active', False) else 'MPC',
            soft_head_enabled=alpha > 0.1,
            alpha_soft=alpha,
        )

    def _build_mpc_health(self, diag: Dict) -> MPCHealthStatus:
        """构建 MPC 健康状态"""
        health = diag.get('mpc_health', {})
        if not isinstance(health, dict):
            health = {}

        return MPCHealthStatus(
            kkt_residual=health.get('kkt_residual', 0),
            condition_number=health.get('condition_number', 0),
            solve_time_ms=diag.get('mpc_solve_time_ms', 0),
            consecutive_near_timeout=health.get('consecutive_near_timeout', 0),
            degradation_warning=health.get('degradation_warning', False),
            can_recover=health.get('can_recover', True),
            healthy=diag.get('mpc_success', False) and not health.get('degradation_warning', False),
        )

    def _build_consistency(self, diag: Dict) -> ConsistencyStatus:
        """构建一致性状态"""
        cons = diag.get('consistency', {})
        if not isinstance(cons, dict):
            cons = {}

        return ConsistencyStatus(
            curvature=cons.get('curvature', 0),
            velocity_dir=cons.get('velocity_dir', 0),
            temporal=cons.get('temporal', 0),
            alpha_soft=cons.get('alpha_soft', 0),
            data_valid=cons.get('data_valid', True),
        )

    def _build_timeout_status(self, diag: Dict) -> TimeoutStatus:
        """构建超时状态"""
        timeout = diag.get('timeout', {})
        if not isinstance(timeout, dict):
            timeout = {}

        return TimeoutStatus(
            odom_timeout=timeout.get('odom_timeout', False),
            traj_timeout=timeout.get('traj_timeout', False),
            traj_grace_exceeded=timeout.get('traj_grace_exceeded', False),
            imu_timeout=timeout.get('imu_timeout', False),
            last_odom_age_ms=timeout.get('last_odom_age_ms', 0),
            last_traj_age_ms=timeout.get('last_traj_age_ms', 0),
            last_imu_age_ms=timeout.get('last_imu_age_ms', 0),
            in_startup_grace=timeout.get('in_startup_grace', False),
        )

    def _build_tracking_status(self, diag: Dict) -> TrackingStatus:
        """构建跟踪状态"""
        tracking = diag.get('tracking', {})
        if not isinstance(tracking, dict):
            tracking = {}

        return TrackingStatus(
            lateral_error=tracking.get('lateral_error', 0),
            longitudinal_error=tracking.get('longitudinal_error', 0),
            heading_error=tracking.get('heading_error', 0),
            prediction_error=tracking.get('prediction_error', 0),
        )

    def _build_estimator_status(self, diag: Dict) -> EstimatorStatus:
        """构建状态估计器状态"""
        est = diag.get('estimator_health', {})
        if not isinstance(est, dict):
            est = {}

        bias = est.get('imu_bias', [0, 0, 0])
        if not isinstance(bias, (list, tuple)) or len(bias) < 3:
            bias = [0, 0, 0]

        # 功能开关：从配置中读取，而不是简单地根据 mock 模式设置
        ekf_config = self.config.get('ekf', {})
        transform_config = self.config.get('transform', {})
        adaptive_config = ekf_config.get('adaptive', {})
        
        # 自适应 EKF：检查是否有自适应参数配置
        ekf_enabled = bool(adaptive_config) and adaptive_config.get('base_slip_thresh', 0) > 0
        
        # 打滑检测：检查打滑检测阈值是否配置
        slip_detection_enabled = adaptive_config.get('base_slip_thresh', 0) > 0
        
        # 漂移校正：从 transform 配置读取
        drift_correction_enabled = transform_config.get('recovery_correction_enabled', True)
        
        # 航向备选：从 ekf 配置读取
        heading_fallback_enabled = ekf_config.get('use_odom_orientation_fallback', True)

        return EstimatorStatus(
            covariance_norm=est.get('covariance_norm', 0),
            innovation_norm=est.get('innovation_norm', 0),
            slip_probability=est.get('slip_probability', 0),
            imu_drift_detected=est.get('imu_drift_detected', False),
            imu_bias=(bias[0], bias[1], bias[2]),
            imu_available=est.get('imu_available', False),
            ekf_enabled=ekf_enabled,
            slip_detection_enabled=slip_detection_enabled,
            drift_correction_enabled=drift_correction_enabled,
            heading_fallback_enabled=heading_fallback_enabled,
        )

    def _build_transform_status(self, diag: Dict) -> TransformStatus:
        """构建坐标变换状态"""
        transform = diag.get('transform', {})
        if not isinstance(transform, dict):
            transform = {}

        fallback_ms = transform.get('fallback_duration_ms', 0)

        return TransformStatus(
            tf2_available=TF2_AVAILABLE,
            fallback_active=fallback_ms > 0,
            fallback_duration_ms=fallback_ms,
            accumulated_drift=transform.get('accumulated_drift', 0),
            target_frame='odom',
            output_frame='base_link',
        )

    def _build_safety_status(self, diag: Dict) -> SafetyStatus:
        """构建安全状态"""
        cmd = diag.get('cmd', {})
        if not isinstance(cmd, dict):
            cmd = {}

        vx = cmd.get('vx', 0)
        vy = cmd.get('vy', 0)
        current_v = math.sqrt(vx ** 2 + vy ** 2)

        return SafetyStatus(
            v_max=self.config.get('constraints', {}).get('v_max', 2.0),
            omega_max=self.config.get('constraints', {}).get('omega_max', 2.0),
            a_max=self.config.get('constraints', {}).get('a_max', 1.5),
            current_v=current_v,
            current_omega=abs(cmd.get('omega', 0)),
            low_speed_protection_active=current_v < 0.1,
            safety_check_passed=True,
            emergency_stop=diag.get('emergency_stop', False),
        )

    def _build_control_command(self, diag: Dict) -> ControlCommand:
        """构建控制命令"""
        cmd = diag.get('cmd', {})
        if not isinstance(cmd, dict):
            cmd = {}

        return ControlCommand(
            vx=cmd.get('vx', 0),
            vy=cmd.get('vy', 0),
            vz=cmd.get('vz', 0),
            omega=cmd.get('omega', 0),
            frame_id=cmd.get('frame_id', 'base_link'),
        )

    def _build_trajectory_data(self) -> TrajectoryData:
        """构建轨迹数据"""
        if self.manager:
            return self._get_trajectory_from_manager()

        # 没有 manager 时返回空数据，面板会显示"无真实数据"
        return TrajectoryData()

    def _get_trajectory_from_manager(self) -> TrajectoryData:
        """从 ControllerManager 获取轨迹数据"""
        data = TrajectoryData()

        if hasattr(self.manager, '_last_trajectory') and self.manager._last_trajectory:
            traj = self.manager._last_trajectory
            data.hard_points = [(p.x, p.y, p.z) for p in traj.points]
            data.num_points = len(traj.points)
            data.dt_sec = traj.dt_sec
            data.total_duration = data.num_points * data.dt_sec
            data.soft_enabled = traj.soft_enabled
            data.confidence = traj.confidence

            if traj.soft_enabled and traj.velocities is not None and len(traj.velocities) > 0:
                data.soft_velocities = [(v[0], v[1], v[2], v[3]) for v in traj.velocities]

        if hasattr(self.manager, '_state_estimator') and self.manager._state_estimator:
            state = self.manager._state_estimator.get_state().state
            data.current_position = (state[0], state[1], state[2])
            data.current_heading = state[6]
            data.current_velocity = (state[3], state[4])

        # 计算最近点
        if data.hard_points and data.current_position:
            pos = data.current_position
            min_dist = float('inf')
            for i, p in enumerate(data.hard_points):
                dist = math.sqrt((p[0] - pos[0]) ** 2 + (p[1] - pos[1]) ** 2)
                if dist < min_dist:
                    min_dist = dist
                    data.nearest_idx = i
            data.nearest_distance = min_dist
            data.lookahead_idx = min(data.nearest_idx + 5, len(data.hard_points) - 1)

        return data

    def _build_statistics(self) -> StatisticsData:
        """构建统计数据"""
        elapsed = time.time() - self._start_time
        hours = int(elapsed // 3600)
        minutes = int((elapsed % 3600) // 60)
        seconds = int(elapsed % 60)

        avg_cycle = sum(self._cycle_times) / len(self._cycle_times) if self._cycle_times else 0
        actual_freq = 1000 / avg_cycle if avg_cycle > 0 else 0

        return StatisticsData(
            elapsed_time=elapsed,
            elapsed_time_str=f'{hours:02d}:{minutes:02d}:{seconds:02d}',
            total_cycles=self._total_cycles,
            actual_freq=actual_freq,
            avg_cycle_ms=avg_cycle,
            max_cycle_ms=max(self._cycle_times) if self._cycle_times else 0,
            min_cycle_ms=min(self._cycle_times) if self._cycle_times else 0,
            mpc_success_rate=(self._mpc_success_count / self._total_cycles * 100
                             if self._total_cycles > 0 else 0),
            state_counts=self._state_counts.copy(),
            backup_switch_count=self._backup_switch_count,
            safety_limit_count=self._safety_limit_count,
            tf2_fallback_count=self._tf2_fallback_count,
            soft_disable_count=self._soft_disable_count,
        )

    def get_history(self) -> Dict[str, Any]:
        """获取历史数据 (用于曲线图)"""
        return {
            'solve_time': list(self._solve_time_history),
            'lateral_error': list(self._lateral_error_history),
            'alpha': list(self._alpha_history),
        }
