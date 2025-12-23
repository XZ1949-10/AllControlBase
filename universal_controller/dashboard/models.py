"""
Dashboard 统一数据模型

所有显示数据的统一定义，确保数据管理集中化。
"""

from dataclasses import dataclass, field
from typing import List, Tuple, Dict, Any, Optional
from enum import IntEnum


class ControllerStateEnum(IntEnum):
    """控制器状态枚举"""
    INIT = 0
    NORMAL = 1
    SOFT_DISABLED = 2
    MPC_DEGRADED = 3
    BACKUP_ACTIVE = 4
    STOPPING = 5
    STOPPED = 6


@dataclass
class DataAvailability:
    """数据可用性状态 - 标记各类数据是否有真实数据"""
    diagnostics_available: bool = False      # 诊断数据是否可用
    trajectory_available: bool = False       # 轨迹数据是否可用
    position_available: bool = False         # 位置数据是否可用
    odom_available: bool = False             # 里程计数据是否可用
    imu_data_available: bool = False         # IMU 数据是否可用
    mpc_data_available: bool = False         # MPC 数据是否可用
    consistency_data_available: bool = False # 一致性数据是否可用
    tracking_data_available: bool = False    # 跟踪数据是否可用
    estimator_data_available: bool = False   # 估计器数据是否可用
    transform_data_available: bool = False   # 变换数据是否可用
    last_update_time: float = 0.0            # 最后更新时间戳
    data_age_ms: float = 0.0                 # 数据年龄 (毫秒)


@dataclass
class EnvironmentStatus:
    """环境状态"""
    ros_available: bool = False
    tf2_available: bool = False
    acados_available: bool = False
    imu_available: bool = False
    is_mock_mode: bool = False  # 默认为 False，只有明确启用模拟模式时才为 True


@dataclass
class PlatformConfig:
    """平台配置"""
    platform: str = 'differential'
    platform_display: str = '差速车'
    ctrl_freq: int = 50
    mpc_horizon: int = 20
    mpc_horizon_degraded: int = 10
    mpc_dt: float = 0.02


@dataclass
class ControllerStatus:
    """控制器状态"""
    state: ControllerStateEnum = ControllerStateEnum.INIT
    state_name: str = 'INIT'
    state_desc: str = '初始化'
    mpc_success: bool = False
    backup_active: bool = False
    current_controller: str = 'MPC'
    soft_head_enabled: bool = False
    alpha_soft: float = 0.0


@dataclass
class MPCHealthStatus:
    """MPC 健康状态"""
    kkt_residual: float = 0.0
    condition_number: float = 0.0
    solve_time_ms: float = 0.0
    consecutive_near_timeout: int = 0
    degradation_warning: bool = False
    can_recover: bool = True
    healthy: bool = True


@dataclass
class ConsistencyStatus:
    """一致性状态"""
    curvature: float = 0.0
    velocity_dir: float = 0.0
    temporal: float = 0.0
    alpha_soft: float = 0.0
    data_valid: bool = True


@dataclass
class TimeoutStatus:
    """超时状态"""
    odom_timeout: bool = False
    traj_timeout: bool = False
    traj_grace_exceeded: bool = False
    imu_timeout: bool = False
    last_odom_age_ms: float = 0.0
    last_traj_age_ms: float = 0.0
    last_imu_age_ms: float = 0.0
    in_startup_grace: bool = False


@dataclass
class TrackingStatus:
    """跟踪状态"""
    lateral_error: float = 0.0
    longitudinal_error: float = 0.0
    heading_error: float = 0.0
    prediction_error: float = 0.0


@dataclass
class EstimatorStatus:
    """状态估计器状态"""
    covariance_norm: float = 0.0
    innovation_norm: float = 0.0
    slip_probability: float = 0.0
    imu_drift_detected: bool = False
    imu_bias: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    imu_available: bool = False
    # EKF 功能开关
    ekf_enabled: bool = False
    slip_detection_enabled: bool = False
    drift_correction_enabled: bool = False
    heading_fallback_enabled: bool = False


@dataclass
class TransformStatus:
    """坐标变换状态"""
    tf2_available: bool = False
    fallback_active: bool = False
    fallback_duration_ms: float = 0.0
    accumulated_drift: float = 0.0
    target_frame: str = 'odom'
    output_frame: str = 'base_link'


@dataclass
class ControlCommand:
    """控制命令"""
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    omega: float = 0.0
    frame_id: str = 'base_link'


@dataclass
class TrajectoryData:
    """轨迹数据"""
    hard_points: List[Tuple[float, float, float]] = field(default_factory=list)
    soft_velocities: List[Tuple[float, float, float, float]] = field(default_factory=list)
    current_position: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    current_heading: float = 0.0
    current_velocity: Tuple[float, float] = (0.0, 0.0)
    nearest_idx: int = 0
    nearest_distance: float = 0.0
    lookahead_idx: int = 0
    num_points: int = 0
    dt_sec: float = 0.1
    total_duration: float = 0.0
    mode: str = 'TRACK'
    soft_enabled: bool = False
    confidence: float = 0.0


@dataclass
class StatisticsData:
    """运行统计"""
    elapsed_time: float = 0.0
    elapsed_time_str: str = '00:00:00'
    total_cycles: int = 0
    actual_freq: float = 0.0
    avg_cycle_ms: float = 0.0
    max_cycle_ms: float = 0.0
    min_cycle_ms: float = 0.0
    mpc_success_rate: float = 0.0
    state_counts: Dict[int, int] = field(default_factory=lambda: {i: 0 for i in range(7)})
    backup_switch_count: int = 0
    safety_limit_count: int = 0
    tf2_fallback_count: int = 0
    soft_disable_count: int = 0


@dataclass
class SafetyStatus:
    """安全状态"""
    v_max: float = 2.0
    omega_max: float = 2.0
    a_max: float = 1.5
    current_v: float = 0.0
    current_omega: float = 0.0
    low_speed_protection_active: bool = False
    safety_check_passed: bool = True
    emergency_stop: bool = False


@dataclass 
class DisplayData:
    """
    Dashboard 显示数据 - 统一数据模型
    
    所有面板都从这个统一的数据结构获取数据，
    避免分散的数据解析和重复的逻辑判断。
    
    数据可用性:
    - availability 字段标记各类数据是否有真实数据
    - 当数据不可用时，面板应显示"不可用"状态而非默认值
    """
    # 数据可用性
    availability: DataAvailability = field(default_factory=DataAvailability)
    
    # 环境与模式
    environment: EnvironmentStatus = field(default_factory=EnvironmentStatus)
    platform: PlatformConfig = field(default_factory=PlatformConfig)
    
    # 控制器状态
    controller: ControllerStatus = field(default_factory=ControllerStatus)
    mpc_health: MPCHealthStatus = field(default_factory=MPCHealthStatus)
    
    # 数据状态
    consistency: ConsistencyStatus = field(default_factory=ConsistencyStatus)
    timeout: TimeoutStatus = field(default_factory=TimeoutStatus)
    tracking: TrackingStatus = field(default_factory=TrackingStatus)
    estimator: EstimatorStatus = field(default_factory=EstimatorStatus)
    transform: TransformStatus = field(default_factory=TransformStatus)
    safety: SafetyStatus = field(default_factory=SafetyStatus)
    
    # 输出
    command: ControlCommand = field(default_factory=ControlCommand)
    trajectory: TrajectoryData = field(default_factory=TrajectoryData)
    
    # 统计
    statistics: StatisticsData = field(default_factory=StatisticsData)
    
    # 元信息
    version: str = 'v3.17.12'
    transition_progress: float = 1.0
