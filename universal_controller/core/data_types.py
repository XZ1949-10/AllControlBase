"""
数据类型定义

本模块定义了控制器使用的核心数据类型。

坐标系说明 (不需要建图/定位):
==================================

系统使用两个坐标系:

1. base_link (机体坐标系)
   - 原点在机器人中心
   - X轴朝前，Y轴朝左，Z轴朝上
   - 随机器人移动和旋转
   - 网络输出的轨迹在此坐标系下 (局部坐标，当前位置为原点)

2. odom (里程计坐标系)
   - 从机器人启动位置开始累积
   - 固定不动（相对于启动位置）
   - 会有漂移，但不需要建图/定位
   - 控制器在此坐标系下工作

数据流:
   Trajectory (base_link) → 坐标变换 → 控制器 (odom) → ControlOutput

关键数据类型:
   - Trajectory: 轨迹数据，frame_id 指定坐标系
   - Odometry: 里程计数据，包含位姿和速度
   - ControlOutput: 控制输出，frame_id 指定输出坐标系
"""
from dataclasses import dataclass, field
from typing import Optional, List, Dict, Any, Tuple, ClassVar
import numpy as np
import copy
import time
import logging

from .enums import TrajectoryMode, ControllerState
from .constants import normalize_angle

# 配置日志
logger = logging.getLogger(__name__)


# =============================================================================
# 轨迹默认配置 (可通过 Trajectory.configure() 修改)
# =============================================================================
class TrajectoryDefaults:
    """轨迹默认配置，可在运行时修改"""
    dt_sec: float = 0.1
    low_speed_thresh: float = 0.1
    default_confidence: float = 0.9
    default_frame_id: str = 'base_link'
    
    @classmethod
    def configure(cls, config: Dict[str, Any]) -> None:
        """
        从配置字典更新默认值
        
        Args:
            config: 配置字典，应包含 'trajectory' 键
        
        Example:
            >>> from universal_controller.config import DEFAULT_CONFIG
            >>> TrajectoryDefaults.configure(DEFAULT_CONFIG)
        """
        traj_config = config.get('trajectory', {})
        if 'default_dt_sec' in traj_config:
            cls.dt_sec = traj_config['default_dt_sec']
        if 'low_speed_thresh' in traj_config:
            cls.low_speed_thresh = traj_config['low_speed_thresh']
        if 'default_confidence' in traj_config:
            cls.default_confidence = traj_config['default_confidence']
        if 'default_frame_id' in traj_config:
            cls.default_frame_id = traj_config['default_frame_id']


# 模拟 ROS Header
@dataclass
class Header:
    """ROS Header 模拟"""
    stamp: float = 0.0  # 时间戳 (秒)
    frame_id: str = ""
    seq: int = 0


@dataclass
class Point3D:
    """3D 点"""
    x: float
    y: float
    z: float


@dataclass
class Twist3D:
    """3D 速度"""
    vx: float
    vy: float
    vz: float
    wz: float


@dataclass
class Trajectory:
    """轨迹数据类
    
    Attributes:
        header: 消息头，包含时间戳和坐标系
        points: Hard 轨迹点列表
        velocities: Soft 速度数组 [N, 4]: [vx, vy, vz, wz]
        dt_sec: 时间步长 (秒)
        confidence: 网络置信度 [0, 1]
        mode: 轨迹模式
        soft_enabled: Soft Head 是否启用
        low_speed_thresh: 低速阈值，用于角速度计算
    
    Note:
        默认值从 TrajectoryDefaults 获取，可通过 TrajectoryDefaults.configure() 修改
    """
    header: Header
    points: List[Point3D]
    velocities: Optional[np.ndarray]  # [N, 4]: [vx, vy, vz, wz]
    dt_sec: float = None  # None 表示使用默认值
    confidence: float = None  # None 表示使用默认值
    mode: TrajectoryMode = TrajectoryMode.MODE_TRACK
    soft_enabled: bool = False
    low_speed_thresh: float = field(default=None, repr=False, compare=False)
    
    def __post_init__(self):
        # 使用 TrajectoryDefaults 填充默认值
        if self.dt_sec is None:
            self.dt_sec = TrajectoryDefaults.dt_sec
        if self.confidence is None:
            self.confidence = TrajectoryDefaults.default_confidence
        if self.low_speed_thresh is None:
            self.low_speed_thresh = TrajectoryDefaults.low_speed_thresh
        
        # 验证 confidence: 必须是有效的有限数值
        # np.clip 对 NaN 不会报错但会保留 NaN，需要先检查
        if not np.isfinite(self.confidence):
            logger.warning(f"Trajectory confidence={self.confidence} invalid (NaN/Inf), using default {TrajectoryDefaults.default_confidence}")
            self.confidence = TrajectoryDefaults.default_confidence
        else:
            self.confidence = np.clip(self.confidence, 0.0, 1.0)
        
        # 验证 dt_sec: 必须是正的有限数值
        if not np.isfinite(self.dt_sec) or self.dt_sec <= 0:
            logger.warning(f"Trajectory dt_sec={self.dt_sec} invalid, using default {TrajectoryDefaults.dt_sec}")
            self.dt_sec = TrajectoryDefaults.dt_sec
        
        # 验证 velocities 维度与 points 匹配
        # 这是一个警告而非错误，因为某些情况下维度不匹配是可接受的
        if self.velocities is not None and len(self.points) > 0:
            if len(self.velocities) != len(self.points):
                logger.warning(
                    f"Trajectory velocities length ({len(self.velocities)}) "
                    f"doesn't match points length ({len(self.points)}). "
                    f"This may cause issues in velocity-based control."
                )
    
    def __len__(self) -> int:
        return len(self.points)
    
    def copy(self) -> 'Trajectory':
        """
        创建轨迹的深拷贝
        
        确保所有数据都是独立的副本，修改副本不会影响原始对象
        """
        new_header = Header(stamp=self.header.stamp, frame_id=self.header.frame_id, seq=self.header.seq)
        # 保持 velocities 的原始状态：None 保持 None，数组则深拷贝
        # np.array(..., copy=True) 对空数组和非空数组都能正确处理
        new_velocities = None if self.velocities is None else np.array(self.velocities, copy=True)
        return Trajectory(
            header=new_header,
            points=[Point3D(p.x, p.y, p.z) for p in self.points],
            velocities=new_velocities,
            dt_sec=self.dt_sec,
            confidence=self.confidence,
            mode=self.mode,
            soft_enabled=self.soft_enabled,
            low_speed_thresh=self.low_speed_thresh
        )
    
    def get_hard_velocities(self) -> np.ndarray:
        """从 Hard 轨迹点计算隐含速度"""
        if len(self.points) < 2:
            return np.zeros((1, 4))
        
        velocities = []
        n = len(self.points)
        
        for i in range(n - 1):
            p0, p1 = self.points[i], self.points[i + 1]
            vx = (p1.x - p0.x) / self.dt_sec
            vy = (p1.y - p0.y) / self.dt_sec
            vz = (p1.z - p0.z) / self.dt_sec
            
            if i < n - 2:
                p2 = self.points[i + 2]
                vx_next = (p2.x - p1.x) / self.dt_sec
                vy_next = (p2.y - p1.y) / self.dt_sec
            elif i > 0:
                p_prev = self.points[i - 1]
                vx_prev = (p0.x - p_prev.x) / self.dt_sec
                vy_prev = (p0.y - p_prev.y) / self.dt_sec
                vx_next = 2 * vx - vx_prev
                vy_next = 2 * vy - vy_prev
            else:
                vx_next, vy_next = vx, vy
            
            speed = np.sqrt(vx**2 + vy**2)
            speed_next = np.sqrt(vx_next**2 + vy_next**2)
            
            # 使用实例属性 low_speed_thresh 而非硬编码值
            if speed > self.low_speed_thresh and speed_next > self.low_speed_thresh:
                heading_curr = np.arctan2(vy, vx)
                heading_next = np.arctan2(vy_next, vx_next)
                dheading = heading_next - heading_curr
                dheading = normalize_angle(dheading)
                wz = dheading / self.dt_sec
            else:
                wz = 0.0
            
            velocities.append([vx, vy, vz, wz])
        
        velocities.append(velocities[-1])
        return np.array(velocities)
    
    def get_velocities(self) -> np.ndarray:
        """获取速度数组（统一接口）"""
        if self.soft_enabled and self.velocities is not None and len(self.velocities) > 0:
            return self.velocities
        return self.get_hard_velocities()
    
    def has_valid_soft_velocities(self) -> bool:
        return self.soft_enabled and self.velocities is not None and len(self.velocities) > 0


@dataclass
class EstimatorOutput:
    """状态估计器输出"""
    state: np.ndarray               # 状态向量 [8]
    covariance: np.ndarray          # 协方差矩阵 [8x8]
    covariance_norm: float
    innovation_norm: float
    imu_bias: np.ndarray            # IMU bias [3]
    slip_probability: float
    anomalies: List[str]
    imu_available: bool = True
    imu_drift_detected: bool = False


@dataclass
class ControlOutput:
    """控制器输出"""
    vx: float
    vy: float = 0.0
    vz: float = 0.0
    omega: float = 0.0
    frame_id: str = field(default_factory=lambda: "")
    success: bool = True
    solve_time_ms: float = 0.0
    health_metrics: Dict[str, Any] = field(default_factory=dict)
    
    def __post_init__(self):
        if not self.frame_id:
            pass  # 允许空 frame_id
    
    @property
    def v(self) -> float:
        return np.sqrt(self.vx**2 + self.vy**2)
    
    @property
    def v_horizontal(self) -> float:
        return np.sqrt(self.vx**2 + self.vy**2)
    
    @property
    def v_3d(self) -> float:
        return np.sqrt(self.vx**2 + self.vy**2 + self.vz**2)
    
    def copy(self) -> 'ControlOutput':
        return ControlOutput(
            vx=self.vx, vy=self.vy, vz=self.vz, omega=self.omega,
            frame_id=self.frame_id, success=self.success,
            solve_time_ms=self.solve_time_ms,
            health_metrics=self.health_metrics.copy() if self.health_metrics else {}
        )


@dataclass
class ConsistencyResult:
    """一致性检查结果
    
    Attributes:
        alpha: 一致性加权平均值，用于调整 soft 轨迹的权重
               范围 [0, 1]，值越高表示 soft 轨迹越可信
        kappa_consistency: 曲率一致性 [0, 1]
        v_dir_consistency: 速度方向一致性 [0, 1]
        temporal_smooth: 时序平滑度 [0, 1]
        should_disable_soft: 是否应该禁用 soft 模式
        data_valid: 数据质量是否有效
        
    data_valid 语义说明:
        - True: 数据质量正常（无 NaN、无 Inf、无异常值）
        - False: 数据质量异常（包含 NaN、Inf 或其他异常值）
        
        注意：data_valid 表示"数据质量"，不表示"数据充足性"。
        - 数据不足（如启动期间）时，data_valid 仍为 True
        - 数据不足会导致某些指标使用中性值（1.0），但不影响 data_valid
        - 下游模块应使用 data_valid 判断是否信任一致性结果
    """
    alpha: float
    kappa_consistency: float
    v_dir_consistency: float
    temporal_smooth: float
    should_disable_soft: bool
    data_valid: bool = True


@dataclass
class SafetyDecision:
    """安全决策"""
    safe: bool
    new_state: Optional[ControllerState]
    reason: str
    limited_cmd: Optional[ControlOutput] = None


@dataclass
class MPCHealthStatus:
    """MPC 健康状态"""
    healthy: bool
    warning: bool
    can_recover: bool
    consecutive_near_timeout: int
    kkt_residual: float
    condition_number: float


@dataclass
class TimeoutStatus:
    """超时状态"""
    odom_timeout: bool
    traj_timeout: bool
    traj_grace_exceeded: bool
    imu_timeout: bool
    last_odom_age_ms: float
    last_traj_age_ms: float
    last_imu_age_ms: float
    in_startup_grace: bool = False


@dataclass
class AttitudeCommand:
    """姿态命令"""
    roll: float
    pitch: float
    yaw: float
    thrust: float


@dataclass
class DiagnosticsV2:
    """诊断消息数据类"""
    header: Header
    state: int
    mpc_success: bool
    mpc_solve_time_ms: float
    backup_active: bool
    
    # MPC 健康状态
    mpc_health_kkt_residual: float
    mpc_health_condition_number: float
    mpc_health_consecutive_near_timeout: int
    mpc_health_degradation_warning: bool
    mpc_health_can_recover: bool
    
    # 一致性指标
    consistency_curvature: float
    consistency_velocity_dir: float
    consistency_temporal: float
    consistency_alpha_soft: float
    consistency_data_valid: bool
    
    # 状态估计器健康
    estimator_covariance_norm: float
    estimator_innovation_norm: float
    estimator_slip_probability: float
    estimator_imu_drift_detected: bool
    estimator_imu_bias: np.ndarray
    estimator_imu_available: bool
    
    # 跟踪误差
    tracking_lateral_error: float
    tracking_longitudinal_error: float
    tracking_heading_error: float
    tracking_prediction_error: float
    
    # 坐标变换状态
    transform_tf2_available: bool
    transform_tf2_injected: bool
    transform_fallback_duration_ms: float
    transform_accumulated_drift: float
    
    # 超时状态
    timeout_odom: bool
    timeout_traj: bool
    timeout_traj_grace_exceeded: bool
    timeout_imu: bool
    timeout_last_odom_age_ms: float
    timeout_last_traj_age_ms: float
    timeout_last_imu_age_ms: float
    timeout_in_startup_grace: bool
    
    # 控制命令
    cmd_vx: float
    cmd_vy: float
    cmd_vz: float
    cmd_omega: float
    cmd_frame_id: str
    
    # 过渡进度
    transition_progress: float
    
    def to_ros_msg(self) -> Dict[str, Any]:
        """转换为 ROS 消息格式的字典"""
        return {
            'header': {'stamp': self.header.stamp, 'frame_id': self.header.frame_id},
            'state': self.state,
            'mpc_success': self.mpc_success,
            'mpc_solve_time_ms': self.mpc_solve_time_ms,
            'backup_active': self.backup_active,
            'mpc_health': {
                'kkt_residual': self.mpc_health_kkt_residual,
                'condition_number': self.mpc_health_condition_number,
                'consecutive_near_timeout': self.mpc_health_consecutive_near_timeout,
                'degradation_warning': self.mpc_health_degradation_warning,
                'can_recover': self.mpc_health_can_recover
            },
            'consistency': {
                'curvature': self.consistency_curvature,
                'velocity_dir': self.consistency_velocity_dir,
                'temporal': self.consistency_temporal,
                'alpha_soft': self.consistency_alpha_soft,
                'data_valid': self.consistency_data_valid
            },
            'estimator_health': {
                'covariance_norm': self.estimator_covariance_norm,
                'innovation_norm': self.estimator_innovation_norm,
                'slip_probability': self.estimator_slip_probability,
                'imu_drift_detected': self.estimator_imu_drift_detected,
                'imu_bias': list(self.estimator_imu_bias),
                'imu_available': self.estimator_imu_available
            },
            'tracking': {
                'lateral_error': self.tracking_lateral_error,
                'longitudinal_error': self.tracking_longitudinal_error,
                'heading_error': self.tracking_heading_error,
                'prediction_error': self.tracking_prediction_error
            },
            'transform': {
                'tf2_available': self.transform_tf2_available,
                'tf2_injected': self.transform_tf2_injected,
                'fallback_duration_ms': self.transform_fallback_duration_ms,
                'accumulated_drift': self.transform_accumulated_drift
            },
            'timeout': {
                'odom_timeout': self.timeout_odom,
                'traj_timeout': self.timeout_traj,
                'traj_grace_exceeded': self.timeout_traj_grace_exceeded,
                'imu_timeout': self.timeout_imu,
                'last_odom_age_ms': self.timeout_last_odom_age_ms,
                'last_traj_age_ms': self.timeout_last_traj_age_ms,
                'last_imu_age_ms': self.timeout_last_imu_age_ms,
                'in_startup_grace': self.timeout_in_startup_grace
            },
            'cmd': {
                'vx': self.cmd_vx,
                'vy': self.cmd_vy,
                'vz': self.cmd_vz,
                'omega': self.cmd_omega,
                'frame_id': self.cmd_frame_id
            },
            'transition_progress': self.transition_progress
        }


# 模拟 ROS 消息类型
@dataclass
class Odometry:
    """Odometry 消息模拟"""
    header: Header = field(default_factory=Header)
    pose_position: Point3D = field(default_factory=lambda: Point3D(0, 0, 0))
    pose_orientation: Tuple[float, float, float, float] = (0, 0, 0, 1)  # quaternion (x, y, z, w)
    twist_linear: Tuple[float, float, float] = (0, 0, 0)  # (vx, vy, vz) 机体坐标系
    twist_angular: Tuple[float, float, float] = (0, 0, 0)  # (wx, wy, wz)


@dataclass
class Imu:
    """IMU 消息模拟"""
    header: Header = field(default_factory=Header)
    orientation: Tuple[float, float, float, float] = (0, 0, 0, 1)
    angular_velocity: Tuple[float, float, float] = (0, 0, 0)
    linear_acceleration: Tuple[float, float, float] = (0, 0, 0)


# =============================================================================
# TF2 相关数据类型 (用于坐标变换)
# =============================================================================

@dataclass
class Vector3:
    """3D 向量 (兼容 geometry_msgs/Vector3)"""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass
class Quaternion:
    """四元数 (兼容 geometry_msgs/Quaternion)"""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    w: float = 1.0


@dataclass
class Transform:
    """变换 (兼容 geometry_msgs/Transform)"""
    translation: Vector3 = field(default_factory=Vector3)
    rotation: Quaternion = field(default_factory=Quaternion)


@dataclass
class TransformStamped:
    """带时间戳的变换 (兼容 geometry_msgs/TransformStamped)"""
    header: Header = field(default_factory=Header)
    child_frame_id: str = ""
    transform: Transform = field(default_factory=Transform)


# =============================================================================
# ROS geometry_msgs 兼容数据类型
# =============================================================================

@dataclass
class Pose:
    """位姿 (兼容 geometry_msgs/Pose)"""
    position: Point3D = field(default_factory=lambda: Point3D(0, 0, 0))
    orientation: Quaternion = field(default_factory=Quaternion)


@dataclass
class Twist:
    """速度 (兼容 geometry_msgs/Twist)"""
    linear: Vector3 = field(default_factory=Vector3)
    angular: Vector3 = field(default_factory=Vector3)


@dataclass
class PoseWithCovariance:
    """带协方差的位姿 (兼容 geometry_msgs/PoseWithCovariance)"""
    pose: Pose = field(default_factory=Pose)
    covariance: list = field(default_factory=lambda: [0.0] * 36)


@dataclass
class TwistWithCovariance:
    """带协方差的速度 (兼容 geometry_msgs/TwistWithCovariance)"""
    twist: Twist = field(default_factory=Twist)
    covariance: list = field(default_factory=lambda: [0.0] * 36)
