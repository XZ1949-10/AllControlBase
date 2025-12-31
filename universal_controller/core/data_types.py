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
    
    # 置信度范围 (用于 clip)
    min_confidence: float = 0.0
    max_confidence: float = 1.0
    
    # 验证参数
    min_dt_sec: float = 0.01
    max_dt_sec: float = 1.0
    min_points: int = 2
    max_points: int = 100
    max_point_distance: float = 10.0
    
    # 验证开关
    validate_enabled: bool = True
    
    @classmethod
    def configure(cls, config: Dict[str, Any]) -> None:
        """
        从配置字典更新默认值
        
        Args:
            config: 配置字典，应包含 'trajectory', 'transform', 'mpc' 键
        
        配置说明:
            dt_sec 继承已在 ParamLoader._sync_dt_config() 中统一处理。
            传入的 config['trajectory']['default_dt_sec'] 已经是最终值。
            
            为了兼容直接使用 DEFAULT_CONFIG 的场景（不经过 ParamLoader），
            此处保留了 mpc.dt 的 fallback 逻辑。
        
        Example:
            >>> from universal_controller.config import DEFAULT_CONFIG
            >>> TrajectoryDefaults.configure(DEFAULT_CONFIG)
        """
        traj_config = config.get('trajectory', {})
        transform_config = config.get('transform', {})
        mpc_config = config.get('mpc', {})
        
        # dt_sec 配置: 优先使用 trajectory.default_dt_sec
        # 如果未配置，则 fallback 到 mpc.dt (兼容直接使用 DEFAULT_CONFIG 的场景)
        if 'default_dt_sec' in traj_config:
            cls.dt_sec = traj_config['default_dt_sec']
        elif 'dt' in mpc_config:
            cls.dt_sec = mpc_config['dt']
        
        if 'low_speed_thresh' in traj_config:
            cls.low_speed_thresh = traj_config['low_speed_thresh']
        if 'default_confidence' in traj_config:
            cls.default_confidence = traj_config['default_confidence']
        
        # 置信度范围参数
        if 'min_confidence' in traj_config:
            cls.min_confidence = traj_config['min_confidence']
        if 'max_confidence' in traj_config:
            cls.max_confidence = traj_config['max_confidence']
        
        # 坐标系配置: 统一从 transform 读取
        # 轨迹的默认坐标系应与 transform.source_frame 一致
        if 'source_frame' in transform_config:
            cls.default_frame_id = transform_config['source_frame']
        
        # 验证参数
        if 'min_dt_sec' in traj_config:
            cls.min_dt_sec = traj_config['min_dt_sec']
        if 'max_dt_sec' in traj_config:
            cls.max_dt_sec = traj_config['max_dt_sec']
        if 'min_points' in traj_config:
            cls.min_points = traj_config['min_points']
        if 'max_points' in traj_config:
            cls.max_points = traj_config['max_points']
        if 'max_point_distance' in traj_config:
            cls.max_point_distance = traj_config['max_point_distance']


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
        
    缓存机制:
        get_hard_velocities() 使用内部缓存，避免重复计算。
        缓存在以下情况下失效：
        - 首次调用
        - points 或 dt_sec 发生变化（通过 _cache_key 检测）
    """
    header: Header
    points: List[Point3D]
    velocities: Optional[np.ndarray]  # [N, 4]: [vx, vy, vz, wz]
    dt_sec: float = None  # None 表示使用默认值
    confidence: float = None  # None 表示使用默认值
    mode: TrajectoryMode = TrajectoryMode.MODE_TRACK
    soft_enabled: bool = False
    low_speed_thresh: float = field(default=None, repr=False, compare=False)
    
    # 缓存字段（不参与比较和 repr）
    _hard_velocities_cache: Optional[np.ndarray] = field(default=None, repr=False, compare=False, init=False)
    _cache_key: Optional[tuple] = field(default=None, repr=False, compare=False, init=False)
    
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
            # 使用配置的 min_confidence 和 max_confidence 进行 clip
            self.confidence = np.clip(
                self.confidence, 
                TrajectoryDefaults.min_confidence, 
                TrajectoryDefaults.max_confidence
            )
        
        # 验证 dt_sec: 必须是正的有限数值
        if not np.isfinite(self.dt_sec) or self.dt_sec <= 0:
            logger.warning(f"Trajectory dt_sec={self.dt_sec} invalid, using default {TrajectoryDefaults.dt_sec}")
            self.dt_sec = TrajectoryDefaults.dt_sec
        
        # 轨迹验证 (使用配置的验证参数)
        if TrajectoryDefaults.validate_enabled:
            self._validate_trajectory()
        
        # 验证 velocities 维度与 points 匹配
        # 这是一个警告而非错误，因为某些情况下维度不匹配是可接受的
        if self.velocities is not None and len(self.points) > 0:
            if len(self.velocities) != len(self.points):
                logger.warning(
                    f"Trajectory velocities length ({len(self.velocities)}) "
                    f"doesn't match points length ({len(self.points)}). "
                    f"This may cause issues in velocity-based control."
                )
    
    def _validate_trajectory(self) -> None:
        """
        验证轨迹数据的有效性
        
        验证内容:
        1. dt_sec 在 [min_dt_sec, max_dt_sec] 范围内
        2. 轨迹点数在 [min_points, max_points] 范围内
        3. 相邻点距离不超过 max_point_distance
        
        验证失败时记录警告，不会抛出异常
        """
        # 验证 dt_sec 范围
        if self.dt_sec < TrajectoryDefaults.min_dt_sec:
            logger.warning(
                f"Trajectory dt_sec={self.dt_sec:.4f}s < min_dt_sec={TrajectoryDefaults.min_dt_sec:.4f}s, "
                f"clamping to min value"
            )
            self.dt_sec = TrajectoryDefaults.min_dt_sec
        elif self.dt_sec > TrajectoryDefaults.max_dt_sec:
            logger.warning(
                f"Trajectory dt_sec={self.dt_sec:.4f}s > max_dt_sec={TrajectoryDefaults.max_dt_sec:.4f}s, "
                f"clamping to max value"
            )
            self.dt_sec = TrajectoryDefaults.max_dt_sec
        
        # 验证轨迹点数
        num_points = len(self.points)
        if num_points < TrajectoryDefaults.min_points:
            logger.warning(
                f"Trajectory has {num_points} points < min_points={TrajectoryDefaults.min_points}, "
                f"control quality may be degraded"
            )
        elif num_points > TrajectoryDefaults.max_points:
            logger.warning(
                f"Trajectory has {num_points} points > max_points={TrajectoryDefaults.max_points}, "
                f"consider truncating for performance"
            )
        
        # 验证相邻点距离
        if len(self.points) >= 2:
            max_dist = TrajectoryDefaults.max_point_distance
            for i in range(len(self.points) - 1):
                p0, p1 = self.points[i], self.points[i + 1]
                dist = np.sqrt((p1.x - p0.x)**2 + (p1.y - p0.y)**2 + (p1.z - p0.z)**2)
                if dist > max_dist:
                    logger.warning(
                        f"Trajectory point distance {dist:.2f}m at index {i} > max_point_distance={max_dist:.2f}m, "
                        f"trajectory may be discontinuous"
                    )
                    break  # 只报告第一个异常，避免日志刷屏
    
    def __len__(self) -> int:
        return len(self.points)
    
    def _compute_cache_key(self) -> tuple:
        """
        计算缓存键，用于检测 points 或 dt_sec 是否变化
        
        使用首尾点 + 中间采样点 + 长度 + dt_sec 的组合进行检测。
        这是性能和可靠性的权衡：
        - 完整比较所有点 O(n) 太慢
        - 只比较首尾点可能漏检中间点变化
        - 采样中间点可以在 O(1) 时间内提高检测可靠性
        """
        n = len(self.points)
        if n == 0:
            return (0, self.dt_sec)
        elif n == 1:
            p = self.points[0]
            return (1, p.x, p.y, p.z, self.dt_sec)
        elif n == 2:
            p0, p1 = self.points[0], self.points[1]
            return (2, p0.x, p0.y, p0.z, p1.x, p1.y, p1.z, self.dt_sec)
        else:
            # 采样首、中、尾三个点，提高中间点变化的检测率
            p0 = self.points[0]
            p_mid = self.points[n // 2]
            p_last = self.points[-1]
            return (n, p0.x, p0.y, p0.z, p_mid.x, p_mid.y, p_mid.z, 
                    p_last.x, p_last.y, p_last.z, self.dt_sec)
    
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
        """从 Hard 轨迹点计算隐含速度
        
        使用缓存机制避免重复计算。缓存在 points 或 dt_sec 变化时自动失效。
        """
        # 检查缓存是否有效
        current_key = self._compute_cache_key()
        if self._hard_velocities_cache is not None and self._cache_key == current_key:
            return self._hard_velocities_cache
        
        # 计算速度
        if len(self.points) < 2:
            result = np.zeros((1, 4))
            self._hard_velocities_cache = result
            self._cache_key = current_key
            return result
        
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
        result = np.array(velocities)
        
        # 更新缓存
        self._hard_velocities_cache = result
        self._cache_key = current_key
        
        return result
    
    def get_velocities(self) -> np.ndarray:
        """获取速度数组（统一接口）"""
        if self.soft_enabled and self.velocities is not None and len(self.velocities) > 0:
            return self.velocities
        return self.get_hard_velocities()
    
    def has_valid_soft_velocities(self) -> bool:
        return self.soft_enabled and self.velocities is not None and len(self.velocities) > 0
    
    def get_blended_velocity(self, index: int, alpha: float) -> np.ndarray:
        """
        获取混合后的速度向量
        
        使用公式: final_vel = alpha * soft_vel + (1 - alpha) * hard_vel
        
        Args:
            index: 轨迹点索引
            alpha: 一致性系数 [0, 1]，alpha=1 表示完全使用 soft，alpha=0 表示完全使用 hard
        
        Returns:
            混合后的速度向量 [vx, vy, vz, wz]
        
        Note:
            - 当 soft_enabled=False 时，返回 hard velocities
            - 当 soft velocities 不可用时，返回 hard velocities
            - 索引超出范围时，使用最后一个有效索引
        """
        hard_velocities = self.get_hard_velocities()
        
        # 确保索引有效
        hard_idx = min(index, len(hard_velocities) - 1) if len(hard_velocities) > 0 else 0
        
        if len(hard_velocities) == 0:
            return np.zeros(4)
        
        hard_vel = hard_velocities[hard_idx]
        
        # 如果 soft 不可用，直接返回 hard
        if not self.has_valid_soft_velocities():
            return hard_vel.copy()
        
        # 确保 soft 索引有效
        soft_idx = min(index, len(self.velocities) - 1)
        soft_vel = self.velocities[soft_idx]
        
        # 确保 soft_vel 是 4 维
        if len(soft_vel) < 4:
            soft_vel_4d = np.zeros(4)
            soft_vel_4d[:len(soft_vel)] = soft_vel
            soft_vel = soft_vel_4d
        
        # 混合: alpha * soft + (1 - alpha) * hard
        return alpha * soft_vel + (1 - alpha) * hard_vel
    
    def get_blended_speed(self, index: int, alpha: float) -> float:
        """
        获取混合后的水平速度大小
        
        Args:
            index: 轨迹点索引
            alpha: 一致性系数 [0, 1]
        
        Returns:
            混合后的水平速度大小 (m/s)
        """
        vel = self.get_blended_velocity(index, alpha)
        return np.sqrt(vel[0]**2 + vel[1]**2)


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
    """MPC 健康状态
    
    字段说明:
    - healthy: 整体健康状态（无警告且求解时间正常）
    - degradation_warning: 是否有降级警告（连续超时、条件数过高或 KKT 残差过大）
    - can_recover: 是否可以从降级状态恢复
    - consecutive_near_timeout: 连续接近超时的次数
    - kkt_residual: KKT 残差（求解精度指标）
    - condition_number: 条件数（数值稳定性指标）
    """
    healthy: bool
    degradation_warning: bool  # 统一命名，原 warning
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
    """诊断消息数据类
    
    所有字段都有默认值，便于部分初始化和测试。
    字段按功能分组，保持逻辑清晰。
    """
    # 基础信息
    header: Header = field(default_factory=Header)
    state: int = 0
    mpc_success: bool = False
    mpc_solve_time_ms: float = 0.0
    backup_active: bool = False
    
    # MPC 健康状态
    mpc_health_kkt_residual: float = 0.0
    mpc_health_condition_number: float = 0.0
    mpc_health_consecutive_near_timeout: int = 0
    mpc_health_degradation_warning: bool = False
    mpc_health_can_recover: bool = True
    
    # 一致性指标
    consistency_curvature: float = 1.0
    consistency_velocity_dir: float = 1.0
    consistency_temporal: float = 1.0
    consistency_alpha_soft: float = 1.0
    consistency_data_valid: bool = True
    
    # 状态估计器健康
    estimator_covariance_norm: float = 0.0
    estimator_innovation_norm: float = 0.0
    estimator_slip_probability: float = 0.0
    estimator_imu_drift_detected: bool = False
    estimator_imu_bias: np.ndarray = field(default_factory=lambda: np.zeros(3))
    estimator_imu_available: bool = True
    
    # 跟踪误差 (所有误差均为绝对值，用于诊断和质量评估)
    tracking_lateral_error: float = 0.0       # 横向误差绝对值 (m)
    tracking_longitudinal_error: float = 0.0  # 纵向误差绝对值 (m)
    tracking_heading_error: float = 0.0       # 航向误差绝对值 (rad)
    tracking_prediction_error: float = float('nan')  # 预测误差 (m)，NaN 表示无数据
    
    # 跟踪质量评估 (基于 tracking 配置的阈值和权重计算)
    tracking_quality_score: float = 0.0       # 加权总分 (0-100)
    tracking_quality_rating: str = 'unknown'  # 质量等级 ('excellent', 'good', 'fair', 'poor')
    
    # 坐标变换状态
    transform_tf2_available: bool = True
    transform_tf2_injected: bool = False
    transform_fallback_duration_ms: float = 0.0
    transform_accumulated_drift: float = 0.0
    transform_source_frame: str = ''
    transform_target_frame: str = ''
    transform_error_message: str = ''
    
    # 超时状态
    timeout_odom: bool = False
    timeout_traj: bool = False
    timeout_traj_grace_exceeded: bool = False
    timeout_imu: bool = False
    timeout_last_odom_age_ms: float = 0.0
    timeout_last_traj_age_ms: float = 0.0
    timeout_last_imu_age_ms: float = 0.0
    timeout_in_startup_grace: bool = False
    
    # 控制命令
    cmd_vx: float = 0.0
    cmd_vy: float = 0.0
    cmd_vz: float = 0.0
    cmd_omega: float = 0.0
    cmd_frame_id: str = ''
    
    # 过渡进度
    transition_progress: float = 0.0
    
    # 安全状态
    safety_check_passed: bool = True
    emergency_stop: bool = False
    
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
                'prediction_error': self.tracking_prediction_error,
                'quality_score': self.tracking_quality_score,
                'quality_rating': self.tracking_quality_rating,
            },
            'transform': {
                'tf2_available': self.transform_tf2_available,
                'tf2_injected': self.transform_tf2_injected,
                'fallback_duration_ms': self.transform_fallback_duration_ms,
                'accumulated_drift': self.transform_accumulated_drift,
                'source_frame': self.transform_source_frame,
                'target_frame': self.transform_target_frame,
                'error_message': self.transform_error_message
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
            'transition_progress': self.transition_progress,
            'safety_check_passed': self.safety_check_passed,
            'emergency_stop': self.emergency_stop
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
