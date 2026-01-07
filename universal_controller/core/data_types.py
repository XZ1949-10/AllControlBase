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
from .constants import normalize_angle, CONFIDENCE_MIN, CONFIDENCE_MAX

# 配置日志
logger = logging.getLogger(__name__)



@dataclass
class TrajectoryConfig:
    """
    Trajectory configuration parameters.
    Passed to ControllerManager to avoid global state.
    """
    dt_sec: float = 0.1
    low_speed_thresh: float = 0.1
    default_confidence: float = 0.9
    default_frame_id: str = 'base_link'
    min_points: int = 2
    max_points: int = 100
    max_point_distance: float = 10.0
    velocity_decay_threshold: float = 0.1
    validate_enabled: bool = True
    legacy_points_enabled: bool = False

    @classmethod
    def from_dict(cls, config: Dict[str, Any]) -> 'TrajectoryConfig':
        """Create from configuration dictionary"""
        traj_config = config.get('trajectory', {})
        transform_config = config.get('transform', {})
        mpc_config = config.get('mpc', {})
        
        dt = traj_config.get('default_dt_sec', mpc_config.get('dt', 0.1))
        
        return cls(
            dt_sec=dt,
            low_speed_thresh=traj_config.get('low_speed_thresh', 0.1),
            default_confidence=traj_config.get('default_confidence', 0.9),
            default_frame_id=transform_config.get('source_frame', 'base_link'),
            min_points=traj_config.get('min_points', 2),
            max_points=traj_config.get('max_points', 100),
            max_point_distance=traj_config.get('max_point_distance', 10.0),
            velocity_decay_threshold=traj_config.get('velocity_decay_threshold', 0.1),
            legacy_points_enabled=traj_config.get('legacy_points_enabled', False)
        )


# 模拟 ROS Header
@dataclass
class Header:
    """ROS Header 模拟"""
    stamp: float = 0.0  # 时间戳 (秒)
    frame_id: str = ""
    seq: int = 0


@dataclass(slots=True)
class Point3D:
    """3D 点"""
    x: float
    y: float
    z: float


@dataclass(slots=True)
class Twist3D:
    """3D 速度"""
    vx: float
    vy: float
    vz: float
    wz: float


@dataclass(slots=True)
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

    缓存机制:
        get_hard_velocities() 使用内部缓存，避免重复计算。
        缓存在以下情况下失效：
        - 首次调用
        - points 或 dt_sec 发生变化（通过 _cache_key 检测）

    线程安全性:
        此类不是线程安全的。如果需要在多线程环境中使用：
        - 方案1: 在外部加锁保护对 Trajectory 对象的访问
        - 方案2: 使用 copy() 方法创建副本，每个线程使用独立副本
        - 方案3: 只在单线程中修改，其他线程只读（读取缓存结果是安全的）
    """
    header: Header
    points: List[Point3D]
    velocities: Optional[np.ndarray]  # [N, 4]: [vx, vy, vz, wz]
    dt_sec: float = 0.1  # 默认 0.1s，不再依赖 TrajectoryDefaults
    confidence: float = 0.9  # 默认 0.9
    mode: TrajectoryMode = TrajectoryMode.MODE_TRACK
    soft_enabled: bool = False
    low_speed_thresh: float = field(default=0.1, repr=False, compare=False)
    
    # 缓存字段（不参与比较和 repr）
    _hard_velocities_cache: Optional[np.ndarray] = field(default=None, repr=False, compare=False, init=False)
    _points_matrix_cache: Optional[np.ndarray] = field(default=None, repr=False, compare=False, init=False)
    _version: int = field(default=0, repr=False, compare=False, init=False)
    _points_are_numpy: bool = field(default=False, repr=False, compare=False, init=False)
    
    # 显式定义私有字段以支持 slots=True
    _cache_key: Optional[tuple] = field(default=None, repr=False, compare=False, init=False)
    _last_validated_version: int = field(default=-1, repr=False, compare=False, init=False)
    _last_validation_result: bool = field(default=True, repr=False, compare=False, init=False)

    def __post_init__(self):
        # 初始化 version 和 cache_key
        # 修复 Bug: 不要重置 version，如果 __setattr__ 已经设置过了
        if not hasattr(self, '_version'):
            object.__setattr__(self, '_version', 0)
        if not hasattr(self, '_cache_key'):
            object.__setattr__(self, '_cache_key', None)
        
        # 优化: 移除已被 __setattr__ 处理过的冗余逻辑
        # 仅处理尚未被处理的情况 (例如直接调用 __init__ 而非通过属性赋值时，尽管 Dataclass 会触发 setattr)
        # 为由于 Dataclass 生成的 __init__ 会对每个字段调用 setattr，
        # 此时 self.points 已经被 __setattr__ 处理并转为 numpy array 和 只读
        # 这里只需要进行最终的卫语句检查
        
        # 确保维度正确 (安全检查)
        if self.points.ndim != 2 or self.points.shape[1] != 3:
            if self.points.ndim == 1 and self.points.size == 0:
                # 修复形状
                object.__setattr__(self, 'points', np.zeros((0, 3), dtype=np.float64))
                self.points.flags.writeable = False
        
        # 标记为 numpy (已废弃 _points_are_numpy 检查，现在总是 API Contract)
        object.__setattr__(self, '_points_are_numpy', True)
        
        # 保护性编程: 禁止原地修改以保证缓存一致性
        if isinstance(self.points, np.ndarray):
            self.points.flags.writeable = False
        
        # 验证 confidence
        if self.confidence is None or not np.isfinite(self.confidence):
            self.confidence = 0.9
        else:
            self.confidence = np.clip(self.confidence, CONFIDENCE_MIN, CONFIDENCE_MAX)
        
        # 验证 dt_sec
        if self.dt_sec is None or not np.isfinite(self.dt_sec) or self.dt_sec <= 0:
            self.dt_sec = 0.1
            
        # 验证 Velocities 维度
        num_points = len(self.points)
        if self.velocities is not None and num_points > 0:
            # 确保 velocities 也是 numpy
            if not isinstance(self.velocities, np.ndarray):
                self.velocities = np.array(self.velocities, dtype=np.float64)
                 
            if len(self.velocities) != num_points:
                if len(self.velocities) > num_points:
                    self.velocities = self.velocities[:num_points]
                else:
                    pad_len = num_points - len(self.velocities)
                    self.velocities = np.pad(self.velocities, ((0, pad_len), (0, 0)), 'constant')
    
    def __setattr__(self, name: str, value: Any) -> None:
        """
        重写 __setattr__ 以自动处理缓存失效
        
        当关键属性被修改时，自动递增 version，从而使缓存失效。
        注意: 这无法捕获 list 的原地修改 (e.g. traj.points.append)。
        对于原地修改，用户必须手动调用 mark_dirty()。
        """
        if name in ('points', 'dt_sec', 'confidence', 'velocities', 'soft_enabled', 'low_speed_thresh'):
            # 处理 points 的特殊赋值逻辑：确保它是 Read-only Numpy Array
            if name == 'points':
                # 类型转换与验证 (复用 __post_init__ 中的逻辑)
                if isinstance(value, np.ndarray):
                    if value.dtype != np.float64:
                        value = value.astype(np.float64)
                    # 确保是 C 连续的，有利于底层性能
                    if not value.flags['C_CONTIGUOUS']:
                        value = np.ascontiguousarray(value)
                else:
                    # 列表转换
                    if not value:
                        value = np.zeros((0, 3), dtype=np.float64)
                    else:
                        try:
                            value = np.array(value, dtype=np.float64)
                        except Exception:
                            # 简化的回退逻辑
                            value = np.array([[p.x, p.y, p.z] for p in value], dtype=np.float64)
                
                # 维度检查
                if value.ndim != 2 or value.shape[1] != 3:
                    if value.ndim == 1 and value.size == 0:
                        value = value.reshape(0, 3)
                
                # 锁定数组
                if hasattr(value, 'flags'):
                     value.flags.writeable = False
                
            # 直接更新 version，移除 try-except 开销 (如果 Trajectory 被频繁创建/修改)
            # 因为 _version 在 __post_init__ 保证存在
            if name != '_version':
                 curr = getattr(self, '_version', 0)
                 object.__setattr__(self, '_version', curr + 1)
            
            # 使用 object.__setattr__
            object.__setattr__(self, name, value)
            return

        # 其他属性直接设置
        object.__setattr__(self, name, value)


    def mark_dirty(self) -> None:
        """
        手动标记轨迹数据已修改
        
        用于在使用原地修改操作（如 points.append, points[i]=...）后
        通知 Trajectory 更新版本号，从而使缓存失效。
        """
        object.__setattr__(self, '_version', self._version + 1)
    
    def validate(self, config: 'TrajectoryConfig') -> bool:
        """
        验证轨迹数据的有效性 (Vectorized)
        
        Args:
            config: 配置对象，提供验证阈值
            
        Returns:
            bool: 验证是否通过
        """
        from .constants import TRAJECTORY_MIN_DT_SEC, TRAJECTORY_MAX_DT_SEC
        
        # 优化: 如果版本没有改变，直接返回上次验证结果
        current_version = getattr(self, '_version', 0)
        last_validated_ver = getattr(self, '_last_validated_version', -1)
        if current_version == last_validated_ver:
            return getattr(self, '_last_validation_result', True)
        
        is_valid = True
        
        # 验证 dt_sec
        if self.dt_sec < TRAJECTORY_MIN_DT_SEC or self.dt_sec > TRAJECTORY_MAX_DT_SEC:
            logger.debug(
                f"Trajectory dt_sec={self.dt_sec} out of range "
                f"[{TRAJECTORY_MIN_DT_SEC}, {TRAJECTORY_MAX_DT_SEC}], clipping"
            )
            self.dt_sec = np.clip(self.dt_sec, TRAJECTORY_MIN_DT_SEC, TRAJECTORY_MAX_DT_SEC)
            # 自动修复被视为有效，不置为 False
            # is_valid = False
        
        # 向量化验证相邻点距离
        # 使用 get_points_matrix() 获取统一的 numpy 视图，避免处理类型差异
        if config.validate_enabled and len(self.points) >= 2:
            points_mat = self.get_points_matrix()
            max_dist_sq = config.max_point_distance ** 2
            
            # 计算相邻点距离平方 [N-1]
            diffs = points_mat[1:] - points_mat[:-1]
            dist_sq = np.sum(diffs**2, axis=1)
            
            # 查找跳变
            jump_indices = np.where(dist_sq > max_dist_sq)[0]
            if len(jump_indices) > 0:
                first_idx = jump_indices[0]
                logger.error(
                    f"Trajectory jump detected at index {first_idx}->{first_idx+1}: "
                    f"{np.sqrt(dist_sq[first_idx]):.2f}m > {config.max_point_distance}m"
                )
                is_valid = False
        
        # 更新缓存
        object.__setattr__(self, '_last_validated_version', current_version)
        object.__setattr__(self, '_last_validation_result', is_valid)
        
        return is_valid
    
    def __len__(self) -> int:
        return len(self.points)
    
    def _compute_cache_key(self) -> tuple:
        """
        基于版本号的高效缓存键计算
        
        现在基于 _version 字段，它是 O(1) 的，比原来的 O(N) 或采样方法更有性能且更健壮。
        """
        return (self._version, self.dt_sec)
            
    def get_points_matrix(self) -> np.ndarray:
        """获取点坐标矩阵 [N, 3] (Strict Numpy Zero-Copy)"""
        # 由于 __post_init__ 保证了 self.points 是 numpy array
        return self.points
    
    def get_slice(self, start: int, end: int) -> 'Trajectory':
        """
        获取轨迹切片（返回独立副本，确保数据隔离）
        
        Args:
            start: 起始索引
            end: 结束索引
            
        Returns:
            Trajectory: 切片后的新轨迹对象（独立副本）
            
        Note:
            为确保数据安全，切片返回的是独立副本而非视图。
            这避免了原轨迹修改影响切片数据的风险。
            对于性能敏感场景，numpy 的小数组复制开销可忽略不计。
        """
        # 处理索引边界
        total_points = len(self.points)
        if start < 0: start = 0
        if end > total_points: end = total_points
        if start > end: start = end
        
        # 创建独立副本而非视图，确保数据隔离
        # np.array(..., copy=True) 显式创建副本
        new_points = np.array(self.points[start:end], copy=True)
        # 设置为只读，保持与原 Trajectory 一致的不可变语义
        new_points.flags.writeable = False
        
        # 切片 Velocities（同样创建副本）
        new_velocities = None
        if self.velocities is not None:
            v_len = len(self.velocities)
            v_start = min(start, v_len)
            v_end = min(end, v_len)
            new_velocities = np.array(self.velocities[v_start:v_end], copy=True)
            
        # 创建新实例（高性能路径：绕过 __init__ 和 __post_init__）
        new_traj = object.__new__(Trajectory)
        
        # 直接赋值
        object.__setattr__(new_traj, 'header', self.header)
        object.__setattr__(new_traj, 'points', new_points)
        object.__setattr__(new_traj, 'velocities', new_velocities)
        object.__setattr__(new_traj, 'dt_sec', self.dt_sec)
        object.__setattr__(new_traj, 'confidence', self.confidence)
        object.__setattr__(new_traj, 'mode', self.mode)
        object.__setattr__(new_traj, 'soft_enabled', self.soft_enabled)
        object.__setattr__(new_traj, 'low_speed_thresh', self.low_speed_thresh)
        
        # 初始化内部字段
        object.__setattr__(new_traj, '_version', 0)
        object.__setattr__(new_traj, '_cache_key', None)
        
        # 复用 Hard Cache（创建独立副本）
        new_hard_vels = None
        current_version = getattr(self, '_version', 0)
        
        if (self._hard_velocities_cache is not None and 
            getattr(self, '_cache_key', None) == (current_version, self.dt_sec)):
            
            cache_len = len(self._hard_velocities_cache)
            c_start = min(start, cache_len)
            c_end = min(end, cache_len)
            
            if c_end > c_start:
                # 创建独立副本，避免缓存污染
                new_hard_vels = np.array(self._hard_velocities_cache[c_start:c_end], copy=True)
        
        object.__setattr__(new_traj, '_hard_velocities_cache', new_hard_vels)
        
        if new_hard_vels is not None:
             object.__setattr__(new_traj, '_cache_key', (0, self.dt_sec))
             
        object.__setattr__(new_traj, '_points_matrix_cache', new_points)
        object.__setattr__(new_traj, '_points_are_numpy', True)
        object.__setattr__(new_traj, '_last_validated_version', -1)
        object.__setattr__(new_traj, '_last_validation_result', True)
        
        return new_traj
    
    def copy(self) -> 'Trajectory':
        new_header = Header(stamp=self.header.stamp, frame_id=self.header.frame_id, seq=self.header.seq)
        new_velocities = None if self.velocities is None else np.array(self.velocities, copy=True)
        
        # Deep copy points (Always Numpy now)
        new_points = np.array(self.points, copy=True)

        return Trajectory(
            header=new_header,
            points=new_points,
            velocities=new_velocities,
            dt_sec=self.dt_sec,
            confidence=self.confidence,
            mode=self.mode,
            soft_enabled=self.soft_enabled,
            low_speed_thresh=self.low_speed_thresh
        )
    
    def get_hard_velocities(self) -> np.ndarray:
        """使用向量化计算 Hard 轨迹隐含速度 [N, 4]"""
        current_key = self._compute_cache_key()
        if self._hard_velocities_cache is not None and self._cache_key == current_key:
            return self._hard_velocities_cache
        
        # 获取点矩阵 (会自动更新 _points_matrix_cache 和 _cache_key)
        points_mat = self.get_points_matrix()
        self._cache_key = current_key # 确保 key 同步
        n_points = len(points_mat)
        
        if n_points < 2:
            result = np.zeros((max(1, n_points), 4))
            self._hard_velocities_cache = result
            return result
        
        # 一阶差分: V[i] = (P[i+1] - P[i]) / dt
        diffs = np.diff(points_mat, axis=0) / self.dt_sec # [N-1, 3]
        
        vx = diffs[:, 0]
        vy = diffs[:, 1]
        vz = diffs[:, 2]
        
        # 扩展到 N 个点: 中间点使用中心差分 V[i] = (v[i-1] + v[i]) / 2
        vx_full = np.empty(n_points)
        vy_full = np.empty(n_points)
        vz_full = np.empty(n_points)
        
        # 边界处理: 首尾直接使用前向/后向差分
        vx_full[0], vx_full[-1] = vx[0], vx[-1]
        vy_full[0], vy_full[-1] = vy[0], vy[-1]
        vz_full[0], vz_full[-1] = vz[0], vz[-1]
        
        # 中间点插值
        if n_points > 2:
            vx_full[1:-1] = (vx[:-1] + vx[1:]) * 0.5
            vy_full[1:-1] = (vy[:-1] + vy[1:]) * 0.5
            vz_full[1:-1] = (vz[:-1] + vz[1:]) * 0.5
            
        # 计算角速度 wz
        # heading = atan2(vy, vx)
        headings = np.arctan2(vy_full, vx_full)
        dheadings = np.diff(headings)
        
        # 归一化角差
        dheadings = normalize_angle(dheadings)
        
        wz_full = np.empty(n_points)
        wz_full[:-1] = dheadings / self.dt_sec
        wz_full[-1] = 0.0 # 最后一个点角速度为 0 或保持
        
        # 低速抑制
        speed_sq = vx_full**2 + vy_full**2
        low_speed_mask = speed_sq < (self.low_speed_thresh**2)
        wz_full[low_speed_mask] = 0.0
        
        result = np.column_stack((vx_full, vy_full, vz_full, wz_full))
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
        """获取混合后的速度向量 (单点)"""
        idx_end = min(index + 1, len(self.points))
        res = self.get_blended_velocities_slice(index, idx_end, alpha)
        if len(res) > 0:
            return res[0]
        return np.zeros(4)

    def get_blended_velocities_slice(self, start: int, end: int, alpha: float) -> np.ndarray:
        """批量获取混合速度 [M, 4]"""
        hard_vels = self.get_hard_velocities()
        n_points = len(hard_vels)
        
        start = max(0, min(start, n_points))
        end = max(start, min(end, n_points))
        
        if start >= end:
            return np.zeros((0, 4))
            
        v_hard_slice = hard_vels[start:end]
        
        if not self.has_valid_soft_velocities() or alpha <= 1e-6:
            return v_hard_slice.copy()
            
        if alpha >= 1.0 - 1e-6:
            return self.velocities[start:end].copy()
            
        v_soft_slice = self.velocities[start:end]
        min_len = min(len(v_hard_slice), len(v_soft_slice))
        
        return alpha * v_soft_slice[:min_len] + (1 - alpha) * v_hard_slice[:min_len]

    def get_blended_speed(self, index: int, alpha: float) -> float:
        """获取混合后的水平速度大小"""
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


@dataclass(slots=True)
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
    extras: Dict[str, Any] = field(default_factory=dict)
    
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
            health_metrics=self.health_metrics.copy() if self.health_metrics else {},
            extras=self.extras.copy() if self.extras else {}
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


@dataclass(slots=True)
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
    
    # 内部缓存字典，避免重复创建
    _cached_ros_msg: Dict[str, Any] = field(default=None, repr=False, init=False)
    _dirty: bool = field(default=True, repr=False, init=False)

    def __setattr__(self, name: str, value: Any) -> None:
        """重写 setattr 以标记 dirty"""
        # 注意: 这里用 object.__setattr__ 避免递归
        if name != '_cached_ros_msg' and name != '_dirty':
            object.__setattr__(self, '_dirty', True)
        object.__setattr__(self, name, value)

    def to_ros_msg(self) -> Dict[str, Any]:
        """转换为 ROS 消息格式的字典 (带缓存)"""
        # 如果缓存有效且未修改，直接返回
        if not getattr(self, '_dirty', True) and getattr(self, '_cached_ros_msg', None) is not None:
            return self._cached_ros_msg

        msg = {
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
        
        object.__setattr__(self, '_cached_ros_msg', msg)
        object.__setattr__(self, '_dirty', False)
        return msg


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
