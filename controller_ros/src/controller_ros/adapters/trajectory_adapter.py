"""
轨迹适配器

ROS 消息: controller_ros/LocalTrajectoryV4
UC 数据类型: universal_controller.core.data_types.Trajectory

轨迹模式映射:
- ROS: 0=MODE_TRACK, 1=MODE_STOP, 2=MODE_HOLD
- UC:  0=MODE_TRACK, 1=MODE_STOP, 2=MODE_HOVER, 3=MODE_EMERGENCY
- 注意: ROS 的 MODE_HOLD 对应 UC 的 MODE_HOVER (数值相同，语义相近)

配置说明:
- 配置参数统一通过 TrajectoryDefaults 类获取
- TrajectoryDefaults 是轨迹配置的单一数据源 (Single Source of Truth)
- 在使用前应调用 TrajectoryDefaults.configure(config) 初始化配置
"""
from typing import Any, Optional, Tuple, Dict
import logging
import numpy as np

from universal_controller.core.data_types import (
    Trajectory as UcTrajectory, Header, Point3D, TrajectoryDefaults
)
from universal_controller.core.enums import TrajectoryMode
from .base import IMsgConverter

logger = logging.getLogger(__name__)

# ============================================================================
# 协议常量 (不应修改)
# ============================================================================

# 速度向量维度 [vx, vy, vz, wz]
# 这是 ROS 消息格式的固定定义，不应该通过配置修改
VELOCITY_DIMENSION = 4

# ============================================================================
# 默认坐标系 (当 ROS 消息的 frame_id 为空时使用)
# ============================================================================
DEFAULT_TRAJECTORY_FRAME_ID = 'base_link'


class TrajectoryAdapter(IMsgConverter):
    """
    轨迹消息适配器
    
    将 ROS LocalTrajectoryV4 转换为 UC Trajectory 数据类型。
    
    速度数组处理策略:
    - 速度点数 > 位置点数: 截断速度数组
    - 速度点数 < 位置点数: 使用最后一个速度点填充 (而非零填充)
    - 速度点数 = 0 且 soft_enabled=True: 禁用 soft 模式
    
    坐标系处理:
    - 如果 ROS 消息的 frame_id 为空，使用默认值 'base_link'
    - 网络输出的轨迹通常在 base_link 坐标系下
    
    配置说明:
    - 所有配置参数通过 TrajectoryDefaults 类获取
    - 在使用前应确保已调用 TrajectoryDefaults.configure(config)
    - 配置参数包括: velocity_decay_threshold, min_dt_sec, max_dt_sec, dt_sec
    - max_coord 使用 constants.py 中的 TRAJECTORY_MAX_COORD 常量
    """
    
    def __init__(self, config: Dict[str, Any] = None):
        """
        初始化轨迹适配器
        
        Args:
            config: 配置字典，可选。如果提供，会调用 TrajectoryDefaults.configure() 更新配置。
                   
        配置说明:
            如果传入 config，会更新 TrajectoryDefaults 的配置。
            所有参数从 TrajectoryDefaults 类属性读取，确保配置的单一数据源。
        """
        super().__init__()
        
        # 如果提供了配置，更新 TrajectoryDefaults
        if config is not None:
            TrajectoryDefaults.configure(config)
        
        # 注意: 不再缓存配置值，直接从 TrajectoryDefaults 读取
        # 这确保了配置的一致性，即使在运行时更新配置也能生效
    
    def _validate_dt_sec(self, dt_sec: float) -> float:
        """
        验证并修正 dt_sec 值
        
        Args:
            dt_sec: 原始时间间隔值
        
        Returns:
            有效的时间间隔值
        """
        # 使用 constants.py 中的常量
        from universal_controller.core.constants import TRAJECTORY_MIN_DT_SEC, TRAJECTORY_MAX_DT_SEC
        min_dt = TRAJECTORY_MIN_DT_SEC
        max_dt = TRAJECTORY_MAX_DT_SEC
        default_dt = TrajectoryDefaults.dt_sec
        
        if dt_sec <= 0 or dt_sec < min_dt or dt_sec > max_dt:
            if dt_sec <= 0:
                logger.warning(
                    f"Invalid dt_sec={dt_sec} (non-positive), using default {default_dt}. "
                    f"This may indicate upstream trajectory generation issues."
                )
            else:
                logger.warning(
                    f"dt_sec={dt_sec} out of valid range [{min_dt}, {max_dt}], "
                    f"using default {default_dt}."
                )
            return default_dt
        return dt_sec
    
    def _process_velocities(self, velocities_flat: list, num_points: int, 
                           soft_enabled: bool, mode: int = 0) -> Tuple[Optional[np.ndarray], bool]:
        """
        处理速度数组
        
        Args:
            velocities_flat: 扁平化的速度数组
            num_points: 位置点数量
            soft_enabled: 是否启用 soft 模式
            mode: 轨迹模式
        
        Returns:
            (velocities, soft_enabled): 处理后的速度数组和 soft 模式状态
        
        策略 (Refactored):
        - 严格校验: 速度点数量必须与位置点数量完全一致
        - 移除隐式填充: 不再自动进行零填充或衰减填充
        - 安全降级: 如果数量不匹配，丢弃所有速度数据并禁用 soft 模式 (回退到纯位置跟踪)
        """
        if not soft_enabled:
            return None, False
        
        if len(velocities_flat) == 0:
            return None, False
        
        flat_len = len(velocities_flat)
        
        # 维度检查
        if flat_len % VELOCITY_DIMENSION != 0:
            logger.warning(
                f"velocities_flat length {flat_len} is not a multiple of {VELOCITY_DIMENSION}, "
                f"discarding velocity data."
            )
            return None, False
            
        velocities = np.array(velocities_flat[:flat_len]).reshape(-1, VELOCITY_DIMENSION)
        num_vel_points = velocities.shape[0]
        
        # 严格数量检查
        if num_vel_points != num_points:
            logger.warning(
                f"Velocity points ({num_vel_points}) != position points ({num_points}). "
                f"Strict validation failed. Discarding velocity data to prevent behavioral ambiguity."
            )
            return None, False
            
        return velocities, True
    
    def _validate_points(self, points: list) -> Tuple[list, bool]:
        """
        验证轨迹点的有效性
        
        Args:
            points: 轨迹点列表
        
        Returns:
            (validated_points, is_valid): 验证后的点列表和是否有效
        """
        if not points:
            return [], False
        
        validated = []
        has_invalid = False
        # 使用 constants.py 中的常量
        from universal_controller.core.constants import TRAJECTORY_MAX_COORD
        max_coord = TRAJECTORY_MAX_COORD
        
        for i, p in enumerate(points):
            # 检查 NaN 和 Inf
            if not (np.isfinite(p.x) and np.isfinite(p.y) and np.isfinite(p.z)):
                logger.warning(f"Trajectory point {i} contains NaN/Inf: ({p.x}, {p.y}, {p.z})")
                has_invalid = True
                continue
            
            # 检查坐标范围
            if abs(p.x) > max_coord or abs(p.y) > max_coord or abs(p.z) > max_coord:
                logger.warning(
                    f"Trajectory point {i} out of range (max={max_coord}m): "
                    f"({p.x}, {p.y}, {p.z})"
                )
                has_invalid = True
                continue
            
            validated.append(Point3D(x=p.x, y=p.y, z=p.z))
        
        if has_invalid:
            logger.warning(
                f"Trajectory validation: {len(points) - len(validated)} invalid points removed, "
                f"{len(validated)} valid points remaining"
            )
        
        return validated, len(validated) > 0
    
    def to_uc(self, ros_msg: Any) -> UcTrajectory:
        """ROS LocalTrajectoryV4 → UC Trajectory"""
        # 严格校验 frame_id
        # 安全性改进: 拒绝隐式坐标系，防止 map/odom/base_link 混淆导致的事故
        frame_id = ros_msg.header.frame_id
        if not frame_id:
            raise ValueError("Trajectory message must have a valid frame_id in header! Implicit assumption of 'base_link' is unsafe.")
        
        # 验证并转换轨迹点
        raw_points = [
            Point3D(x=p.x, y=p.y, z=p.z)
            for p in ros_msg.points
        ]
        points, points_valid = self._validate_points(raw_points)
        num_points = len(points)
        
        # 处理空轨迹的边界情况
        if num_points == 0:
            logger.warning(
                "Received empty trajectory (0 points), returning MODE_STOP trajectory. "
                "This may indicate upstream trajectory generation issues."
            )
            return UcTrajectory(
                header=Header(
                    stamp=self._ros_time_to_sec(ros_msg.header.stamp),
                    frame_id=frame_id
                ),
                points=[],
                velocities=None,
                dt_sec=self._validate_dt_sec(ros_msg.dt_sec),
                confidence=0.0,
                mode=TrajectoryMode.MODE_STOP,
                soft_enabled=False
            )
        
        # 处理速度数组 (传入 mode 用于决定填充策略)
        velocities, soft_enabled = self._process_velocities(
            ros_msg.velocities_flat, 
            num_points, 
            ros_msg.soft_enabled,
            ros_msg.mode  # 传入轨迹模式
        )
        
        # 转换轨迹模式
        # ROS: 0=MODE_TRACK, 1=MODE_STOP, 2=MODE_HOLD
        # UC:  0=MODE_TRACK, 1=MODE_STOP, 2=MODE_HOVER, 3=MODE_EMERGENCY
        try:
            mode = TrajectoryMode(ros_msg.mode)
        except ValueError:
            logger.warning(f"Unknown trajectory mode {ros_msg.mode}, using MODE_TRACK")
            mode = TrajectoryMode.MODE_TRACK
        
        # 验证 dt_sec
        dt_sec = self._validate_dt_sec(ros_msg.dt_sec)
        
        # 验证并限制置信度在 [0, 1] 范围内
        confidence = ros_msg.confidence
        if confidence < 0.0 or confidence > 1.0:
            logger.warning(
                f"Trajectory confidence {confidence} out of valid range [0, 1], "
                f"clamping to valid range."
            )
            confidence = max(0.0, min(1.0, confidence))
        
        return UcTrajectory(
            header=Header(
                stamp=self._ros_time_to_sec(ros_msg.header.stamp),
                frame_id=frame_id
            ),
            points=points,
            velocities=velocities,
            dt_sec=dt_sec,
            confidence=confidence,
            mode=mode,
            soft_enabled=soft_enabled
        )
    
    def to_ros(self, uc_data: UcTrajectory) -> Any:
        """UC Trajectory → ROS LocalTrajectoryV4"""
        # 延迟导入避免循环依赖
        try:
            from controller_ros.msg import LocalTrajectoryV4
            from geometry_msgs.msg import Point
        except ImportError:
            raise ImportError("ROS messages not available")
        
        ros_msg = LocalTrajectoryV4()
        ros_msg.header.stamp = self._sec_to_ros_time(uc_data.header.stamp)
        ros_msg.header.frame_id = uc_data.header.frame_id or DEFAULT_TRAJECTORY_FRAME_ID
        
        # 安全地转换 mode 值，支持枚举和整数
        if isinstance(uc_data.mode, TrajectoryMode):
            ros_msg.mode = uc_data.mode.value
        else:
            ros_msg.mode = int(uc_data.mode)
        
        ros_msg.dt_sec = float(uc_data.dt_sec)
        ros_msg.confidence = float(uc_data.confidence)
        ros_msg.soft_enabled = uc_data.soft_enabled
        
        ros_msg.points = [
            Point(x=float(p.x), y=float(p.y), z=float(p.z))
            for p in uc_data.points
        ]
        
        if uc_data.velocities is not None and len(uc_data.velocities) > 0:
            ros_msg.velocities_flat = uc_data.velocities.flatten().tolist()
        else:
            ros_msg.velocities_flat = []
        
        return ros_msg
