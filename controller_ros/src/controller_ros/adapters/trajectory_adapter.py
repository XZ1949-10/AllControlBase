"""
轨迹适配器

ROS 消息: controller_ros/LocalTrajectoryV4
UC 数据类型: universal_controller.core.data_types.Trajectory

轨迹模式映射:
- ROS: 0=MODE_TRACK, 1=MODE_STOP, 2=MODE_HOLD
- UC:  0=MODE_TRACK, 1=MODE_STOP, 2=MODE_HOVER, 3=MODE_EMERGENCY
- 注意: ROS 的 MODE_HOLD 对应 UC 的 MODE_HOVER (数值相同，语义相近)
"""
from typing import Any, Optional, Tuple
import logging
import numpy as np

from universal_controller.core.data_types import (
    Trajectory as UcTrajectory, Header, Point3D
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
# 默认配置值 (可通过配置覆盖)
# ============================================================================

# 默认坐标系，当 ROS 消息的 frame_id 为空时使用
DEFAULT_TRAJECTORY_FRAME_ID = 'base_link'

# 轨迹验证默认配置
DEFAULT_TRAJECTORY_CONFIG = {
    'max_coord': 100.0,              # 最大合理坐标值 (米)
    'velocity_decay_threshold': 0.1,  # 速度衰减填充阈值 (m/s)
    'min_dt_sec': 0.001,             # 最小时间间隔 (秒)
    'max_dt_sec': 10.0,              # 最大时间间隔 (秒)
    'default_dt_sec': 0.1,           # 默认时间间隔 (秒)
}


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
    
    配置参数 (通过 config 字典传入):
    - max_coord: 最大合理坐标值 (米)
    - velocity_decay_threshold: 速度衰减填充阈值 (m/s)
    - min_dt_sec: 最小时间间隔 (秒)
    - max_dt_sec: 最大时间间隔 (秒)
    - default_dt_sec: 默认时间间隔 (秒)
    """
    
    def __init__(self, config: dict = None):
        """
        初始化轨迹适配器
        
        Args:
            config: 配置字典，可选。支持的配置项见 DEFAULT_TRAJECTORY_CONFIG
        """
        super().__init__()
        
        # 合并配置
        self._config = DEFAULT_TRAJECTORY_CONFIG.copy()
        if config is not None:
            self._config.update(config)
        
        # 缓存常用配置值
        self._max_coord = self._config['max_coord']
        self._velocity_decay_threshold = self._config['velocity_decay_threshold']
        self._min_dt_sec = self._config['min_dt_sec']
        self._max_dt_sec = self._config['max_dt_sec']
        self._default_dt_sec = self._config['default_dt_sec']
    
    def _validate_dt_sec(self, dt_sec: float) -> float:
        """
        验证并修正 dt_sec 值
        
        Args:
            dt_sec: 原始时间间隔值
        
        Returns:
            有效的时间间隔值
        """
        if dt_sec <= 0 or dt_sec < self._min_dt_sec or dt_sec > self._max_dt_sec:
            if dt_sec <= 0:
                logger.warning(
                    f"Invalid dt_sec={dt_sec} (non-positive), using default {self._default_dt_sec}. "
                    f"This may indicate upstream trajectory generation issues."
                )
            else:
                logger.warning(
                    f"dt_sec={dt_sec} out of valid range [{self._min_dt_sec}, {self._max_dt_sec}], "
                    f"using default {self._default_dt_sec}."
                )
            return self._default_dt_sec
        return dt_sec
    
    def _process_velocities(self, velocities_flat: list, num_points: int, 
                           soft_enabled: bool, mode: int = 0) -> Tuple[Optional[np.ndarray], bool]:
        """
        处理速度数组
        
        Args:
            velocities_flat: 扁平化的速度数组
            num_points: 位置点数量
            soft_enabled: 是否启用 soft 模式
            mode: 轨迹模式 (用于决定填充策略)
        
        Returns:
            (velocities, soft_enabled): 处理后的速度数组和 soft 模式状态
        
        填充策略:
        - MODE_STOP/MODE_EMERGENCY: 使用零速度填充 (平滑停车)
        - 其他模式: 使用最后一个速度点填充 (保持运动连续性)
        """
        if not soft_enabled:
            return None, False
        
        if len(velocities_flat) == 0:
            logger.debug("soft_enabled=True but no velocity data, disabling soft mode")
            return None, False
        
        flat_len = len(velocities_flat)
        
        # 检查长度是否为 VELOCITY_DIMENSION 的倍数
        if flat_len % VELOCITY_DIMENSION != 0:
            logger.warning(
                f"velocities_flat length {flat_len} is not a multiple of {VELOCITY_DIMENSION} "
                f"(expected multiple of {VELOCITY_DIMENSION} for [vx, vy, vz, wz]), "
                f"truncating to {(flat_len // VELOCITY_DIMENSION) * VELOCITY_DIMENSION}. "
                f"This may indicate upstream data corruption."
            )
            flat_len = (flat_len // VELOCITY_DIMENSION) * VELOCITY_DIMENSION
        
        if flat_len == 0:
            logger.debug("No valid velocity data after truncation, disabling soft mode")
            return None, False
        
        velocities = np.array(velocities_flat[:flat_len]).reshape(-1, VELOCITY_DIMENSION)
        num_vel_points = velocities.shape[0]
        
        # 检查速度点数与位置点数是否匹配
        if num_vel_points != num_points:
            if num_vel_points > num_points:
                # 速度点多于位置点，截断
                logger.debug(
                    f"Velocity points ({num_vel_points}) > position points ({num_points}), truncating"
                )
                velocities = velocities[:num_points]
            else:
                # 速度点少于位置点，需要填充
                # 根据轨迹模式决定填充策略
                padding_count = num_points - num_vel_points
                
                # 判断是否为停止模式
                is_stop_mode = mode in (
                    TrajectoryMode.MODE_STOP.value, 
                    TrajectoryMode.MODE_EMERGENCY.value
                )
                
                if is_stop_mode:
                    # 停止模式：使用零速度填充，实现平滑停车
                    padding = np.zeros((padding_count, VELOCITY_DIMENSION))
                    logger.debug(
                        f"Velocity points ({num_vel_points}) < position points ({num_points}), "
                        f"padding with zeros for stop mode"
                    )
                else:
                    # 跟踪模式：使用衰减速度填充
                    # 设计说明：直接复制最后一个速度点可能导致轨迹末端继续高速运动
                    # 使用线性衰减到零，确保轨迹末端平滑减速
                    last_vel = velocities[-1, :]
                    last_vel_magnitude = np.sqrt(last_vel[0]**2 + last_vel[1]**2 + last_vel[2]**2)
                    
                    if last_vel_magnitude > self._velocity_decay_threshold:
                        logger.debug(
                            f"Velocity points ({num_vel_points}) < position points ({num_points}), "
                            f"padding with decaying velocity (magnitude={last_vel_magnitude:.2f} m/s)"
                        )
                        # 线性衰减到零：确保速度连续性
                        # 使用 (padding_count - i) / (padding_count + 1) 公式:
                        # - i=0: decay = padding_count / (padding_count + 1) ≈ 1 (接近原速度)
                        # - i=padding_count-1: decay = 1 / (padding_count + 1) (接近零但不为零)
                        # 这确保了从 last_vel 到填充点的平滑过渡
                        padding = np.zeros((padding_count, VELOCITY_DIMENSION))
                        for i in range(padding_count):
                            decay_factor = (padding_count - i) / (padding_count + 1)
                            padding[i, :] = last_vel * decay_factor
                    else:
                        # 速度很小，直接使用零填充
                        logger.debug(
                            f"Velocity points ({num_vel_points}) < position points ({num_points}), "
                            f"padding with zeros (last velocity magnitude={last_vel_magnitude:.2f} m/s is small)"
                        )
                        padding = np.zeros((padding_count, VELOCITY_DIMENSION))
                
                velocities = np.vstack([velocities, padding])
        
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
        
        for i, p in enumerate(points):
            # 检查 NaN 和 Inf
            if not (np.isfinite(p.x) and np.isfinite(p.y) and np.isfinite(p.z)):
                logger.warning(f"Trajectory point {i} contains NaN/Inf: ({p.x}, {p.y}, {p.z})")
                has_invalid = True
                continue
            
            # 检查坐标范围
            if abs(p.x) > self._max_coord or abs(p.y) > self._max_coord or abs(p.z) > self._max_coord:
                logger.warning(
                    f"Trajectory point {i} out of range (max={self._max_coord}m): "
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
        # 处理 frame_id，空字符串使用默认值
        frame_id = ros_msg.header.frame_id
        if not frame_id:
            frame_id = DEFAULT_TRAJECTORY_FRAME_ID
        
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
