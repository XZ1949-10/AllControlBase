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

# 默认坐标系，当 ROS 消息的 frame_id 为空时使用
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
    """
    
    def _process_velocities(self, velocities_flat: list, num_points: int, 
                           soft_enabled: bool) -> Tuple[Optional[np.ndarray], bool]:
        """
        处理速度数组
        
        Args:
            velocities_flat: 扁平化的速度数组
            num_points: 位置点数量
            soft_enabled: 是否启用 soft 模式
        
        Returns:
            (velocities, soft_enabled): 处理后的速度数组和 soft 模式状态
        """
        if not soft_enabled:
            return None, False
        
        if len(velocities_flat) == 0:
            logger.debug("soft_enabled=True but no velocity data, disabling soft mode")
            return None, False
        
        flat_len = len(velocities_flat)
        
        # 检查长度是否为 4 的倍数
        if flat_len % 4 != 0:
            logger.warning(
                f"velocities_flat length {flat_len} is not a multiple of 4 "
                f"(expected multiple of 4 for [vx, vy, vz, wz]), truncating to {(flat_len // 4) * 4}. "
                f"This may indicate upstream data corruption."
            )
            flat_len = (flat_len // 4) * 4
        
        if flat_len == 0:
            logger.debug("No valid velocity data after truncation, disabling soft mode")
            return None, False
        
        velocities = np.array(velocities_flat[:flat_len]).reshape(-1, 4)
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
                # 速度点少于位置点，使用最后一个速度点填充 (比零填充更合理)
                logger.warning(
                    f"Velocity points ({num_vel_points}) < position points ({num_points}), "
                    f"padding with last velocity"
                )
                last_vel = velocities[-1:, :]  # 保持 2D 形状
                padding = np.tile(last_vel, (num_points - num_vel_points, 1))
                velocities = np.vstack([velocities, padding])
        
        return velocities, True
    
    def to_uc(self, ros_msg: Any) -> UcTrajectory:
        """ROS LocalTrajectoryV4 → UC Trajectory"""
        # 处理 frame_id，空字符串使用默认值
        frame_id = ros_msg.header.frame_id
        if not frame_id:
            frame_id = DEFAULT_TRAJECTORY_FRAME_ID
        
        # 转换轨迹点
        points = [
            Point3D(x=p.x, y=p.y, z=p.z)
            for p in ros_msg.points
        ]
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
                dt_sec=ros_msg.dt_sec if ros_msg.dt_sec > 0 else 0.1,
                confidence=0.0,
                mode=TrajectoryMode.MODE_STOP,
                soft_enabled=False
            )
        
        # 处理速度数组
        velocities, soft_enabled = self._process_velocities(
            ros_msg.velocities_flat, 
            num_points, 
            ros_msg.soft_enabled
        )
        
        # 转换轨迹模式
        # ROS: 0=MODE_TRACK, 1=MODE_STOP, 2=MODE_HOLD
        # UC:  0=MODE_TRACK, 1=MODE_STOP, 2=MODE_HOVER, 3=MODE_EMERGENCY
        try:
            mode = TrajectoryMode(ros_msg.mode)
        except ValueError:
            logger.warning(f"Unknown trajectory mode {ros_msg.mode}, using MODE_TRACK")
            mode = TrajectoryMode.MODE_TRACK
        
        return UcTrajectory(
            header=Header(
                stamp=self._ros_time_to_sec(ros_msg.header.stamp),
                frame_id=frame_id
            ),
            points=points,
            velocities=velocities,
            dt_sec=ros_msg.dt_sec,
            confidence=ros_msg.confidence,
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
