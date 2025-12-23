"""
轨迹适配器

ROS 消息: controller_ros/LocalTrajectoryV4
UC 数据类型: universal_controller.core.data_types.Trajectory
"""
from typing import Any
import numpy as np

from universal_controller.core.data_types import (
    Trajectory as UcTrajectory, Header, Point3D
)
from universal_controller.core.enums import TrajectoryMode
from .base import IMsgConverter


class TrajectoryAdapter(IMsgConverter):
    """
    轨迹消息适配器
    
    将 ROS LocalTrajectoryV4 转换为 UC Trajectory 数据类型。
    """
    
    def to_uc(self, ros_msg: Any) -> UcTrajectory:
        """ROS LocalTrajectoryV4 → UC Trajectory"""
        # 转换轨迹点
        points = [
            Point3D(x=p.x, y=p.y, z=p.z)
            for p in ros_msg.points
        ]
        
        # 转换速度数组
        velocities = None
        if ros_msg.soft_enabled and len(ros_msg.velocities_flat) > 0:
            # 从扁平数组重建 [N, 4] 数组
            velocities = np.array(ros_msg.velocities_flat).reshape(-1, 4)
        
        # 转换轨迹模式
        try:
            mode = TrajectoryMode(ros_msg.mode)
        except ValueError:
            mode = TrajectoryMode.MODE_TRACK
        
        return UcTrajectory(
            header=Header(
                stamp=self._ros_time_to_sec(ros_msg.header.stamp),
                frame_id=ros_msg.header.frame_id
            ),
            points=points,
            velocities=velocities,
            dt_sec=ros_msg.dt_sec,
            confidence=ros_msg.confidence,
            mode=mode,
            soft_enabled=ros_msg.soft_enabled
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
        ros_msg.header.frame_id = uc_data.header.frame_id
        ros_msg.mode = int(uc_data.mode.value) if hasattr(uc_data.mode, 'value') else int(uc_data.mode)
        ros_msg.dt_sec = float(uc_data.dt_sec)
        ros_msg.confidence = float(uc_data.confidence)
        ros_msg.soft_enabled = uc_data.soft_enabled
        
        ros_msg.points = [
            Point(x=float(p.x), y=float(p.y), z=float(p.z))
            for p in uc_data.points
        ]
        
        if uc_data.velocities is not None:
            ros_msg.velocities_flat = uc_data.velocities.flatten().tolist()
        else:
            ros_msg.velocities_flat = []
        
        return ros_msg
