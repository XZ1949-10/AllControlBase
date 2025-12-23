"""
里程计适配器

ROS 消息: nav_msgs/Odometry
UC 数据类型: universal_controller.core.data_types.Odometry
"""
from typing import Any

from universal_controller.core.data_types import (
    Odometry as UcOdometry, Header, Point3D
)
from .base import IMsgConverter


class OdomAdapter(IMsgConverter):
    """
    Odometry 消息适配器
    
    将 ROS nav_msgs/Odometry 转换为 UC Odometry 数据类型。
    """
    
    def to_uc(self, ros_msg: Any) -> UcOdometry:
        """ROS Odometry → UC Odometry"""
        return UcOdometry(
            header=Header(
                stamp=self._ros_time_to_sec(ros_msg.header.stamp),
                frame_id=ros_msg.header.frame_id
            ),
            pose_position=Point3D(
                x=ros_msg.pose.pose.position.x,
                y=ros_msg.pose.pose.position.y,
                z=ros_msg.pose.pose.position.z
            ),
            pose_orientation=(
                ros_msg.pose.pose.orientation.x,
                ros_msg.pose.pose.orientation.y,
                ros_msg.pose.pose.orientation.z,
                ros_msg.pose.pose.orientation.w
            ),
            twist_linear=(
                ros_msg.twist.twist.linear.x,
                ros_msg.twist.twist.linear.y,
                ros_msg.twist.twist.linear.z
            ),
            twist_angular=(
                ros_msg.twist.twist.angular.x,
                ros_msg.twist.twist.angular.y,
                ros_msg.twist.twist.angular.z
            )
        )
    
    def to_ros(self, uc_data: UcOdometry) -> Any:
        """UC Odometry → ROS Odometry (通常不需要)"""
        raise NotImplementedError("Odom is input-only")
