"""
IMU 适配器

ROS 消息: sensor_msgs/Imu
UC 数据类型: universal_controller.core.data_types.Imu
"""
from typing import Any

from universal_controller.core.data_types import (
    Imu as UcImu, Header
)
from .base import IMsgConverter


class ImuAdapter(IMsgConverter):
    """
    IMU 消息适配器
    
    将 ROS sensor_msgs/Imu 转换为 UC Imu 数据类型。
    注意: IMU 是输入数据，不需要 to_ros() 方法。
    """
    
    def to_uc(self, ros_msg: Any) -> UcImu:
        """ROS Imu → UC Imu"""
        return UcImu(
            header=Header(
                stamp=self._ros_time_to_sec(ros_msg.header.stamp),
                frame_id=ros_msg.header.frame_id
            ),
            orientation=(
                ros_msg.orientation.x,
                ros_msg.orientation.y,
                ros_msg.orientation.z,
                ros_msg.orientation.w
            ),
            angular_velocity=(
                ros_msg.angular_velocity.x,
                ros_msg.angular_velocity.y,
                ros_msg.angular_velocity.z
            ),
            linear_acceleration=(
                ros_msg.linear_acceleration.x,
                ros_msg.linear_acceleration.y,
                ros_msg.linear_acceleration.z
            )
        )
