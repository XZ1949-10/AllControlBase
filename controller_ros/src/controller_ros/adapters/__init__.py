"""
适配器层 - ROS 消息与 universal_controller 数据类型的双向转换

适配器列表:
- OdomAdapter: nav_msgs/Odometry <-> UC Odometry
- ImuAdapter: sensor_msgs/Imu <-> UC Imu
- TrajectoryAdapter: LocalTrajectoryV4 <-> UC Trajectory
- OutputAdapter: UnifiedCmd <-> UC ControlOutput
- AttitudeAdapter: AttitudeCmd <-> UC AttitudeCommand (四旋翼平台)
"""
from .base import IMsgConverter
from .odom_adapter import OdomAdapter
from .imu_adapter import ImuAdapter
from .trajectory_adapter import TrajectoryAdapter
from .output_adapter import OutputAdapter
from .attitude_adapter import AttitudeAdapter

__all__ = [
    'IMsgConverter',
    'OdomAdapter',
    'ImuAdapter',
    'TrajectoryAdapter',
    'OutputAdapter',
    'AttitudeAdapter',
]
