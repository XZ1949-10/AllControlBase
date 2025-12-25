"""
IO 层 - 管理 ROS 数据和发布

模块:
- DataManager: 统一数据管理器 (ROS 无关)
- PublisherManager: ROS2 发布管理器
- ServiceManager: ROS2 服务管理器
- ROS1PublisherManager: ROS1 发布管理器
- ROS1ServiceManager: ROS1 服务管理器

注意：
- PublisherManager, ServiceManager 依赖 ROS2 (rclpy)
- ROS1PublisherManager, ROS1ServiceManager 依赖 ROS1 (rospy)
- 在非 ROS 环境下，只有 DataManager 可用
"""
from .data_manager import DataManager
from ..utils.ros_compat import ROS_VERSION

# 根据 ROS 版本导入对应的管理器
PublisherManager = None
ServiceManager = None
ROS1PublisherManager = None
ROS1ServiceManager = None

if ROS_VERSION == 2:
    try:
        from .publishers import PublisherManager
        from .services import ServiceManager
    except ImportError:
        pass
elif ROS_VERSION == 1:
    try:
        from .ros1_publishers import ROS1PublisherManager
        from .ros1_services import ROS1ServiceManager
    except ImportError:
        pass

__all__ = [
    'DataManager',
    # ROS2
    'PublisherManager',
    'ServiceManager',
    # ROS1
    'ROS1PublisherManager',
    'ROS1ServiceManager',
]
