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

# ROS2 特定的类，延迟导入
try:
    from .publishers import PublisherManager
    from .services import ServiceManager
    _ROS2_AVAILABLE = True
except ImportError:
    PublisherManager = None
    ServiceManager = None
    _ROS2_AVAILABLE = False

# ROS1 特定的类，延迟导入
try:
    from .ros1_publishers import ROS1PublisherManager
    from .ros1_services import ROS1ServiceManager
    _ROS1_AVAILABLE = True
except ImportError:
    ROS1PublisherManager = None
    ROS1ServiceManager = None
    _ROS1_AVAILABLE = False

__all__ = [
    'DataManager',
    # ROS2
    'PublisherManager',
    'ServiceManager',
    # ROS1
    'ROS1PublisherManager',
    'ROS1ServiceManager',
]
