"""
IO 层 - 管理 ROS 订阅和发布

注意：SubscriberManager, PublisherManager, ServiceManager 依赖 ROS2 (rclpy)。
在非 ROS 环境下，只有 DataManager 可用。
"""
from .data_manager import DataManager

# ROS2 特定的类，延迟导入以支持非 ROS 环境
try:
    from .subscribers import SubscriberManager
    from .publishers import PublisherManager
    from .services import ServiceManager
    _ROS2_AVAILABLE = True
except ImportError:
    SubscriberManager = None
    PublisherManager = None
    ServiceManager = None
    _ROS2_AVAILABLE = False

__all__ = [
    'DataManager',
    'SubscriberManager',
    'PublisherManager',
    'ServiceManager',
]
