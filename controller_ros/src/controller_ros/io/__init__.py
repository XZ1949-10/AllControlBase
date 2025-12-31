"""
IO 层 - 管理 ROS 数据和发布

模块:
- DataManager: 统一数据管理器 (ROS 无关)
- ROS1PublisherManager: ROS1 发布管理器
- ROS1ServiceManager: ROS1 服务管理器

ROS 版本支持:
- ROS1 Noetic: 主要支持版本
"""
from .data_manager import DataManager

# ROS1 管理器直接导入
try:
    from .ros1_publishers import ROS1PublisherManager
    from .ros1_services import ROS1ServiceManager
except ImportError:
    ROS1PublisherManager = None
    ROS1ServiceManager = None

__all__ = [
    'DataManager',
    'ROS1PublisherManager',
    'ROS1ServiceManager',
]
