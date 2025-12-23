"""
节点层 - 主节点实现

注意：ControllerNode 依赖 ROS2 (rclpy)。
在非 ROS 环境下，只有 ControllerNodeBase 可用。
"""
from .base_node import ControllerNodeBase

# ROS2 特定的类，延迟导入以支持非 ROS 环境
try:
    from .controller_node import ControllerNode, main
    _ROS2_AVAILABLE = True
except ImportError:
    ControllerNode = None
    main = None
    _ROS2_AVAILABLE = False

__all__ = [
    'ControllerNodeBase',
    'ControllerNode',
    'main',
]
