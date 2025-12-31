"""
Node Layer - Main node implementations

Supports both ROS1 (Noetic) and ROS2 (Humble).
In non-ROS environments, only ControllerNodeBase is available.
"""
from .base_node import ControllerNodeBase

# ROS2 specific class, lazy import for non-ROS environments
try:
    from .controller_node import ControllerNode
    _ROS2_AVAILABLE = True
except ImportError:
    ControllerNode = None
    _ROS2_AVAILABLE = False

# ROS1 specific class, lazy import for non-ROS environments
try:
    from .controller_node_ros1 import ControllerNodeROS1
    _ROS1_AVAILABLE = True
except ImportError:
    ControllerNodeROS1 = None
    _ROS1_AVAILABLE = False

__all__ = [
    'ControllerNodeBase',
    'ControllerNode',
    'ControllerNodeROS1',
]
