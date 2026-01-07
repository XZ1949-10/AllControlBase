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

# ROS1 specific class
# NOTE: The legacy controller_node_ros1.py module is deprecated.
# For ROS1, use scripts/controller_node.py instead.
# We keep ControllerNodeROS1 = None for backward compatibility with code
# that checks for its existence, but we do NOT import the deprecated module.
ControllerNodeROS1 = None
_ROS1_AVAILABLE = False

__all__ = [
    'ControllerNodeBase',
    'ControllerNode',
    'ControllerNodeROS1',
]
