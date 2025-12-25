"""
ROS 桥接层

提供 ROS1/ROS2 兼容的接口，隔离 ROS 版本差异。
"""
from typing import Dict, Any, Optional, Callable
from abc import ABC, abstractmethod
import threading
import time
import logging

logger = logging.getLogger(__name__)

# 检测 ROS 版本
ROS_VERSION = 0
try:
    import rospy
    ROS_VERSION = 1
except ImportError:
    pass

if ROS_VERSION == 0:
    try:
        import rclpy
        ROS_VERSION = 2
    except ImportError:
        pass


class ROSBridgeBase(ABC):
    """ROS 桥接基类"""
    
    @abstractmethod
    def init(self, node_name: str):
        """初始化 ROS"""
        pass
    
    @abstractmethod
    def shutdown(self):
        """关闭 ROS"""
        pass
    
    @abstractmethod
    def get_time(self) -> float:
        """获取当前时间 (秒)"""
        pass
    
    @abstractmethod
    def log_info(self, msg: str):
        """记录信息日志"""
        pass
    
    @abstractmethod
    def log_warn(self, msg: str):
        """记录警告日志"""
        pass
    
    @abstractmethod
    def create_subscriber(self, msg_type, topic: str, callback: Callable, queue_size: int = 10):
        """创建订阅"""
        pass
    
    @abstractmethod
    def create_publisher(self, msg_type, topic: str, queue_size: int = 10):
        """创建发布器"""
        pass
    
    @abstractmethod
    def publish(self, publisher, msg):
        """发布消息"""
        pass
    
    @abstractmethod
    def spin_once(self, timeout: float = 0.1):
        """处理一次回调"""
        pass
    
    @abstractmethod
    def is_shutdown(self) -> bool:
        """检查 ROS 是否已关闭"""
        pass


class ROS1Bridge(ROSBridgeBase):
    """ROS1 桥接实现"""
    
    def __init__(self):
        self._node_name = None
        self._subscribers = []
        self._publishers = []
    
    def init(self, node_name: str):
        import rospy
        rospy.init_node(node_name, anonymous=False)
        self._node_name = node_name
    
    def shutdown(self):
        import rospy
        rospy.signal_shutdown("Visualizer shutdown")
    
    def get_time(self) -> float:
        import rospy
        try:
            return rospy.Time.now().to_sec()
        except:
            return time.time()
    
    def log_info(self, msg: str):
        import rospy
        rospy.loginfo(msg)
    
    def log_warn(self, msg: str):
        import rospy
        rospy.logwarn(msg)
    
    def create_subscriber(self, msg_type, topic: str, callback: Callable, queue_size: int = 10):
        import rospy
        sub = rospy.Subscriber(topic, msg_type, callback, queue_size=queue_size)
        self._subscribers.append(sub)
        return sub
    
    def create_publisher(self, msg_type, topic: str, queue_size: int = 10):
        import rospy
        pub = rospy.Publisher(topic, msg_type, queue_size=queue_size)
        self._publishers.append(pub)
        return pub
    
    def publish(self, publisher, msg):
        publisher.publish(msg)
    
    def spin_once(self, timeout: float = 0.1):
        import rospy
        # ROS1 使用回调队列，不需要显式 spin_once
        # 但我们可以短暂 sleep 让回调有机会执行
        rospy.sleep(timeout)
    
    def is_shutdown(self) -> bool:
        import rospy
        return rospy.is_shutdown()


class ROS2Bridge(ROSBridgeBase):
    """ROS2 桥接实现"""
    
    def __init__(self):
        self._node = None
        self._executor = None
    
    def init(self, node_name: str):
        import rclpy
        from rclpy.node import Node
        
        if not rclpy.ok():
            rclpy.init()
        
        self._node = rclpy.create_node(node_name)
    
    def shutdown(self):
        import rclpy
        if self._node:
            self._node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    
    def get_time(self) -> float:
        if self._node:
            return self._node.get_clock().now().nanoseconds * 1e-9
        return time.time()
    
    def log_info(self, msg: str):
        if self._node:
            self._node.get_logger().info(msg)
    
    def log_warn(self, msg: str):
        if self._node:
            self._node.get_logger().warn(msg)
    
    def create_subscriber(self, msg_type, topic: str, callback: Callable, queue_size: int = 10):
        if self._node:
            return self._node.create_subscription(msg_type, topic, callback, queue_size)
        return None
    
    def create_publisher(self, msg_type, topic: str, queue_size: int = 10):
        if self._node:
            return self._node.create_publisher(msg_type, topic, queue_size)
        return None
    
    def publish(self, publisher, msg):
        if publisher:
            publisher.publish(msg)
    
    def spin_once(self, timeout: float = 0.1):
        import rclpy
        if self._node:
            rclpy.spin_once(self._node, timeout_sec=timeout)
    
    def is_shutdown(self) -> bool:
        import rclpy
        return not rclpy.ok()
    
    @property
    def node(self):
        return self._node


def create_ros_bridge() -> ROSBridgeBase:
    """创建 ROS 桥接实例"""
    if ROS_VERSION == 1:
        return ROS1Bridge()
    elif ROS_VERSION == 2:
        return ROS2Bridge()
    else:
        raise RuntimeError("No ROS installation found (neither rospy nor rclpy)")


# 导出
__all__ = [
    'ROS_VERSION',
    'ROSBridgeBase',
    'ROS1Bridge', 
    'ROS2Bridge',
    'create_ros_bridge',
]
