"""
适配器基类接口
"""
from abc import ABC, abstractmethod
from typing import Any, TypeVar

T_ROS = TypeVar('T_ROS')  # ROS 消息类型
T_UC = TypeVar('T_UC')    # UC 数据类型


class IMsgConverter(ABC):
    """
    消息转换器接口
    
    定义 ROS 消息与 universal_controller 数据类型之间的双向转换。
    """
    
    @abstractmethod
    def to_uc(self, ros_msg: T_ROS) -> T_UC:
        """
        ROS 消息 → UC 数据类型
        
        Args:
            ros_msg: ROS 消息对象
        
        Returns:
            universal_controller 数据类型对象
        """
        pass
    
    @abstractmethod
    def to_ros(self, uc_data: T_UC) -> T_ROS:
        """
        UC 数据类型 → ROS 消息
        
        Args:
            uc_data: universal_controller 数据类型对象
        
        Returns:
            ROS 消息对象
        """
        pass
    
    def _ros_time_to_sec(self, stamp) -> float:
        """ROS 时间戳转秒"""
        return stamp.sec + stamp.nanosec * 1e-9
    
    def _sec_to_ros_time(self, sec: float):
        """秒转 ROS 时间戳"""
        from builtin_interfaces.msg import Time
        t = Time()
        t.sec = int(sec)
        t.nanosec = int((sec - t.sec) * 1e9)
        return t
