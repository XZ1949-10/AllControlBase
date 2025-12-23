"""
姿态命令适配器

UC 数据类型: universal_controller.core.data_types.AttitudeCommand
ROS 消息: controller_ros/AttitudeCmd

仅用于四旋翼平台的姿态内环控制。
"""
from typing import Any, Optional, Callable

from universal_controller.core.data_types import AttitudeCommand as UcAttitudeCommand
from .base import IMsgConverter


class AttitudeAdapter(IMsgConverter):
    """
    姿态命令适配器
    
    将 UC AttitudeCommand 转换为 ROS AttitudeCmd 消息。
    """
    
    def __init__(self, get_time_func: Optional[Callable[[], float]] = None):
        """
        初始化姿态适配器
        
        Args:
            get_time_func: 获取当前时间的函数，用于支持仿真时间
        """
        self._get_time_func = get_time_func
    
    def _get_current_time(self) -> float:
        """获取当前时间（秒）"""
        if self._get_time_func is not None:
            return self._get_time_func()
        import time
        return time.time()
    
    def to_uc(self, ros_msg: Any) -> UcAttitudeCommand:
        """ROS AttitudeCmd → UC AttitudeCommand"""
        return UcAttitudeCommand(
            roll=ros_msg.roll,
            pitch=ros_msg.pitch,
            yaw=ros_msg.yaw,
            thrust=ros_msg.thrust
        )
    
    def to_ros(self, uc_data: UcAttitudeCommand, 
               yaw_mode: int = 0, 
               is_hovering: bool = False) -> Any:
        """
        UC AttitudeCommand → ROS AttitudeCmd
        
        Args:
            uc_data: UC 姿态命令
            yaw_mode: 航向模式 (0=FOLLOW_VELOCITY, 1=FIXED, 2=TARGET_POINT, 3=MANUAL)
            is_hovering: 是否处于悬停状态
        
        Returns:
            ROS AttitudeCmd 消息
        """
        try:
            from controller_ros.msg import AttitudeCmd
        except ImportError:
            raise ImportError("ROS messages not available")
        
        msg = AttitudeCmd()
        msg.header.stamp = self._sec_to_ros_time(self._get_current_time())
        msg.header.frame_id = 'base_link'
        msg.roll = float(uc_data.roll)
        msg.pitch = float(uc_data.pitch)
        msg.yaw = float(uc_data.yaw)
        msg.thrust = float(uc_data.thrust)
        msg.yaw_mode = int(yaw_mode)
        msg.is_hovering = bool(is_hovering)
        return msg
    
    def create_hover_cmd(self, yaw: float = 0.0, thrust: float = 1.0) -> Any:
        """
        创建悬停姿态命令
        
        Args:
            yaw: 目标航向 (rad)
            thrust: 推力 (归一化，1.0 = 悬停)
        
        Returns:
            ROS AttitudeCmd 消息
        """
        try:
            from controller_ros.msg import AttitudeCmd
        except ImportError:
            raise ImportError("ROS messages not available")
        
        msg = AttitudeCmd()
        msg.header.stamp = self._sec_to_ros_time(self._get_current_time())
        msg.header.frame_id = 'base_link'
        msg.roll = 0.0
        msg.pitch = 0.0
        msg.yaw = float(yaw)
        msg.thrust = float(thrust)
        msg.yaw_mode = 1  # FIXED
        msg.is_hovering = True
        return msg
