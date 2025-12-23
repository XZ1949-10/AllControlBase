"""
控制输出适配器

UC 数据类型: universal_controller.core.data_types.ControlOutput
ROS 消息: controller_ros/UnifiedCmd
"""
import time as _time  # 导入到模块顶部，避免函数内重复导入
from typing import Any, Dict, Optional, Callable

from universal_controller.core.data_types import ControlOutput as UcControlOutput
from .base import IMsgConverter


class OutputAdapter(IMsgConverter):
    """
    控制输出适配器
    
    将 UC ControlOutput 转换为 ROS UnifiedCmd 消息。
    """
    
    def __init__(self, default_frame_id: str = 'base_link',
                 get_time_func: Optional[Callable[[], float]] = None):
        """
        初始化输出适配器
        
        Args:
            default_frame_id: 默认输出坐标系
            get_time_func: 获取当前时间的函数，用于支持仿真时间
                          如果为 None，使用系统时间
        """
        self._default_frame_id = default_frame_id
        self._get_time_func = get_time_func
    
    def _get_current_time(self) -> float:
        """获取当前时间（秒）"""
        if self._get_time_func is not None:
            return self._get_time_func()
        return _time.time()
    
    def to_uc(self, ros_msg: Any) -> UcControlOutput:
        """ROS UnifiedCmd → UC ControlOutput"""
        return UcControlOutput(
            vx=ros_msg.vx,
            vy=ros_msg.vy,
            vz=ros_msg.vz,
            omega=ros_msg.omega,
            frame_id=ros_msg.header.frame_id or self._default_frame_id,
            success=ros_msg.success,
            solve_time_ms=ros_msg.solve_time_ms
        )
    
    def to_ros(self, uc_data: UcControlOutput) -> Any:
        """UC ControlOutput → ROS UnifiedCmd"""
        # 延迟导入避免循环依赖
        try:
            from controller_ros.msg import UnifiedCmd
        except ImportError:
            raise ImportError("ROS messages not available")
        
        msg = UnifiedCmd()
        msg.header.stamp = self._sec_to_ros_time(self._get_current_time())
        msg.header.frame_id = uc_data.frame_id or self._default_frame_id
        msg.vx = float(uc_data.vx)
        msg.vy = float(uc_data.vy)
        msg.vz = float(uc_data.vz)
        msg.omega = float(uc_data.omega)
        msg.success = uc_data.success
        msg.solve_time_ms = float(uc_data.solve_time_ms)
        return msg
    
    def create_stop_cmd(self) -> Any:
        """创建停止命令"""
        try:
            from controller_ros.msg import UnifiedCmd
        except ImportError:
            raise ImportError("ROS messages not available")
        
        msg = UnifiedCmd()
        msg.header.stamp = self._sec_to_ros_time(self._get_current_time())
        msg.header.frame_id = self._default_frame_id
        msg.vx = 0.0
        msg.vy = 0.0
        msg.vz = 0.0
        msg.omega = 0.0
        msg.success = True
        msg.solve_time_ms = 0.0
        return msg
