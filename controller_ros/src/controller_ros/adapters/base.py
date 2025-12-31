"""
适配器基类接口

支持 ROS1 和 ROS2 双版本。

提供通用的验证和转换功能，减少子类代码重复。
"""
from abc import ABC, abstractmethod
from typing import Any, TypeVar, Optional, Callable, Tuple
import time as _time
import math

from ..utils.ros_compat import ROS_VERSION, ros_time_to_sec, sec_to_ros_time

T_ROS = TypeVar('T_ROS')  # ROS 消息类型
T_UC = TypeVar('T_UC')    # UC 数据类型


class IMsgConverter(ABC):
    """
    消息转换器接口
    
    定义 ROS 消息与 universal_controller 数据类型之间的双向转换。
    支持 ROS1 和 ROS2。
    
    提供通用的验证方法供子类使用：
    - _validate_timestamp: 验证时间戳有效性
    - _validate_finite: 验证数值是否有限
    - _validate_range: 验证数值是否在范围内
    """
    
    def __init__(self, get_time_func: Optional[Callable[[], float]] = None):
        """
        初始化消息转换器
        
        Args:
            get_time_func: 获取当前时间的函数，用于支持仿真时间
                          如果为 None，使用系统时间
        """
        self._get_time_func = get_time_func
    
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
    
    def to_ros(self, uc_data: T_UC) -> T_ROS:
        """
        UC 数据类型 → ROS 消息
        
        Args:
            uc_data: universal_controller 数据类型对象
        
        Returns:
            ROS 消息对象
        
        Note:
            默认实现抛出 NotImplementedError。
            只有需要输出的适配器（如 OutputAdapter）需要重写此方法。
        """
        raise NotImplementedError(f"{self.__class__.__name__} does not support to_ros()")
    
    def _get_current_time(self) -> float:
        """
        获取当前时间（秒）
        
        处理仿真时间未初始化（返回 0）的情况，回退到系统时间。
        """
        if self._get_time_func is not None:
            t = self._get_time_func()
            # 仿真时间未初始化时可能返回 0，回退到系统时间
            if t > 0:
                return t
        return _time.time()
    
    def _ros_time_to_sec(self, stamp) -> float:
        """ROS 时间戳转秒 - 委托给 ros_compat"""
        return ros_time_to_sec(stamp)
    
    def _sec_to_ros_time(self, sec: float):
        """秒转 ROS 时间戳 - 委托给 ros_compat"""
        return sec_to_ros_time(sec)
    
    # ==================== 通用验证方法 ====================
    
    def _validate_timestamp(self, stamp_sec: float, max_age_sec: float = 10.0) -> Tuple[bool, str]:
        """
        验证时间戳有效性
        
        Args:
            stamp_sec: 时间戳（秒）
            max_age_sec: 最大允许年龄（秒）
        
        Returns:
            (is_valid, error_message): 是否有效和错误信息
        """
        if not math.isfinite(stamp_sec):
            return False, f"Invalid timestamp: {stamp_sec} (not finite)"
        
        if stamp_sec < 0:
            return False, f"Invalid timestamp: {stamp_sec} (negative)"
        
        current_time = self._get_current_time()
        age = current_time - stamp_sec
        
        if age > max_age_sec:
            return False, f"Timestamp too old: age={age:.2f}s > max={max_age_sec}s"
        
        if age < -1.0:  # 允许 1 秒的时钟偏差
            return False, f"Timestamp in future: age={age:.2f}s"
        
        return True, ""
    
    def _validate_finite(self, value: float, name: str) -> Tuple[bool, str]:
        """
        验证数值是否有限（非 NaN、非 Inf）
        
        Args:
            value: 要验证的数值
            name: 数值名称（用于错误信息）
        
        Returns:
            (is_valid, error_message): 是否有效和错误信息
        """
        if not math.isfinite(value):
            return False, f"{name} is not finite: {value}"
        return True, ""
    
    def _validate_range(self, value: float, name: str, 
                       min_val: Optional[float] = None, 
                       max_val: Optional[float] = None) -> Tuple[bool, str]:
        """
        验证数值是否在范围内
        
        Args:
            value: 要验证的数值
            name: 数值名称（用于错误信息）
            min_val: 最小值（None 表示无下限）
            max_val: 最大值（None 表示无上限）
        
        Returns:
            (is_valid, error_message): 是否有效和错误信息
        """
        if not math.isfinite(value):
            return False, f"{name} is not finite: {value}"
        
        if min_val is not None and value < min_val:
            return False, f"{name}={value} < min={min_val}"
        
        if max_val is not None and value > max_val:
            return False, f"{name}={value} > max={max_val}"
        
        return True, ""
    
    def _validate_vector3_finite(self, x: float, y: float, z: float, 
                                 name: str) -> Tuple[bool, str]:
        """
        验证 3D 向量的所有分量是否有限
        
        Args:
            x, y, z: 向量分量
            name: 向量名称（用于错误信息）
        
        Returns:
            (is_valid, error_message): 是否有效和错误信息
        """
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
            return False, f"{name} contains non-finite values: ({x}, {y}, {z})"
        return True, ""
    
    def _validate_quaternion(self, x: float, y: float, z: float, w: float,
                            tolerance: float = 0.01) -> Tuple[bool, str]:
        """
        验证四元数是否有效（归一化）
        
        Args:
            x, y, z, w: 四元数分量
            tolerance: 归一化容差
        
        Returns:
            (is_valid, error_message): 是否有效和错误信息
        """
        if not all(math.isfinite(v) for v in [x, y, z, w]):
            return False, f"Quaternion contains non-finite values: ({x}, {y}, {z}, {w})"
        
        norm = math.sqrt(x*x + y*y + z*z + w*w)
        if abs(norm - 1.0) > tolerance:
            return False, f"Quaternion not normalized: norm={norm:.4f}"
        
        return True, ""
