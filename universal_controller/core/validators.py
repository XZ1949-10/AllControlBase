"""
数据验证工具

本模块提供通用的数据验证函数，用于检查输入数据的有效性。

使用指南:
=========

这些验证函数是可选的工具，模块可以根据需要使用。
验证函数遵循以下约定：

1. 返回布尔值表示验证是否通过
2. 不抛出异常（除非明确要求）
3. 对于无效数据，返回 False 并可选地记录警告

示例:
    from universal_controller.core.validators import (
        is_finite, is_positive, is_valid_quaternion
    )
    
    if not is_finite(value):
        return default_value
    
    if not is_valid_quaternion(q):
        logger.warning("Invalid quaternion")
        return None

设计原则:
=========

- 验证函数应该是纯函数，不修改输入
- 验证函数应该快速执行，适合在控制循环中使用
- 复杂的验证逻辑应该保留在使用它的模块中
"""

import numpy as np
from typing import Tuple, Union, Optional
import logging

from .constants import (
    EPSILON, EPSILON_SMALL, EPSILON_VELOCITY,
    QUATERNION_NORM_SQ_MIN, QUATERNION_NORM_SQ_MAX,
)

logger = logging.getLogger(__name__)


# =============================================================================
# 数值验证
# =============================================================================

def is_finite(value: Union[float, np.ndarray]) -> bool:
    """
    检查值是否为有限数（非 NaN、非 Inf）
    
    Args:
        value: 标量或数组
    
    Returns:
        True 如果所有值都是有限的
    """
    return np.all(np.isfinite(value))


def is_positive(value: float, allow_zero: bool = False) -> bool:
    """
    检查值是否为正数
    
    Args:
        value: 要检查的值
        allow_zero: 是否允许零
    
    Returns:
        True 如果值为正数（或零，如果 allow_zero=True）
    """
    if not np.isfinite(value):
        return False
    if allow_zero:
        return value >= 0
    return value > 0


def is_in_range(value: float, min_val: Optional[float], max_val: Optional[float]) -> bool:
    """
    检查值是否在指定范围内
    
    Args:
        value: 要检查的值
        min_val: 最小值（None 表示无下限）
        max_val: 最大值（None 表示无上限）
    
    Returns:
        True 如果值在范围内
    """
    if not np.isfinite(value):
        return False
    if min_val is not None and value < min_val:
        return False
    if max_val is not None and value > max_val:
        return False
    return True


# =============================================================================
# 四元数验证
# =============================================================================

def is_valid_quaternion(q: Tuple[float, float, float, float]) -> bool:
    """
    检查四元数是否有效
    
    有效的四元数应该：
    1. 所有分量都是有限数
    2. 范数在合理范围内（可以归一化）
    
    Args:
        q: 四元数 (x, y, z, w)
    
    Returns:
        True 如果四元数有效
    """
    try:
        x, y, z, w = float(q[0]), float(q[1]), float(q[2]), float(q[3])
    except (TypeError, ValueError, IndexError):
        return False
    
    # 检查有限性
    if not (np.isfinite(x) and np.isfinite(y) and np.isfinite(z) and np.isfinite(w)):
        return False
    
    # 检查范数
    norm_sq = x*x + y*y + z*z + w*w
    return QUATERNION_NORM_SQ_MIN < norm_sq < QUATERNION_NORM_SQ_MAX


def normalize_quaternion(q: Tuple[float, float, float, float]) -> Optional[Tuple[float, float, float, float]]:
    """
    归一化四元数
    
    Args:
        q: 四元数 (x, y, z, w)
    
    Returns:
        归一化后的四元数，如果输入无效则返回 None
    """
    if not is_valid_quaternion(q):
        return None
    
    x, y, z, w = float(q[0]), float(q[1]), float(q[2]), float(q[3])
    norm = np.sqrt(x*x + y*y + z*z + w*w)
    return (x/norm, y/norm, z/norm, w/norm)


# =============================================================================
# 速度验证
# =============================================================================

def is_valid_velocity(v: Union[float, np.ndarray], max_velocity: float = 100.0) -> bool:
    """
    检查速度是否有效
    
    Args:
        v: 速度值或向量
        max_velocity: 最大合理速度（用于检测异常值）
    
    Returns:
        True 如果速度有效
    """
    if not is_finite(v):
        return False
    
    if np.isscalar(v):
        return abs(v) <= max_velocity
    else:
        return np.all(np.abs(v) <= max_velocity)


def is_low_speed(v: Union[float, np.ndarray], threshold: float = None) -> bool:
    """
    检查是否为低速状态
    
    Args:
        v: 速度值或向量
        threshold: 低速阈值，默认使用 EPSILON_VELOCITY
    
    Returns:
        True 如果速度低于阈值
    """
    if threshold is None:
        threshold = EPSILON_VELOCITY
    
    if np.isscalar(v):
        return abs(v) < threshold
    else:
        return np.linalg.norm(v) < threshold


# =============================================================================
# 时间验证
# =============================================================================

def is_valid_dt(dt: float, min_dt: float = 1e-6, max_dt: float = 10.0) -> bool:
    """
    检查时间步长是否有效
    
    Args:
        dt: 时间步长（秒）
        min_dt: 最小有效时间步长
        max_dt: 最大有效时间步长
    
    Returns:
        True 如果时间步长有效
    """
    if not np.isfinite(dt):
        return False
    return min_dt <= dt <= max_dt


def is_valid_timestamp(stamp: float, max_age: float = 3600.0) -> bool:
    """
    检查时间戳是否有效
    
    Args:
        stamp: 时间戳（秒）
        max_age: 最大允许的时间戳年龄（秒）
    
    Returns:
        True 如果时间戳有效
    """
    import time
    if not np.isfinite(stamp):
        return False
    if stamp <= 0:
        return False
    
    current_time = time.time()
    age = current_time - stamp
    return -max_age <= age <= max_age


# =============================================================================
# 数组验证
# =============================================================================

def is_valid_array(arr: np.ndarray, expected_shape: Tuple[int, ...] = None,
                   allow_empty: bool = False) -> bool:
    """
    检查数组是否有效
    
    Args:
        arr: 要检查的数组
        expected_shape: 期望的形状（None 表示不检查形状）
        allow_empty: 是否允许空数组
    
    Returns:
        True 如果数组有效
    """
    if arr is None:
        return False
    
    if not isinstance(arr, np.ndarray):
        return False
    
    if not allow_empty and arr.size == 0:
        return False
    
    if not is_finite(arr):
        return False
    
    if expected_shape is not None and arr.shape != expected_shape:
        return False
    
    return True


# =============================================================================
# 导出列表
# =============================================================================

__all__ = [
    # 数值验证
    'is_finite',
    'is_positive',
    'is_in_range',
    # 四元数验证
    'is_valid_quaternion',
    'normalize_quaternion',
    # 速度验证
    'is_valid_velocity',
    'is_low_speed',
    # 时间验证
    'is_valid_dt',
    'is_valid_timestamp',
    # 数组验证
    'is_valid_array',
]
