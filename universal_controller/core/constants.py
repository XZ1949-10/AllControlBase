"""
通用常量和基础数学函数定义

本模块定义了整个控制器系统使用的通用常量和不依赖其他模块的基础数学函数。

常量分类:
=========

1. 数值稳定性常量 (Numerical Stability)
   - 用于避免除零、数值溢出等问题
   - 基于 IEEE 754 双精度浮点数特性选择

2. 物理常量 (Physical Constants)
   - 重力加速度等物理量的默认值

3. 时间常量 (Time Constants)
   - 超时、时间戳等相关的特殊值

基础数学函数:
=============

本模块包含不依赖其他模块的基础数学函数（如角度归一化），
这些函数被放在这里以避免循环导入问题。

设计原则:
=========

- 通用常量放在本模块，供多个模块共享
- 不依赖其他模块的基础数学函数放在本模块
- 模块专用常量保留在各自模块中，但应添加清晰的文档说明
- 常量命名使用全大写加下划线分隔
- 每个常量必须有详细的文档说明其用途和选择依据

使用示例:
=========

    from universal_controller.core.constants import (
        EPSILON, EPSILON_SMALL, EPSILON_ANGLE,
        DEFAULT_GRAVITY, NEVER_RECEIVED_TIME_MS,
        normalize_angle, angle_difference
    )
    
    # 避免除零
    if denominator > EPSILON:
        result = numerator / denominator
    
    # 角度归一化
    theta = normalize_angle(theta + delta)
    
    # 角度差计算
    error = angle_difference(target, current)
"""

import numpy as np


# =============================================================================
# 基础数学函数 (不依赖其他模块，避免循环导入)
# =============================================================================

def normalize_angle(angle: float) -> float:
    """
    将角度归一化到 [-π, π] 范围
    
    使用 arctan2(sin, cos) 方法，数值稳定且高效。
    
    Args:
        angle: 输入角度 (弧度)
    
    Returns:
        归一化后的角度 (弧度)，范围 [-π, π]
    
    Examples:
        >>> normalize_angle(3 * np.pi)  # 约等于 -π
        >>> normalize_angle(-2 * np.pi)  # 约等于 0
    """
    return np.arctan2(np.sin(angle), np.cos(angle))


def angle_difference(angle1: float, angle2: float) -> float:
    """
    计算两个角度之间的最短差值
    
    结果表示从 angle2 到 angle1 的最短旋转方向和角度。
    正值表示逆时针旋转，负值表示顺时针旋转。
    
    Args:
        angle1: 目标角度 (弧度)
        angle2: 起始角度 (弧度)
    
    Returns:
        角度差 (弧度)，范围 [-π, π]，表示从 angle2 到 angle1 的最短旋转
    
    Examples:
        >>> angle_difference(0.1, -0.1)  # 约等于 0.2
        >>> angle_difference(-np.pi + 0.1, np.pi - 0.1)  # 约等于 0.2 (跨越 ±π)
    """
    return normalize_angle(angle1 - angle2)

# =============================================================================
# 数值稳定性常量 (Numerical Stability Constants)
# =============================================================================

# 通用小量阈值
# 用于一般的数值比较和避免除零
# 选择依据: 1e-6 是 float32 精度的合理阈值，对于 float64 更加安全
EPSILON = 1e-6

# 更严格的小量阈值
# 用于需要更高精度的计算，如协方差矩阵操作
# 选择依据: 1e-9 接近 float32 的机器精度，对于 float64 仍有足够余量
EPSILON_SMALL = 1e-9

# 角度计算阈值
# 用于角度比较和归一化
# 选择依据: 1e-9 rad ≈ 5.7e-8 度，足够精确
EPSILON_ANGLE = 1e-9

# 速度计算最小阈值
# 用于避免低速时的数值不稳定（如航向角计算）
# 选择依据: 1e-6 m/s 是实际不可能的速度，可安全用于除法
EPSILON_VELOCITY = 1e-6

# 最小分母值
# 用于通用的除法保护
# 选择依据: 与 EPSILON_SMALL 相同，确保除法结果在合理范围内
MIN_DENOMINATOR = 1e-9

# 最小线段长度
# 用于几何计算中判断点是否重合
# 选择依据: 1e-6 m = 1 微米，远小于任何实际测量精度
MIN_SEGMENT_LENGTH = 1e-6

# 最小相对叉积
# 用于判断向量是否共线
# 选择依据: 相对于分母的比例阈值，1e-6 表示非常接近共线
MIN_RELATIVE_CROSS = 1e-6


# =============================================================================
# 物理常量 (Physical Constants)
# =============================================================================

# 默认重力加速度 (m/s²)
# 标准重力加速度，可通过配置覆盖
DEFAULT_GRAVITY = 9.81


# =============================================================================
# 时间常量 (Time Constants)
# =============================================================================

# 表示"从未收到消息"的超时值 (毫秒)
# 使用大的有限值代替无穷大，避免 JSON 序列化问题
# 1e9 ms ≈ 11.5 天，足够表示"非常长时间"
NEVER_RECEIVED_TIME_MS = 1e9


# =============================================================================
# 四元数验证常量 (Quaternion Validation Constants)
# =============================================================================

# 四元数范数平方的有效范围
# 
# 设计说明：
# - 单位四元数的范数平方应该为 1.0
# - 由于数值误差，允许一定的偏差
# - 超出范围的四元数被认为无效
#
# 阈值选择：
# - MIN: 0.25 (范数 > 0.5) - 低于此值无法可靠归一化
# - MAX: 4.0 (范数 < 2.0) - 高于此值认为数据明显错误
#
# 范围 [0.5, 2.0] 的理由：
# - 正常的数值误差不会导致范数偏离 1.0 超过 2 倍
# - 如果范数 < 0.5 或 > 2.0，说明数据本身有问题
QUATERNION_NORM_SQ_MIN = 0.25
QUATERNION_NORM_SQ_MAX = 4.0


# =============================================================================
# 导出列表
# =============================================================================

__all__ = [
    # 数值稳定性常量
    'EPSILON',
    'EPSILON_SMALL',
    'EPSILON_ANGLE',
    'EPSILON_VELOCITY',
    'MIN_DENOMINATOR',
    'MIN_SEGMENT_LENGTH',
    'MIN_RELATIVE_CROSS',
    # 物理常量
    'DEFAULT_GRAVITY',
    # 时间常量
    'NEVER_RECEIVED_TIME_MS',
    # 四元数验证常量
    'QUATERNION_NORM_SQ_MIN',
    'QUATERNION_NORM_SQ_MAX',
    # 基础数学函数
    'normalize_angle',
    'angle_difference',
]
