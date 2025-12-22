"""
ROS 兼容层

支持两种运行模式:
1. 真实 ROS 环境: 使用 rospy, tf2_ros 等
2. 独立模式: 使用模拟实现，用于测试和非 ROS 环境

使用方法:
    from universal_controller.core.ros_compat import (
        rospy, tf2_ros, tf2_geometry_msgs, tft,
        Header, Odometry, Imu, TwistStamped,
        ROS_AVAILABLE, TF2_AVAILABLE
    )
"""
import time
import numpy as np
from typing import Tuple

# ============================================================================
# 数值常量
# ============================================================================

# 数值稳定性阈值
EPSILON = 1e-6          # 通用小量阈值
EPSILON_SMALL = 1e-9    # 更严格的小量阈值
EPSILON_ANGLE = 1e-9    # 角度计算阈值

# ============================================================================
# ROS 检测
# ============================================================================

ROS_AVAILABLE = False
TF2_AVAILABLE = False

try:
    import rospy
    from std_msgs.msg import Header as RosHeader
    from nav_msgs.msg import Odometry as RosOdometry
    from sensor_msgs.msg import Imu as RosImu
    from geometry_msgs.msg import TwistStamped as RosTwistStamped
    from geometry_msgs.msg import TransformStamped, PointStamped, Quaternion
    ROS_AVAILABLE = True
    
    try:
        import tf2_ros
        import tf2_geometry_msgs
        import tf.transformations as tft
        TF2_AVAILABLE = True
    except ImportError:
        TF2_AVAILABLE = False
        tft = None
        tf2_ros = None
        tf2_geometry_msgs = None
        
except ImportError:
    rospy = None
    RosHeader = None
    RosOdometry = None
    RosImu = None
    RosTwistStamped = None
    TransformStamped = None
    PointStamped = None
    Quaternion = None
    tf2_ros = None
    tf2_geometry_msgs = None
    tft = None


# ============================================================================
# 数学工具函数 (不依赖 ROS)
# ============================================================================

def euler_from_quaternion(q: Tuple[float, float, float, float]) -> Tuple[float, float, float]:
    """
    从四元数计算欧拉角 (roll, pitch, yaw)
    
    Args:
        q: 四元数 (x, y, z, w)
    
    Returns:
        (roll, pitch, yaw) 弧度
    """
    x, y, z, w = q
    
    # 归一化四元数，确保数值稳定性
    norm_sq = x*x + y*y + z*z + w*w
    if norm_sq < 1e-10:
        # 无效四元数，返回零欧拉角
        return (0.0, 0.0, 0.0)
    if abs(norm_sq - 1.0) > 1e-6:
        # 需要归一化
        norm = np.sqrt(norm_sq)
        x, y, z, w = x/norm, y/norm, z/norm, w/norm
    
    # Roll
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)
    # Pitch (带数值稳定性保护)
    sinp = 2 * (w * y - z * x)
    sinp = np.clip(sinp, -1.0, 1.0)  # 确保在有效范围内
    if abs(sinp) >= 1.0 - 1e-9:
        # 万向节锁情况
        pitch = np.copysign(np.pi / 2, sinp)
    else:
        pitch = np.arcsin(sinp)
    # Yaw
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def quaternion_from_euler(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    """
    从欧拉角计算四元数 (x, y, z, w)
    
    Args:
        roll, pitch, yaw: 欧拉角 (弧度)
    
    Returns:
        四元数 (x, y, z, w)
    """
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)
    
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return (x, y, z, w)


def normalize_angle(angle: float) -> float:
    """
    将角度归一化到 [-π, π] 范围
    
    Args:
        angle: 输入角度 (弧度)
    
    Returns:
        归一化后的角度 (弧度)，范围 [-π, π]
    """
    return np.arctan2(np.sin(angle), np.cos(angle))


def angle_difference(angle1: float, angle2: float) -> float:
    """
    计算两个角度之间的最短差值
    
    Args:
        angle1: 第一个角度 (弧度)
        angle2: 第二个角度 (弧度)
    
    Returns:
        角度差 (弧度)，范围 [-π, π]，表示从 angle2 到 angle1 的最短旋转
    """
    return normalize_angle(angle1 - angle2)


# ============================================================================
# 导入模拟实现 (用于非 ROS 环境)
# ============================================================================

if not ROS_AVAILABLE:
    from ..mock.ros_mock import MockRospy, MockTF2Ros, MockTFTransformations
    from ..mock.ros_mock import LookupException, ExtrapolationException, ConnectivityException
    
    # 使用模拟实现
    rospy = MockRospy()
    tf2_ros = MockTF2Ros()
    tft = MockTFTransformations()

if not TF2_AVAILABLE and ROS_AVAILABLE:
    # ROS 可用但 TF2 不可用
    from ..mock.ros_mock import MockTF2Ros, MockTFTransformations
    from ..mock.ros_mock import LookupException, ExtrapolationException, ConnectivityException
    
    tf2_ros = MockTF2Ros()
    tft = MockTFTransformations()


# ============================================================================
# 统一的异常类型
# ============================================================================

if TF2_AVAILABLE:
    TF2LookupException = tf2_ros.LookupException
    TF2ExtrapolationException = tf2_ros.ExtrapolationException
    TF2ConnectivityException = tf2_ros.ConnectivityException
else:
    from ..mock.ros_mock import LookupException, ExtrapolationException, ConnectivityException
    TF2LookupException = LookupException
    TF2ExtrapolationException = ExtrapolationException
    TF2ConnectivityException = ConnectivityException


# ============================================================================
# 工具函数
# ============================================================================

def get_current_time() -> float:
    """获取当前时间（秒）"""
    if ROS_AVAILABLE:
        try:
            return rospy.Time.now().to_sec()
        except Exception:
            # ROS 未初始化或其他异常，回退到系统时间
            return time.time()
    return time.time()


def get_monotonic_time() -> float:
    """
    获取单调时钟时间（秒）
    
    使用 time.monotonic() 而非 time.time()，避免系统时间跳变
    （如 NTP 同步）导致的时间间隔计算错误。
    
    适用场景：
    - 计算时间间隔 (dt)
    - 超时检测
    - 性能测量
    
    不适用场景：
    - 需要与外部系统同步的时间戳
    - 日志记录的绝对时间
    """
    return time.monotonic()


def create_time(sec: float):
    """创建时间对象"""
    if ROS_AVAILABLE:
        return rospy.Time.from_sec(sec)
    from ..mock.ros_mock import MockRospy
    return MockRospy.Time.from_sec(sec)


def create_duration(sec: float):
    """创建时长对象"""
    if ROS_AVAILABLE:
        return rospy.Duration.from_sec(sec)
    from ..mock.ros_mock import MockRospy
    return MockRospy.Duration.from_sec(sec)
