"""
ROS 兼容层

支持两种运行模式:
1. 真实 ROS 环境: 使用 rospy, tf2_ros 等
2. 独立模式: 使用替代实现，用于测试和非 ROS 环境

使用方法:
    from universal_controller.core.ros_compat import (
        rospy, tf2_ros, tf2_geometry_msgs, tft,
        Header, Odometry, Imu, TwistStamped,
        ROS_AVAILABLE, TF2_AVAILABLE
    )

命名说明:
    - 独立模式下的替代实现使用 "Standalone" 前缀
    - 这些是生产代码的一部分，不是测试 mock
"""
import time
import numpy as np
from typing import Any, Tuple

# ============================================================================
# 数值常量和基础数学函数 (从 constants 模块导入，保持向后兼容)
# ============================================================================

from .constants import (
    EPSILON, EPSILON_SMALL, EPSILON_ANGLE,
    normalize_angle, angle_difference
)

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
    if abs(norm_sq - 1.0) > EPSILON:
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


# 注意: normalize_angle 和 angle_difference 已从 constants 模块导入
# 保留以下注释说明函数来源，便于代码审查
# normalize_angle(angle) -> float: 将角度归一化到 [-π, π]
# angle_difference(angle1, angle2) -> float: 计算两角度的最短差值


# ============================================================================
# 导入兼容层实现 (用于非 ROS 环境)
# ============================================================================

if not ROS_AVAILABLE:
    from ..compat.ros_compat_impl import (
        StandaloneRospy, StandaloneTF2Ros, StandaloneTFTransformations,
        LookupException, ExtrapolationException, ConnectivityException
    )
    
    # 使用独立运行实现
    rospy = StandaloneRospy()
    tf2_ros = StandaloneTF2Ros()
    tft = StandaloneTFTransformations()

if not TF2_AVAILABLE and ROS_AVAILABLE:
    # ROS 可用但 TF2 不可用
    from ..compat.ros_compat_impl import (
        StandaloneTF2Ros, StandaloneTFTransformations,
        LookupException, ExtrapolationException, ConnectivityException
    )
    
    tf2_ros = StandaloneTF2Ros()
    tft = StandaloneTFTransformations()


# ============================================================================
# 统一的异常类型
# ============================================================================

if TF2_AVAILABLE:
    TF2LookupException = tf2_ros.LookupException
    TF2ExtrapolationException = tf2_ros.ExtrapolationException
    TF2ConnectivityException = tf2_ros.ConnectivityException
else:
    from ..compat.ros_compat_impl import LookupException, ExtrapolationException, ConnectivityException
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


def create_time(sec: float) -> Any:
    """创建时间对象"""
    if ROS_AVAILABLE:
        return rospy.Time.from_sec(sec)
    from ..compat.ros_compat_impl import StandaloneRospy
    return StandaloneRospy.Time.from_sec(sec)


def create_duration(sec: float) -> Any:
    """创建时长对象"""
    if ROS_AVAILABLE:
        return rospy.Duration.from_sec(sec)
    from ..compat.ros_compat_impl import StandaloneRospy
    return StandaloneRospy.Duration.from_sec(sec)


def transform_pose(pose, transform):
    """
    执行 Pose 变换
    
    Args:
        pose: GeometryMsgs PoseStamped
        transform: GeometryMsgs TransformStamped
    """
    if TF2_AVAILABLE:
        # tf2_geometry_msgs 会自动注册到 tf2_ros
        # 但有时需要显式调用 do_transform_pose
        # 注意: tf2_geometry_msgs 模块本身没有导出 do_transform_pose，它是 monkey patch 的
        # 正确用法通常是: tf2_geometry_msgs.do_transform_pose(pose, transform)
        return tf2_geometry_msgs.do_transform_pose(pose, transform)
    else:
        from ..compat.ros_compat_impl import do_transform_pose
        return do_transform_pose(pose, transform)


# ============================================================================
# 类型导出 (用于类型标注和直接实例化)
# ============================================================================

if ROS_AVAILABLE:
    Time = rospy.Time
    Duration = rospy.Duration
else:
    from ..compat.ros_compat_impl import StandaloneRospy
    Time = StandaloneRospy.Time
    Duration = StandaloneRospy.Duration


# ============================================================================
# 坐标变换工具函数
# ============================================================================

def get_transform_matrix(translation: dict, rotation: dict) -> np.ndarray:
    """
    从平移和四元数构建 4x4 变换矩阵
    
    Args:
        translation: 平移向量 {'x': float, 'y': float, 'z': float}
        rotation: 四元数 {'x': float, 'y': float, 'z': float, 'w': float}
        
    Returns:
        4x4 变换矩阵
    """
    # 提取四元数
    qx = rotation.get('x', 0.0)
    qy = rotation.get('y', 0.0)
    qz = rotation.get('z', 0.0)
    qw = rotation.get('w', 1.0)
    
    # 归一化四元数
    norm = np.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if norm < 1e-10:
        qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
    else:
        qx, qy, qz, qw = qx/norm, qy/norm, qz/norm, qw/norm
    
    # 构建旋转矩阵
    R = np.array([
        [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw), 1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx*qx + qy*qy)]
    ])
    
    # 构建 4x4 变换矩阵
    T = np.eye(4)
    T[:3, :3] = R
    T[0, 3] = translation.get('x', 0.0)
    T[1, 3] = translation.get('y', 0.0)
    T[2, 3] = translation.get('z', 0.0)
    
    return T


def apply_transform_to_trajectory(traj: 'Trajectory', transform_matrix: np.ndarray, 
                                  target_frame: str) -> 'Trajectory':
    """
    将变换矩阵应用到轨迹上
    
    Args:
        traj: 输入轨迹
        transform_matrix: 4x4 变换矩阵
        target_frame: 目标坐标系名称
        
    Returns:
        变换后的新轨迹
    """
    from .data_types import Trajectory, Point3D, Header
    
    # 创建新轨迹副本
    new_traj = traj.copy()
    new_traj.header.frame_id = target_frame
    
    # 提取旋转矩阵和平移向量
    R = transform_matrix[:3, :3]
    t = transform_matrix[:3, 3]
    
    # 向量化变换所有点
    if len(traj.points) > 0:
        # 构建点矩阵 [N, 3]
        points = np.array([[p.x, p.y, p.z] for p in traj.points])
        
        # 应用变换: p' = R @ p + t
        transformed_points = (R @ points.T).T + t
        
        # 更新轨迹点
        new_traj.points = [
            Point3D(x=p[0], y=p[1], z=p[2]) 
            for p in transformed_points
        ]
    
    # 变换速度向量 (只需旋转，不需平移)
    if traj.velocities is not None and len(traj.velocities) > 0:
        # velocities: [N, 4] = [vx, vy, vz, wz]
        # 线速度需要旋转，角速度 wz 在 2D 情况下不变
        linear_vels = traj.velocities[:, :3]  # [N, 3]
        angular_vels = traj.velocities[:, 3:4]  # [N, 1]
        
        transformed_linear = (R @ linear_vels.T).T  # [N, 3]
        new_traj.velocities = np.hstack([transformed_linear, angular_vels])
    
    return new_traj
