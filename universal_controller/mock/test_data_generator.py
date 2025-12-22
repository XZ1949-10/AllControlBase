"""
测试数据生成器

生成用于测试的轨迹、里程计、IMU 等数据。
"""
import time
import math
import numpy as np
from typing import List, Optional, Tuple

# 导入数据类型
from ..core.data_types import (
    Trajectory, Point3D, Header, Odometry, Imu
)
from ..core.enums import TrajectoryMode


def create_test_trajectory(
    num_points: int = 20,
    dt: float = 0.1,
    trajectory_type: str = 'sine',
    frame_id: str = 'base_link',
    confidence: float = 0.9,
    soft_enabled: bool = False,
    **kwargs
) -> Trajectory:
    """
    创建测试轨迹
    
    Args:
        num_points: 轨迹点数
        dt: 时间步长 (秒)
        trajectory_type: 轨迹类型 ('sine', 'circle', 'straight', 'figure8')
        frame_id: 坐标系 ID，默认 'base_link' (网络输出的局部坐标系)
        confidence: 置信度
        soft_enabled: 是否启用 Soft Head
        **kwargs: 轨迹类型特定参数
    
    Returns:
        Trajectory 对象
        
    Note:
        默认 frame_id='base_link' 模拟网络输出的局部轨迹，
        轨迹点是相对于当前机器人位置的局部坐标。
    """
    points = []
    velocities = [] if soft_enabled else None
    
    if trajectory_type == 'sine':
        # 正弦波轨迹
        amplitude = kwargs.get('amplitude', 0.5)
        wavelength = kwargs.get('wavelength', 3.0)
        speed = kwargs.get('speed', 0.5)
        
        vel_list = []
        for i in range(num_points):
            x = i * speed * dt
            y = amplitude * math.sin(2 * math.pi * x / wavelength)
            z = 0.0
            points.append(Point3D(x, y, z))
            
            if soft_enabled:
                vx = speed
                vy = amplitude * (2 * math.pi / wavelength) * math.cos(2 * math.pi * x / wavelength) * speed
                vel_list.append([vx, vy, 0.0, 0.0])
        
        if soft_enabled and vel_list:
            velocities = np.array(vel_list)
    
    elif trajectory_type == 'circle':
        # 圆形轨迹
        radius = kwargs.get('radius', 2.0)
        angular_speed = kwargs.get('angular_speed', 0.5)
        
        vel_list = []
        for i in range(num_points):
            theta = i * angular_speed * dt
            x = radius * math.cos(theta)
            y = radius * math.sin(theta)
            z = 0.0
            points.append(Point3D(x, y, z))
            
            if soft_enabled:
                vx = -radius * angular_speed * math.sin(theta)
                vy = radius * angular_speed * math.cos(theta)
                vel_list.append([vx, vy, 0.0, angular_speed])
        
        if soft_enabled and vel_list:
            velocities = np.array(vel_list)
    
    elif trajectory_type == 'straight':
        # 直线轨迹
        speed = kwargs.get('speed', 1.0)
        direction = kwargs.get('direction', 0.0)  # 弧度
        
        vel_list = []
        for i in range(num_points):
            dist = i * speed * dt
            x = dist * math.cos(direction)
            y = dist * math.sin(direction)
            z = 0.0
            points.append(Point3D(x, y, z))
            
            if soft_enabled:
                vx = speed * math.cos(direction)
                vy = speed * math.sin(direction)
                vel_list.append([vx, vy, 0.0, 0.0])
        
        if soft_enabled and vel_list:
            velocities = np.array(vel_list)
    
    elif trajectory_type == 'figure8':
        # 8 字形轨迹
        scale = kwargs.get('scale', 2.0)
        angular_speed = kwargs.get('angular_speed', 0.3)
        
        vel_list = []
        for i in range(num_points):
            t = i * angular_speed * dt
            x = scale * math.sin(t)
            y = scale * math.sin(t) * math.cos(t)
            z = 0.0
            points.append(Point3D(x, y, z))
            
            if soft_enabled:
                vx = scale * angular_speed * math.cos(t)
                vy = scale * angular_speed * (math.cos(t)**2 - math.sin(t)**2)
                vel_list.append([vx, vy, 0.0, 0.0])
        
        if soft_enabled and vel_list:
            velocities = np.array(vel_list)
    
    else:
        raise ValueError(f"Unknown trajectory type: {trajectory_type}")
    
    return Trajectory(
        header=Header(stamp=time.time(), frame_id=frame_id),
        points=points,
        velocities=velocities,
        dt_sec=dt,
        confidence=confidence,
        soft_enabled=soft_enabled
    )


def create_test_odom(
    x: float = 0.0,
    y: float = 0.0,
    z: float = 0.0,
    theta: float = 0.0,
    vx: float = 0.0,
    vy: float = 0.0,
    vz: float = 0.0,
    omega: float = 0.0,
    frame_id: str = 'odom',
    child_frame_id: str = 'base_link'
) -> Odometry:
    """
    创建测试里程计数据
    
    Args:
        x, y, z: 位置 (米)
        theta: 航向角 (弧度)
        vx, vy, vz: 线速度 (米/秒)，机体坐标系
        omega: 角速度 (弧度/秒)
        frame_id: 父坐标系
        child_frame_id: 子坐标系
    
    Returns:
        Odometry 对象
    """
    # 从航向角计算四元数 (仅 yaw)
    qx = 0.0
    qy = 0.0
    qz = math.sin(theta / 2)
    qw = math.cos(theta / 2)
    
    return Odometry(
        header=Header(stamp=time.time(), frame_id=frame_id),
        pose_position=Point3D(x, y, z),
        pose_orientation=(qx, qy, qz, qw),
        twist_linear=(vx, vy, vz),
        twist_angular=(0.0, 0.0, omega)
    )


def create_test_imu(
    orientation: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0),
    angular_velocity: Tuple[float, float, float] = (0.0, 0.0, 0.0),
    linear_acceleration: Tuple[float, float, float] = (0.0, 0.0, 9.81),
    frame_id: str = 'imu_link'
) -> Imu:
    """
    创建测试 IMU 数据
    
    Args:
        orientation: 四元数 (x, y, z, w)
        angular_velocity: 角速度 (rad/s)
        linear_acceleration: 线加速度 (m/s²)
        frame_id: 坐标系
    
    Returns:
        Imu 对象
    """
    return Imu(
        header=Header(stamp=time.time(), frame_id=frame_id),
        orientation=orientation,
        angular_velocity=angular_velocity,
        linear_acceleration=linear_acceleration
    )


def create_test_state_sequence(
    num_steps: int = 100,
    dt: float = 0.02,
    trajectory_type: str = 'sine',
    initial_state: Tuple[float, float, float] = (0.0, 0.0, 0.0),
    **kwargs
) -> List[Tuple[Odometry, Trajectory]]:
    """
    创建测试状态序列 (里程计 + 轨迹)
    
    用于模拟控制循环的输入数据。
    
    注意: 此函数生成的轨迹是在世界坐标系下的 (frame_id='world')，
    因为它会根据当前位置偏移轨迹点。这模拟的是已经变换后的轨迹。
    
    如果需要测试坐标变换功能，应该使用 create_test_trajectory() 
    生成局部坐标系的轨迹 (frame_id='base_link')。
    
    Args:
        num_steps: 步数
        dt: 时间步长
        trajectory_type: 轨迹类型
        initial_state: 初始状态 (x, y, theta)
        **kwargs: 轨迹参数
    
    Returns:
        (Odometry, Trajectory) 元组列表
    """
    sequence = []
    x, y, theta = initial_state
    vx, vy = 0.0, 0.0
    
    for i in range(num_steps):
        # 创建轨迹 (从当前位置开始)
        # 注意: 这里使用 frame_id='world' 因为我们会手动偏移轨迹点
        traj = create_test_trajectory(
            num_points=20,
            dt=0.1,
            trajectory_type=trajectory_type,
            frame_id='world',  # 偏移后的轨迹在世界坐标系
            **kwargs
        )
        
        # 偏移轨迹到当前位置 (模拟坐标变换后的结果)
        for j, pt in enumerate(traj.points):
            # 旋转并平移
            px = pt.x * math.cos(theta) - pt.y * math.sin(theta) + x
            py = pt.x * math.sin(theta) + pt.y * math.cos(theta) + y
            traj.points[j] = Point3D(px, py, pt.z)
        
        # 创建里程计
        odom = create_test_odom(x, y, 0.0, theta, vx, vy)
        
        sequence.append((odom, traj))
        
        # 简单的运动学更新 (模拟跟踪)
        if len(traj.points) > 1:
            target = traj.points[1]
            dx = target.x - x
            dy = target.y - y
            dist = math.sqrt(dx*dx + dy*dy)
            
            if dist > 0.01:
                target_theta = math.atan2(dy, dx)
                theta_error = target_theta - theta
                # 归一化角度
                while theta_error > math.pi:
                    theta_error -= 2 * math.pi
                while theta_error < -math.pi:
                    theta_error += 2 * math.pi
                
                # 简单的比例控制
                omega = 2.0 * theta_error
                omega = max(-1.0, min(1.0, omega))
                
                speed = min(1.0, dist / dt)
                vx = speed
                
                # 更新状态
                x += vx * math.cos(theta) * dt
                y += vx * math.sin(theta) * dt
                theta += omega * dt
    
    return sequence


def create_local_trajectory_with_transform(
    robot_x: float,
    robot_y: float,
    robot_theta: float,
    num_points: int = 20,
    dt: float = 0.1,
    trajectory_type: str = 'straight',
    soft_enabled: bool = False,
    **kwargs
) -> Tuple[Trajectory, Trajectory]:
    """
    创建局部轨迹和对应的世界坐标系轨迹
    
    用于测试坐标变换功能。
    
    Args:
        robot_x, robot_y, robot_theta: 机器人在世界坐标系中的位姿
        num_points: 轨迹点数
        dt: 时间步长
        trajectory_type: 轨迹类型
        soft_enabled: 是否启用 Soft Head
        **kwargs: 轨迹参数
    
    Returns:
        (local_traj, world_traj): 局部坐标系轨迹和世界坐标系轨迹
    """
    # 创建局部坐标系轨迹
    local_traj = create_test_trajectory(
        num_points=num_points,
        dt=dt,
        trajectory_type=trajectory_type,
        frame_id='base_link',
        soft_enabled=soft_enabled,
        **kwargs
    )
    
    # 手动变换到世界坐标系 (用于验证)
    cos_theta = math.cos(robot_theta)
    sin_theta = math.sin(robot_theta)
    
    world_points = []
    for p in local_traj.points:
        wx = p.x * cos_theta - p.y * sin_theta + robot_x
        wy = p.x * sin_theta + p.y * cos_theta + robot_y
        world_points.append(Point3D(wx, wy, p.z))
    
    world_velocities = None
    if local_traj.velocities is not None:
        world_velocities = local_traj.velocities.copy()
        for i in range(len(world_velocities)):
            vx, vy = world_velocities[i, 0], world_velocities[i, 1]
            world_velocities[i, 0] = vx * cos_theta - vy * sin_theta
            world_velocities[i, 1] = vx * sin_theta + vy * cos_theta
    
    world_traj = Trajectory(
        header=Header(stamp=local_traj.header.stamp, frame_id='world'),
        points=world_points,
        velocities=world_velocities,
        dt_sec=local_traj.dt_sec,
        confidence=local_traj.confidence,
        soft_enabled=local_traj.soft_enabled
    )
    
    return local_traj, world_traj
