"""
适配器层单元测试
"""
import pytest
import sys
import os
import numpy as np

# 添加 src 目录和 test 目录到路径
_test_dir = os.path.dirname(__file__)
sys.path.insert(0, os.path.join(_test_dir, '..', 'src'))
sys.path.insert(0, _test_dir)

# 测试不依赖 ROS，使用 mock 数据
from universal_controller.core.data_types import (
    Odometry, Imu, Trajectory, ControlOutput, Header, Point3D
)
from universal_controller.core.enums import TrajectoryMode

# 导入共享的 Mock 类 (从 conftest.py 或 fixtures 目录)
from conftest import (
    MockRosTime, MockRosHeader, MockRosPoint, MockRosQuaternion,
    MockRosVector3, MockRosPose, MockRosTwist, MockRosPoseWithCovariance,
    MockRosTwistWithCovariance, MockRosOdometry, MockRosImu, MockRosTrajectory
)


def test_odom_adapter_to_uc():
    """测试里程计适配器 ROS → UC"""
    from controller_ros.adapters.odom_adapter import OdomAdapter
    
    adapter = OdomAdapter()
    ros_msg = MockRosOdometry()
    
    uc_odom = adapter.to_uc(ros_msg)
    
    assert isinstance(uc_odom, Odometry)
    assert uc_odom.pose_position.x == 1.0
    assert uc_odom.pose_position.y == 2.0
    assert uc_odom.twist_linear[0] == 0.5
    assert uc_odom.twist_angular[2] == 0.1
    assert uc_odom.header.frame_id == 'odom'
    # 时间戳: 1000 + 0.5 = 1000.5
    assert abs(uc_odom.header.stamp - 1000.5) < 0.001


def test_imu_adapter_to_uc():
    """测试 IMU 适配器 ROS → UC"""
    from controller_ros.adapters.imu_adapter import ImuAdapter
    
    adapter = ImuAdapter()
    ros_msg = MockRosImu()
    
    uc_imu = adapter.to_uc(ros_msg)
    
    assert isinstance(uc_imu, Imu)
    assert uc_imu.orientation[3] == 1.0  # w
    assert abs(uc_imu.linear_acceleration[2] - 9.81) < 0.01
    assert uc_imu.header.frame_id == 'imu_link'


def test_trajectory_adapter_to_uc():
    """测试轨迹适配器 ROS → UC"""
    from controller_ros.adapters.trajectory_adapter import TrajectoryAdapter
    
    adapter = TrajectoryAdapter()
    ros_msg = MockRosTrajectory()
    
    uc_traj = adapter.to_uc(ros_msg)
    
    assert isinstance(uc_traj, Trajectory)
    assert len(uc_traj.points) == 10
    assert uc_traj.points[0].x == 0.0
    assert abs(uc_traj.points[1].x - 0.1) < 0.001
    assert uc_traj.dt_sec == 0.1
    assert uc_traj.confidence == 0.9
    assert uc_traj.soft_enabled == False
    assert uc_traj.header.frame_id == 'base_link'


def test_trajectory_adapter_soft_mode():
    """测试轨迹适配器 Soft 模式"""
    from controller_ros.adapters.trajectory_adapter import TrajectoryAdapter
    
    adapter = TrajectoryAdapter()
    ros_msg = MockRosTrajectory()
    ros_msg.soft_enabled = True
    ros_msg.velocities_flat = [1.0, 0.0, 0.0, 0.0] * 10  # 10 个点，每点 4 个值
    
    uc_traj = adapter.to_uc(ros_msg)
    
    assert uc_traj.soft_enabled == True
    assert uc_traj.velocities is not None
    assert uc_traj.velocities.shape == (10, 4)
    assert uc_traj.velocities[0, 0] == 1.0


def test_trajectory_adapter_soft_mode_empty_velocities():
    """测试轨迹适配器 Soft 模式但无速度数据"""
    from controller_ros.adapters.trajectory_adapter import TrajectoryAdapter
    
    adapter = TrajectoryAdapter()
    ros_msg = MockRosTrajectory()
    ros_msg.soft_enabled = True
    ros_msg.velocities_flat = []  # 空速度数据
    
    uc_traj = adapter.to_uc(ros_msg)
    
    # soft_enabled 应该被禁用
    assert uc_traj.soft_enabled == False
    assert uc_traj.velocities is None


def test_trajectory_adapter_soft_mode_invalid_length():
    """测试轨迹适配器 Soft 模式但速度数据长度无效"""
    from controller_ros.adapters.trajectory_adapter import TrajectoryAdapter
    
    adapter = TrajectoryAdapter()
    ros_msg = MockRosTrajectory()
    ros_msg.soft_enabled = True
    ros_msg.velocities_flat = [1.0, 0.0, 0.0]  # 长度不是 4 的倍数，截断后为 0
    
    uc_traj = adapter.to_uc(ros_msg)
    
    # soft_enabled 应该被禁用
    assert uc_traj.soft_enabled == False
    assert uc_traj.velocities is None


def test_trajectory_adapter_soft_mode_partial_truncate():
    """测试轨迹适配器 Soft 模式部分截断 - Refactored for Strict Mode
    
    设计说明 (New):
    - 当速度点数量不匹配时 (无论多少)，应直接拒绝速度数据
    """
    from controller_ros.adapters.trajectory_adapter import TrajectoryAdapter
    
    adapter = TrajectoryAdapter()
    ros_msg = MockRosTrajectory()
    ros_msg.soft_enabled = True
    ros_msg.velocities_flat = [1.0, 0.0, 0.0, 0.0, 2.0, 0.0]  # 6 个值 -> 1 个点 (10 位置点)
    
    uc_traj = adapter.to_uc(ros_msg)
    
    # 严格模式下，数量不匹配应导致 velocities 为 None，soft_enabled 设为 False
    assert uc_traj.soft_enabled == False
    assert uc_traj.velocities is None


def test_trajectory_adapter_velocity_points_mismatch():
    """测试轨迹适配器速度点数与位置点数不匹配 - Refactored for Strict Mode"""
    from controller_ros.adapters.trajectory_adapter import TrajectoryAdapter
    
    adapter = TrajectoryAdapter()
    ros_msg = MockRosTrajectory()
    ros_msg.points = [MockRosPoint(i * 0.1, 0, 0) for i in range(5)]  # 5 个位置点
    ros_msg.soft_enabled = True
    ros_msg.velocities_flat = [1.0, 0.0, 0.0, 0.0] * 10  # 10 个速度点
    
    uc_traj = adapter.to_uc(ros_msg)
    
    # 严格模式下，数量不匹配应导致 velocities 为 None
    assert uc_traj.soft_enabled == False
    assert uc_traj.velocities is None


def test_trajectory_adapter_velocity_padding():
    """测试轨迹适配器速度点填充 - Refactored for Strict Mode"""
    from controller_ros.adapters.trajectory_adapter import TrajectoryAdapter
    
    adapter = TrajectoryAdapter()
    ros_msg = MockRosTrajectory()
    ros_msg.points = [MockRosPoint(i * 0.1, 0, 0) for i in range(10)]  # 10 个位置点
    ros_msg.soft_enabled = True
    # 只有 3 个速度点
    ros_msg.velocities_flat = [
        1.0, 0.1, 0.0, 0.01,
        2.0, 0.2, 0.0, 0.02,
        3.0, 0.3, 0.0, 0.03,
    ]
    
    uc_traj = adapter.to_uc(ros_msg)
    
    # 严格模式下，必须拒绝
    assert uc_traj.soft_enabled == False
    assert uc_traj.velocities is None


def test_trajectory_adapter_velocity_padding_stop_mode():
    """测试轨迹适配器在停止模式下使用零速度填充 - Refactored for Strict Mode"""
    # 停止模式下虽然以前允许填充，但为了统一严格性，现在也应要求上游提供完整的零速度数组
    # 或者上游完全不提供速度 (velocities_flat=[])，此时 adapter 会返回 None velocities，
    # ControllerManager 可能会根据 MODE_STOP 自己处理。
    # 这里测试的是: 如果提供了不完整的数据，Adapter 应该拒绝，而不是尝试修复。
    from controller_ros.adapters.trajectory_adapter import TrajectoryAdapter
    
    adapter = TrajectoryAdapter()
    ros_msg = MockRosTrajectory()
    ros_msg.points = [MockRosPoint(i * 0.1, 0, 0) for i in range(10)]
    ros_msg.mode = 1  # MODE_STOP
    ros_msg.soft_enabled = True
    # 3 个速度点 vs 10 个位置点
    ros_msg.velocities_flat = [
        1.0, 0.1, 0.0, 0.01,
        2.0, 0.2, 0.0, 0.02,
        3.0, 0.3, 0.0, 0.03,
    ]
    
    uc_traj = adapter.to_uc(ros_msg)
    
    assert uc_traj.soft_enabled == False
    assert uc_traj.velocities is None


def test_trajectory_adapter_velocity_padding_emergency_mode():
    """测试轨迹适配器在紧急模式下使用零速度填充 - Refactored for Strict Mode"""
    from controller_ros.adapters.trajectory_adapter import TrajectoryAdapter
    
    adapter = TrajectoryAdapter()
    ros_msg = MockRosTrajectory()
    ros_msg.points = [MockRosPoint(i * 0.1, 0, 0) for i in range(5)]
    ros_msg.mode = 3  # MODE_EMERGENCY
    ros_msg.soft_enabled = True
    # 2 个速度点 vs 5 个位置点
    ros_msg.velocities_flat = [
        1.0, 0.0, 0.0, 0.1,
        0.5, 0.0, 0.0, 0.05,
    ]
    
    uc_traj = adapter.to_uc(ros_msg)
    
    assert uc_traj.soft_enabled == False
    assert uc_traj.velocities is None


def test_trajectory_adapter_confidence_clamping():
    """测试轨迹适配器置信度范围限制"""
    from controller_ros.adapters.trajectory_adapter import TrajectoryAdapter
    
    adapter = TrajectoryAdapter()
    
    # 测试置信度超出上限
    ros_msg = MockRosTrajectory()
    ros_msg.confidence = 1.5
    uc_traj = adapter.to_uc(ros_msg)
    assert uc_traj.confidence == 1.0, "Confidence should be clamped to 1.0"
    
    # 测试置信度低于下限
    ros_msg.confidence = -0.5
    uc_traj = adapter.to_uc(ros_msg)
    assert uc_traj.confidence == 0.0, "Confidence should be clamped to 0.0"
    
    # 测试正常置信度
    ros_msg.confidence = 0.8
    uc_traj = adapter.to_uc(ros_msg)
    assert uc_traj.confidence == 0.8, "Normal confidence should not be changed"
