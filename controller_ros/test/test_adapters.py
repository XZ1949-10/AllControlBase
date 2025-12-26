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
    """测试轨迹适配器 Soft 模式部分截断
    
    设计说明：
    - 当速度点少于位置点时，使用线性衰减填充
    - 衰减从最后一个速度点开始，线性衰减到零
    - 这确保轨迹末端平滑减速，避免高速运动到轨迹末端
    """
    from controller_ros.adapters.trajectory_adapter import TrajectoryAdapter
    
    adapter = TrajectoryAdapter()
    ros_msg = MockRosTrajectory()
    ros_msg.soft_enabled = True
    ros_msg.velocities_flat = [1.0, 0.0, 0.0, 0.0, 2.0, 0.0]  # 6 个值，截断为 4 个 (1 个速度点)
    
    uc_traj = adapter.to_uc(ros_msg)
    
    assert uc_traj.soft_enabled == True
    assert uc_traj.velocities is not None
    # 1 个有效速度点，但有 10 个位置点，需要填充 9 个点
    assert uc_traj.velocities.shape == (10, 4)
    assert uc_traj.velocities[0, 0] == 1.0
    
    # 填充的点应该线性衰减到零
    # 填充 9 个点，衰减因子: (9-1-i)/9 = (8-i)/9
    # i=0 (index 1): 8/9 ≈ 0.889
    # i=8 (index 9): 0/9 = 0
    padding_count = 9
    for i in range(padding_count):
        expected_decay = (padding_count - 1 - i) / padding_count
        expected_vx = 1.0 * expected_decay
        assert abs(uc_traj.velocities[1 + i, 0] - expected_vx) < 0.001, \
            f"Point {1+i} should have decayed velocity {expected_vx}, got {uc_traj.velocities[1+i, 0]}"
    
    # 最后一个填充点应该接近零
    assert abs(uc_traj.velocities[9, 0]) < 0.001


def test_trajectory_adapter_velocity_points_mismatch():
    """测试轨迹适配器速度点数与位置点数不匹配"""
    from controller_ros.adapters.trajectory_adapter import TrajectoryAdapter
    
    adapter = TrajectoryAdapter()
    ros_msg = MockRosTrajectory()
    ros_msg.points = [MockRosPoint(i * 0.1, 0, 0) for i in range(5)]  # 5 个位置点
    ros_msg.soft_enabled = True
    ros_msg.velocities_flat = [1.0, 0.0, 0.0, 0.0] * 10  # 10 个速度点
    
    uc_traj = adapter.to_uc(ros_msg)
    
    assert uc_traj.soft_enabled == True
    assert uc_traj.velocities is not None
    # 速度点应该被截断到与位置点相同
    assert uc_traj.velocities.shape == (5, 4)
    assert len(uc_traj.points) == 5


def test_trajectory_adapter_velocity_padding():
    """测试轨迹适配器速度点填充 (速度点少于位置点)
    
    设计说明：
    - 当速度点少于位置点时，使用线性衰减填充
    - 衰减从最后一个速度点开始，线性衰减到零
    - 这确保轨迹末端平滑减速，避免高速运动到轨迹末端
    """
    from controller_ros.adapters.trajectory_adapter import TrajectoryAdapter
    
    adapter = TrajectoryAdapter()
    ros_msg = MockRosTrajectory()
    ros_msg.points = [MockRosPoint(i * 0.1, 0, 0) for i in range(10)]  # 10 个位置点
    ros_msg.soft_enabled = True
    # 只有 3 个速度点，最后一个是 [3.0, 0.3, 0.0, 0.03]
    ros_msg.velocities_flat = [
        1.0, 0.1, 0.0, 0.01,
        2.0, 0.2, 0.0, 0.02,
        3.0, 0.3, 0.0, 0.03,
    ]
    
    uc_traj = adapter.to_uc(ros_msg)
    
    assert uc_traj.soft_enabled == True
    assert uc_traj.velocities is not None
    assert uc_traj.velocities.shape == (10, 4)
    
    # 前 3 个点应该是原始值
    assert uc_traj.velocities[0, 0] == 1.0
    assert uc_traj.velocities[1, 0] == 2.0
    assert uc_traj.velocities[2, 0] == 3.0
    
    # 后 7 个点应该线性衰减到零
    # 最后一个速度点是 [3.0, 0.3, 0.0, 0.03]
    # 填充 7 个点，衰减因子: (7-1-i)/7 = (6-i)/7
    padding_count = 7
    last_vel = np.array([3.0, 0.3, 0.0, 0.03])
    for i in range(padding_count):
        expected_decay = (padding_count - 1 - i) / padding_count
        expected_vel = last_vel * expected_decay
        actual_vel = uc_traj.velocities[3 + i]
        assert np.allclose(actual_vel, expected_vel, atol=0.001), \
            f"Point {3+i} should have decayed velocity {expected_vel}, got {actual_vel}"
    
    # 最后一个填充点应该接近零
    assert np.allclose(uc_traj.velocities[9], [0, 0, 0, 0], atol=0.001)


def test_output_adapter_to_ros():
    """测试输出适配器 UC → ROS"""
    from controller_ros.adapters.output_adapter import OutputAdapter
    
    adapter = OutputAdapter(default_frame_id='base_link')
    
    uc_cmd = ControlOutput(
        vx=1.0, vy=0.5, vz=0.0, omega=0.2,
        frame_id='world', success=True, solve_time_ms=5.0
    )
    
    # 由于没有实际的 ROS 消息类型，这里只测试适配器逻辑
    # 实际测试需要在 ROS 环境中进行
    assert adapter._default_frame_id == 'base_link'


def test_output_adapter_stop_cmd():
    """测试输出适配器停止命令"""
    from controller_ros.adapters.output_adapter import OutputAdapter
    
    adapter = OutputAdapter(default_frame_id='base_link')
    
    # 测试默认 frame_id
    assert adapter._default_frame_id == 'base_link'


def test_trajectory_adapter_empty_trajectory():
    """测试轨迹适配器处理空轨迹"""
    from controller_ros.adapters.trajectory_adapter import TrajectoryAdapter
    
    adapter = TrajectoryAdapter()
    ros_msg = MockRosTrajectory(num_points=0)
    ros_msg.dt_sec = 0.1
    ros_msg.confidence = 0.9
    ros_msg.soft_enabled = False
    
    uc_traj = adapter.to_uc(ros_msg)
    
    # 应该返回 MODE_STOP 轨迹
    assert isinstance(uc_traj, Trajectory)
    assert len(uc_traj.points) == 0
    assert uc_traj.mode == TrajectoryMode.MODE_STOP
    assert uc_traj.confidence == 0.0
    assert uc_traj.soft_enabled == False
    assert uc_traj.velocities is None


def test_trajectory_adapter_empty_trajectory_with_soft():
    """测试轨迹适配器处理空轨迹 (soft_enabled=True)"""
    from controller_ros.adapters.trajectory_adapter import TrajectoryAdapter
    
    adapter = TrajectoryAdapter()
    ros_msg = MockRosTrajectory(num_points=0)
    ros_msg.soft_enabled = True
    ros_msg.velocities_flat = [1.0, 0.0, 0.0, 0.0]  # 有速度数据但无位置点
    
    uc_traj = adapter.to_uc(ros_msg)
    
    # 应该返回 MODE_STOP 轨迹，忽略速度数据
    assert len(uc_traj.points) == 0
    assert uc_traj.mode == TrajectoryMode.MODE_STOP
    assert uc_traj.soft_enabled == False
    assert uc_traj.velocities is None


def test_trajectory_adapter_empty_frame_id():
    """测试轨迹适配器处理空 frame_id"""
    from controller_ros.adapters.trajectory_adapter import TrajectoryAdapter
    
    adapter = TrajectoryAdapter()
    ros_msg = MockRosTrajectory(frame_id='')
    
    uc_traj = adapter.to_uc(ros_msg)
    
    # 应该使用默认 frame_id
    assert uc_traj.header.frame_id == 'base_link'


def test_trajectory_adapter_zero_dt_sec():
    """测试轨迹适配器处理零 dt_sec (空轨迹情况)"""
    from controller_ros.adapters.trajectory_adapter import TrajectoryAdapter
    
    adapter = TrajectoryAdapter()
    ros_msg = MockRosTrajectory(num_points=0, dt_sec=0.0)
    
    uc_traj = adapter.to_uc(ros_msg)
    
    # 空轨迹时应该使用默认 dt_sec
    assert uc_traj.dt_sec == 0.1


if __name__ == '__main__':
    pytest.main([__file__, '-v'])


def test_trajectory_adapter_velocity_padding_stop_mode():
    """测试轨迹适配器在停止模式下使用零速度填充"""
    from controller_ros.adapters.trajectory_adapter import TrajectoryAdapter
    
    adapter = TrajectoryAdapter()
    ros_msg = MockRosTrajectory()
    ros_msg.points = [MockRosPoint(i * 0.1, 0, 0) for i in range(10)]  # 10 个位置点
    ros_msg.mode = 1  # MODE_STOP
    ros_msg.soft_enabled = True
    # 只有 3 个速度点
    ros_msg.velocities_flat = [
        1.0, 0.1, 0.0, 0.01,
        2.0, 0.2, 0.0, 0.02,
        3.0, 0.3, 0.0, 0.03,
    ]
    
    uc_traj = adapter.to_uc(ros_msg)
    
    assert uc_traj.soft_enabled == True
    assert uc_traj.velocities is not None
    assert uc_traj.velocities.shape == (10, 4)
    
    # 前 3 个点应该是原始值
    assert uc_traj.velocities[0, 0] == 1.0
    assert uc_traj.velocities[1, 0] == 2.0
    assert uc_traj.velocities[2, 0] == 3.0
    
    # 后 7 个点应该用零填充 (停止模式)
    for i in range(3, 10):
        assert uc_traj.velocities[i, 0] == 0.0, f"Point {i} should be padded with zero for stop mode"
        assert uc_traj.velocities[i, 1] == 0.0
        assert uc_traj.velocities[i, 2] == 0.0
        assert uc_traj.velocities[i, 3] == 0.0


def test_trajectory_adapter_velocity_padding_emergency_mode():
    """测试轨迹适配器在紧急模式下使用零速度填充"""
    from controller_ros.adapters.trajectory_adapter import TrajectoryAdapter
    
    adapter = TrajectoryAdapter()
    ros_msg = MockRosTrajectory()
    ros_msg.points = [MockRosPoint(i * 0.1, 0, 0) for i in range(5)]  # 5 个位置点
    ros_msg.mode = 3  # MODE_EMERGENCY
    ros_msg.soft_enabled = True
    # 只有 2 个速度点
    ros_msg.velocities_flat = [
        1.0, 0.0, 0.0, 0.1,
        0.5, 0.0, 0.0, 0.05,
    ]
    
    uc_traj = adapter.to_uc(ros_msg)
    
    assert uc_traj.soft_enabled == True
    assert uc_traj.velocities is not None
    assert uc_traj.velocities.shape == (5, 4)
    
    # 前 2 个点应该是原始值
    assert uc_traj.velocities[0, 0] == 1.0
    assert uc_traj.velocities[1, 0] == 0.5
    
    # 后 3 个点应该用零填充 (紧急模式)
    for i in range(2, 5):
        assert uc_traj.velocities[i, 0] == 0.0, f"Point {i} should be padded with zero for emergency mode"


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
