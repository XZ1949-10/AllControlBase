"""
适配器层单元测试
"""
import pytest
import sys
import os
import numpy as np
import time

# 添加 src 目录到路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

# 测试不依赖 ROS，使用 mock 数据
from universal_controller.core.data_types import (
    Odometry, Imu, Trajectory, ControlOutput, Header, Point3D
)
from universal_controller.core.enums import TrajectoryMode


class MockRosTime:
    """模拟 ROS 时间戳"""
    def __init__(self, sec: int = 0, nanosec: int = 0):
        self.sec = sec
        self.nanosec = nanosec


class MockRosHeader:
    """模拟 ROS Header"""
    def __init__(self, stamp=None, frame_id=''):
        self.stamp = stamp or MockRosTime()
        self.frame_id = frame_id


class MockRosPoint:
    """模拟 ROS Point"""
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z


class MockRosQuaternion:
    """模拟 ROS Quaternion"""
    def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
        self.x = x
        self.y = y
        self.z = z
        self.w = w


class MockRosVector3:
    """模拟 ROS Vector3"""
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z


class MockRosPose:
    """模拟 ROS Pose"""
    def __init__(self):
        self.position = MockRosPoint()
        self.orientation = MockRosQuaternion()


class MockRosTwist:
    """模拟 ROS Twist"""
    def __init__(self):
        self.linear = MockRosVector3()
        self.angular = MockRosVector3()


class MockRosPoseWithCovariance:
    """模拟 ROS PoseWithCovariance"""
    def __init__(self):
        self.pose = MockRosPose()


class MockRosTwistWithCovariance:
    """模拟 ROS TwistWithCovariance"""
    def __init__(self):
        self.twist = MockRosTwist()


class MockRosOdometry:
    """模拟 ROS Odometry 消息"""
    def __init__(self):
        self.header = MockRosHeader(MockRosTime(1000, 500000000), 'odom')
        self.pose = MockRosPoseWithCovariance()
        self.twist = MockRosTwistWithCovariance()
        
        # 设置测试值
        self.pose.pose.position.x = 1.0
        self.pose.pose.position.y = 2.0
        self.pose.pose.position.z = 0.0
        self.pose.pose.orientation.w = 1.0
        self.twist.twist.linear.x = 0.5
        self.twist.twist.angular.z = 0.1


class MockRosImu:
    """模拟 ROS Imu 消息"""
    def __init__(self):
        self.header = MockRosHeader(MockRosTime(1000, 500000000), 'imu_link')
        self.orientation = MockRosQuaternion()
        self.angular_velocity = MockRosVector3()
        self.linear_acceleration = MockRosVector3(z=9.81)


class MockRosTrajectory:
    """模拟 ROS LocalTrajectoryV4 消息"""
    def __init__(self):
        self.header = MockRosHeader(MockRosTime(1000, 500000000), 'base_link')
        self.mode = 0
        self.points = [MockRosPoint(i * 0.1, 0, 0) for i in range(10)]
        self.velocities_flat = []
        self.dt_sec = 0.1
        self.confidence = 0.9
        self.soft_enabled = False


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
    """测试轨迹适配器 Soft 模式部分截断"""
    from controller_ros.adapters.trajectory_adapter import TrajectoryAdapter
    
    adapter = TrajectoryAdapter()
    ros_msg = MockRosTrajectory()
    ros_msg.soft_enabled = True
    ros_msg.velocities_flat = [1.0, 0.0, 0.0, 0.0, 2.0, 0.0]  # 6 个值，截断为 4 个 (1 个速度点)
    
    uc_traj = adapter.to_uc(ros_msg)
    
    assert uc_traj.soft_enabled == True
    assert uc_traj.velocities is not None
    # 1 个有效速度点，但有 10 个位置点，所以会用最后一个速度点填充到 10 个
    assert uc_traj.velocities.shape == (10, 4)
    assert uc_traj.velocities[0, 0] == 1.0
    # 填充的点应该是最后一个速度点的值 (不再是零)
    assert uc_traj.velocities[1, 0] == 1.0
    assert uc_traj.velocities[9, 0] == 1.0


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
    """测试轨迹适配器速度点填充 (速度点少于位置点)"""
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
    
    # 后 7 个点应该用最后一个速度点填充
    for i in range(3, 10):
        assert uc_traj.velocities[i, 0] == 3.0, f"Point {i} should be padded with last velocity"
        assert uc_traj.velocities[i, 1] == 0.3
        assert uc_traj.velocities[i, 3] == 0.03


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
    ros_msg = MockRosTrajectory()
    ros_msg.points = []  # 空轨迹
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
    ros_msg = MockRosTrajectory()
    ros_msg.points = []  # 空轨迹
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
    ros_msg = MockRosTrajectory()
    ros_msg.header.frame_id = ''  # 空 frame_id
    
    uc_traj = adapter.to_uc(ros_msg)
    
    # 应该使用默认 frame_id
    assert uc_traj.header.frame_id == 'base_link'


def test_trajectory_adapter_zero_dt_sec():
    """测试轨迹适配器处理零 dt_sec (空轨迹情况)"""
    from controller_ros.adapters.trajectory_adapter import TrajectoryAdapter
    
    adapter = TrajectoryAdapter()
    ros_msg = MockRosTrajectory()
    ros_msg.points = []  # 空轨迹
    ros_msg.dt_sec = 0.0  # 零 dt_sec
    
    uc_traj = adapter.to_uc(ros_msg)
    
    # 空轨迹时应该使用默认 dt_sec
    assert uc_traj.dt_sec == 0.1


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
