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


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
