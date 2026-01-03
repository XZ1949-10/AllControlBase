"""
适配器边界条件测试

测试 ROS 适配器在各种边界条件下的行为。

测试覆盖:
1. 空数据处理
2. 无效数据处理
3. 数据维度不匹配
4. 坐标系处理
"""
import pytest
import sys
import os
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from universal_controller.config.default_config import DEFAULT_CONFIG
from universal_controller.core.enums import TrajectoryMode


# Mock 类定义
class MockTime:
    def __init__(self, sec=0, nsec=0):
        self.secs = sec
        self.nsecs = nsec


class MockHeader:
    def __init__(self, stamp=None, frame_id=""):
        self.stamp = stamp or MockTime()
        self.frame_id = frame_id


class MockPoint:
    def __init__(self, x=0, y=0, z=0):
        self.x = x
        self.y = y
        self.z = z


class MockQuaternion:
    def __init__(self, x=0, y=0, z=0, w=1):
        self.x = x
        self.y = y
        self.z = z
        self.w = w


class MockVector3:
    def __init__(self, x=0, y=0, z=0):
        self.x = x
        self.y = y
        self.z = z


class MockPose:
    def __init__(self):
        self.position = MockPoint()
        self.orientation = MockQuaternion()


class MockTwist:
    def __init__(self):
        self.linear = MockVector3()
        self.angular = MockVector3()


class MockPoseWithCov:
    def __init__(self):
        self.pose = MockPose()


class MockTwistWithCov:
    def __init__(self):
        self.twist = MockTwist()


class MockOdometry:
    def __init__(self):
        self.header = MockHeader()
        self.pose = MockPoseWithCov()
        self.twist = MockTwistWithCov()


class MockTrajectory:
    def __init__(self):
        self.header = MockHeader(frame_id="base_link")  # 使用有效的 frame_id
        self.points = []
        self.velocities_flat = []
        self.dt_sec = 0.1
        self.confidence = 0.9
        self.mode = 0
        self.soft_enabled = False


class TestTrajectoryAdapterEdgeCases:
    """测试轨迹适配器边界条件"""
    
    def test_empty_trajectory(self):
        """测试空轨迹"""
        from controller_ros.adapters.trajectory_adapter import TrajectoryAdapter
        
        adapter = TrajectoryAdapter(DEFAULT_CONFIG)
        
        ros_msg = MockTrajectory()
        ros_msg.points = []
        
        uc_traj = adapter.to_uc(ros_msg)
        
        assert len(uc_traj.points) == 0
        assert uc_traj.mode == TrajectoryMode.MODE_STOP
    
    def test_single_point_trajectory(self):
        """测试单点轨迹"""
        from controller_ros.adapters.trajectory_adapter import TrajectoryAdapter
        
        adapter = TrajectoryAdapter(DEFAULT_CONFIG)
        
        ros_msg = MockTrajectory()
        ros_msg.points = [MockPoint(1.0, 2.0, 0.0)]
        
        uc_traj = adapter.to_uc(ros_msg)
        
        assert len(uc_traj.points) == 1
        assert uc_traj.points[0].x == 1.0
    
    def test_velocity_dimension_mismatch(self):
        """测试速度维度不匹配"""
        from controller_ros.adapters.trajectory_adapter import TrajectoryAdapter
        
        adapter = TrajectoryAdapter(DEFAULT_CONFIG)
        
        ros_msg = MockTrajectory()
        ros_msg.points = [MockPoint(i * 0.1, 0, 0) for i in range(5)]
        # 速度数组长度不是 4 的倍数
        ros_msg.velocities_flat = [0.5, 0.0, 0.0]  # 只有 3 个元素
        ros_msg.soft_enabled = True
        
        uc_traj = adapter.to_uc(ros_msg)
        
        # 应该禁用 soft 模式或截断
        # 具体行为取决于实现
        assert uc_traj is not None
    
    def test_more_velocities_than_points(self):
        """测试速度点多于位置点"""
        from controller_ros.adapters.trajectory_adapter import TrajectoryAdapter
        
        adapter = TrajectoryAdapter(DEFAULT_CONFIG)
        
        ros_msg = MockTrajectory()
        ros_msg.points = [MockPoint(i * 0.1, 0, 0) for i in range(3)]
        ros_msg.velocities_flat = [0.5, 0.0, 0.0, 0.0] * 10  # 10 个速度点
        ros_msg.soft_enabled = True
        
        uc_traj = adapter.to_uc(ros_msg)
        
        # 速度应该被截断到与位置点数相同
        assert uc_traj.velocities.shape[0] == 3
    
    def test_fewer_velocities_than_points(self):
        """测试速度点少于位置点"""
        from controller_ros.adapters.trajectory_adapter import TrajectoryAdapter
        
        adapter = TrajectoryAdapter(DEFAULT_CONFIG)
        
        ros_msg = MockTrajectory()
        ros_msg.points = [MockPoint(i * 0.1, 0, 0) for i in range(10)]
        ros_msg.velocities_flat = [0.5, 0.0, 0.0, 0.0] * 3  # 只有 3 个速度点
        ros_msg.soft_enabled = True
        
        uc_traj = adapter.to_uc(ros_msg)
        
        # 速度应该被填充到与位置点数相同
        assert uc_traj.velocities.shape[0] == 10
    
    def test_empty_frame_id(self):
        """测试空 frame_id 会抛出异常 (安全性改进)"""
        from controller_ros.adapters.trajectory_adapter import TrajectoryAdapter
        
        adapter = TrajectoryAdapter(DEFAULT_CONFIG)
        
        ros_msg = MockTrajectory()
        ros_msg.header.frame_id = ""
        ros_msg.points = [MockPoint(0, 0, 0)]
        
        # 空 frame_id 应该抛出 ValueError (安全性改进: 拒绝隐式坐标系)
        with pytest.raises(ValueError, match="valid frame_id"):
            adapter.to_uc(ros_msg)
    
    def test_invalid_dt_sec(self):
        """测试无效 dt_sec"""
        from controller_ros.adapters.trajectory_adapter import TrajectoryAdapter
        
        adapter = TrajectoryAdapter(DEFAULT_CONFIG)
        
        ros_msg = MockTrajectory()
        ros_msg.points = [MockPoint(0, 0, 0)]
        ros_msg.dt_sec = -0.1  # 无效值
        
        uc_traj = adapter.to_uc(ros_msg)
        
        # 应该使用默认值
        assert uc_traj.dt_sec > 0
    
    def test_invalid_confidence(self):
        """测试无效置信度"""
        from controller_ros.adapters.trajectory_adapter import TrajectoryAdapter
        
        adapter = TrajectoryAdapter(DEFAULT_CONFIG)
        
        ros_msg = MockTrajectory()
        ros_msg.points = [MockPoint(0, 0, 0)]
        ros_msg.confidence = 1.5  # 超出范围
        
        uc_traj = adapter.to_uc(ros_msg)
        
        # 应该被裁剪到 [0, 1]
        assert 0.0 <= uc_traj.confidence <= 1.0
    
    def test_nan_point_coordinates(self):
        """测试 NaN 坐标"""
        from controller_ros.adapters.trajectory_adapter import TrajectoryAdapter
        
        adapter = TrajectoryAdapter(DEFAULT_CONFIG)
        
        ros_msg = MockTrajectory()
        ros_msg.points = [
            MockPoint(0, 0, 0),
            MockPoint(float('nan'), 0, 0),  # NaN 坐标
            MockPoint(0.2, 0, 0),
        ]
        
        uc_traj = adapter.to_uc(ros_msg)
        
        # NaN 点应该被过滤掉
        assert len(uc_traj.points) == 2
    
    def test_inf_point_coordinates(self):
        """测试 Inf 坐标"""
        from controller_ros.adapters.trajectory_adapter import TrajectoryAdapter
        
        adapter = TrajectoryAdapter(DEFAULT_CONFIG)
        
        ros_msg = MockTrajectory()
        ros_msg.points = [
            MockPoint(0, 0, 0),
            MockPoint(float('inf'), 0, 0),  # Inf 坐标
            MockPoint(0.2, 0, 0),
        ]
        
        uc_traj = adapter.to_uc(ros_msg)
        
        # Inf 点应该被过滤掉
        assert len(uc_traj.points) == 2
    
    def test_unknown_trajectory_mode(self):
        """测试未知轨迹模式"""
        from controller_ros.adapters.trajectory_adapter import TrajectoryAdapter
        
        adapter = TrajectoryAdapter(DEFAULT_CONFIG)
        
        ros_msg = MockTrajectory()
        ros_msg.points = [MockPoint(0, 0, 0)]
        ros_msg.mode = 999  # 未知模式
        
        uc_traj = adapter.to_uc(ros_msg)
        
        # 应该使用默认模式
        assert uc_traj.mode == TrajectoryMode.MODE_TRACK


class TestOdomAdapterEdgeCases:
    """测试里程计适配器边界条件"""
    
    def test_basic_odom_conversion(self):
        """测试基本里程计转换"""
        from controller_ros.adapters.odom_adapter import OdomAdapter
        
        adapter = OdomAdapter()
        
        ros_msg = MockOdometry()
        ros_msg.pose.pose.position.x = 1.0
        ros_msg.pose.pose.position.y = 2.0
        ros_msg.twist.twist.linear.x = 0.5
        
        uc_odom = adapter.to_uc(ros_msg)
        
        assert uc_odom.pose_position.x == 1.0
        assert uc_odom.pose_position.y == 2.0
        assert uc_odom.twist_linear[0] == 0.5
    
    def test_odom_with_rotation(self):
        """测试带旋转的里程计"""
        from controller_ros.adapters.odom_adapter import OdomAdapter
        
        adapter = OdomAdapter()
        
        ros_msg = MockOdometry()
        ros_msg.pose.pose.orientation.z = 0.707
        ros_msg.pose.pose.orientation.w = 0.707
        ros_msg.twist.twist.angular.z = 0.5
        
        uc_odom = adapter.to_uc(ros_msg)
        
        assert uc_odom.pose_orientation[2] == 0.707
        assert uc_odom.twist_angular[2] == 0.5


class TestImuAdapterEdgeCases:
    """测试 IMU 适配器边界条件"""
    
    def test_basic_imu_conversion(self):
        """测试基本 IMU 转换"""
        from controller_ros.adapters.imu_adapter import ImuAdapter
        
        adapter = ImuAdapter()
        
        class MockImu:
            def __init__(self):
                self.header = MockHeader()
                self.orientation = MockQuaternion()
                self.angular_velocity = MockVector3(0, 0, 0.5)
                self.linear_acceleration = MockVector3(0, 0, 9.8)
        
        ros_msg = MockImu()
        uc_imu = adapter.to_uc(ros_msg)
        
        assert uc_imu.angular_velocity[2] == 0.5
        assert uc_imu.linear_acceleration[2] == 9.8


class TestOutputAdapterEdgeCases:
    """测试输出适配器边界条件"""
    
    def test_control_output_conversion(self):
        """测试控制输出转换"""
        from controller_ros.adapters.output_adapter import OutputAdapter
        from universal_controller.core.data_types import ControlOutput
        
        adapter = OutputAdapter()
        
        uc_cmd = ControlOutput(
            vx=1.0,
            vy=0.0,
            vz=0.0,
            omega=0.5,
            frame_id="base_link",
            success=True
        )
        
        # 转换为 ROS 消息需要 ROS 环境，这里只测试接口存在
        assert hasattr(adapter, 'to_ros')
    
    def test_control_output_with_extras(self):
        """测试带额外数据的控制输出"""
        from controller_ros.adapters.output_adapter import OutputAdapter
        from universal_controller.core.data_types import ControlOutput
        
        adapter = OutputAdapter()
        
        uc_cmd = ControlOutput(
            vx=1.0,
            vy=0.0,
            vz=0.0,
            omega=0.5,
            frame_id="base_link",
            success=True,
            extras={'attitude_cmd': {'roll': 0.1, 'pitch': 0.2}}
        )
        
        assert 'attitude_cmd' in uc_cmd.extras


class TestVelocityDecayFilling:
    """测试速度衰减填充 (问题 8 补充测试)"""
    
    def test_velocity_decay_with_large_padding(self):
        """测试大量填充点时的速度衰减行为"""
        from controller_ros.adapters.trajectory_adapter import TrajectoryAdapter
        
        adapter = TrajectoryAdapter(DEFAULT_CONFIG)
        
        ros_msg = MockTrajectory()
        # 100 个位置点，但只有 5 个速度点
        ros_msg.points = [MockPoint(i * 0.1, 0, 0) for i in range(100)]
        ros_msg.velocities_flat = [1.0, 0.0, 0.0, 0.0] * 5  # 只有 5 个速度点
        ros_msg.soft_enabled = True
        ros_msg.mode = 0  # MODE_TRACK
        
        uc_traj = adapter.to_uc(ros_msg)
        
        # 验证填充后的速度数组维度正确
        assert uc_traj.velocities is not None
        assert uc_traj.velocities.shape == (100, 4)
        
        # 验证前 5 个速度点保持原值
        for i in range(5):
            assert np.isclose(uc_traj.velocities[i, 0], 1.0)
        
        # 验证填充点的速度是衰减的
        # 第 6 个点 (index 5) 应该接近原速度
        assert uc_traj.velocities[5, 0] > 0.9
        
        # 最后一个点的速度应该接近零但不为零
        last_vel_magnitude = np.sqrt(np.sum(uc_traj.velocities[-1, :3]**2))
        assert last_vel_magnitude > 0  # 不为零
        assert last_vel_magnitude < 0.1  # 接近零
    
    def test_velocity_decay_is_monotonic(self):
        """测试速度衰减是单调递减的"""
        from controller_ros.adapters.trajectory_adapter import TrajectoryAdapter
        
        adapter = TrajectoryAdapter(DEFAULT_CONFIG)
        
        ros_msg = MockTrajectory()
        ros_msg.points = [MockPoint(i * 0.1, 0, 0) for i in range(50)]
        ros_msg.velocities_flat = [2.0, 0.0, 0.0, 0.0] * 10  # 10 个速度点
        ros_msg.soft_enabled = True
        ros_msg.mode = 0
        
        uc_traj = adapter.to_uc(ros_msg)
        
        # 验证填充部分的速度是单调递减的
        for i in range(10, 49):  # 从第 11 个点到倒数第二个点
            current_speed = np.sqrt(np.sum(uc_traj.velocities[i, :3]**2))
            next_speed = np.sqrt(np.sum(uc_traj.velocities[i+1, :3]**2))
            assert current_speed >= next_speed, f"Speed at {i} ({current_speed}) < speed at {i+1} ({next_speed})"
    
    def test_velocity_decay_with_small_velocity(self):
        """测试小速度时使用零填充"""
        from controller_ros.adapters.trajectory_adapter import TrajectoryAdapter
        from universal_controller.core.data_types import TrajectoryDefaults
        
        adapter = TrajectoryAdapter(DEFAULT_CONFIG)
        
        # 使用小于阈值的速度
        small_vel = TrajectoryDefaults.velocity_decay_threshold * 0.5
        
        ros_msg = MockTrajectory()
        ros_msg.points = [MockPoint(i * 0.1, 0, 0) for i in range(20)]
        ros_msg.velocities_flat = [small_vel, 0.0, 0.0, 0.0] * 5
        ros_msg.soft_enabled = True
        ros_msg.mode = 0
        
        uc_traj = adapter.to_uc(ros_msg)
        
        # 填充部分应该是零速度
        for i in range(5, 20):
            assert np.allclose(uc_traj.velocities[i, :], 0.0)
    
    def test_velocity_zero_fill_for_stop_mode(self):
        """测试停止模式使用零填充"""
        from controller_ros.adapters.trajectory_adapter import TrajectoryAdapter
        
        adapter = TrajectoryAdapter(DEFAULT_CONFIG)
        
        ros_msg = MockTrajectory()
        ros_msg.points = [MockPoint(i * 0.1, 0, 0) for i in range(30)]
        ros_msg.velocities_flat = [1.0, 0.0, 0.0, 0.0] * 10
        ros_msg.soft_enabled = True
        ros_msg.mode = 1  # MODE_STOP
        
        uc_traj = adapter.to_uc(ros_msg)
        
        # 停止模式下，填充部分应该是零速度
        for i in range(10, 30):
            assert np.allclose(uc_traj.velocities[i, :], 0.0)
    
    def test_velocity_decay_preserves_direction(self):
        """测试速度衰减保持方向"""
        from controller_ros.adapters.trajectory_adapter import TrajectoryAdapter
        
        adapter = TrajectoryAdapter(DEFAULT_CONFIG)
        
        # 使用有方向的速度 (vx=1, vy=1)
        ros_msg = MockTrajectory()
        ros_msg.points = [MockPoint(i * 0.1, i * 0.1, 0) for i in range(20)]
        ros_msg.velocities_flat = [1.0, 1.0, 0.0, 0.5] * 5  # 有 vy 和 wz
        ros_msg.soft_enabled = True
        ros_msg.mode = 0
        
        uc_traj = adapter.to_uc(ros_msg)
        
        # 验证填充点的速度方向与最后一个原始速度点一致
        last_original = uc_traj.velocities[4, :]
        for i in range(5, 20):
            filled = uc_traj.velocities[i, :]
            # 检查方向一致性 (通过检查各分量的符号)
            for j in range(4):
                if abs(last_original[j]) > 1e-6 and abs(filled[j]) > 1e-6:
                    assert np.sign(last_original[j]) == np.sign(filled[j])
    
    def test_velocity_padding_count_boundary(self):
        """测试填充数量边界情况"""
        from controller_ros.adapters.trajectory_adapter import TrajectoryAdapter
        
        adapter = TrajectoryAdapter(DEFAULT_CONFIG)
        
        # 测试只需要填充 1 个点的情况
        ros_msg = MockTrajectory()
        ros_msg.points = [MockPoint(i * 0.1, 0, 0) for i in range(10)]
        ros_msg.velocities_flat = [1.0, 0.0, 0.0, 0.0] * 9  # 9 个速度点，需要填充 1 个
        ros_msg.soft_enabled = True
        ros_msg.mode = 0
        
        uc_traj = adapter.to_uc(ros_msg)
        
        assert uc_traj.velocities.shape == (10, 4)
        # 最后一个填充点的衰减因子 = 1/2 = 0.5
        assert np.isclose(uc_traj.velocities[9, 0], 0.5, atol=0.01)


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
