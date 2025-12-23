"""
DataManager 单元测试
"""
import pytest
import sys
import os
import time
import threading

# 添加 src 目录到路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from universal_controller.core.data_types import Odometry, Imu, Trajectory


# ==================== Mock 类 ====================

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


# ==================== 测试 ====================

def test_data_manager_initialization():
    """测试 DataManager 初始化"""
    from controller_ros.io.data_manager import DataManager
    
    dm = DataManager()
    
    assert dm.get_latest_odom() is None
    assert dm.get_latest_imu() is None
    assert dm.get_latest_trajectory() is None
    assert not dm.has_required_data()


def test_data_manager_update_odom():
    """测试更新里程计数据"""
    from controller_ros.io.data_manager import DataManager
    
    dm = DataManager()
    ros_msg = MockRosOdometry()
    
    uc_odom = dm.update_odom(ros_msg)
    
    assert isinstance(uc_odom, Odometry)
    assert uc_odom.pose_position.x == 1.0
    assert uc_odom.pose_position.y == 2.0
    
    # 验证缓存
    cached = dm.get_latest_odom()
    assert cached is uc_odom


def test_data_manager_update_imu():
    """测试更新 IMU 数据"""
    from controller_ros.io.data_manager import DataManager
    
    dm = DataManager()
    ros_msg = MockRosImu()
    
    uc_imu = dm.update_imu(ros_msg)
    
    assert isinstance(uc_imu, Imu)
    assert abs(uc_imu.linear_acceleration[2] - 9.81) < 0.01
    
    # 验证缓存
    cached = dm.get_latest_imu()
    assert cached is uc_imu


def test_data_manager_update_trajectory():
    """测试更新轨迹数据"""
    from controller_ros.io.data_manager import DataManager
    
    dm = DataManager()
    ros_msg = MockRosTrajectory()
    
    uc_traj = dm.update_trajectory(ros_msg)
    
    assert isinstance(uc_traj, Trajectory)
    assert len(uc_traj.points) == 10
    
    # 验证缓存
    cached = dm.get_latest_trajectory()
    assert cached is uc_traj


def test_data_manager_has_required_data():
    """测试必需数据检查"""
    from controller_ros.io.data_manager import DataManager
    
    dm = DataManager()
    
    # 初始状态：无数据
    assert not dm.has_required_data()
    
    # 只有 odom
    dm.update_odom(MockRosOdometry())
    assert not dm.has_required_data()
    
    # 有 odom 和 trajectory
    dm.update_trajectory(MockRosTrajectory())
    assert dm.has_required_data()


def test_data_manager_get_all_latest():
    """测试获取所有最新数据"""
    from controller_ros.io.data_manager import DataManager
    
    dm = DataManager()
    dm.update_odom(MockRosOdometry())
    dm.update_imu(MockRosImu())
    dm.update_trajectory(MockRosTrajectory())
    
    all_data = dm.get_all_latest()
    
    assert 'odom' in all_data
    assert 'imu' in all_data
    assert 'trajectory' in all_data
    assert all_data['odom'] is not None
    assert all_data['imu'] is not None
    assert all_data['trajectory'] is not None


def test_data_manager_data_ages():
    """测试数据年龄计算"""
    from controller_ros.io.data_manager import DataManager
    
    # 使用可控的时间函数
    current_time = [0.0]
    def get_time():
        return current_time[0]
    
    dm = DataManager(get_time_func=get_time)
    
    # 初始状态：所有数据年龄为无穷大
    ages = dm.get_data_ages()
    assert ages['odom'] == float('inf')
    assert ages['imu'] == float('inf')
    assert ages['trajectory'] == float('inf')
    
    # 更新 odom（时间 = 0）
    dm.update_odom(MockRosOdometry())
    
    # 时间前进 0.1 秒
    current_time[0] = 0.1
    
    ages = dm.get_data_ages()
    assert abs(ages['odom'] - 0.1) < 0.001
    assert ages['imu'] == float('inf')


def test_data_manager_is_data_fresh():
    """测试数据新鲜度检查"""
    from controller_ros.io.data_manager import DataManager
    
    current_time = [0.0]
    def get_time():
        return current_time[0]
    
    dm = DataManager(get_time_func=get_time)
    
    # 更新数据
    dm.update_odom(MockRosOdometry())
    dm.update_trajectory(MockRosTrajectory())
    
    # 数据刚更新，应该是新鲜的
    assert dm.is_data_fresh({'odom': 0.1, 'trajectory': 0.2})
    
    # 时间前进 0.15 秒
    current_time[0] = 0.15
    
    # odom 超时（0.15 > 0.1），trajectory 未超时（0.15 < 0.2）
    assert not dm.is_data_fresh({'odom': 0.1, 'trajectory': 0.2})
    assert dm.is_data_fresh({'odom': 0.2, 'trajectory': 0.2})


def test_data_manager_clear():
    """测试清除数据"""
    from controller_ros.io.data_manager import DataManager
    
    dm = DataManager()
    dm.update_odom(MockRosOdometry())
    dm.update_imu(MockRosImu())
    dm.update_trajectory(MockRosTrajectory())
    
    assert dm.has_required_data()
    
    dm.clear()
    
    assert not dm.has_required_data()
    assert dm.get_latest_odom() is None
    assert dm.get_latest_imu() is None
    assert dm.get_latest_trajectory() is None


def test_data_manager_thread_safety():
    """测试线程安全性"""
    from controller_ros.io.data_manager import DataManager
    
    dm = DataManager()
    errors = []
    
    def writer():
        for i in range(100):
            try:
                dm.update_odom(MockRosOdometry())
                dm.update_imu(MockRosImu())
                dm.update_trajectory(MockRosTrajectory())
            except Exception as e:
                errors.append(e)
    
    def reader():
        for i in range(100):
            try:
                dm.get_latest_odom()
                dm.get_latest_imu()
                dm.get_latest_trajectory()
                dm.get_data_ages()
                dm.has_required_data()
            except Exception as e:
                errors.append(e)
    
    threads = [
        threading.Thread(target=writer),
        threading.Thread(target=writer),
        threading.Thread(target=reader),
        threading.Thread(target=reader),
    ]
    
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    
    assert len(errors) == 0, f"Thread safety errors: {errors}"


def test_data_manager_custom_time_func():
    """测试自定义时间函数"""
    from controller_ros.io.data_manager import DataManager
    
    mock_time = [100.0]
    def custom_time():
        return mock_time[0]
    
    dm = DataManager(get_time_func=custom_time)
    dm.update_odom(MockRosOdometry())
    
    timestamps = dm.get_timestamps()
    assert timestamps['odom'] == 100.0
    
    # 时间前进
    mock_time[0] = 100.5
    ages = dm.get_data_ages()
    assert abs(ages['odom'] - 0.5) < 0.001


def test_data_manager_clock_rollback():
    """测试时钟回退场景（如仿真时间重置）"""
    from controller_ros.io.data_manager import DataManager
    
    mock_time = [100.0]
    def custom_time():
        return mock_time[0]
    
    dm = DataManager(get_time_func=custom_time)
    dm.update_odom(MockRosOdometry())
    
    # 先正常获取一次年龄，建立基准时间
    dm.get_data_ages()
    
    # 时间回退（模拟仿真重置）
    mock_time[0] = 50.0
    
    ages = dm.get_data_ages()
    
    # 年龄应该被 clamp 到 0，而不是负值
    assert ages['odom'] >= 0.0, f"Age should not be negative, got {ages['odom']}"
    assert ages['odom'] == 0.0, f"Age should be clamped to 0 on clock rollback"
    
    # 应该检测到时钟回退
    assert dm.did_clock_jump_back() == True


def test_data_manager_clock_jump_flag():
    """测试时钟回退标志"""
    from controller_ros.io.data_manager import DataManager
    
    mock_time = [100.0]
    def custom_time():
        return mock_time[0]
    
    dm = DataManager(get_time_func=custom_time)
    
    # 初始状态：无时钟回退
    assert dm.did_clock_jump_back() == False
    
    dm.update_odom(MockRosOdometry())
    dm.get_data_ages()  # 触发时间检查，建立基准
    
    # 正常时间前进
    mock_time[0] = 101.0
    dm.get_data_ages()
    assert dm.did_clock_jump_back() == False
    
    # 时钟回退
    mock_time[0] = 50.0
    dm.get_data_ages()
    assert dm.did_clock_jump_back() == True
    
    # 清除标志
    dm.clear_clock_jump_flag()
    assert dm.did_clock_jump_back() == False


def test_data_manager_clear_resets_clock_state():
    """测试 clear() 重置时钟状态"""
    from controller_ros.io.data_manager import DataManager
    
    mock_time = [100.0]
    def custom_time():
        return mock_time[0]
    
    dm = DataManager(get_time_func=custom_time)
    dm.update_odom(MockRosOdometry())
    
    # 先建立基准时间
    dm.get_data_ages()
    
    # 触发时钟回退
    mock_time[0] = 50.0
    dm.get_data_ages()
    assert dm.did_clock_jump_back() == True
    
    # clear 应该重置所有状态
    dm.clear()
    assert dm.did_clock_jump_back() == False


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
