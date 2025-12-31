"""
DataManager unit tests
"""
import pytest
import threading

from universal_controller.core.data_types import Odometry, Imu, Trajectory

from conftest import (
    MockRosOdometry, MockRosImu, MockRosTrajectory
)


def test_data_manager_initialization():
    """Test DataManager initialization"""
    from controller_ros.io.data_manager import DataManager
    
    dm = DataManager()
    
    assert dm.get_latest_odom() is None
    assert dm.get_latest_imu() is None
    assert dm.get_latest_trajectory() is None
    assert not dm.has_required_data()


def test_data_manager_update_odom():
    """Test odom update"""
    from controller_ros.io.data_manager import DataManager
    
    dm = DataManager()
    ros_msg = MockRosOdometry()
    
    uc_odom = dm.update_odom(ros_msg)
    
    assert isinstance(uc_odom, Odometry)
    assert uc_odom.pose_position.x == 1.0
    assert uc_odom.pose_position.y == 2.0
    
    cached = dm.get_latest_odom()
    assert cached is uc_odom


def test_data_manager_update_imu():
    """Test IMU update"""
    from controller_ros.io.data_manager import DataManager
    
    dm = DataManager()
    ros_msg = MockRosImu()
    
    uc_imu = dm.update_imu(ros_msg)
    
    assert isinstance(uc_imu, Imu)
    assert abs(uc_imu.linear_acceleration[2] - 9.81) < 0.01
    
    cached = dm.get_latest_imu()
    assert cached is uc_imu


def test_data_manager_update_trajectory():
    """Test trajectory update"""
    from controller_ros.io.data_manager import DataManager
    
    dm = DataManager()
    ros_msg = MockRosTrajectory()
    
    uc_traj = dm.update_trajectory(ros_msg)
    
    assert isinstance(uc_traj, Trajectory)
    assert len(uc_traj.points) == 10
    
    cached = dm.get_latest_trajectory()
    assert cached is uc_traj


def test_data_manager_has_required_data():
    """Test required data check"""
    from controller_ros.io.data_manager import DataManager
    
    dm = DataManager()
    
    assert not dm.has_required_data()
    
    dm.update_odom(MockRosOdometry())
    assert not dm.has_required_data()
    
    dm.update_trajectory(MockRosTrajectory())
    assert dm.has_required_data()


def test_data_manager_get_all_latest():
    """Test get all latest data"""
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
    """Test data age calculation"""
    from controller_ros.io.data_manager import DataManager
    
    current_time = [0.0]
    def get_time():
        return current_time[0]
    
    dm = DataManager(get_time_func=get_time)
    
    ages = dm.get_data_ages()
    assert ages['odom'] == float('inf')
    assert ages['imu'] == float('inf')
    assert ages['trajectory'] == float('inf')
    
    dm.update_odom(MockRosOdometry())
    
    current_time[0] = 0.1
    
    ages = dm.get_data_ages()
    assert abs(ages['odom'] - 0.1) < 0.001
    assert ages['imu'] == float('inf')


def test_data_manager_is_data_fresh():
    """Test data freshness check"""
    from controller_ros.io.data_manager import DataManager
    
    current_time = [0.0]
    def get_time():
        return current_time[0]
    
    dm = DataManager(get_time_func=get_time)
    
    dm.update_odom(MockRosOdometry())
    dm.update_trajectory(MockRosTrajectory())
    
    assert dm.is_data_fresh({'odom': 0.1, 'trajectory': 0.2})
    
    current_time[0] = 0.15
    
    assert not dm.is_data_fresh({'odom': 0.1, 'trajectory': 0.2})
    assert dm.is_data_fresh({'odom': 0.2, 'trajectory': 0.2})


def test_data_manager_reset():
    """Test data reset"""
    from controller_ros.io.data_manager import DataManager
    
    dm = DataManager()
    dm.update_odom(MockRosOdometry())
    dm.update_imu(MockRosImu())
    dm.update_trajectory(MockRosTrajectory())
    
    assert dm.has_required_data()
    
    dm.reset()
    
    assert not dm.has_required_data()
    assert dm.get_latest_odom() is None
    assert dm.get_latest_imu() is None
    assert dm.get_latest_trajectory() is None


def test_data_manager_thread_safety():
    """Test thread safety"""
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
    """Test custom time function"""
    from controller_ros.io.data_manager import DataManager
    
    mock_time = [100.0]
    def custom_time():
        return mock_time[0]
    
    dm = DataManager(get_time_func=custom_time)
    dm.update_odom(MockRosOdometry())
    
    timestamps = dm.get_timestamps()
    assert timestamps['odom'] == 100.0
    
    mock_time[0] = 100.5
    ages = dm.get_data_ages()
    assert abs(ages['odom'] - 0.5) < 0.001


def test_data_manager_clock_rollback():
    """Test clock rollback scenario"""
    from controller_ros.io.data_manager import DataManager
    
    mock_time = [100.0]
    def custom_time():
        return mock_time[0]
    
    dm = DataManager(get_time_func=custom_time)
    dm.update_odom(MockRosOdometry())
    
    dm.get_data_ages()
    
    mock_time[0] = 50.0
    
    ages = dm.get_data_ages()
    
    assert ages['odom'] == float('inf')
    assert dm.did_clock_jump_back() == True
    assert dm.is_data_valid() == False
    
    dm.update_odom(MockRosOdometry())
    assert dm.is_data_valid() == True
    
    ages = dm.get_data_ages()
    assert ages['odom'] == 0.0


def test_data_manager_clock_jump_flag():
    """Test clock jump flag"""
    from controller_ros.io.data_manager import DataManager
    
    mock_time = [100.0]
    def custom_time():
        return mock_time[0]
    
    dm = DataManager(get_time_func=custom_time)
    
    assert dm.did_clock_jump_back() == False
    
    dm.update_odom(MockRosOdometry())
    dm.get_data_ages()
    
    mock_time[0] = 101.0
    dm.get_data_ages()
    assert dm.did_clock_jump_back() == False
    
    mock_time[0] = 50.0
    dm.get_data_ages()
    assert dm.did_clock_jump_back() == True
    
    dm.clear_clock_jump_flag()
    assert dm.did_clock_jump_back() == False


def test_data_manager_reset_resets_clock_state():
    """Test reset() resets clock state"""
    from controller_ros.io.data_manager import DataManager
    
    mock_time = [100.0]
    def custom_time():
        return mock_time[0]
    
    dm = DataManager(get_time_func=custom_time)
    dm.update_odom(MockRosOdometry())
    
    dm.get_data_ages()
    
    mock_time[0] = 50.0
    dm.get_data_ages()
    assert dm.did_clock_jump_back() == True
    
    dm.reset()
    assert dm.did_clock_jump_back() == False


def test_data_manager_separated_data_validity_tracking():
    """Test separated data validity tracking"""
    from controller_ros.io.data_manager import DataManager
    
    mock_time = [100.0]
    def custom_time():
        return mock_time[0]
    
    dm = DataManager(get_time_func=custom_time)
    
    dm.update_odom(MockRosOdometry())
    dm.update_imu(MockRosImu())
    dm.update_trajectory(MockRosTrajectory())
    
    dm.get_data_ages()
    
    assert dm.is_data_valid() == True
    assert dm.is_data_type_valid('odom') == True
    assert dm.is_data_type_valid('imu') == True
    assert dm.is_data_type_valid('trajectory') == True
    
    mock_time[0] = 50.0
    dm.get_data_ages()
    
    assert dm.is_data_valid() == False
    assert dm.is_data_type_valid('odom') == False
    assert dm.is_data_type_valid('imu') == False
    assert dm.is_data_type_valid('trajectory') == False
    
    dm.update_odom(MockRosOdometry())
    
    assert dm.is_data_valid() == False
    assert dm.is_data_type_valid('odom') == True
    assert dm.is_data_type_valid('imu') == False
    assert dm.is_data_type_valid('trajectory') == False
    
    dm.update_imu(MockRosImu())
    
    assert dm.is_data_valid() == False
    assert dm.is_data_type_valid('odom') == True
    assert dm.is_data_type_valid('imu') == True
    assert dm.is_data_type_valid('trajectory') == False
    
    dm.update_trajectory(MockRosTrajectory())
    
    assert dm.is_data_valid() == True
    assert dm.is_data_type_valid('odom') == True
    assert dm.is_data_type_valid('imu') == True
    assert dm.is_data_type_valid('trajectory') == True


def test_data_manager_data_ages_with_separated_validity():
    """Test data ages with separated validity"""
    from controller_ros.io.data_manager import DataManager
    
    mock_time = [100.0]
    def custom_time():
        return mock_time[0]
    
    dm = DataManager(get_time_func=custom_time)
    
    dm.update_odom(MockRosOdometry())
    dm.update_imu(MockRosImu())
    dm.update_trajectory(MockRosTrajectory())
    
    dm.get_data_ages()
    
    mock_time[0] = 50.0
    ages = dm.get_data_ages()
    
    assert ages['odom'] == float('inf')
    assert ages['imu'] == float('inf')
    assert ages['trajectory'] == float('inf')
    
    dm.update_odom(MockRosOdometry())
    
    ages = dm.get_data_ages()
    
    assert ages['odom'] == 0.0
    assert ages['imu'] == float('inf')
    assert ages['trajectory'] == float('inf')
    
    mock_time[0] = 50.1
    ages = dm.get_data_ages()
    
    assert abs(ages['odom'] - 0.1) < 0.001
    assert ages['imu'] == float('inf')
    assert ages['trajectory'] == float('inf')


def test_data_manager_clock_jump_only_invalidates_existing_data():
    """Test clock jump only invalidates existing data"""
    from controller_ros.io.data_manager import DataManager
    
    mock_time = [100.0]
    def custom_time():
        return mock_time[0]
    
    dm = DataManager(get_time_func=custom_time)
    
    dm.update_odom(MockRosOdometry())
    
    dm.get_data_ages()
    
    mock_time[0] = 50.0
    dm.get_data_ages()
    
    assert dm.is_data_type_valid('odom') == False
    assert dm.is_data_type_valid('imu') == True
    assert dm.is_data_type_valid('trajectory') == True
    
    ages = dm.get_data_ages()
    assert ages['odom'] == float('inf')
    assert ages['imu'] == float('inf')
    assert ages['trajectory'] == float('inf')


def test_data_manager_clock_jump_callback_timing():
    """Test clock jump callback timing"""
    from controller_ros.io.data_manager import DataManager
    
    mock_time = [100.0]
    def custom_time():
        return mock_time[0]
    
    callback_called = [False]
    callback_event = [None]
    
    def callback(event):
        callback_called[0] = True
        callback_event[0] = event
    
    dm = DataManager(get_time_func=custom_time, on_clock_jump=callback)
    dm.update_odom(MockRosOdometry())
    dm.get_data_ages()
    
    mock_time[0] = 50.0
    dm.get_data_ages()
    
    assert callback_called[0] == True
    assert callback_event[0] is not None
    assert callback_event[0].is_backward == True
    assert callback_event[0].jump_delta == -50.0


def test_data_manager_health_details_with_separated_validity():
    """Test health details with separated validity"""
    from controller_ros.io.data_manager import DataManager
    
    mock_time = [100.0]
    def custom_time():
        return mock_time[0]
    
    dm = DataManager(get_time_func=custom_time)
    dm.update_odom(MockRosOdometry())
    dm.update_imu(MockRosImu())
    dm.update_trajectory(MockRosTrajectory())
    dm.get_data_ages()
    
    mock_time[0] = 50.0
    dm.get_data_ages()
    
    dm.update_odom(MockRosOdometry())
    
    health = dm._get_health_details()
    
    assert 'data_invalidated' in health
    assert health['data_invalidated']['odom'] == False
    assert health['data_invalidated']['imu'] == True
    assert health['data_invalidated']['trajectory'] == True
    
    assert health['data_valid'] == False


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
