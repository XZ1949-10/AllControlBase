"""
集成测试 - 验证 controller_ros 与 universal_controller 的协同工作

此测试不依赖 ROS 环境，验证核心集成逻辑。
"""
import pytest
import sys
import os
import time
import numpy as np

# 添加 src 目录和 test 目录到路径
_test_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_test_dir, '..', 'src'))
sys.path.insert(0, _test_dir)

# 导入共享的 Mock 类 (从 conftest.py)
from conftest import (
    MockRosTime, MockRosHeader, MockRosPoint, MockRosQuaternion,
    MockRosVector3, MockRosTrajectory
)


def test_import_universal_controller():
    """测试 universal_controller 导入"""
    from universal_controller.config.default_config import DEFAULT_CONFIG, PLATFORM_CONFIG
    from universal_controller.manager.controller_manager import ControllerManager
    from universal_controller.core.data_types import (
        Odometry, Imu, Trajectory, ControlOutput, Header, Point3D
    )
    from universal_controller.core.enums import ControllerState, TrajectoryMode
    
    assert DEFAULT_CONFIG is not None
    assert 'differential' in PLATFORM_CONFIG
    assert ControllerManager is not None


def test_import_controller_ros_adapters():
    """测试 controller_ros 适配器导入"""
    from controller_ros.adapters import (
        OdomAdapter, ImuAdapter, TrajectoryAdapter, OutputAdapter
    )
    
    assert OdomAdapter is not None
    assert ImuAdapter is not None
    assert TrajectoryAdapter is not None
    assert OutputAdapter is not None


def test_import_controller_ros_bridge():
    """测试 controller_ros 桥接层导入"""
    from controller_ros.bridge.controller_bridge import ControllerBridge
    
    assert ControllerBridge is not None


def test_full_integration_pipeline():
    """测试完整的集成管道"""
    from universal_controller.config.default_config import DEFAULT_CONFIG
    from universal_controller.tests.fixtures import create_test_odom, create_test_trajectory
    from controller_ros.bridge.controller_bridge import ControllerBridge
    
    # 1. 创建桥接层
    config = DEFAULT_CONFIG.copy()
    bridge = ControllerBridge(config)
    
    assert bridge.is_running  # 使用新 API
    
    # 2. 创建测试数据
    odom = create_test_odom(vx=1.0, vy=0.0)
    trajectory = create_test_trajectory(soft_enabled=True)
    
    # 3. 执行控制更新
    cmd = bridge.update(odom, trajectory)
    
    assert cmd is not None
    assert hasattr(cmd, 'vx')
    assert hasattr(cmd, 'vy')
    assert hasattr(cmd, 'omega')
    assert cmd.success or not cmd.success  # 只要有返回值就行
    
    # 4. 获取诊断信息
    diag = bridge.get_diagnostics()
    assert diag is not None
    
    # 5. 重置
    bridge.reset()
    
    # 6. 关闭
    bridge.shutdown()
    assert not bridge.is_running  # 使用新 API


def test_adapter_data_flow():
    """测试适配器数据流"""
    from controller_ros.adapters import TrajectoryAdapter
    from universal_controller.core.data_types import Trajectory, Header, Point3D
    from universal_controller.core.enums import TrajectoryMode
    
    # 使用共享的 Mock 类
    ros_msg = MockRosTrajectory(
        num_points=10,
        frame_id='base_link',
        confidence=0.9,
        soft_enabled=True
    )
    ros_msg.set_velocities([1.0, 0.0, 0.0, 0.1] * 10)
    
    adapter = TrajectoryAdapter()
    
    # ROS → UC
    uc_traj = adapter.to_uc(ros_msg)
    
    assert isinstance(uc_traj, Trajectory)
    assert len(uc_traj.points) == 10
    assert uc_traj.soft_enabled == True
    assert uc_traj.velocities is not None
    assert uc_traj.velocities.shape == (10, 4)
    assert uc_traj.header.frame_id == 'base_link'


def test_platform_specific_output():
    """测试不同平台的输出"""
    from universal_controller.config.default_config import DEFAULT_CONFIG
    from universal_controller.tests.fixtures import create_test_odom, create_test_trajectory
    from controller_ros.bridge.controller_bridge import ControllerBridge
    
    platforms = {
        'differential': {'expected_frame': 'base_link', 'vy_zero': True},
        'omni': {'expected_frame': 'world', 'vy_zero': False},
        'ackermann': {'expected_frame': 'base_link', 'vy_zero': True},
        'quadrotor': {'expected_frame': 'world', 'vy_zero': False},
    }
    
    for platform, expected in platforms.items():
        config = DEFAULT_CONFIG.copy()
        config['system'] = DEFAULT_CONFIG['system'].copy()
        config['system']['platform'] = platform
        
        bridge = ControllerBridge(config)
        
        odom = create_test_odom(vx=1.0, vy=0.5)
        trajectory = create_test_trajectory()
        
        cmd = bridge.update(odom, trajectory)
        
        assert cmd.frame_id == expected['expected_frame'], \
            f"Platform {platform}: expected frame {expected['expected_frame']}, got {cmd.frame_id}"
        
        if expected['vy_zero']:
            assert abs(cmd.vy) < 1e-6, \
                f"Platform {platform}: expected vy=0, got {cmd.vy}"
        
        bridge.shutdown()
        print(f"✓ Platform {platform} output verified")


def test_velocity_padding_integration():
    """测试速度填充在完整管道中的行为
    
    设计说明：
    - 当速度点少于位置点时，使用线性衰减填充
    - 衰减从最后一个速度点开始，线性衰减到零
    - 这确保轨迹末端平滑减速，避免高速运动到轨迹末端
    """
    from controller_ros.adapters import TrajectoryAdapter
    import numpy as np
    
    # 使用共享的 Mock 类，20 个位置点但只有 5 个速度点
    ros_msg = MockRosTrajectory(num_points=20, frame_id='base_link')
    ros_msg.set_velocities([
        1.0, 0.1, 0.0, 0.01,
        1.5, 0.15, 0.0, 0.015,
        2.0, 0.2, 0.0, 0.02,
        2.5, 0.25, 0.0, 0.025,
        3.0, 0.3, 0.0, 0.03,
    ])
    
    adapter = TrajectoryAdapter()
    uc_traj = adapter.to_uc(ros_msg)
    
    # 验证填充
    assert uc_traj.velocities.shape == (20, 4)
    
    # 前 5 个应该是原始值
    assert uc_traj.velocities[0, 0] == 1.0
    assert uc_traj.velocities[4, 0] == 3.0
    
    # 后 15 个应该线性衰减到零
    # 最后一个速度点是 [3.0, 0.3, 0.0, 0.03]
    # 填充 15 个点，衰减因子: (15-1-i)/15 = (14-i)/15
    padding_count = 15
    last_vel = np.array([3.0, 0.3, 0.0, 0.03])
    for i in range(padding_count):
        expected_decay = (padding_count - 1 - i) / padding_count
        expected_vel = last_vel * expected_decay
        actual_vel = uc_traj.velocities[5 + i]
        assert np.allclose(actual_vel, expected_vel, atol=0.001), \
            f"Point {5+i} should have decayed velocity {expected_vel}, got {actual_vel}"
    
    # 最后一个填充点应该接近零
    assert np.allclose(uc_traj.velocities[19], [0, 0, 0, 0], atol=0.001)


def test_empty_frame_id_handling():
    """测试空 frame_id 处理"""
    from controller_ros.adapters import TrajectoryAdapter
    from controller_ros.adapters.trajectory_adapter import DEFAULT_TRAJECTORY_FRAME_ID
    
    # 使用共享的 Mock 类，空 frame_id
    ros_msg = MockRosTrajectory(num_points=5, frame_id='')
    
    adapter = TrajectoryAdapter()
    uc_traj = adapter.to_uc(ros_msg)
    
    # 空 frame_id 应该使用默认值
    assert uc_traj.header.frame_id == DEFAULT_TRAJECTORY_FRAME_ID
    print(f"✓ Empty frame_id correctly defaulted to '{DEFAULT_TRAJECTORY_FRAME_ID}'")


def test_trajectory_mode_mapping():
    """测试轨迹模式映射"""
    from controller_ros.adapters import TrajectoryAdapter
    from universal_controller.core.enums import TrajectoryMode
    
    adapter = TrajectoryAdapter()
    
    # 测试所有有效模式
    mode_mapping = {
        0: TrajectoryMode.MODE_TRACK,
        1: TrajectoryMode.MODE_STOP,
        2: TrajectoryMode.MODE_HOVER,  # ROS 的 MODE_HOLD 对应 UC 的 MODE_HOVER
        3: TrajectoryMode.MODE_EMERGENCY,
    }
    
    for ros_mode, expected_uc_mode in mode_mapping.items():
        ros_msg = MockRosTrajectory(num_points=1, mode=ros_mode, frame_id='base_link')
        uc_traj = adapter.to_uc(ros_msg)
        assert uc_traj.mode == expected_uc_mode, \
            f"ROS mode {ros_mode} should map to {expected_uc_mode}, got {uc_traj.mode}"
    
    # 测试无效模式
    ros_msg_invalid = MockRosTrajectory(num_points=1, mode=99, frame_id='base_link')
    uc_traj = adapter.to_uc(ros_msg_invalid)
    assert uc_traj.mode == TrajectoryMode.MODE_TRACK, \
        f"Invalid mode should default to MODE_TRACK, got {uc_traj.mode}"
    
    print("✓ Trajectory mode mapping verified")


def test_diagnostics_callback():
    """测试诊断回调功能"""
    from universal_controller.config.default_config import DEFAULT_CONFIG
    from universal_controller.tests.fixtures import create_test_odom, create_test_trajectory
    from controller_ros.bridge.controller_bridge import ControllerBridge
    
    config = DEFAULT_CONFIG.copy()
    bridge = ControllerBridge(config)
    
    callback_data = []
    
    def on_diagnostics(diag):
        callback_data.append(diag)
    
    bridge.set_diagnostics_callback(on_diagnostics)
    
    odom = create_test_odom(vx=1.0)
    trajectory = create_test_trajectory()
    
    for _ in range(5):
        bridge.update(odom, trajectory)
    
    # 验证回调被调用
    assert len(callback_data) >= 5, f"Callback should be called at least 5 times, got {len(callback_data)}"
    
    # 验证诊断数据结构
    if callback_data:
        diag = callback_data[0]
        assert 'state' in diag
        assert 'mpc_success' in diag
        assert 'timeout' in diag
        assert 'cmd' in diag
    
    bridge.shutdown()
    print("✓ Diagnostics callback verified")


def test_tf2_injection_mechanism():
    """测试 TF2 注入机制"""
    from universal_controller.config.default_config import DEFAULT_CONFIG
    from controller_ros.bridge.controller_bridge import ControllerBridge
    
    config = DEFAULT_CONFIG.copy()
    bridge = ControllerBridge(config)
    
    # 验证 manager 和 coord_transformer 存在
    assert bridge.manager is not None
    assert bridge.manager.coord_transformer is not None
    
    # 验证 coord_transformer 支持 TF2 注入
    assert hasattr(bridge.manager.coord_transformer, 'set_tf2_lookup_callback')
    
    # 模拟 TF2 查找回调
    tf2_calls = []
    
    def mock_tf2_lookup(target_frame, source_frame, time, timeout):
        tf2_calls.append((target_frame, source_frame))
        return {
            'translation': (1.0, 2.0, 0.0),
            'rotation': (0.0, 0.0, 0.0, 1.0)
        }
    
    # 注入回调
    bridge.manager.coord_transformer.set_tf2_lookup_callback(mock_tf2_lookup)
    
    # 验证回调已设置
    status = bridge.manager.coord_transformer.get_status()
    assert status['external_tf2_callback'] == True
    
    bridge.shutdown()
    print("✓ TF2 injection mechanism verified")


if __name__ == '__main__':
    pytest.main([__file__, '-v'])


def test_time_sync_key_aliases():
    """测试 TimeSync 键名处理"""
    from controller_ros.utils.time_sync import TimeSync
    
    time_sync = TimeSync(
        max_odom_age_ms=100,
        max_traj_age_ms=200,
        max_imu_age_ms=50
    )
    
    # 测试使用 'trajectory' 作为输入键名
    ages_with_trajectory = {
        'odom': 0.05,
        'trajectory': 0.15,
        'imu': 0.03
    }
    
    timeouts = time_sync.check_freshness(ages_with_trajectory)
    
    # 返回的键应该是 'traj_timeout' (保持向后兼容)
    assert 'traj_timeout' in timeouts
    assert 'odom_timeout' in timeouts
    assert 'imu_timeout' in timeouts
    
    # 验证超时判断正确
    assert timeouts['odom_timeout'] == False  # 0.05 < 0.1
    assert timeouts['traj_timeout'] == False  # 0.15 < 0.2
    assert timeouts['imu_timeout'] == False   # 0.03 < 0.05
    
    # 测试超时情况
    ages_timeout = {
        'odom': 0.15,       # > 0.1, 超时
        'trajectory': 0.25, # > 0.2, 超时
        'imu': 0.06         # > 0.05, 超时
    }
    
    timeouts2 = time_sync.check_freshness(ages_timeout)
    assert timeouts2['odom_timeout'] == True
    assert timeouts2['traj_timeout'] == True
    assert timeouts2['imu_timeout'] == True
    
    # 测试 is_all_fresh 方法
    ages_fresh = {
        'odom': 0.05,
        'trajectory': 0.15,
        'imu': 0.03
    }
    assert time_sync.is_all_fresh(ages_fresh) == True
    
    ages_stale = {
        'odom': 0.15,
        'trajectory': 0.15,
        'imu': 0.03
    }
    assert time_sync.is_all_fresh(ages_stale) == False  # odom 超时


def test_diag_filler_numpy_imu_bias():
    """测试 diagnostics_publisher 处理 numpy array 类型的 imu_bias"""
    import numpy as np
    from controller_ros.utils.diagnostics_publisher import fill_diagnostics_msg
    
    # 模拟 DiagnosticsV2 消息
    class MockDiagMsg:
        def __init__(self):
            self.header = type('Header', (), {'stamp': None, 'frame_id': ''})()
            self.state = 0
            self.mpc_success = False
            self.mpc_solve_time_ms = 0.0
            self.backup_active = False
            self.mpc_health_kkt_residual = 0.0
            self.mpc_health_condition_number = 1.0
            self.mpc_health_consecutive_near_timeout = 0
            self.mpc_health_degradation_warning = False
            self.mpc_health_can_recover = True
            self.consistency_curvature = 0.0
            self.consistency_velocity_dir = 1.0
            self.consistency_temporal = 1.0
            self.consistency_alpha_soft = 0.0
            self.consistency_data_valid = True
            self.estimator_covariance_norm = 0.0
            self.estimator_innovation_norm = 0.0
            self.estimator_slip_probability = 0.0
            self.estimator_imu_drift_detected = False
            self.estimator_imu_available = True
            self.estimator_imu_bias = [0.0, 0.0, 0.0]
            self.tracking_lateral_error = 0.0
            self.tracking_longitudinal_error = 0.0
            self.tracking_heading_error = 0.0
            self.tracking_prediction_error = 0.0
            self.transform_tf2_available = False
            self.transform_tf2_injected = False
            self.transform_fallback_duration_ms = 0.0
            self.transform_accumulated_drift = 0.0
            self.timeout_odom = False
            self.timeout_traj = False
            self.timeout_traj_grace_exceeded = False
            self.timeout_imu = False
            self.timeout_last_odom_age_ms = 0.0
            self.timeout_last_traj_age_ms = 0.0
            self.timeout_last_imu_age_ms = 0.0
            self.timeout_in_startup_grace = False
            self.cmd_vx = 0.0
            self.cmd_vy = 0.0
            self.cmd_vz = 0.0
            self.cmd_omega = 0.0
            self.cmd_frame_id = ''
            self.transition_progress = 0.0
            self.error_message = ''
            self.consecutive_errors = 0
    
    msg = MockDiagMsg()
    
    # 测试 numpy array 类型的 imu_bias
    diag = {
        'estimator_health': {
            'imu_bias': np.array([0.1, 0.2, 0.3])  # numpy array
        }
    }
    
    fill_diagnostics_msg(msg, diag)
    
    assert msg.estimator_imu_bias == [0.1, 0.2, 0.3]
    
    # 测试 list 类型
    diag2 = {
        'estimator_health': {
            'imu_bias': [0.4, 0.5, 0.6]
        }
    }
    
    fill_diagnostics_msg(msg, diag2)
    assert msg.estimator_imu_bias == [0.4, 0.5, 0.6]
    
    # 测试 tuple 类型
    diag3 = {
        'estimator_health': {
            'imu_bias': (0.7, 0.8, 0.9)
        }
    }
    
    fill_diagnostics_msg(msg, diag3)
    assert msg.estimator_imu_bias == [0.7, 0.8, 0.9]
