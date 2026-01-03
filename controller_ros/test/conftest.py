"""
pytest 配置和共享 fixtures

提供所有测试共享的 Mock 类和 fixtures，避免代码重复。

Mock 类现在统一定义在 fixtures/ 目录中：
- fixtures/ros_message_mocks.py: ROS 消息类型 Mock
- fixtures/bridge_mocks.py: 桥接层 Mock
"""
import pytest
import sys
import os

# 使用统一的路径管理器


# 添加 fixtures 目录
_fixtures_dir = os.path.join(_test_dir, 'fixtures')
if _test_dir not in sys.path:
    sys.path.insert(0, _test_dir)

# =============================================================================
# 从 fixtures 模块导入所有 Mock 类
# =============================================================================

from fixtures import (
    # ROS 基础类型
    MockRosTime,
    MockRosHeader,
    MockRosPoint,
    MockRosQuaternion,
    MockRosVector3,
    MockRosPose,
    MockRosTwist,
    MockRosPoseWithCovariance,
    MockRosTwistWithCovariance,
    # ROS 消息类型
    MockRosOdometry,
    MockRosImu,
    MockRosTrajectory,
    # 工厂函数
    create_mock_odom,
    create_mock_imu,
    create_mock_trajectory,
    # 桥接层 Mock
    MockTFBridge,
    MockControllerBridge,
)


# =============================================================================
# pytest fixtures
# =============================================================================

@pytest.fixture
def mock_odom():
    """创建默认的 Mock Odometry"""
    return MockRosOdometry(x=1.0, y=2.0, vx=0.5, omega_z=0.1)


@pytest.fixture
def mock_imu():
    """创建默认的 Mock IMU"""
    return MockRosImu()


@pytest.fixture
def mock_trajectory():
    """创建默认的 Mock Trajectory"""
    return MockRosTrajectory()


@pytest.fixture
def mock_tf_bridge():
    """创建默认的 Mock TF Bridge"""
    return MockTFBridge()


@pytest.fixture
def mock_time_func():
    """创建可控的时间函数"""
    current_time = [0.0]
    
    def get_time():
        return current_time[0]
    
    def set_time(t):
        current_time[0] = t
    
    get_time.set = set_time
    return get_time


# =============================================================================
# 导出所有 Mock 类供直接导入
# =============================================================================

__all__ = [
    # ROS 基础类型
    'MockRosTime',
    'MockRosHeader',
    'MockRosPoint',
    'MockRosQuaternion',
    'MockRosVector3',
    'MockRosPose',
    'MockRosTwist',
    'MockRosPoseWithCovariance',
    'MockRosTwistWithCovariance',
    # ROS 消息类型
    'MockRosOdometry',
    'MockRosImu',
    'MockRosTrajectory',
    # 工厂函数
    'create_mock_odom',
    'create_mock_imu',
    'create_mock_trajectory',
    # 桥接层 Mock
    'MockTFBridge',
    'MockControllerBridge',
]
