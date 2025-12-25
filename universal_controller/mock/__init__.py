"""
模拟数据模块 (已弃用)

⚠️ 此模块已弃用，将在 v4.0 版本中移除。

迁移指南
========

ROS 兼容层 (独立运行模式):
--------------------------
用于非 ROS 环境下的独立运行，这是生产代码的一部分。

    from universal_controller.compat import (
        StandaloneRospy,      # rospy 替代
        StandaloneTF2Buffer,  # tf2_ros.Buffer 替代
    )

数据类型:
---------
所有数据类型已移至 core.data_types:

    from universal_controller.core.data_types import (
        Pose, Twist, PoseWithCovariance, TwistWithCovariance,
        Vector3, Quaternion, Transform, TransformStamped,
        Odometry, Imu, Header, Point3D,
    )

测试数据生成 (仅用于测试):
--------------------------
用于单元测试和集成测试，不应在生产代码中使用。

    from universal_controller.tests.fixtures import (
        create_test_trajectory,
        create_test_odom,
        generate_mock_diagnostics,
    )
"""
import warnings

# 发出弃用警告
warnings.warn(
    "universal_controller.mock 模块已弃用，将在 v4.0 版本中移除。"
    "请使用 universal_controller.compat (ROS 兼容层) 和 "
    "universal_controller.core.data_types (数据类型)。"
    "测试数据生成器请从 universal_controller.tests.fixtures 导入。",
    DeprecationWarning,
    stacklevel=2
)

# =============================================================================
# 向后兼容导出 - 仅保留最小必要的别名
# =============================================================================

# ROS 兼容层 (从 compat 重新导出)
from ..compat.ros_compat_impl import (
    # 推荐名称
    StandaloneRospy,
    StandaloneTF2Buffer,
    StandaloneTF2Ros,
    StandaloneTFTransformations,
    # 向后兼容别名
    MockRospy,
    MockTF2BufferCore,
    MockTF2Ros,
    MockTFTransformations,
    # 异常类型
    LookupException,
    ExtrapolationException,
    ConnectivityException,
)

# 数据类型别名 (从 core.data_types 重新导出，带弃用标记)
from ..core.data_types import (
    Vector3, Quaternion, Transform, Header, TransformStamped,
    Point3D, Odometry, Imu, Pose, Twist,
    PoseWithCovariance, TwistWithCovariance,
)

# 向后兼容的 Mock* 数据类型别名
MockVector3 = Vector3
MockQuaternion = Quaternion
MockTransform = Transform
MockHeader = Header
MockTransformStamped = TransformStamped
MockPoint = Point3D
MockOdometry = Odometry
MockImu = Imu
MockPose = Pose
MockTwist = Twist
MockPoseWithCovariance = PoseWithCovariance
MockTwistWithCovariance = TwistWithCovariance

__all__ = [
    # ROS 兼容实现 (推荐)
    'StandaloneRospy', 'StandaloneTF2Buffer', 
    'StandaloneTF2Ros', 'StandaloneTFTransformations',
    # ROS 兼容实现 (向后兼容别名)
    'MockRospy', 'MockTF2BufferCore', 'MockTF2Ros', 'MockTFTransformations',
    # 异常类型
    'LookupException', 'ExtrapolationException', 'ConnectivityException',
    # 数据类型 (推荐从 core.data_types 导入)
    'Vector3', 'Quaternion', 'Transform', 'Header', 'TransformStamped',
    'Point3D', 'Odometry', 'Imu', 'Pose', 'Twist',
    'PoseWithCovariance', 'TwistWithCovariance',
    # 数据类型别名 (已弃用)
    'MockVector3', 'MockQuaternion', 'MockTransform', 'MockHeader',
    'MockTransformStamped', 'MockPoint', 'MockOdometry', 'MockImu',
    'MockPose', 'MockTwist', 'MockPoseWithCovariance', 'MockTwistWithCovariance',
]
