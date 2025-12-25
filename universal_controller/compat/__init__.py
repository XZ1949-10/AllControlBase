"""
ROS 兼容层模块 - 独立运行模式支持

重要说明
========
此模块是**生产代码**的一部分，不是测试 mock。

它提供了 ROS 模块的替代实现，使 universal_controller 可以在非 ROS 环境下独立运行。

使用场景
--------
- 在没有 ROS 的环境中运行控制器算法
- 单元测试中模拟 ROS 环境
- 开发和调试时不依赖 ROS

自动切换机制
============
universal_controller 会自动检测 ROS 环境:

1. ROS 环境可用时:
   - 使用真实的 rospy, tf2_ros 等模块
   - 此模块不会被使用

2. ROS 环境不可用时:
   - 自动使用此模块提供的替代实现
   - 提供完整的功能支持

您通常不需要直接导入此模块，除非:
- 您需要在测试中强制使用独立模式
- 您正在开发需要 ROS 兼容性的代码

包含内容
========
- StandaloneRospy: rospy 模块的替代实现
- StandaloneTF2Buffer: tf2_ros.Buffer 的替代实现
- StandaloneTF2Ros: tf2_ros 模块的替代实现
- StandaloneTFTransformations: tf.transformations 的替代实现

命名约定
========
- "Standalone" 前缀表示独立运行模式的实现
- "Mock" 前缀别名已弃用，保留仅为向后兼容

数据类型
========
数据类型应从 universal_controller.core.data_types 导入:

    from universal_controller.core.data_types import (
        Vector3, Quaternion, Transform, TransformStamped,
        Header, Point3D, Odometry, Imu,
        Pose, Twist, PoseWithCovariance, TwistWithCovariance,
    )

测试工具
========
- 模拟诊断数据生成器位于 tests/fixtures/mock_diagnostics.py
- 测试数据生成器位于 tests/fixtures/test_data_generator.py
- 这些测试工具不应在生产代码中使用
"""

from .ros_compat_impl import (
    # 推荐使用的名称
    StandaloneRospy,
    StandaloneTF2Buffer,
    StandaloneTF2Ros,
    StandaloneTFTransformations,
    # 异常类型
    LookupException,
    ExtrapolationException,
    ConnectivityException,
    # 向后兼容别名 (已弃用，不在 __all__ 中导出)
    MockRospy,
    MockTF2BufferCore,
    MockTF2Ros,
    MockTFTransformations,
)

__all__ = [
    # 推荐使用的名称
    'StandaloneRospy',
    'StandaloneTF2Buffer',
    'StandaloneTF2Ros',
    'StandaloneTFTransformations',
    # 异常类型
    'LookupException',
    'ExtrapolationException',
    'ConnectivityException',
]
