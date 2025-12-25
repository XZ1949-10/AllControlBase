"""
通用控制器 (Universal Controller)

版本: v3.18.0
日期: 2024-12-22

基于 MPC 的通用轨迹跟踪控制器，支持多平台部署。

特性:
- 跨平台部署: 统一状态空间支持地面车辆和无人机
- Hard/Soft 融合: 多维一致性门控，硬主干保证可用性，软建议提升平滑性
- 安全可靠: 热备控制器 + 渐进式降级，异常时安全停车
- 高实时性: ACADOS 求解器，15ms 内完成优化
- 插件化架构: 核心模块可替换，便于扩展和维护
- ROS 兼容: 支持 ROS 环境和独立运行模式，自动检测切换
- 强类型诊断: DiagnosticsInput 数据类提供类型安全
- 职责分离: DiagnosticsPublisher 独立处理诊断发布

支持平台:
- 阿克曼转向车辆 (Ackermann)
- 差速驱动车辆 (Differential)
- 全向移动车辆 (Omni)
- 四旋翼无人机 (Quadrotor)

使用示例:
    from universal_controller import ControllerManager, DEFAULT_CONFIG
    
    config = DEFAULT_CONFIG.copy()
    manager = ControllerManager(config)
    manager.initialize_default_components()
    
    cmd = manager.update(odom, trajectory)
"""

__version__ = "3.18.1"
__author__ = "Universal Controller Team"

# 导出主要类和配置
from .manager.controller_manager import ControllerManager
from .config.default_config import DEFAULT_CONFIG, PLATFORM_CONFIG, get_config_value
from .core.enums import ControllerState, PlatformType, HeadingMode, TransformStatus, TrajectoryMode
from .core.data_types import (
    Trajectory, ControlOutput, ConsistencyResult, EstimatorOutput,
    Point3D, Header, Odometry, Imu, DiagnosticsV2, TimeoutStatus,
    SafetyDecision, MPCHealthStatus,
    # TF2 相关数据类型
    Vector3, Quaternion, Transform, TransformStamped,
    # ROS geometry_msgs 兼容类型
    Pose, Twist, PoseWithCovariance, TwistWithCovariance,
)
from .core.diagnostics_input import DiagnosticsInput
from .core.interfaces import (
    IStateEstimator, ITrajectoryTracker, IConsistencyChecker,
    ISafetyMonitor, ISmoothTransition, ICoordinateTransformer
)

__all__ = [
    # 版本
    '__version__',
    # 管理器
    'ControllerManager',
    # 配置
    'DEFAULT_CONFIG', 'PLATFORM_CONFIG', 'get_config_value',
    # 枚举
    'ControllerState', 'PlatformType', 'HeadingMode', 'TransformStatus', 'TrajectoryMode',
    # 数据类型
    'Trajectory', 'ControlOutput', 'ConsistencyResult', 'EstimatorOutput',
    'Point3D', 'Header', 'Odometry', 'Imu', 'DiagnosticsV2', 'TimeoutStatus',
    'SafetyDecision', 'MPCHealthStatus', 'DiagnosticsInput',
    # TF2 相关数据类型
    'Vector3', 'Quaternion', 'Transform', 'TransformStamped',
    # ROS geometry_msgs 兼容类型
    'Pose', 'Twist', 'PoseWithCovariance', 'TwistWithCovariance',
    # 接口
    'IStateEstimator', 'ITrajectoryTracker', 'IConsistencyChecker',
    'ISafetyMonitor', 'ISmoothTransition', 'ICoordinateTransformer',
]
