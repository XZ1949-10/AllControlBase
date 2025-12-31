"""
工具层 - 通用工具函数

导出说明:
- get_time_sec: 推荐使用的时间获取函数
- TF2Compat: TF2 兼容层，通常通过 TFBridge 使用
- TOPICS_DEFAULTS: 话题默认配置常量
- TRANSFORM_ROS_DEFAULTS: Transform ROS 扩展配置常量
- msg_availability: 消息可用性检查模块
"""
from .param_loader import ParamLoader, TOPICS_DEFAULTS, TRANSFORM_ROS_DEFAULTS
from .visualizer_param_loader import VisualizerParamLoader
from .param_utils import (
    IParamStrategy, ROS1Strategy, ROS2Strategy, DefaultStrategy,
    get_strategy, load_params_recursive, convert_param_type
)
from .diagnostics_publisher import (
    fill_diagnostics_msg,
    DiagnosticsThrottler,
    safe_float,
    safe_float_list,
)
from .ros_compat import (
    ROS_VERSION, ROS_AVAILABLE, TF2_AVAILABLE,
    get_time_sec, get_monotonic_time,
    ros_time_to_sec, sec_to_ros_time,
    log_info, log_warn, log_error, log_warn_throttle,
    TF2Compat
)
from .tf2_injection_manager import TF2InjectionManager
from .msg_availability import (
    CUSTOM_MSGS_AVAILABLE,
    get_msg_import_error,
    check_msgs_available,
    LocalTrajectoryV4,
    UnifiedCmd,
    DiagnosticsV2,
    AttitudeCmd,
)

__all__ = [
    # 参数加载
    'ParamLoader',
    'VisualizerParamLoader',
    # 参数工具 (供扩展使用)
    'IParamStrategy',
    'ROS1Strategy',
    'ROS2Strategy', 
    'DefaultStrategy',
    'get_strategy',
    'load_params_recursive',
    'convert_param_type',
    # 常量
    'TOPICS_DEFAULTS',
    'TRANSFORM_ROS_DEFAULTS',
    # 诊断工具
    'fill_diagnostics_msg',
    'DiagnosticsThrottler',
    'safe_float',
    'safe_float_list',
    # ROS 兼容层
    'ROS_VERSION',
    'ROS_AVAILABLE', 
    'TF2_AVAILABLE',
    'get_time_sec',
    'get_monotonic_time',
    'ros_time_to_sec',
    'sec_to_ros_time',
    'log_info',
    'log_warn',
    'log_error',
    'log_warn_throttle',
    'TF2Compat',
    # TF2 注入管理
    'TF2InjectionManager',
    # 消息可用性
    'CUSTOM_MSGS_AVAILABLE',
    'get_msg_import_error',
    'check_msgs_available',
    'LocalTrajectoryV4',
    'UnifiedCmd',
    'DiagnosticsV2',
    'AttitudeCmd',
]
