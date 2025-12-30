"""
工具层 - 通用工具函数

导出说明:
- get_time_sec: 推荐使用的时间获取函数
- get_current_time: 已废弃，请使用 get_time_sec
- TF2Compat: TF2 兼容层，通常通过 TFBridge 使用
- TOPICS_DEFAULTS: 话题默认配置常量
- TF_DEFAULTS: TF 默认配置常量

注意: TimeSync 已废弃，超时检测统一由 universal_controller 的 TimeoutMonitor 处理
"""
from .param_loader import ParamLoader, TOPICS_DEFAULTS, TF_DEFAULTS
from .visualizer_param_loader import VisualizerParamLoader
from .diagnostics_publisher import (
    fill_diagnostics_msg,
    DiagnosticsThrottler,
    safe_float,
    safe_float_list,
)
from .ros_compat import (
    ROS_VERSION, ROS_AVAILABLE, TF2_AVAILABLE,
    get_time_sec, get_current_time, get_monotonic_time,
    ros_time_to_sec, sec_to_ros_time,
    log_info, log_warn, log_error, log_warn_throttle,
    TF2Compat
)
from .tf2_injection_manager import TF2InjectionManager

__all__ = [
    # 参数加载
    'ParamLoader',
    'VisualizerParamLoader',
    # 常量
    'TOPICS_DEFAULTS',
    'TF_DEFAULTS',
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
    'get_current_time',  # 已废弃，保留向后兼容
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
]
