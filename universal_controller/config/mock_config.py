"""模拟数据配置

控制是否允许在各模块中使用模拟数据。

默认情况下，所有模块都不允许使用模拟数据。
只有在配置中明确启用时，才会使用模拟数据。

使用场景:
- 开发调试时可以启用模拟数据
- 生产环境必须禁用模拟数据
- Dashboard 测试模式可以启用模拟数据
"""

# 模拟数据配置
MOCK_CONFIG = {
    # 全局开关：是否允许使用模拟数据
    # 设为 True 时，各模块可以根据自己的配置使用模拟数据
    # 设为 False 时，所有模块都不允许使用模拟数据
    'allow_mock_data': False,
    
    # Dashboard 模拟数据配置
    'dashboard': {
        'allow_mock_diagnostics': False,  # 是否允许模拟诊断数据
        'allow_mock_trajectory': False,   # 是否允许模拟轨迹数据
        'allow_mock_position': False,     # 是否允许模拟位置数据
    },
    
    # ROS 兼容层配置
    'ros_compat': {
        'allow_standalone_mode': False,    # 是否允许独立运行模式 (非 ROS 环境)
        'allow_mock_tf2': False,          # 是否允许模拟 TF2 数据
    },
    
    # 控制器配置
    'controller': {
        'allow_mock_odom': False,         # 是否允许模拟里程计数据
        'allow_mock_imu': False,          # 是否允许模拟 IMU 数据
        'allow_mock_trajectory': False,   # 是否允许模拟轨迹数据
    },
}

# 模拟数据配置验证规则
MOCK_VALIDATION_RULES = {
    'mock.allow_mock_data': (None, None, '全局模拟数据开关 (bool)'),
    'mock.dashboard.allow_mock_diagnostics': (None, None, 'Dashboard 模拟诊断数据开关 (bool)'),
    'mock.dashboard.allow_mock_trajectory': (None, None, 'Dashboard 模拟轨迹数据开关 (bool)'),
    'mock.dashboard.allow_mock_position': (None, None, 'Dashboard 模拟位置数据开关 (bool)'),
}


def is_mock_allowed(config: dict, module: str = None, feature: str = None) -> bool:
    """
    检查是否允许使用模拟数据
    
    Args:
        config: 配置字典
        module: 模块名称 ('dashboard', 'ros_compat', 'controller')
        feature: 功能名称 (如 'allow_mock_diagnostics')
    
    Returns:
        是否允许使用模拟数据
    
    Example:
        >>> config = {'mock': {'allow_mock_data': True, 'dashboard': {'allow_mock_diagnostics': True}}}
        >>> is_mock_allowed(config, 'dashboard', 'allow_mock_diagnostics')
        True
        >>> is_mock_allowed(config, 'dashboard', 'allow_mock_trajectory')
        False
    """
    mock_config = config.get('mock', {})
    
    # 首先检查全局开关
    if not mock_config.get('allow_mock_data', False):
        return False
    
    # 如果没有指定模块，只检查全局开关
    if module is None:
        return True
    
    # 检查模块级别配置
    module_config = mock_config.get(module, {})
    
    # 如果没有指定功能，检查模块是否有任何允许的功能
    if feature is None:
        return any(v for k, v in module_config.items() if k.startswith('allow_'))
    
    # 检查具体功能
    return module_config.get(feature, False)


__all__ = [
    'MOCK_CONFIG',
    'MOCK_VALIDATION_RULES',
    'is_mock_allowed',
]
