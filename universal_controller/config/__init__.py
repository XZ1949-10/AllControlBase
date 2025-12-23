"""配置模块

提供统一的配置接口，支持：
- 平台配置 (PLATFORM_CONFIG)
- 默认配置 (DEFAULT_CONFIG)
- 配置验证 (validate_config)

配置文件结构:
- platform_config.py: 平台运动学配置
- system_config.py: 系统基础配置
- mpc_config.py: MPC 控制器配置
- safety_config.py: 安全和约束配置
- ekf_config.py: EKF 状态估计器配置
- attitude_config.py: 姿态控制配置
- modules_config.py: 其他模块配置
- validation.py: 配置验证逻辑

使用示例:
    from universal_controller.config import DEFAULT_CONFIG, PLATFORM_CONFIG
    
    # 创建配置副本
    config = DEFAULT_CONFIG.copy()
    config['system']['platform'] = 'quadrotor'
    
    # 验证配置
    from universal_controller.config import validate_config
    errors = validate_config(config, raise_on_error=False)
"""

# 主要导出 (向后兼容)
from .default_config import (
    # 核心配置
    DEFAULT_CONFIG,
    PLATFORM_CONFIG,
    # 验证
    validate_config,
    get_config_value,
    ConfigValidationError,
    CONFIG_VALIDATION_RULES,
    # Mock 配置
    MOCK_CONFIG,
    is_mock_allowed,
)

# 可选: 导出子模块配置
from .default_config import (
    SYSTEM_CONFIG,
    MPC_CONFIG,
    SAFETY_CONFIG,
    CONSTRAINTS_CONFIG,
    EKF_CONFIG,
    ATTITUDE_CONFIG,
    CONSISTENCY_CONFIG,
    TRANSFORM_CONFIG,
    TRANSITION_CONFIG,
    BACKUP_CONFIG,
    WATCHDOG_CONFIG,
    DIAGNOSTICS_CONFIG,
    TRAJECTORY_CONFIG,
)

__all__ = [
    # 核心导出
    'DEFAULT_CONFIG',
    'PLATFORM_CONFIG',
    'validate_config',
    'get_config_value',
    'ConfigValidationError',
    'CONFIG_VALIDATION_RULES',
    # Mock 配置
    'MOCK_CONFIG',
    'is_mock_allowed',
    # 子模块配置
    'SYSTEM_CONFIG',
    'MPC_CONFIG',
    'SAFETY_CONFIG',
    'CONSTRAINTS_CONFIG',
    'EKF_CONFIG',
    'ATTITUDE_CONFIG',
    'CONSISTENCY_CONFIG',
    'TRANSFORM_CONFIG',
    'TRANSITION_CONFIG',
    'BACKUP_CONFIG',
    'WATCHDOG_CONFIG',
    'DIAGNOSTICS_CONFIG',
    'TRAJECTORY_CONFIG',
]
