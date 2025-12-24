"""默认配置

本模块合并所有配置子模块，提供统一的配置接口。

配置结构:
- platform_config.py: 平台配置 (PLATFORM_CONFIG)
- system_config.py: 系统基础配置
- mpc_config.py: MPC 配置
- safety_config.py: 安全和约束配置
- ekf_config.py: EKF 配置
- attitude_config.py: 姿态控制配置
- modules_config.py: 其他模块配置
- validation.py: 配置验证

使用示例:
    from universal_controller.config import DEFAULT_CONFIG, PLATFORM_CONFIG
    
    config = DEFAULT_CONFIG.copy()
    config['system']['platform'] = 'quadrotor'
"""
from typing import Dict, Any

# 导入平台配置
from .platform_config import PLATFORM_CONFIG

# 导入各模块配置
from .system_config import (
    SYSTEM_CONFIG, 
    WATCHDOG_CONFIG, 
    DIAGNOSTICS_CONFIG,
    TRACKING_CONFIG,
    SYSTEM_VALIDATION_RULES,
)
from .mpc_config import MPC_CONFIG, MPC_VALIDATION_RULES
from .safety_config import (
    CONSTRAINTS_CONFIG, 
    SAFETY_CONFIG,
    SAFETY_VALIDATION_RULES,
)
from .ekf_config import EKF_CONFIG, EKF_VALIDATION_RULES
from .attitude_config import ATTITUDE_CONFIG, ATTITUDE_VALIDATION_RULES
from .modules_config import (
    CONSISTENCY_CONFIG,
    TRANSFORM_CONFIG,
    TRANSITION_CONFIG,
    BACKUP_CONFIG,
    MODULES_VALIDATION_RULES,
)
from .trajectory_config import TRAJECTORY_CONFIG, TRAJECTORY_VALIDATION_RULES
from .mock_config import MOCK_CONFIG, MOCK_VALIDATION_RULES, is_mock_allowed

# 导入验证模块
from .validation import (
    ConfigValidationError,
    get_config_value,
    validate_config as _validate_config,
    validate_logical_consistency,
    validate_full_config,
)


# =============================================================================
# 合并所有配置
# =============================================================================
DEFAULT_CONFIG: Dict[str, Any] = {
    'system': SYSTEM_CONFIG.copy(),
    'mpc': MPC_CONFIG.copy(),
    'watchdog': WATCHDOG_CONFIG.copy(),
    'diagnostics': DIAGNOSTICS_CONFIG.copy(),
    'consistency': CONSISTENCY_CONFIG.copy(),
    'safety': SAFETY_CONFIG.copy(),
    'transform': TRANSFORM_CONFIG.copy(),
    'transition': TRANSITION_CONFIG.copy(),
    'backup': BACKUP_CONFIG.copy(),
    'constraints': CONSTRAINTS_CONFIG.copy(),
    'ekf': EKF_CONFIG.copy(),
    'attitude': ATTITUDE_CONFIG.copy(),
    'trajectory': TRAJECTORY_CONFIG.copy(),
    'tracking': TRACKING_CONFIG.copy(),
    'mock': MOCK_CONFIG.copy(),
}


# =============================================================================
# 合并所有验证规则
# =============================================================================
CONFIG_VALIDATION_RULES: Dict[str, tuple] = {}
CONFIG_VALIDATION_RULES.update(SYSTEM_VALIDATION_RULES)
CONFIG_VALIDATION_RULES.update(MPC_VALIDATION_RULES)
CONFIG_VALIDATION_RULES.update(SAFETY_VALIDATION_RULES)
CONFIG_VALIDATION_RULES.update(EKF_VALIDATION_RULES)
CONFIG_VALIDATION_RULES.update(ATTITUDE_VALIDATION_RULES)
CONFIG_VALIDATION_RULES.update(MODULES_VALIDATION_RULES)
CONFIG_VALIDATION_RULES.update(TRAJECTORY_VALIDATION_RULES)
CONFIG_VALIDATION_RULES.update(MOCK_VALIDATION_RULES)


# =============================================================================
# 向后兼容的验证函数
# =============================================================================
def validate_config(config: Dict[str, Any], raise_on_error: bool = True) -> list:
    """
    验证配置参数（向后兼容接口）
    
    Args:
        config: 配置字典
        raise_on_error: 是否在发现错误时抛出异常
    
    Returns:
        错误列表，每个元素为 (key_path, error_message)
    
    Raises:
        ConfigValidationError: 当 raise_on_error=True 且发现错误时
    
    Example:
        >>> config = DEFAULT_CONFIG.copy()
        >>> config['mpc']['horizon'] = -1
        >>> errors = validate_config(config, raise_on_error=False)
        >>> print(errors)
        [('mpc.horizon', 'MPC 预测时域 值 -1 超出范围 [1, 100]')]
    """
    return validate_full_config(config, CONFIG_VALIDATION_RULES, raise_on_error)


# =============================================================================
# 导出
# =============================================================================
__all__ = [
    # 配置
    'DEFAULT_CONFIG',
    'PLATFORM_CONFIG',
    # 验证
    'validate_config',
    'get_config_value',
    'ConfigValidationError',
    'CONFIG_VALIDATION_RULES',
    # Mock 配置
    'MOCK_CONFIG',
    'is_mock_allowed',
    # 子模块配置 (可选导出)
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
    'TRACKING_CONFIG',
]
