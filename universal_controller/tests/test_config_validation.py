"""配置验证测试"""
import pytest
from universal_controller.config.default_config import (
    DEFAULT_CONFIG, 
    validate_config, 
    ConfigValidationError,
    get_config_value
)


def test_default_config_valid():
    """测试默认配置应该通过验证"""
    errors = validate_config(DEFAULT_CONFIG, raise_on_error=False)
    assert len(errors) == 0, f"Default config has errors: {errors}"


def test_validate_config_invalid_horizon():
    """测试无效的 MPC horizon"""
    config = DEFAULT_CONFIG.copy()
    config['mpc'] = DEFAULT_CONFIG['mpc'].copy()
    config['mpc']['horizon'] = -1
    
    errors = validate_config(config, raise_on_error=False)
    assert len(errors) > 0
    assert any('horizon' in key for key, _ in errors)


def test_validate_config_invalid_ctrl_freq():
    """测试无效的控制频率"""
    config = DEFAULT_CONFIG.copy()
    config['system'] = DEFAULT_CONFIG['system'].copy()
    config['system']['ctrl_freq'] = 0
    
    errors = validate_config(config, raise_on_error=False)
    assert len(errors) > 0
    assert any('ctrl_freq' in key for key, _ in errors)


def test_validate_config_logical_consistency_horizon():
    """测试 horizon 逻辑一致性"""
    config = DEFAULT_CONFIG.copy()
    config['mpc'] = DEFAULT_CONFIG['mpc'].copy()
    config['mpc']['horizon'] = 10
    config['mpc']['horizon_degraded'] = 20  # 降级 horizon 大于正常 horizon
    
    errors = validate_config(config, raise_on_error=False)
    assert len(errors) > 0
    assert any('horizon_degraded' in key for key, _ in errors)


def test_validate_config_logical_consistency_lookahead():
    """测试前瞻距离逻辑一致性"""
    config = DEFAULT_CONFIG.copy()
    config['backup'] = DEFAULT_CONFIG['backup'].copy()
    config['backup']['min_lookahead'] = 5.0
    config['backup']['max_lookahead'] = 2.0  # min > max
    
    errors = validate_config(config, raise_on_error=False)
    assert len(errors) > 0
    assert any('min_lookahead' in key for key, _ in errors)


def test_validate_config_logical_consistency_velocity():
    """测试速度约束逻辑一致性"""
    config = DEFAULT_CONFIG.copy()
    config['constraints'] = DEFAULT_CONFIG['constraints'].copy()
    config['constraints']['v_min'] = 5.0
    config['constraints']['v_max'] = 2.0  # min > max
    
    errors = validate_config(config, raise_on_error=False)
    assert len(errors) > 0
    assert any('v_min' in key for key, _ in errors)


def test_validate_config_raise_on_error():
    """测试 raise_on_error 参数"""
    config = DEFAULT_CONFIG.copy()
    config['mpc'] = DEFAULT_CONFIG['mpc'].copy()
    config['mpc']['horizon'] = -1
    
    with pytest.raises(ConfigValidationError):
        validate_config(config, raise_on_error=True)


def test_validate_config_type_error():
    """测试类型错误检测"""
    config = DEFAULT_CONFIG.copy()
    config['mpc'] = DEFAULT_CONFIG['mpc'].copy()
    config['mpc']['horizon'] = "invalid"  # 应该是数字
    
    errors = validate_config(config, raise_on_error=False)
    assert len(errors) > 0
    assert any('类型错误' in msg for _, msg in errors)


def test_get_config_value_nested():
    """测试嵌套配置值获取"""
    value = get_config_value(DEFAULT_CONFIG, 'mpc.weights.position')
    assert value == 10.0


def test_get_config_value_default():
    """测试默认值获取"""
    value = get_config_value(DEFAULT_CONFIG, 'nonexistent.key', default=42)
    assert value == 42


def test_get_config_value_from_default():
    """测试从备选配置获取缺失值"""
    config = {'system': {'ctrl_freq': 100}}  # 缺少其他配置
    # 使用 fallback_config 参数显式指定备选配置
    value = get_config_value(config, 'mpc.horizon', fallback_config=DEFAULT_CONFIG)
    assert value == 20  # 从 DEFAULT_CONFIG 获取


def test_validate_config_alpha_max_constraint():
    """测试角加速度约束验证 - alpha_max 必须大于 0"""
    config = DEFAULT_CONFIG.copy()
    config['constraints'] = DEFAULT_CONFIG['constraints'].copy()
    config['constraints']['alpha_max'] = 0  # 无效值
    
    errors = validate_config(config, raise_on_error=False)
    assert len(errors) > 0
    assert any('alpha_max' in key for key, _ in errors)
    
    # 测试负值
    config['constraints']['alpha_max'] = -1.0
    errors = validate_config(config, raise_on_error=False)
    assert len(errors) > 0
    assert any('alpha_max' in key for key, _ in errors)


def test_validate_config_az_max_constraint():
    """测试垂直加速度约束验证 - az_max 必须大于 0"""
    config = DEFAULT_CONFIG.copy()
    config['constraints'] = DEFAULT_CONFIG['constraints'].copy()
    config['constraints']['az_max'] = 0  # 无效值
    
    errors = validate_config(config, raise_on_error=False)
    assert len(errors) > 0
    assert any('az_max' in key for key, _ in errors)
    
    # 测试负值
    config['constraints']['az_max'] = -5.0
    errors = validate_config(config, raise_on_error=False)
    assert len(errors) > 0
    assert any('az_max' in key for key, _ in errors)


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
