"""配置验证模块

提供配置参数的验证功能：
- 范围检查
- 类型检查
- 逻辑一致性检查
"""
from typing import Dict, Any, List, Tuple, Optional


class ConfigValidationError(ValueError):
    """配置验证错误"""
    pass


def get_config_value(
    config: Dict[str, Any], 
    key_path: str, 
    default: Any = None,
    fallback_config: Optional[Dict[str, Any]] = None
) -> Any:
    """
    从配置字典中获取值，支持点分隔的路径
    
    Args:
        config: 配置字典
        key_path: 点分隔的键路径，如 'mpc.horizon'
        default: 默认值
        fallback_config: 备选配置字典，当 config 中找不到时从此获取
    
    Returns:
        配置值或默认值
    
    Example:
        >>> config = {'mpc': {'horizon': 20}}
        >>> get_config_value(config, 'mpc.horizon')
        20
    """
    keys = key_path.split('.')
    value = config
    for key in keys:
        if isinstance(value, dict) and key in value:
            value = value[key]
        else:
            # 尝试从备选配置获取
            if fallback_config is not None:
                return get_config_value(fallback_config, key_path, default, None)
            return default
    return value


def validate_config(
    config: Dict[str, Any], 
    validation_rules: Dict[str, Tuple],
    raise_on_error: bool = True
) -> List[Tuple[str, str]]:
    """
    验证配置参数
    
    Args:
        config: 配置字典
        validation_rules: 验证规则字典，格式为 {key_path: (min, max, description)}
        raise_on_error: 是否在发现错误时抛出异常
    
    Returns:
        错误列表，每个元素为 (key_path, error_message)
    
    Raises:
        ConfigValidationError: 当 raise_on_error=True 且发现错误时
    """
    errors = []
    
    for key_path, (min_val, max_val, description) in validation_rules.items():
        value = get_config_value(config, key_path)
        
        if value is None:
            continue  # 使用默认值，跳过验证
        
        # 类型检查
        if not isinstance(value, (int, float)):
            errors.append((key_path, f'{description} 类型错误，期望数值，实际为 {type(value).__name__}'))
            continue
        
        # 范围检查
        if min_val is not None and value < min_val:
            errors.append((key_path, f'{description} 值 {value} 小于最小值 {min_val}'))
        elif max_val is not None and value > max_val:
            errors.append((key_path, f'{description} 值 {value} 大于最大值 {max_val}'))
    
    if errors and raise_on_error:
        error_messages = '\n'.join([f'  - {key}: {msg}' for key, msg in errors])
        raise ConfigValidationError(f'配置验证失败:\n{error_messages}')
    
    return errors


def validate_logical_consistency(config: Dict[str, Any]) -> List[Tuple[str, str]]:
    """
    验证配置的逻辑一致性
    
    检查配置参数之间的逻辑关系，例如：
    - min_lookahead < max_lookahead
    - horizon_degraded <= horizon
    - v_min <= v_max
    
    Args:
        config: 配置字典
    
    Returns:
        错误列表
    """
    errors = []
    
    def _is_numeric(value) -> bool:
        """检查值是否为数值类型"""
        return isinstance(value, (int, float)) and not isinstance(value, bool)
    
    # MPC horizon 一致性
    horizon = get_config_value(config, 'mpc.horizon')
    horizon_degraded = get_config_value(config, 'mpc.horizon_degraded')
    if _is_numeric(horizon) and _is_numeric(horizon_degraded):
        if horizon_degraded > horizon:
            errors.append(('mpc.horizon_degraded', 
                          f'降级 horizon ({horizon_degraded}) 不应大于正常 horizon ({horizon})'))
    
    # 前瞻距离一致性
    min_lookahead = get_config_value(config, 'backup.min_lookahead')
    max_lookahead = get_config_value(config, 'backup.max_lookahead')
    if _is_numeric(min_lookahead) and _is_numeric(max_lookahead):
        if min_lookahead > max_lookahead:
            errors.append(('backup.min_lookahead', 
                          f'最小前瞻距离 ({min_lookahead}) 不应大于最大前瞻距离 ({max_lookahead})'))
    
    # 速度约束一致性
    v_min = get_config_value(config, 'constraints.v_min')
    v_max = get_config_value(config, 'constraints.v_max')
    if _is_numeric(v_min) and _is_numeric(v_max):
        if v_min > v_max:
            errors.append(('constraints.v_min', 
                          f'最小速度 ({v_min}) 不应大于最大速度 ({v_max})'))
    
    # 推力约束一致性
    thrust_min = get_config_value(config, 'attitude.thrust_min')
    thrust_max = get_config_value(config, 'attitude.thrust_max')
    if _is_numeric(thrust_min) and _is_numeric(thrust_max):
        if thrust_min > thrust_max:
            errors.append(('attitude.thrust_min', 
                          f'最小推力 ({thrust_min}) 不应大于最大推力 ({thrust_max})'))
    
    # 超时一致性
    traj_timeout = get_config_value(config, 'watchdog.traj_timeout_ms')
    traj_grace = get_config_value(config, 'watchdog.traj_grace_ms')
    if _is_numeric(traj_timeout) and _is_numeric(traj_grace):
        if traj_grace > traj_timeout:
            errors.append(('watchdog.traj_grace_ms', 
                          f'轨迹宽限期 ({traj_grace}ms) 不应大于轨迹超时 ({traj_timeout}ms)'))
    
    # 轨迹配置一致性
    min_dt = get_config_value(config, 'trajectory.min_dt_sec')
    max_dt = get_config_value(config, 'trajectory.max_dt_sec')
    default_dt = get_config_value(config, 'trajectory.default_dt_sec')
    if _is_numeric(min_dt) and _is_numeric(max_dt):
        if min_dt > max_dt:
            errors.append(('trajectory.min_dt_sec', 
                          f'最小时间步长 ({min_dt}) 不应大于最大时间步长 ({max_dt})'))
    if _is_numeric(default_dt) and _is_numeric(min_dt) and _is_numeric(max_dt):
        if default_dt < min_dt or default_dt > max_dt:
            errors.append(('trajectory.default_dt_sec', 
                          f'默认时间步长 ({default_dt}) 应在 [{min_dt}, {max_dt}] 范围内'))
    
    min_points = get_config_value(config, 'trajectory.min_points')
    max_points = get_config_value(config, 'trajectory.max_points')
    default_points = get_config_value(config, 'trajectory.default_num_points')
    if _is_numeric(min_points) and _is_numeric(max_points):
        if min_points > max_points:
            errors.append(('trajectory.min_points', 
                          f'最小轨迹点数 ({min_points}) 不应大于最大轨迹点数 ({max_points})'))
    if _is_numeric(default_points) and _is_numeric(min_points) and _is_numeric(max_points):
        if default_points < min_points or default_points > max_points:
            errors.append(('trajectory.default_num_points', 
                          f'默认轨迹点数 ({default_points}) 应在 [{min_points}, {max_points}] 范围内'))
    
    # low_speed_thresh 一致性检查
    # 注意: consistency 模块现在统一从 trajectory.low_speed_thresh 读取
    # 如果用户在 consistency 配置中显式设置了不同的值，发出警告
    traj_low_speed = get_config_value(config, 'trajectory.low_speed_thresh')
    consistency_low_speed = get_config_value(config, 'consistency.low_speed_thresh')
    if _is_numeric(traj_low_speed) and _is_numeric(consistency_low_speed):
        if abs(traj_low_speed - consistency_low_speed) > 1e-6:
            errors.append(('consistency.low_speed_thresh', 
                          f'一致性检查低速阈值 ({consistency_low_speed}) 与轨迹低速阈值 ({traj_low_speed}) 不一致。'
                          f'建议移除 consistency.low_speed_thresh 配置，统一使用 trajectory.low_speed_thresh'))
    
    # Transform 配置一致性检查
    fallback_duration = get_config_value(config, 'transform.fallback_duration_limit_ms')
    fallback_critical = get_config_value(config, 'transform.fallback_critical_limit_ms')
    if _is_numeric(fallback_duration) and _is_numeric(fallback_critical):
        if fallback_duration > fallback_critical:
            errors.append(('transform.fallback_duration_limit_ms', 
                          f'TF2 降级警告阈值 ({fallback_duration}ms) 不应大于临界阈值 ({fallback_critical}ms)'))
    
    return errors


def validate_full_config(
    config: Dict[str, Any], 
    validation_rules: Dict[str, Tuple],
    raise_on_error: bool = True
) -> List[Tuple[str, str]]:
    """
    完整配置验证（包括范围检查和逻辑一致性检查）
    
    Args:
        config: 配置字典
        validation_rules: 验证规则字典
        raise_on_error: 是否在发现错误时抛出异常
    
    Returns:
        错误列表
    
    Raises:
        ConfigValidationError: 当 raise_on_error=True 且发现错误时
    """
    # 范围检查
    errors = validate_config(config, validation_rules, raise_on_error=False)
    
    # 逻辑一致性检查
    logical_errors = validate_logical_consistency(config)
    errors.extend(logical_errors)
    
    if errors and raise_on_error:
        error_messages = '\n'.join([f'  - {key}: {msg}' for key, msg in errors])
        raise ConfigValidationError(f'配置验证失败:\n{error_messages}')
    
    return errors
