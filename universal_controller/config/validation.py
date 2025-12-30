"""配置验证模块

提供配置参数的验证功能：
- 范围检查
- 类型检查
- 逻辑一致性检查
"""
from typing import Dict, Any, List, Tuple, Optional

from ..core.exceptions import ConfigValidationError
from ..core.constants import EPSILON


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
    - MPC 权重必须为正值
    - EKF 数值稳定性参数必须为正值
    
    Args:
        config: 配置字典
    
    Returns:
        错误列表
    """
    errors = []
    
    def _is_numeric(value) -> bool:
        """检查值是否为数值类型"""
        return isinstance(value, (int, float)) and not isinstance(value, bool)
    
    # EKF 数值稳定性参数验证
    MIN_VELOCITY_FOR_JACOBIAN = EPSILON  # Jacobian 计算的最小速度阈值
    min_vel_jacobian = get_config_value(config, 'ekf.min_velocity_for_jacobian')
    if min_vel_jacobian is not None and _is_numeric(min_vel_jacobian):
        if min_vel_jacobian < MIN_VELOCITY_FOR_JACOBIAN:
            errors.append(('ekf.min_velocity_for_jacobian',
                          f'Jacobian 最小速度阈值 ({min_vel_jacobian}) 过小，'
                          f'可能导致数值不稳定，建议设置为 >= {MIN_VELOCITY_FOR_JACOBIAN}'))
    
    # EKF 协方差最小特征值验证
    MIN_EIGENVALUE = 1e-10
    min_eigenvalue = get_config_value(config, 'ekf.covariance.min_eigenvalue')
    if min_eigenvalue is not None and _is_numeric(min_eigenvalue):
        if min_eigenvalue < MIN_EIGENVALUE:
            errors.append(('ekf.covariance.min_eigenvalue',
                          f'协方差最小特征值 ({min_eigenvalue}) 过小，'
                          f'可能导致协方差矩阵奇异，建议设置为 >= {MIN_EIGENVALUE}'))
    
    # MPC 权重验证 - 必须为正值
    MIN_MPC_WEIGHT = EPSILON
    mpc_weight_keys = [
        ('mpc.weights.position', 'MPC 位置权重'),
        ('mpc.weights.velocity', 'MPC 速度权重'),
        ('mpc.weights.heading', 'MPC 航向权重'),
        ('mpc.weights.control_accel', 'MPC 加速度控制权重'),
        ('mpc.weights.control_alpha', 'MPC 角加速度控制权重'),
    ]
    for key_path, description in mpc_weight_keys:
        weight = get_config_value(config, key_path)
        if weight is not None and _is_numeric(weight):
            if weight < MIN_MPC_WEIGHT:
                errors.append((key_path, 
                              f'{description} ({weight}) 过小，可能导致求解器数值问题，'
                              f'建议设置为 >= {MIN_MPC_WEIGHT}'))
    
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
    lookahead_dist = get_config_value(config, 'backup.lookahead_dist')
    if _is_numeric(min_lookahead) and _is_numeric(max_lookahead):
        if min_lookahead > max_lookahead:
            errors.append(('backup.min_lookahead', 
                          f'最小前瞻距离 ({min_lookahead}) 不应大于最大前瞻距离 ({max_lookahead})'))
    if _is_numeric(lookahead_dist) and _is_numeric(min_lookahead) and _is_numeric(max_lookahead):
        if lookahead_dist < min_lookahead or lookahead_dist > max_lookahead:
            errors.append(('backup.lookahead_dist', 
                          f'默认前瞻距离 ({lookahead_dist}) 应在 [{min_lookahead}, {max_lookahead}] 范围内'))
    
    # 速度约束一致性
    v_min = get_config_value(config, 'constraints.v_min')
    v_max = get_config_value(config, 'constraints.v_max')
    if _is_numeric(v_min) and _is_numeric(v_max):
        if v_min > v_max:
            errors.append(('constraints.v_min', 
                          f'最小速度 ({v_min}) 不应大于最大速度 ({v_max})'))
    
    # 角速度约束验证 - omega_max 必须大于 0
    # omega_max = 0 会导致机器人无法转向，这是一个严重的配置错误
    omega_max = get_config_value(config, 'constraints.omega_max')
    if omega_max is not None and _is_numeric(omega_max):
        if omega_max <= 0:
            errors.append(('constraints.omega_max', 
                          f'最大角速度 ({omega_max}) 必须大于 0，否则机器人无法转向'))
    
    # 低速角速度约束验证 - omega_max_low 必须大于 0 且不超过 omega_max
    # omega_max_low 用于低速时的角速度限制，防止低速大转弯导致的不稳定
    omega_max_low = get_config_value(config, 'constraints.omega_max_low')
    if omega_max_low is not None and _is_numeric(omega_max_low):
        if omega_max_low <= 0:
            errors.append(('constraints.omega_max_low', 
                          f'低速最大角速度 ({omega_max_low}) 必须大于 0'))
        elif omega_max is not None and _is_numeric(omega_max) and omega_max_low > omega_max:
            errors.append(('constraints.omega_max_low', 
                          f'低速最大角速度 ({omega_max_low}) 不应大于最大角速度 ({omega_max})'))
    
    # 加速度约束验证 - a_max 必须大于 0
    a_max = get_config_value(config, 'constraints.a_max')
    if a_max is not None and _is_numeric(a_max):
        if a_max <= 0:
            errors.append(('constraints.a_max', 
                          f'最大加速度 ({a_max}) 必须大于 0，否则机器人无法加速'))
    
    # 角加速度约束验证 - alpha_max 必须大于 0
    # alpha_max 用于 MPC 控制输入约束和安全监控
    alpha_max = get_config_value(config, 'constraints.alpha_max')
    if alpha_max is not None and _is_numeric(alpha_max):
        if alpha_max <= 0:
            errors.append(('constraints.alpha_max', 
                          f'最大角加速度 ({alpha_max}) 必须大于 0，否则机器人无法改变角速度'))
    
    # 垂直加速度约束验证 - az_max 必须大于 0
    # az_max 用于安全监控中的垂直加速度限制检测
    az_max = get_config_value(config, 'constraints.az_max')
    if az_max is not None and _is_numeric(az_max):
        if az_max <= 0:
            errors.append(('constraints.az_max', 
                          f'最大垂直加速度 ({az_max}) 必须大于 0'))
    
    # 推力约束一致性
    thrust_min = get_config_value(config, 'attitude.thrust_min')
    thrust_max = get_config_value(config, 'attitude.thrust_max')
    if _is_numeric(thrust_min) and _is_numeric(thrust_max):
        if thrust_min > thrust_max:
            errors.append(('attitude.thrust_min', 
                          f'最小推力 ({thrust_min}) 不应大于最大推力 ({thrust_max})'))
    
    # 超时一致性
    # 注意: traj_grace_ms 是超时后的额外宽限期，不是超时的一部分
    # 安全停止延迟 = traj_timeout_ms + traj_grace_ms
    # 因此 traj_grace_ms 可以大于 traj_timeout_ms（虽然不常见）
    # 这里不做强制验证，只在 traj_grace_ms 明显过大时发出警告
    traj_timeout = get_config_value(config, 'watchdog.traj_timeout_ms')
    traj_grace = get_config_value(config, 'watchdog.traj_grace_ms')
    if _is_numeric(traj_timeout) and _is_numeric(traj_grace):
        # 只有当 traj_timeout > 0 (启用超时检测) 时才检查
        if traj_timeout > 0 and traj_grace > traj_timeout * 3:
            # 宽限期超过超时的 3 倍，可能是配置错误
            errors.append(('watchdog.traj_grace_ms', 
                          f'轨迹宽限期 ({traj_grace}ms) 远大于轨迹超时 ({traj_timeout}ms)，'
                          f'请确认这是预期配置。安全停止延迟 = {traj_timeout + traj_grace}ms'))
    
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
    
    # MPC dt 与轨迹 dt 一致性检查
    # MPC 的时间步长应该与轨迹的时间步长匹配，否则预测会不准确
    mpc_dt = get_config_value(config, 'mpc.dt')
    if _is_numeric(mpc_dt) and _is_numeric(default_dt):
        if abs(mpc_dt - default_dt) > EPSILON:
            errors.append(('mpc.dt', 
                          f'MPC 时间步长 ({mpc_dt}) 与轨迹时间步长 ({default_dt}) 不一致，'
                          f'这可能导致轨迹跟踪不准确'))
    
    min_points = get_config_value(config, 'trajectory.min_points')
    max_points = get_config_value(config, 'trajectory.max_points')
    if _is_numeric(min_points) and _is_numeric(max_points):
        if min_points > max_points:
            errors.append(('trajectory.min_points', 
                          f'最小轨迹点数 ({min_points}) 不应大于最大轨迹点数 ({max_points})'))
    
    # low_speed_thresh 一致性检查
    # 设计说明: low_speed_thresh 的唯一定义点是 trajectory.low_speed_thresh
    # consistency 模块直接从 trajectory 配置读取此值
    # 如果用户在 consistency 配置中设置了此值，记录警告但不阻止启动
    # 注意：废弃参数使用 logger.warning 而非添加到 errors，因为这不是致命错误
    consistency_low_speed = get_config_value(config, 'consistency.low_speed_thresh')
    if consistency_low_speed is not None:
        import logging
        _logger = logging.getLogger(__name__)
        _logger.warning(
            "配置废弃警告: consistency.low_speed_thresh 已废弃，请移除此配置。"
            "统一使用 trajectory.low_speed_thresh"
        )
    
    # Transform 配置一致性检查
    fallback_duration = get_config_value(config, 'transform.fallback_duration_limit_ms')
    fallback_critical = get_config_value(config, 'transform.fallback_critical_limit_ms')
    if _is_numeric(fallback_duration) and _is_numeric(fallback_critical):
        if fallback_duration > fallback_critical:
            errors.append(('transform.fallback_duration_limit_ms', 
                          f'TF2 降级警告阈值 ({fallback_duration}ms) 不应大于临界阈值 ({fallback_critical}ms)'))
    
    # Safety 加速度预热参数一致性检查
    # accel_warmup_margin_multiplier 应该 <= accel_warmup_margin_max
    # 否则 multiplier 会被隐式截断为 max，用户可能不知道配置未生效
    accel_warmup_multiplier = get_config_value(config, 'safety.accel_warmup_margin_multiplier')
    accel_warmup_max = get_config_value(config, 'safety.accel_warmup_margin_max')
    if _is_numeric(accel_warmup_multiplier) and _is_numeric(accel_warmup_max):
        if accel_warmup_multiplier > accel_warmup_max:
            errors.append(('safety.accel_warmup_margin_multiplier', 
                          f'预热裕度倍数 ({accel_warmup_multiplier}) 大于上限 ({accel_warmup_max})，'
                          f'实际生效值将被截断为 {accel_warmup_max}。'
                          f'请调整配置使 multiplier <= max'))
    
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
