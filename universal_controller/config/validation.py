"""配置验证模块

提供配置参数的验证功能：
- 范围检查
- 类型检查
- 逻辑一致性检查
- 错误严重级别分类

错误严重级别:
- FATAL: 致命错误，必须阻止启动（如 v_max <= 0）
- ERROR: 严重错误，默认阻止启动，可通过参数跳过
- WARNING: 警告，记录但不阻止启动
"""
from typing import Dict, Any, List, Tuple, Optional
from enum import Enum

from ..core.exceptions import ConfigValidationError
from ..core.constants import EPSILON


class ValidationSeverity(Enum):
    """验证错误严重级别"""
    FATAL = 'fatal'      # 致命错误，必须阻止启动
    ERROR = 'error'      # 严重错误，默认阻止启动
    WARNING = 'warning'  # 警告，记录但不阻止启动


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


def validate_logical_consistency(config: Dict[str, Any]) -> List[Tuple[str, str, ValidationSeverity]]:
    """
    验证配置的逻辑一致性
    
    检查配置参数之间的逻辑关系，返回带严重级别的错误列表。
    
    严重级别:
    - FATAL: 致命错误，必须阻止启动（如 v_max <= 0）
    - ERROR: 严重错误，默认阻止启动
    - WARNING: 警告，记录但不阻止启动
    
    Args:
        config: 配置字典
    
    Returns:
        错误列表，每个元素为 (key_path, error_message, severity)
    """
    errors = []
    
    def _is_numeric(value) -> bool:
        """检查值是否为数值类型"""
        return isinstance(value, (int, float)) and not isinstance(value, bool)
    
    def add_error(key: str, msg: str, severity: ValidationSeverity = ValidationSeverity.ERROR):
        """添加错误到列表"""
        errors.append((key, msg, severity))
    
    # ==========================================================================
    # 安全关键参数禁用警告 (WARNING 级别)
    # ==========================================================================
    
    # 检查 odom 超时是否被禁用
    odom_timeout = get_config_value(config, 'watchdog.odom_timeout_ms')
    if odom_timeout is not None and _is_numeric(odom_timeout) and odom_timeout <= 0:
        add_error('watchdog.odom_timeout_ms',
                  "里程计超时检测已禁用 (值 <= 0)，如果里程计数据丢失系统将无法检测",
                  ValidationSeverity.WARNING)
    
    # 检查轨迹超时是否被禁用
    traj_timeout = get_config_value(config, 'watchdog.traj_timeout_ms')
    if traj_timeout is not None and _is_numeric(traj_timeout) and traj_timeout <= 0:
        add_error('watchdog.traj_timeout_ms',
                  "轨迹超时检测已禁用 (值 <= 0)，如果轨迹数据丢失系统将无法检测",
                  ValidationSeverity.WARNING)
    
    # ==========================================================================
    # 致命错误 (FATAL 级别) - 这些错误会导致系统无法正常工作
    # ==========================================================================
    
    # 速度约束验证 - v_max 必须大于 0
    v_max = get_config_value(config, 'constraints.v_max')
    if v_max is not None and _is_numeric(v_max):
        if v_max <= 0:
            add_error('constraints.v_max',
                      f'最大速度 ({v_max}) 必须大于 0，否则机器人无法移动',
                      ValidationSeverity.FATAL)
    
    # 角速度约束验证 - omega_max 必须大于 0
    omega_max = get_config_value(config, 'constraints.omega_max')
    if omega_max is not None and _is_numeric(omega_max):
        if omega_max <= 0:
            add_error('constraints.omega_max', 
                      f'最大角速度 ({omega_max}) 必须大于 0，否则机器人无法转向',
                      ValidationSeverity.FATAL)
    
    # 加速度约束验证
    a_max = get_config_value(config, 'constraints.a_max')
    if a_max is not None and _is_numeric(a_max):
        if a_max <= 0:
            add_error('constraints.a_max', 
                      f'最大加速度 ({a_max}) 必须大于 0，否则机器人无法加速',
                      ValidationSeverity.FATAL)
    
    # 角加速度约束验证
    alpha_max = get_config_value(config, 'constraints.alpha_max')
    if alpha_max is not None and _is_numeric(alpha_max):
        if alpha_max <= 0:
            add_error('constraints.alpha_max', 
                      f'最大角加速度 ({alpha_max}) 必须大于 0，否则机器人无法改变角速度',
                      ValidationSeverity.FATAL)
    
    # ==========================================================================
    # 严重错误 (ERROR 级别) - 这些错误可能导致系统行为异常
    # ==========================================================================
    
    # EKF 数值稳定性参数验证
    MIN_VELOCITY_FOR_JACOBIAN = EPSILON
    min_vel_jacobian = get_config_value(config, 'ekf.min_velocity_for_jacobian')
    if min_vel_jacobian is not None and _is_numeric(min_vel_jacobian):
        if min_vel_jacobian < MIN_VELOCITY_FOR_JACOBIAN:
            add_error('ekf.min_velocity_for_jacobian',
                      f'Jacobian 最小速度阈值 ({min_vel_jacobian}) 过小，'
                      f'可能导致数值不稳定，建议 >= {MIN_VELOCITY_FOR_JACOBIAN}',
                      ValidationSeverity.ERROR)
    
    # EKF 协方差最小特征值验证
    MIN_EIGENVALUE = 1e-10
    min_eigenvalue = get_config_value(config, 'ekf.covariance.min_eigenvalue')
    if min_eigenvalue is not None and _is_numeric(min_eigenvalue):
        if min_eigenvalue < MIN_EIGENVALUE:
            add_error('ekf.covariance.min_eigenvalue',
                      f'协方差最小特征值 ({min_eigenvalue}) 过小，'
                      f'可能导致协方差矩阵奇异，建议 >= {MIN_EIGENVALUE}',
                      ValidationSeverity.ERROR)
    
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
                add_error(key_path, 
                          f'{description} ({weight}) 过小，可能导致求解器数值问题',
                          ValidationSeverity.ERROR)
    
    # MPC horizon 一致性
    horizon = get_config_value(config, 'mpc.horizon')
    horizon_degraded = get_config_value(config, 'mpc.horizon_degraded')
    if _is_numeric(horizon) and _is_numeric(horizon_degraded):
        if horizon_degraded > horizon:
            add_error('mpc.horizon_degraded', 
                      f'降级 horizon ({horizon_degraded}) 不应大于正常 horizon ({horizon})',
                      ValidationSeverity.ERROR)
    
    # MPC horizon 与轨迹点数的关系验证
    min_points = get_config_value(config, 'trajectory.min_points')
    max_points = get_config_value(config, 'trajectory.max_points')
    if _is_numeric(horizon) and _is_numeric(max_points):
        if horizon > max_points:
            add_error('mpc.horizon',
                      f'MPC horizon ({horizon}) 不应大于最大轨迹点数 ({max_points})',
                      ValidationSeverity.ERROR)
    
    # 前瞻距离一致性
    min_lookahead = get_config_value(config, 'backup.min_lookahead')
    max_lookahead = get_config_value(config, 'backup.max_lookahead')
    lookahead_dist = get_config_value(config, 'backup.lookahead_dist')
    if _is_numeric(min_lookahead) and _is_numeric(max_lookahead):
        if min_lookahead > max_lookahead:
            add_error('backup.min_lookahead', 
                      f'最小前瞻距离 ({min_lookahead}) 不应大于最大前瞻距离 ({max_lookahead})',
                      ValidationSeverity.ERROR)
    if _is_numeric(lookahead_dist) and _is_numeric(min_lookahead) and _is_numeric(max_lookahead):
        if lookahead_dist < min_lookahead or lookahead_dist > max_lookahead:
            add_error('backup.lookahead_dist', 
                      f'默认前瞻距离 ({lookahead_dist}) 应在 [{min_lookahead}, {max_lookahead}] 范围内',
                      ValidationSeverity.ERROR)
    
    # 速度约束一致性
    v_min = get_config_value(config, 'constraints.v_min')
    if _is_numeric(v_min) and _is_numeric(v_max):
        if v_min > v_max:
            add_error('constraints.v_min', 
                      f'最小速度 ({v_min}) 不应大于最大速度 ({v_max})',
                      ValidationSeverity.ERROR)
    
    # 低速角速度约束验证
    omega_max_low = get_config_value(config, 'constraints.omega_max_low')
    if omega_max_low is not None and _is_numeric(omega_max_low):
        if omega_max_low <= 0:
            add_error('constraints.omega_max_low', 
                      f'低速最大角速度 ({omega_max_low}) 必须大于 0',
                      ValidationSeverity.ERROR)
        elif omega_max is not None and _is_numeric(omega_max) and omega_max_low > omega_max:
            add_error('constraints.omega_max_low', 
                      f'低速最大角速度 ({omega_max_low}) 不应大于最大角速度 ({omega_max})',
                      ValidationSeverity.ERROR)
    
    # 垂直加速度约束验证
    az_max = get_config_value(config, 'constraints.az_max')
    if az_max is not None and _is_numeric(az_max):
        if az_max <= 0:
            add_error('constraints.az_max', 
                      f'最大垂直加速度 ({az_max}) 必须大于 0',
                      ValidationSeverity.ERROR)
    
    # 推力约束一致性
    thrust_min = get_config_value(config, 'attitude.thrust_min')
    thrust_max = get_config_value(config, 'attitude.thrust_max')
    if _is_numeric(thrust_min) and _is_numeric(thrust_max):
        if thrust_min > thrust_max:
            add_error('attitude.thrust_min', 
                      f'最小推力 ({thrust_min}) 不应大于最大推力 ({thrust_max})',
                      ValidationSeverity.ERROR)
    
    # ==========================================================================
    # 警告 (WARNING 级别) - 这些问题不会阻止启动，但可能影响性能
    # ==========================================================================
    
    # 超时一致性
    traj_timeout_val = get_config_value(config, 'watchdog.traj_timeout_ms')
    traj_grace = get_config_value(config, 'watchdog.traj_grace_ms')
    if _is_numeric(traj_timeout_val) and _is_numeric(traj_grace):
        if traj_timeout_val > 0 and traj_grace > traj_timeout_val * 3:
            add_error('watchdog.traj_grace_ms', 
                      f'轨迹宽限期 ({traj_grace}ms) 远大于轨迹超时 ({traj_timeout_val}ms)',
                      ValidationSeverity.WARNING)
    
    # 轨迹配置一致性
    min_dt = get_config_value(config, 'trajectory.min_dt_sec')
    max_dt = get_config_value(config, 'trajectory.max_dt_sec')
    default_dt = get_config_value(config, 'trajectory.default_dt_sec')
    if _is_numeric(min_dt) and _is_numeric(max_dt):
        if min_dt > max_dt:
            add_error('trajectory.min_dt_sec', 
                      f'最小时间步长 ({min_dt}) 不应大于最大时间步长 ({max_dt})',
                      ValidationSeverity.ERROR)
    if _is_numeric(default_dt) and _is_numeric(min_dt) and _is_numeric(max_dt):
        if default_dt < min_dt or default_dt > max_dt:
            add_error('trajectory.default_dt_sec', 
                      f'默认时间步长 ({default_dt}) 应在 [{min_dt}, {max_dt}] 范围内',
                      ValidationSeverity.ERROR)
    
    # MPC dt 与轨迹 dt 一致性检查
    # 注意: trajectory.default_dt_sec 会自动继承 mpc.dt（如果未显式配置）
    # 因此只有当两者都显式配置且不一致时才发出警告
    mpc_dt = get_config_value(config, 'mpc.dt')
    
    # 检查 trajectory.default_dt_sec 是否被显式配置
    # 如果 config 中存在 trajectory 字典且其中有 default_dt_sec 键，则认为是显式配置
    traj_config = config.get('trajectory', {})
    default_dt_explicitly_set = isinstance(traj_config, dict) and 'default_dt_sec' in traj_config
    
    if _is_numeric(mpc_dt) and _is_numeric(default_dt) and default_dt_explicitly_set:
        if abs(mpc_dt - default_dt) > EPSILON:
            add_error('mpc.dt', 
                      f'MPC 时间步长 ({mpc_dt}) 与显式配置的轨迹时间步长 ({default_dt}) 不一致，'
                      f'建议只配置 mpc.dt，trajectory.default_dt_sec 会自动继承',
                      ValidationSeverity.WARNING)
    
    if _is_numeric(min_points) and _is_numeric(max_points):
        if min_points > max_points:
            add_error('trajectory.min_points', 
                      f'最小轨迹点数 ({min_points}) 不应大于最大轨迹点数 ({max_points})',
                      ValidationSeverity.ERROR)
    
    # low_speed_thresh 废弃参数检查
    consistency_low_speed = get_config_value(config, 'consistency.low_speed_thresh')
    if consistency_low_speed is not None:
        add_error('consistency.low_speed_thresh',
                  "此配置已废弃，请移除并使用 trajectory.low_speed_thresh",
                  ValidationSeverity.WARNING)
    
    # Transform 配置一致性检查
    fallback_duration = get_config_value(config, 'transform.fallback_duration_limit_ms')
    fallback_critical = get_config_value(config, 'transform.fallback_critical_limit_ms')
    if _is_numeric(fallback_duration) and _is_numeric(fallback_critical):
        if fallback_duration > fallback_critical:
            add_error('transform.fallback_duration_limit_ms', 
                      f'TF2 降级警告阈值 ({fallback_duration}ms) 不应大于临界阈值 ({fallback_critical}ms)',
                      ValidationSeverity.ERROR)
    
    # Safety 加速度预热参数一致性检查
    accel_warmup_multiplier = get_config_value(config, 'safety.accel_warmup_margin_multiplier')
    accel_warmup_max = get_config_value(config, 'safety.accel_warmup_margin_max')
    if _is_numeric(accel_warmup_multiplier) and _is_numeric(accel_warmup_max):
        if accel_warmup_multiplier > accel_warmup_max:
            add_error('safety.accel_warmup_margin_multiplier', 
                      f'预热裕度倍数 ({accel_warmup_multiplier}) 大于上限 ({accel_warmup_max})',
                      ValidationSeverity.WARNING)
    
    # 跟踪质量权重总和检查
    tracking_weights = get_config_value(config, 'tracking.weights', {})
    if isinstance(tracking_weights, dict):
        lateral_w = tracking_weights.get('lateral', 0.4)
        longitudinal_w = tracking_weights.get('longitudinal', 0.4)
        heading_w = tracking_weights.get('heading', 0.2)
        if _is_numeric(lateral_w) and _is_numeric(longitudinal_w) and _is_numeric(heading_w):
            weight_sum = lateral_w + longitudinal_w + heading_w
            if abs(weight_sum - 1.0) > 0.01:
                add_error('tracking.weights',
                          f"权重总和 ({weight_sum:.2f}) 不等于 1.0，建议调整",
                          ValidationSeverity.WARNING)
    
    return errors


def validate_full_config(
    config: Dict[str, Any], 
    validation_rules: Dict[str, Tuple],
    raise_on_error: bool = True,
    allow_warnings: bool = True
) -> List[Tuple[str, str, ValidationSeverity]]:
    """
    完整配置验证（包括范围检查和逻辑一致性检查）
    
    Args:
        config: 配置字典
        validation_rules: 验证规则字典
        raise_on_error: 是否在发现 FATAL/ERROR 级别错误时抛出异常
        allow_warnings: 是否允许 WARNING 级别的问题（不影响启动）
    
    Returns:
        错误列表，每个元素为 (key_path, error_message, severity)
    
    Raises:
        ConfigValidationError: 当 raise_on_error=True 且发现 FATAL/ERROR 级别错误时
    """
    import logging
    _logger = logging.getLogger(__name__)
    
    # 范围检查 - 转换为带严重级别的格式
    range_errors = validate_config(config, validation_rules, raise_on_error=False)
    errors = [(key, msg, ValidationSeverity.ERROR) for key, msg in range_errors]
    
    # 逻辑一致性检查 - 已经带有严重级别
    logical_errors = validate_logical_consistency(config)
    errors.extend(logical_errors)
    
    # 分类错误
    fatal_errors = [(k, m, s) for k, m, s in errors if s == ValidationSeverity.FATAL]
    error_errors = [(k, m, s) for k, m, s in errors if s == ValidationSeverity.ERROR]
    warning_errors = [(k, m, s) for k, m, s in errors if s == ValidationSeverity.WARNING]
    
    # 记录警告
    for key, msg, _ in warning_errors:
        _logger.warning(f"配置警告 [{key}]: {msg}")
    
    # 处理致命错误和严重错误
    blocking_errors = fatal_errors + error_errors
    if blocking_errors and raise_on_error:
        # 致命错误始终阻止启动
        if fatal_errors:
            fatal_msgs = '\n'.join([f'  - [FATAL] {key}: {msg}' for key, msg, _ in fatal_errors])
            raise ConfigValidationError(f'配置存在致命错误，无法启动:\n{fatal_msgs}')
        
        # 严重错误默认阻止启动
        if error_errors:
            error_msgs = '\n'.join([f'  - [ERROR] {key}: {msg}' for key, msg, _ in error_errors])
            raise ConfigValidationError(f'配置验证失败:\n{error_msgs}')
    
    return errors


# 向后兼容：提供不带严重级别的简化接口
def validate_config_simple(
    config: Dict[str, Any], 
    validation_rules: Dict[str, Tuple],
    raise_on_error: bool = True
) -> List[Tuple[str, str]]:
    """
    简化的配置验证接口（向后兼容）
    
    返回不带严重级别的错误列表。
    """
    errors = validate_full_config(config, validation_rules, raise_on_error)
    return [(key, msg) for key, msg, _ in errors]
