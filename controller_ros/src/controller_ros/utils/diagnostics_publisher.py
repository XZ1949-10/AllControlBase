"""
统一诊断发布工具

提供 ROS1 和 ROS2 共享的诊断发布逻辑，避免代码重复。

模块内容:
- safe_float, safe_float_list: 安全数值转换函数
- DiagnosticsThrottler: 诊断发布节流器（控制发布频率）
- fill_diagnostics_msg: 填充 DiagnosticsV2 消息
"""
from typing import Dict, Any, Optional, Callable
import math
import threading


def safe_float(val: Any, default: float = 0.0, preserve_nan: bool = False) -> float:
    """
    安全转换为 float，处理 None、nan、inf
    
    Args:
        val: 要转换的值
        default: 转换失败时的默认值
        preserve_nan: 是否保留 NaN 值（用于表示"无数据"语义）
    
    Returns:
        转换后的 float 值
    
    Note:
        当 preserve_nan=True 时，NaN 值会被保留而不是替换为 default。
        这对于需要区分"无数据"和"零值"的场景很重要，如 prediction_error。
        ROS float32 消息支持 NaN 值。
    """
    if val is None:
        return default
    try:
        f = float(val)
        if math.isnan(f):
            return f if preserve_nan else default
        if math.isinf(f):
            return default
        return f
    except (TypeError, ValueError):
        return default


def safe_float_list(vals: Any, length: int = 3, default: float = 0.0) -> list:
    """
    安全转换为 float 列表，处理 None、nan、inf 和各种数组类型
    
    Args:
        vals: 要转换的值（可以是 list, tuple, numpy array 或 None）
        length: 期望的列表长度
        default: 转换失败时的默认值
    
    Returns:
        转换后的 float 列表
    """
    if vals is None:
        return [default] * length
    
    try:
        # 处理 numpy array
        if hasattr(vals, 'tolist'):
            vals = vals.tolist()
        
        # 确保是可迭代的
        if not isinstance(vals, (list, tuple)):
            return [default] * length
        
        # 转换每个元素
        result = []
        for i in range(length):
            if i < len(vals):
                result.append(safe_float(vals[i], default))
            else:
                result.append(default)
        return result
    except (TypeError, IndexError):
        return [default] * length


class DiagnosticsThrottler:
    """
    诊断发布节流器
    
    控制诊断信息的发布频率，支持：
    - 按固定间隔发布（每 N 次控制循环发布一次）
    - 状态变化时立即发布
    - 强制发布
    - 首次调用立即发布
    
    线程安全性:
        本类是线程安全的。虽然在当前架构下（控制回调使用 MutuallyExclusiveCallbackGroup）
        不会出现并发调用，但为了代码健壮性和未来可能的架构变化，使用锁保护状态。
    
    供 ROS1 和 ROS2 节点共享使用。
    """
    
    def __init__(self, publish_rate: int = 10):
        """
        初始化诊断节流器
        
        Args:
            publish_rate: 发布间隔（每 N 次控制循环发布一次诊断）
        """
        self._lock = threading.Lock()
        self._publish_rate = max(1, publish_rate)
        self._counter = 0
        self._last_state: Optional[int] = None
        # 首次调用标志，确保启动后立即发布一次诊断
        self._first_call = True
    
    def should_publish(self, diag: Dict[str, Any], force: bool = False) -> bool:
        """
        判断是否应该发布诊断信息
        
        Args:
            diag: 诊断数据字典
            force: 是否强制发布
        
        Returns:
            是否应该发布
        """
        # 获取当前状态
        current_state = diag.get('state', 0)
        
        with self._lock:
            # 首次调用立即发布
            if self._first_call:
                self._first_call = False
                self._last_state = current_state
                self._counter = 0
                return True
            
            # 检测状态变化
            state_changed = (
                self._last_state is not None and 
                current_state != self._last_state
            )
            self._last_state = current_state
            
            # 更新计数器
            self._counter += 1
            
            # 判断是否发布
            if force or state_changed or self._counter >= self._publish_rate:
                self._counter = 0
                return True
            
            return False
    
    def get_current_state(self) -> Optional[int]:
        """获取当前状态（用于状态话题发布）"""
        with self._lock:
            return self._last_state
    
    def reset(self) -> None:
        """重置状态"""
        with self._lock:
            self._counter = 0
            self._last_state = None
            self._first_call = True


def _safe_set_attr(msg: Any, attr: str, value: Any) -> bool:
    """
    安全设置消息属性
    
    Args:
        msg: ROS 消息对象
        attr: 属性名
        value: 要设置的值
    
    Returns:
        是否设置成功
    """
    try:
        setattr(msg, attr, value)
        return True
    except (AttributeError, TypeError):
        return False


def fill_diagnostics_msg(msg: Any, diag: Dict[str, Any], 
                         get_time_func: Optional[Callable] = None) -> None:
    """
    填充 DiagnosticsV2 消息的所有字段
    
    使用 safe_float 和 safe_float_list 确保数值安全。
    对每个字段的赋值进行异常保护，确保单个字段失败不影响其他字段。
    
    Args:
        msg: DiagnosticsV2 消息对象
        diag: 诊断数据字典
        get_time_func: 获取当前时间戳的函数 (返回 ROS Time 消息)
    
    Note:
        如果消息定义与代码不匹配（如字段缺失或类型变化），
        会静默跳过该字段，不会抛出异常。
    """
    # Header
    try:
        if get_time_func is not None:
            msg.header.stamp = get_time_func()
        msg.header.frame_id = 'controller'
    except AttributeError:
        pass  # 消息可能没有 header
    
    # 基本状态
    _safe_set_attr(msg, 'state', int(diag.get('state', 0)))
    _safe_set_attr(msg, 'mpc_success', bool(diag.get('mpc_success', False)))
    _safe_set_attr(msg, 'mpc_solve_time_ms', safe_float(diag.get('mpc_solve_time_ms', 0.0)))
    _safe_set_attr(msg, 'backup_active', bool(diag.get('backup_active', False)))
    
    # MPC 健康状态
    mpc_health = diag.get('mpc_health', {})
    if not isinstance(mpc_health, dict):
        mpc_health = {}
    _safe_set_attr(msg, 'mpc_health_kkt_residual', safe_float(mpc_health.get('kkt_residual', 0.0)))
    _safe_set_attr(msg, 'mpc_health_condition_number', safe_float(mpc_health.get('condition_number', 1.0)))
    _safe_set_attr(msg, 'mpc_health_consecutive_near_timeout', int(mpc_health.get('consecutive_near_timeout', 0)))
    _safe_set_attr(msg, 'mpc_health_degradation_warning', bool(mpc_health.get('degradation_warning', False)))
    _safe_set_attr(msg, 'mpc_health_can_recover', bool(mpc_health.get('can_recover', True)))
    
    # 一致性指标
    consistency = diag.get('consistency', {})
    if not isinstance(consistency, dict):
        consistency = {}
    _safe_set_attr(msg, 'consistency_curvature', safe_float(consistency.get('curvature', 0.0)))
    _safe_set_attr(msg, 'consistency_velocity_dir', safe_float(consistency.get('velocity_dir', 1.0)))
    _safe_set_attr(msg, 'consistency_temporal', safe_float(consistency.get('temporal', 1.0)))
    _safe_set_attr(msg, 'consistency_alpha_soft', safe_float(consistency.get('alpha_soft', 0.0)))
    _safe_set_attr(msg, 'consistency_data_valid', bool(consistency.get('data_valid', True)))
    
    # 状态估计器健康
    estimator = diag.get('estimator_health', {})
    if not isinstance(estimator, dict):
        estimator = {}
    _safe_set_attr(msg, 'estimator_covariance_norm', safe_float(estimator.get('covariance_norm', 0.0)))
    _safe_set_attr(msg, 'estimator_innovation_norm', safe_float(estimator.get('innovation_norm', 0.0)))
    _safe_set_attr(msg, 'estimator_slip_probability', safe_float(estimator.get('slip_probability', 0.0)))
    _safe_set_attr(msg, 'estimator_imu_drift_detected', bool(estimator.get('imu_drift_detected', False)))
    _safe_set_attr(msg, 'estimator_imu_available', bool(estimator.get('imu_available', False)))
    
    # IMU bias - 使用安全转换
    _safe_set_attr(msg, 'estimator_imu_bias', safe_float_list(
        estimator.get('imu_bias', [0.0, 0.0, 0.0]), 
        length=3, 
        default=0.0
    ))
    
    # 跟踪误差
    tracking = diag.get('tracking', {})
    if not isinstance(tracking, dict):
        tracking = {}
    _safe_set_attr(msg, 'tracking_lateral_error', safe_float(tracking.get('lateral_error', 0.0)))
    _safe_set_attr(msg, 'tracking_longitudinal_error', safe_float(tracking.get('longitudinal_error', 0.0)))
    _safe_set_attr(msg, 'tracking_heading_error', safe_float(tracking.get('heading_error', 0.0)))
    # prediction_error 使用 NaN 表示"无数据"，需要保留 NaN 语义
    _safe_set_attr(msg, 'tracking_prediction_error', safe_float(
        tracking.get('prediction_error', float('nan')), 
        default=float('nan'), 
        preserve_nan=True
    ))
    
    # 坐标变换状态
    transform = diag.get('transform', {})
    if not isinstance(transform, dict):
        transform = {}
    _safe_set_attr(msg, 'transform_tf2_available', bool(transform.get('tf2_available', False)))
    _safe_set_attr(msg, 'transform_tf2_injected', bool(transform.get('tf2_injected', False)))
    _safe_set_attr(msg, 'transform_fallback_duration_ms', safe_float(transform.get('fallback_duration_ms', 0.0)))
    _safe_set_attr(msg, 'transform_accumulated_drift', safe_float(transform.get('accumulated_drift', 0.0)))
    _safe_set_attr(msg, 'transform_source_frame', str(transform.get('source_frame', '')))
    _safe_set_attr(msg, 'transform_target_frame', str(transform.get('target_frame', '')))
    _safe_set_attr(msg, 'transform_error_message', str(transform.get('error_message', '')))
    
    # 超时状态
    timeout = diag.get('timeout', {})
    if not isinstance(timeout, dict):
        timeout = {}
    _safe_set_attr(msg, 'timeout_odom', bool(timeout.get('odom_timeout', False)))
    _safe_set_attr(msg, 'timeout_traj', bool(timeout.get('traj_timeout', False)))
    _safe_set_attr(msg, 'timeout_traj_grace_exceeded', bool(timeout.get('traj_grace_exceeded', False)))
    _safe_set_attr(msg, 'timeout_imu', bool(timeout.get('imu_timeout', False)))
    _safe_set_attr(msg, 'timeout_last_odom_age_ms', safe_float(timeout.get('last_odom_age_ms', 0.0)))
    _safe_set_attr(msg, 'timeout_last_traj_age_ms', safe_float(timeout.get('last_traj_age_ms', 0.0)))
    _safe_set_attr(msg, 'timeout_last_imu_age_ms', safe_float(timeout.get('last_imu_age_ms', 0.0)))
    _safe_set_attr(msg, 'timeout_in_startup_grace', bool(timeout.get('in_startup_grace', False)))
    
    # 控制命令
    cmd = diag.get('cmd', {})
    if not isinstance(cmd, dict):
        cmd = {}
    _safe_set_attr(msg, 'cmd_vx', safe_float(cmd.get('vx', 0.0)))
    _safe_set_attr(msg, 'cmd_vy', safe_float(cmd.get('vy', 0.0)))
    _safe_set_attr(msg, 'cmd_vz', safe_float(cmd.get('vz', 0.0)))
    _safe_set_attr(msg, 'cmd_omega', safe_float(cmd.get('omega', 0.0)))
    _safe_set_attr(msg, 'cmd_frame_id', str(cmd.get('frame_id', '')))
    
    _safe_set_attr(msg, 'transition_progress', safe_float(diag.get('transition_progress', 0.0)))
    
    # 错误信息
    _safe_set_attr(msg, 'error_message', str(diag.get('error_message', '')))
    _safe_set_attr(msg, 'consecutive_errors', int(diag.get('consecutive_errors', 0)))
    
    # 安全状态
    _safe_set_attr(msg, 'safety_check_passed', bool(diag.get('safety_check_passed', True)))
    _safe_set_attr(msg, 'emergency_stop', bool(diag.get('emergency_stop', False)))
