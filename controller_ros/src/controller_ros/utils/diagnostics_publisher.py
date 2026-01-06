"""
统一诊断发布工具

提供 ROS1 和 ROS2 共享的诊断发布逻辑，避免代码重复。

性能优化说明 (v2):
- 移除了基于反射和字符串拼接的动态填充逻辑，改为显式赋值。
- 针对 Dict 和 Object 分别提供优化路径。
- 减少了函数调用开销。
"""
from typing import Dict, Any, Optional, Callable
import math
import threading


def safe_float(val: Any, default: float = 0.0, preserve_nan: bool = False) -> float:
    """安全转换为 float，处理 None、nan、inf (High Performance Optimized)"""
    if val is None:
        return default
    
    # Fast path for common types
    if isinstance(val, (float, int)):
        if not preserve_nan and math.isnan(val):
            return default
        if math.isinf(val):
            return default
        return float(val)
        
    # Fallback for other types (str, etc)
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
    """安全转换为 float 列表"""
    if vals is None:
        return [default] * length
    try:
        if hasattr(vals, 'tolist'):
            vals = vals.tolist()
        if not isinstance(vals, (list, tuple)):
            return [default] * length
        
        result = [default] * length
        for i, v in enumerate(vals[:length]):
            result[i] = safe_float(v, default)
        return result
    except Exception:
        return [default] * length


class DiagnosticsThrottler:
    """诊断发布节流器 (无需修改)"""
    def __init__(self, publish_rate: int = 10):
        self._lock = threading.Lock()
        self._publish_rate = max(1, publish_rate)
        self._counter = 0
        self._last_state: Optional[int] = None
        self._first_call = True
    
    def should_publish(self, diag: Dict[str, Any], force: bool = False) -> bool:
        if isinstance(diag, dict):
            current_state = diag.get('state', 0)
        else:
            current_state = getattr(diag, 'state', 0)
        
        with self._lock:
            if self._first_call:
                self._first_call = False
                self._last_state = current_state
                self._counter = 0
                return True
            
            state_changed = (self._last_state is not None and current_state != self._last_state)
            self._last_state = current_state
            self._counter += 1
            
            if force or state_changed or self._counter >= self._publish_rate:
                self._counter = 0
                return True
            return False
    
    def get_current_state(self) -> Optional[int]:
        with self._lock:
            return self._last_state
    
    def reset(self) -> None:
        with self._lock:
            self._counter = 0
            self._last_state = None
            self._first_call = True


def fill_diagnostics_msg(msg: Any, diag: Any, get_time_func: Optional[Callable] = None) -> None:
    """
    填充 DiagnosticsV2 消息 (高性能显式赋值版)
    """
    # Header
    if get_time_func is not None:
        try:
            msg.header.stamp = get_time_func()
        except AttributeError:
            pass
    try:
        msg.header.frame_id = 'controller'
    except AttributeError:
        pass

    # 分发到不同类型的处理逻辑
    if isinstance(diag, dict):
        _fill_from_dict(msg, diag)
    else:
        _fill_from_object(msg, diag)


def _fill_from_dict(msg: Any, d: Dict[str, Any]) -> None:
    """从字典填充 (主路径，极致性能)"""
    # 根级别字段
    msg.state = int(d.get('state', 0))
    msg.mpc_success = bool(d.get('mpc_success', False))
    msg.mpc_solve_time_ms = safe_float(d.get('mpc_solve_time_ms'))
    msg.backup_active = bool(d.get('backup_active', False))
    msg.transition_progress = safe_float(d.get('transition_progress'))
    msg.error_message = str(d.get('error_message', ''))
    msg.consecutive_errors = int(d.get('consecutive_errors', 0))
    msg.safety_check_passed = bool(d.get('safety_check_passed', True))
    msg.emergency_stop = bool(d.get('emergency_stop', False))

    # MPC Health
    mpc = d.get('mpc_health', {})
    msg.mpc_health_kkt_residual = safe_float(mpc.get('kkt_residual'))
    msg.mpc_health_condition_number = safe_float(mpc.get('condition_number'), default=1.0)
    msg.mpc_health_consecutive_near_timeout = int(mpc.get('consecutive_near_timeout', 0))
    msg.mpc_health_degradation_warning = bool(mpc.get('degradation_warning', False))
    msg.mpc_health_can_recover = bool(mpc.get('can_recover', True))

    # Consistency
    cons = d.get('consistency', {})
    msg.consistency_curvature = safe_float(cons.get('curvature'))
    msg.consistency_velocity_dir = safe_float(cons.get('velocity_dir'), default=1.0)
    msg.consistency_temporal = safe_float(cons.get('temporal'), default=1.0)
    msg.consistency_alpha_soft = safe_float(cons.get('alpha_soft'))
    msg.consistency_data_valid = bool(cons.get('data_valid', True))

    # Estimator Health
    est = d.get('estimator_health', {})
    msg.estimator_covariance_norm = safe_float(est.get('covariance_norm'))
    msg.estimator_innovation_norm = safe_float(est.get('innovation_norm'))
    msg.estimator_slip_probability = safe_float(est.get('slip_probability'))
    msg.estimator_imu_drift_detected = bool(est.get('imu_drift_detected', False))
    msg.estimator_imu_available = bool(est.get('imu_available', False))
    msg.estimator_imu_bias = safe_float_list(est.get('imu_bias'), 3)

    # Tracking
    trk = d.get('tracking', {})
    msg.tracking_lateral_error = safe_float(trk.get('lateral_error'))
    msg.tracking_longitudinal_error = safe_float(trk.get('longitudinal_error'))
    msg.tracking_heading_error = safe_float(trk.get('heading_error'))
    msg.tracking_prediction_error = safe_float(trk.get('prediction_error', float('nan')), preserve_nan=True)
    # New fields if they exist
    msg.tracking_quality_score = safe_float(trk.get('quality_score'))
    msg.tracking_quality_rating = str(trk.get('quality_rating', 'unknown'))

    # Transform
    tf = d.get('transform', {})
    msg.transform_tf2_available = bool(tf.get('tf2_available', False))
    msg.transform_tf2_injected = bool(tf.get('tf2_injected', False))
    msg.transform_fallback_duration_ms = safe_float(tf.get('fallback_duration_ms'))
    msg.transform_accumulated_drift = safe_float(tf.get('accumulated_drift'))
    msg.transform_source_frame = str(tf.get('source_frame', ''))
    msg.transform_target_frame = str(tf.get('target_frame', ''))
    msg.transform_error_message = str(tf.get('error_message', ''))

    # Timeout (Mapping adjustments)
    to = d.get('timeout', {})
    msg.timeout_odom = bool(to.get('odom_timeout', False))
    msg.timeout_traj = bool(to.get('traj_timeout', False))
    msg.timeout_traj_grace_exceeded = bool(to.get('traj_grace_exceeded', False))
    msg.timeout_imu = bool(to.get('imu_timeout', False))
    msg.timeout_last_odom_age_ms = safe_float(to.get('last_odom_age_ms'))
    msg.timeout_last_traj_age_ms = safe_float(to.get('last_traj_age_ms'))
    msg.timeout_last_imu_age_ms = safe_float(to.get('last_imu_age_ms'))
    msg.timeout_in_startup_grace = bool(to.get('in_startup_grace', False))

    # Control Command
    cmd = d.get('cmd', {})
    msg.cmd_vx = safe_float(cmd.get('vx'))
    msg.cmd_vy = safe_float(cmd.get('vy'))
    msg.cmd_vz = safe_float(cmd.get('vz'))
    msg.cmd_omega = safe_float(cmd.get('omega'))
    msg.cmd_frame_id = str(cmd.get('frame_id', ''))


# 记录已验证的类型，避免重复检查
_VALIDATED_TYPES = set()


def _fill_from_object(msg: Any, o: Any) -> None:
    """从对象填充 (兼容路径，使用 getattr)"""
    # 1. 结构验证 (仅首次运行时检查)
    # 这为了防止 silent failure: 如果底层字段改名了，getattr 会默默返回 0
    global _VALIDATED_TYPES
    obj_type = type(o)
    if obj_type not in _VALIDATED_TYPES:
        _validate_object_structure(o)
        _VALIDATED_TYPES.add(obj_type)

    # 根级别
    msg.state = int(getattr(o, 'state', 0))
    msg.mpc_success = bool(getattr(o, 'mpc_success', False))
    msg.mpc_solve_time_ms = safe_float(getattr(o, 'mpc_solve_time_ms', 0.0))
    msg.backup_active = bool(getattr(o, 'backup_active', False))
    msg.transition_progress = safe_float(getattr(o, 'transition_progress', 0.0))
    msg.error_message = str(getattr(o, 'error_message', ''))
    msg.consecutive_errors = int(getattr(o, 'consecutive_errors', 0))
    msg.safety_check_passed = bool(getattr(o, 'safety_check_passed', True))
    msg.emergency_stop = bool(getattr(o, 'emergency_stop', False))
    
    # 假设对象是平铺的 (DiagnosticsV2 like object)
    # 如果对象结构与 DiagnosticsV2 一致，直接按字段名获取
    # 这里为了安全起见，我们只能假设它要么是平铺的，要么有子对象
    # 鉴于当前代码库中 DiagnosticsV2 已经是平铺的，我们直接尝试 getattr(o, 'field_name')
    
    # MPC Health
    msg.mpc_health_kkt_residual = safe_float(getattr(o, 'mpc_health_kkt_residual', 0.0))
    msg.mpc_health_condition_number = safe_float(getattr(o, 'mpc_health_condition_number', 1.0))
    msg.mpc_health_consecutive_near_timeout = int(getattr(o, 'mpc_health_consecutive_near_timeout', 0))
    msg.mpc_health_degradation_warning = bool(getattr(o, 'mpc_health_degradation_warning', False))
    msg.mpc_health_can_recover = bool(getattr(o, 'mpc_health_can_recover', True))

    # Consistency
    msg.consistency_curvature = safe_float(getattr(o, 'consistency_curvature', 0.0))
    msg.consistency_velocity_dir = safe_float(getattr(o, 'consistency_velocity_dir', 1.0))
    msg.consistency_temporal = safe_float(getattr(o, 'consistency_temporal', 1.0))
    msg.consistency_alpha_soft = safe_float(getattr(o, 'consistency_alpha_soft', 0.0))
    msg.consistency_data_valid = bool(getattr(o, 'consistency_data_valid', True))
    
    # Estimator Health
    msg.estimator_covariance_norm = safe_float(getattr(o, 'estimator_covariance_norm', 0.0))
    msg.estimator_innovation_norm = safe_float(getattr(o, 'estimator_innovation_norm', 0.0))
    msg.estimator_slip_probability = safe_float(getattr(o, 'estimator_slip_probability', 0.0))
    msg.estimator_imu_drift_detected = bool(getattr(o, 'estimator_imu_drift_detected', False))
    msg.estimator_imu_available = bool(getattr(o, 'estimator_imu_available', False))
    msg.estimator_imu_bias = safe_float_list(getattr(o, 'estimator_imu_bias', None), 3)

    # Tracking
    msg.tracking_lateral_error = safe_float(getattr(o, 'tracking_lateral_error', 0.0))
    msg.tracking_longitudinal_error = safe_float(getattr(o, 'tracking_longitudinal_error', 0.0))
    msg.tracking_heading_error = safe_float(getattr(o, 'tracking_heading_error', 0.0))
    msg.tracking_prediction_error = safe_float(getattr(o, 'tracking_prediction_error', float('nan')), preserve_nan=True)
    msg.tracking_quality_score = safe_float(getattr(o, 'tracking_quality_score', 0.0))
    msg.tracking_quality_rating = str(getattr(o, 'tracking_quality_rating', 'unknown'))

    # Transform
    msg.transform_tf2_available = bool(getattr(o, 'transform_tf2_available', False))
    msg.transform_tf2_injected = bool(getattr(o, 'transform_tf2_injected', False))
    msg.transform_fallback_duration_ms = safe_float(getattr(o, 'transform_fallback_duration_ms', 0.0))
    msg.transform_accumulated_drift = safe_float(getattr(o, 'transform_accumulated_drift', 0.0))
    msg.transform_source_frame = str(getattr(o, 'transform_source_frame', ''))
    msg.transform_target_frame = str(getattr(o, 'transform_target_frame', ''))
    msg.transform_error_message = str(getattr(o, 'transform_error_message', ''))

    # Timeout
    msg.timeout_odom = bool(getattr(o, 'timeout_odom', False))
    msg.timeout_traj = bool(getattr(o, 'timeout_traj', False))
    msg.timeout_traj_grace_exceeded = bool(getattr(o, 'timeout_traj_grace_exceeded', False))
    msg.timeout_imu = bool(getattr(o, 'timeout_imu', False))
    msg.timeout_last_odom_age_ms = safe_float(getattr(o, 'timeout_last_odom_age_ms', 0.0))
    msg.timeout_last_traj_age_ms = safe_float(getattr(o, 'timeout_last_traj_age_ms', 0.0))
    msg.timeout_last_imu_age_ms = safe_float(getattr(o, 'timeout_last_imu_age_ms', 0.0))
    msg.timeout_in_startup_grace = bool(getattr(o, 'timeout_in_startup_grace', False))

    # Control Command
    msg.cmd_vx = safe_float(getattr(o, 'cmd_vx', 0.0))
    msg.cmd_vy = safe_float(getattr(o, 'cmd_vy', 0.0))
    msg.cmd_vz = safe_float(getattr(o, 'cmd_vz', 0.0))
    msg.cmd_omega = safe_float(getattr(o, 'cmd_omega', 0.0))
    msg.cmd_frame_id = str(getattr(o, 'cmd_frame_id', ''))


def _validate_object_structure(o: Any) -> None:
    """
    验证对象结构，确保关键字段存在
    
    这是为了解决 getattr(o, 'field', default) 掩盖字段重命名/丢失的问题。
    """
    import logging
    
    # 定义关键字段集合 (采样检查)
    critical_fields = [
        'state', 'mpc_success', 
        'mpc_health_kkt_residual',
        'consistency_curvature',
        'estimator_covariance_norm',
        'tracking_lateral_error'
    ]
    
    missing = []
    for field in critical_fields:
        if not hasattr(o, field):
            missing.append(field)
            
    if missing:
        logging.getLogger(__name__).warning(
            f"DiagnosticsV2 Structure Mismatch: Object {type(o).__name__} is missing fields: {missing}. "
            f"Diagnostics publisher will default these to 0/False, which may hide bugs. "
            f"Please ensure Universal Controller data types match DiagnosticsV2 definition."
        )

