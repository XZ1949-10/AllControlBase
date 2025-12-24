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


def safe_float(val: Any, default: float = 0.0) -> float:
    """
    安全转换为 float，处理 None、nan、inf
    
    Args:
        val: 要转换的值
        default: 转换失败时的默认值
    
    Returns:
        转换后的 float 值
    """
    if val is None:
        return default
    try:
        f = float(val)
        if math.isnan(f) or math.isinf(f):
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
    
    首次调用行为:
        首次调用 should_publish() 时会立即返回 True，确保控制器启动后
        立即发布一次诊断信息。这通过将 _counter 初始化为 publish_rate - 1
        实现，使得首次 _counter += 1 后满足发布条件。
    
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
        
        Note:
            _counter 初始化为 publish_rate - 1，这样首次调用 should_publish()
            时 _counter 会递增到 publish_rate，满足发布条件，实现"首次立即发布"。
        """
        self._lock = threading.Lock()
        self._publish_rate = max(1, publish_rate)
        # 初始化为 publish_rate - 1，确保首次调用时立即发布
        # 首次调用: _counter += 1 -> _counter == publish_rate -> 满足发布条件
        self._counter = self._publish_rate - 1
        self._last_state: Optional[int] = None
    
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
            self._counter = self._publish_rate - 1
            self._last_state = None


def fill_diagnostics_msg(msg: Any, diag: Dict[str, Any], 
                         get_time_func: Optional[Callable] = None) -> None:
    """
    填充 DiagnosticsV2 消息的所有字段
    
    使用 safe_float 和 safe_float_list 确保数值安全。
    
    Args:
        msg: DiagnosticsV2 消息对象
        diag: 诊断数据字典
        get_time_func: 获取当前时间戳的函数 (返回 ROS Time 消息)
    """
    # Header
    if get_time_func is not None:
        msg.header.stamp = get_time_func()
    msg.header.frame_id = 'controller'
    
    # 基本状态
    msg.state = int(diag.get('state', 0))
    msg.mpc_success = bool(diag.get('mpc_success', False))
    msg.mpc_solve_time_ms = safe_float(diag.get('mpc_solve_time_ms', 0.0))
    msg.backup_active = bool(diag.get('backup_active', False))
    
    # MPC 健康状态
    mpc_health = diag.get('mpc_health', {})
    msg.mpc_health_kkt_residual = safe_float(mpc_health.get('kkt_residual', 0.0))
    msg.mpc_health_condition_number = safe_float(mpc_health.get('condition_number', 1.0))
    msg.mpc_health_consecutive_near_timeout = int(mpc_health.get('consecutive_near_timeout', 0))
    msg.mpc_health_degradation_warning = bool(mpc_health.get('degradation_warning', False))
    msg.mpc_health_can_recover = bool(mpc_health.get('can_recover', True))
    
    # 一致性指标
    consistency = diag.get('consistency', {})
    msg.consistency_curvature = safe_float(consistency.get('curvature', 0.0))
    msg.consistency_velocity_dir = safe_float(consistency.get('velocity_dir', 1.0))
    msg.consistency_temporal = safe_float(consistency.get('temporal', 1.0))
    msg.consistency_alpha_soft = safe_float(consistency.get('alpha_soft', 0.0))
    msg.consistency_data_valid = bool(consistency.get('data_valid', True))
    
    # 状态估计器健康
    estimator = diag.get('estimator_health', {})
    msg.estimator_covariance_norm = safe_float(estimator.get('covariance_norm', 0.0))
    msg.estimator_innovation_norm = safe_float(estimator.get('innovation_norm', 0.0))
    msg.estimator_slip_probability = safe_float(estimator.get('slip_probability', 0.0))
    msg.estimator_imu_drift_detected = bool(estimator.get('imu_drift_detected', False))
    msg.estimator_imu_available = bool(estimator.get('imu_available', True))
    
    # IMU bias - 使用安全转换
    msg.estimator_imu_bias = safe_float_list(
        estimator.get('imu_bias', [0.0, 0.0, 0.0]), 
        length=3, 
        default=0.0
    )
    
    # 跟踪误差
    tracking = diag.get('tracking', {})
    msg.tracking_lateral_error = safe_float(tracking.get('lateral_error', 0.0))
    msg.tracking_longitudinal_error = safe_float(tracking.get('longitudinal_error', 0.0))
    msg.tracking_heading_error = safe_float(tracking.get('heading_error', 0.0))
    msg.tracking_prediction_error = safe_float(tracking.get('prediction_error', 0.0))
    
    # 坐标变换状态
    transform = diag.get('transform', {})
    msg.transform_tf2_available = bool(transform.get('tf2_available', False))
    msg.transform_tf2_injected = bool(transform.get('tf2_injected', False))
    msg.transform_fallback_duration_ms = safe_float(transform.get('fallback_duration_ms', 0.0))
    msg.transform_accumulated_drift = safe_float(transform.get('accumulated_drift', 0.0))
    
    # 超时状态
    timeout = diag.get('timeout', {})
    msg.timeout_odom = bool(timeout.get('odom_timeout', False))
    msg.timeout_traj = bool(timeout.get('traj_timeout', False))
    msg.timeout_traj_grace_exceeded = bool(timeout.get('traj_grace_exceeded', False))
    msg.timeout_imu = bool(timeout.get('imu_timeout', False))
    msg.timeout_last_odom_age_ms = safe_float(timeout.get('last_odom_age_ms', 0.0))
    msg.timeout_last_traj_age_ms = safe_float(timeout.get('last_traj_age_ms', 0.0))
    msg.timeout_last_imu_age_ms = safe_float(timeout.get('last_imu_age_ms', 0.0))
    msg.timeout_in_startup_grace = bool(timeout.get('in_startup_grace', False))
    
    # 控制命令
    cmd = diag.get('cmd', {})
    msg.cmd_vx = safe_float(cmd.get('vx', 0.0))
    msg.cmd_vy = safe_float(cmd.get('vy', 0.0))
    msg.cmd_vz = safe_float(cmd.get('vz', 0.0))
    msg.cmd_omega = safe_float(cmd.get('omega', 0.0))
    msg.cmd_frame_id = str(cmd.get('frame_id', ''))
    
    msg.transition_progress = safe_float(diag.get('transition_progress', 0.0))
    
    # 错误信息
    msg.error_message = str(diag.get('error_message', ''))
    msg.consecutive_errors = int(diag.get('consecutive_errors', 0))
    
    # 紧急停止状态
    msg.emergency_stop = bool(diag.get('emergency_stop', False))
