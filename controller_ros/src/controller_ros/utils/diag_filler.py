"""
诊断消息填充工具

提供统一的诊断消息填充逻辑，避免代码重复。
"""
from typing import Dict, Any, Optional, Callable


def fill_diagnostics_msg(msg: Any, diag: Dict[str, Any], 
                         get_time_func: Optional[Callable] = None) -> None:
    """
    填充 DiagnosticsV2 消息的所有字段
    
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
    msg.state = diag.get('state', 0)
    msg.mpc_success = diag.get('mpc_success', False)
    msg.mpc_solve_time_ms = float(diag.get('mpc_solve_time_ms', 0.0))
    msg.backup_active = diag.get('backup_active', False)
    
    # MPC 健康状态
    mpc_health = diag.get('mpc_health', {})
    msg.mpc_health_kkt_residual = float(mpc_health.get('kkt_residual', 0.0))
    msg.mpc_health_condition_number = float(mpc_health.get('condition_number', 1.0))
    msg.mpc_health_consecutive_near_timeout = int(mpc_health.get('consecutive_near_timeout', 0))
    msg.mpc_health_degradation_warning = mpc_health.get('degradation_warning', False)
    msg.mpc_health_can_recover = mpc_health.get('can_recover', True)
    
    # 一致性指标
    consistency = diag.get('consistency', {})
    msg.consistency_curvature = float(consistency.get('curvature', 0.0))
    msg.consistency_velocity_dir = float(consistency.get('velocity_dir', 1.0))
    msg.consistency_temporal = float(consistency.get('temporal', 1.0))
    msg.consistency_alpha_soft = float(consistency.get('alpha_soft', 0.0))
    msg.consistency_data_valid = consistency.get('data_valid', True)
    
    # 状态估计器健康
    estimator = diag.get('estimator_health', {})
    msg.estimator_covariance_norm = float(estimator.get('covariance_norm', 0.0))
    msg.estimator_innovation_norm = float(estimator.get('innovation_norm', 0.0))
    msg.estimator_slip_probability = float(estimator.get('slip_probability', 0.0))
    msg.estimator_imu_drift_detected = estimator.get('imu_drift_detected', False)
    msg.estimator_imu_available = estimator.get('imu_available', True)
    
    # IMU bias - 支持 list, tuple 和 numpy array，以及 None 值
    imu_bias = estimator.get('imu_bias', [0.0, 0.0, 0.0])
    try:
        # 处理 None 值
        if imu_bias is None:
            msg.estimator_imu_bias = [0.0, 0.0, 0.0]
        else:
            # 尝试转换为列表 (支持 numpy array)
            if hasattr(imu_bias, 'tolist'):
                imu_bias = imu_bias.tolist()
            if isinstance(imu_bias, (list, tuple)) and len(imu_bias) >= 3:
                # 安全转换每个元素，处理可能的 None 值
                bias_values = []
                for i in range(3):
                    val = imu_bias[i]
                    bias_values.append(float(val) if val is not None else 0.0)
                msg.estimator_imu_bias = bias_values
            else:
                msg.estimator_imu_bias = [0.0, 0.0, 0.0]
    except (TypeError, ValueError, IndexError):
        msg.estimator_imu_bias = [0.0, 0.0, 0.0]
    
    # 跟踪误差
    tracking = diag.get('tracking', {})
    msg.tracking_lateral_error = float(tracking.get('lateral_error', 0.0))
    msg.tracking_longitudinal_error = float(tracking.get('longitudinal_error', 0.0))
    msg.tracking_heading_error = float(tracking.get('heading_error', 0.0))
    msg.tracking_prediction_error = float(tracking.get('prediction_error', 0.0))
    
    # 坐标变换状态
    transform = diag.get('transform', {})
    msg.transform_tf2_available = transform.get('tf2_available', False)
    msg.transform_fallback_duration_ms = float(transform.get('fallback_duration_ms', 0.0))
    msg.transform_accumulated_drift = float(transform.get('accumulated_drift', 0.0))
    
    # 超时状态
    timeout = diag.get('timeout', {})
    msg.timeout_odom = timeout.get('odom_timeout', False)
    msg.timeout_traj = timeout.get('traj_timeout', False)
    msg.timeout_traj_grace_exceeded = timeout.get('traj_grace_exceeded', False)
    msg.timeout_imu = timeout.get('imu_timeout', False)
    msg.timeout_last_odom_age_ms = float(timeout.get('last_odom_age_ms', 0.0))
    msg.timeout_last_traj_age_ms = float(timeout.get('last_traj_age_ms', 0.0))
    msg.timeout_last_imu_age_ms = float(timeout.get('last_imu_age_ms', 0.0))
    msg.timeout_in_startup_grace = timeout.get('in_startup_grace', False)
    
    # 控制命令
    cmd = diag.get('cmd', {})
    msg.cmd_vx = float(cmd.get('vx', 0.0))
    msg.cmd_vy = float(cmd.get('vy', 0.0))
    msg.cmd_vz = float(cmd.get('vz', 0.0))
    msg.cmd_omega = float(cmd.get('omega', 0.0))
    msg.cmd_frame_id = cmd.get('frame_id', '')
    
    msg.transition_progress = float(diag.get('transition_progress', 0.0))
