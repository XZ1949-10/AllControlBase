"""安全配置

包含安全相关的所有配置：
- 速度/加速度约束
- 安全监控参数
- 状态机参数
- 加速度滤波参数
"""

# 运动约束配置
CONSTRAINTS_CONFIG = {
    'v_max': 2.0,           # 最大速度 (m/s)
    'v_min': 0.0,           # 最小速度 (m/s)
    'omega_max': 2.0,       # 最大角速度 (rad/s)
    'omega_max_low': 1.0,   # 低速时最大角速度 (rad/s)
    'v_low_thresh': 0.1,    # 低速阈值 (m/s)
    'a_max': 1.5,           # 最大加速度 (m/s²)
    'az_max': 1.0,          # 最大垂直加速度 (m/s²)
    'alpha_max': 3.0,       # 最大角加速度 (rad/s²)
    'vx_max': 1.5,          # X 方向最大速度 (m/s)
    'vx_min': -1.5,         # X 方向最小速度 (m/s)
    'vy_max': 1.5,          # Y 方向最大速度 (m/s)
    'vy_min': -1.5,         # Y 方向最小速度 (m/s)
    'vz_max': 2.0,          # Z 方向最大速度 (m/s)
}

# 安全监控配置
SAFETY_CONFIG = {
    'v_stop_thresh': 0.05,      # 停车速度阈值 (m/s)
    'vz_stop_thresh': 0.1,      # 垂直停车速度阈值 (m/s)
    'stopping_timeout': 5.0,    # 停车超时时间 (秒)
    'emergency_decel': 3.0,     # 紧急减速度 (m/s²)
    'velocity_margin': 1.1,     # 速度限制裕度
    'accel_margin': 1.5,        # 加速度限制裕度
    
    # 加速度滤波参数
    'accel_filter_window': 3,        # 滑动窗口大小
    'accel_filter_alpha': 0.3,       # 低通滤波系数
    'accel_filter_warmup_alpha': 0.5,  # 滤波器预热期间的系数
    'accel_filter_warmup_period': 3,   # 滤波器预热期长度
    'min_dt_for_accel': 0.001,       # 加速度计算的最小时间间隔 (秒)
    'max_dt_for_accel': 1.0,         # 加速度计算的最大时间间隔 (秒)
    
    # 状态机配置
    'state_machine': {
        'alpha_recovery_thresh': 5,        # α 恢复计数阈值
        'alpha_recovery_value': 0.3,       # α 恢复值
        'alpha_disable_thresh': 0.0,       # α 禁用阈值 (0 = 禁用此检查，适用于无速度信息的轨迹)
        'mpc_recovery_thresh': 5,          # MPC 恢复计数阈值
        'mpc_fail_window_size': 10,        # MPC 失败检测滑动窗口大小
        'mpc_fail_thresh': 3,              # 窗口内失败次数阈值
        'mpc_fail_ratio_thresh': 0.5,      # 失败率阈值
        'mpc_recovery_history_min': 3,     # MPC 恢复检测最小历史记录数
        'mpc_recovery_recent_count': 5,    # MPC 恢复检测最近检查次数
        'mpc_recovery_tolerance': 0,       # MPC 恢复检测容错次数 (最近 N 次中允许的失败次数)
        'mpc_recovery_success_ratio': 0.8, # MPC 恢复检测成功率阈值 (整体成功率要求)
    },
}

# 安全配置验证规则
SAFETY_VALIDATION_RULES = {
    # 约束配置
    'constraints.v_max': (0.01, 100.0, '最大速度 (m/s)'),
    'constraints.omega_max': (0.01, 50.0, '最大角速度 (rad/s)'),
    'constraints.a_max': (0.01, 50.0, '最大加速度 (m/s²)'),
    'constraints.alpha_max': (0.01, 100.0, '最大角加速度 (rad/s²)'),
    # 安全配置
    'safety.v_stop_thresh': (0.0, 1.0, '停止速度阈值 (m/s)'),
    'safety.stopping_timeout': (0.1, 60.0, '停止超时 (秒)'),
    'safety.velocity_margin': (1.0, 2.0, '速度裕度'),
    'safety.accel_margin': (1.0, 3.0, '加速度裕度'),
    'safety.accel_filter_window': (1, 20, '加速度滤波窗口大小'),
    'safety.accel_filter_alpha': (0.0, 1.0, '加速度滤波系数'),
    'safety.accel_filter_warmup_period': (1, 20, '滤波器预热期长度'),
    # 状态机配置
    'safety.state_machine.mpc_recovery_success_ratio': (0.0, 1.0, 'MPC 恢复成功率阈值'),
}
