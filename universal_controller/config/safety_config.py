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
    'velocity_margin': 1.1,     # 速度限制裕度 (安全检查时的速度上限倍数)
    'accel_margin': 1.5,        # 加速度限制裕度 (安全检查时的加速度上限倍数)
    
    # 注意: 低速保护参数统一在 CONSTRAINTS_CONFIG 中定义:
    # - omega_max_low: 低速时最大角速度
    # - v_low_thresh: 低速阈值
    
    # 加速度滤波参数
    'accel_filter_window': 3,        # 滑动窗口大小
    'accel_filter_alpha': 0.3,       # 低通滤波系数
    'accel_filter_warmup_alpha': 0.5,  # 滤波器预热期间的系数
    'accel_filter_warmup_period': 3,   # 滤波器预热期长度
    'accel_warmup_margin_multiplier': 1.5,  # 预热期间裕度倍数
    'accel_warmup_margin_max': 2.0,    # 预热期间裕度上限 (防止配置错误)
    'accel_absolute_max_multiplier': 2.0,   # 绝对加速度上限倍数 (硬性安全限制)
    'min_dt_for_accel': 0.001,       # 加速度计算的最小时间间隔 (秒)
    'max_dt_for_accel': 1.0,         # 加速度计算的最大时间间隔 (秒)
    
    # 状态机配置
    #
    # Alpha (α) 说明:
    # ===============
    # α 是一致性检查器计算的轨迹可信度，范围 [0, 1]
    # - α = 1.0: 完全信任 soft head 输出的速度
    # - α = 0.0: 完全使用几何计算的速度
    #
    # 当轨迹不包含速度信息时 (如纯位置轨迹):
    # - 一致性检查器无法计算有意义的 α 值
    # - 应设置 alpha_disable_thresh = 0.0 禁用 α 检查
    # - 此时控制器将完全使用几何计算的速度
    #
    # MPC 失败检测说明:
    # =================
    # 使用滑动窗口检测 MPC 是否持续失败:
    # - mpc_fail_window_size: 滑动窗口大小
    # - mpc_fail_thresh: 窗口内失败次数阈值
    # - mpc_fail_ratio_thresh: 失败率阈值
    # 当失败次数 >= mpc_fail_thresh 或失败率 >= mpc_fail_ratio_thresh 时，
    # 状态机会切换到 BACKUP_ACTIVE 状态
    #
    # MPC 恢复检测说明:
    # =================
    # 在 BACKUP_ACTIVE 状态下，检测 MPC 是否可以恢复:
    # - mpc_recovery_thresh: 连续成功次数阈值 (用于 MPC_DEGRADED → NORMAL)
    # - mpc_recovery_history_min: 最小历史记录数
    # - mpc_recovery_recent_count: 最近检查次数
    # - mpc_recovery_tolerance: 最近 N 次中允许的失败次数
    # - mpc_recovery_success_ratio: 整体成功率要求
    #
    'state_machine': {
        # Alpha 恢复参数
        'alpha_recovery_thresh': 5,        # α 恢复计数阈值
        'alpha_recovery_value': 0.3,       # α 恢复值
        'alpha_disable_thresh': 0.0,       # α 禁用阈值 (0 = 禁用此检查，适用于无速度信息的轨迹)
        
        # MPC 恢复参数
        'mpc_recovery_thresh': 5,          # MPC 恢复计数阈值
        
        # MPC 失败检测参数
        'mpc_fail_window_size': 10,        # MPC 失败检测滑动窗口大小
        'mpc_fail_thresh': 3,              # 窗口内失败次数阈值
        'mpc_fail_ratio_thresh': 0.5,      # 失败率阈值
        
        # MPC 恢复检测参数
        'mpc_recovery_history_min': 3,     # MPC 恢复检测最小历史记录数
        'mpc_recovery_recent_count': 5,    # MPC 恢复检测最近检查次数
        'mpc_recovery_tolerance': 0,       # MPC 恢复检测容错次数 (最近 N 次中允许的失败次数)
        'mpc_recovery_success_ratio': 0.8, # MPC 恢复检测成功率阈值 (整体成功率要求)
        
        # 状态超时监控参数
        # 用于检测系统是否长时间处于降级状态
        'degraded_state_timeout': 30.0,    # MPC_DEGRADED 状态超时 (秒)
        'backup_state_timeout': 60.0,      # BACKUP_ACTIVE 状态超时 (秒)
        'enable_state_timeout_stop': False,  # 是否启用状态超时自动停止 (默认仅告警)
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
    'safety.accel_warmup_margin_multiplier': (1.0, 5.0, '预热期间裕度倍数'),
    'safety.accel_warmup_margin_max': (1.0, 5.0, '预热期间裕度上限'),
    'safety.accel_absolute_max_multiplier': (1.0, 10.0, '绝对加速度上限倍数'),
    # 状态机配置
    'safety.state_machine.alpha_recovery_thresh': (1, 100, 'α 恢复计数阈值'),
    'safety.state_machine.alpha_recovery_value': (0.0, 1.0, 'α 恢复值'),
    'safety.state_machine.alpha_disable_thresh': (0.0, 1.0, 'α 禁用阈值'),
    'safety.state_machine.mpc_recovery_thresh': (1, 100, 'MPC 恢复计数阈值'),
    'safety.state_machine.mpc_fail_window_size': (1, 100, 'MPC 失败检测滑动窗口大小'),
    'safety.state_machine.mpc_fail_thresh': (1, 100, 'MPC 失败次数阈值'),
    'safety.state_machine.mpc_fail_ratio_thresh': (0.0, 1.0, 'MPC 失败率阈值'),
    'safety.state_machine.mpc_recovery_success_ratio': (0.0, 1.0, 'MPC 恢复成功率阈值'),
    'safety.state_machine.degraded_state_timeout': (0.0, 3600.0, 'MPC_DEGRADED 状态超时 (秒)'),
    'safety.state_machine.backup_state_timeout': (0.0, 3600.0, 'BACKUP_ACTIVE 状态超时 (秒)'),
}
