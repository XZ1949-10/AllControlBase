"""MPC 配置

模型预测控制器的配置参数：
- 预测时域
- 代价函数权重
- 健康监控参数
- Fallback 求解器参数
- ACADOS 求解器参数
"""

MPC_CONFIG = {
    'horizon': 20,                # MPC 预测时域
    'horizon_degraded': 10,       # 降级时的预测时域
    'dt': 0.02,                   # 时间步长 (秒)
    
    # 代价函数权重
    'weights': {
        'position': 10.0,         # 位置跟踪权重 (Q_pos)
        'velocity': 1.0,          # 速度跟踪权重 (Q_vel)
        'heading': 5.0,           # 航向跟踪权重 (Q_heading)
        # 控制输入权重 (R 矩阵) - 惩罚控制输入的大小，实现平滑控制
        'control_accel': 0.1,     # 加速度控制权重 (用于 ax, ay, az)
        'control_alpha': 0.1,     # 角加速度控制权重 (用于 alpha)
        # 向后兼容别名 (已弃用，请使用 control_accel/control_alpha)
        'control_v': 0.1,         # [已弃用] 同 control_accel
        'control_omega': 0.1,     # [已弃用] 同 control_alpha
    },
    
    # MPC 健康监控参数
    'health_monitor': {
        'time_warning_thresh_ms': 8,       # 求解时间警告阈值 (ms)
        'time_critical_thresh_ms': 15,     # 求解时间临界阈值 (ms)
        'time_recovery_thresh_ms': 6,      # 求解时间恢复阈值 (ms)
        'condition_number_thresh': 1e8,    # 条件数警告阈值
        'condition_number_recovery': 1e5,  # 条件数恢复阈值
        'kkt_residual_thresh': 1e-3,       # KKT 残差阈值
        'consecutive_warning_limit': 3,    # 连续警告次数限制
        'consecutive_recovery_limit': 5,   # 连续恢复次数限制
        'recovery_multiplier': 2.0,        # 备选恢复条件的倍数
        'consecutive_good_for_decay': 2,   # 连续良好次数达到此值后开始衰减
        'timeout_decay_rate': 2,           # 超时计数衰减速率
    },
    
    # Fallback 求解器参数 (ACADOS 不可用时使用)
    'fallback': {
        'lookahead_steps': 3,              # 前瞻步数
        'heading_kp': 1.5,                 # 航向控制增益
        'max_curvature': 5.0,              # 最大曲率限制 (1/m)
        'min_distance_thresh': 0.1,        # 最小距离阈值 (m)
        'min_turn_speed': 0.1,             # 阿克曼车辆最小转向速度 (m/s)
        'default_speed_ratio': 0.5,        # 无 soft 速度时的默认速度比例
    },
    
    # ACADOS 求解器参数
    'solver': {
        'nlp_max_iter': 50,                # NLP 求解器最大迭代次数
        'qp_solver': 'PARTIAL_CONDENSING_HPIPM',  # QP 求解器类型
        'integrator_type': 'ERK',          # 积分器类型
        'nlp_solver_type': 'SQP_RTI',      # NLP 求解器类型
    },
}

# MPC 配置验证规则
MPC_VALIDATION_RULES = {
    'mpc.horizon': (1, 100, 'MPC 预测时域'),
    'mpc.horizon_degraded': (1, 100, 'MPC 降级预测时域'),
    'mpc.dt': (0.001, 1.0, 'MPC 时间步长 (秒)'),
    'mpc.fallback.min_distance_thresh': (0.001, 1.0, 'Fallback 最小距离阈值 (m)'),
    'mpc.fallback.min_turn_speed': (0.0, 1.0, 'Fallback 最小转向速度 (m/s)'),
    'mpc.fallback.default_speed_ratio': (0.0, 1.0, 'Fallback 默认速度比例'),
    # MPC 权重 (必须为非负数)
    'mpc.weights.position': (0.0, None, 'MPC 位置权重'),
    'mpc.weights.velocity': (0.0, None, 'MPC 速度权重'),
    'mpc.weights.heading': (0.0, None, 'MPC 航向权重'),
    'mpc.weights.control_accel': (0.0, None, 'MPC 加速度控制权重'),
    'mpc.weights.control_alpha': (0.0, None, 'MPC 角加速度控制权重'),
    # 向后兼容 (已弃用)
    'mpc.weights.control_v': (0.0, None, 'MPC 加速度控制权重 (已弃用，请用 control_accel)'),
    'mpc.weights.control_omega': (0.0, None, 'MPC 角加速度控制权重 (已弃用，请用 control_alpha)'),
    # ACADOS 求解器参数
    'mpc.solver.nlp_max_iter': (1, 1000, 'NLP 求解器最大迭代次数'),
    # MPC 健康监控参数
    'mpc.health_monitor.time_warning_thresh_ms': (0.1, 1000, '求解时间警告阈值 (ms)'),
    'mpc.health_monitor.time_critical_thresh_ms': (0.1, 1000, '求解时间临界阈值 (ms)'),
    'mpc.health_monitor.consecutive_warning_limit': (1, 100, '连续警告次数限制'),
}
