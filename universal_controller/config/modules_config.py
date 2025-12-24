"""其他模块配置

包含以下模块的配置：
- 一致性检查器
- 坐标变换器
- 平滑过渡
- 备份控制器 (Pure Pursuit)

坐标系说明 (不需要建图/定位):
==================================

    base_link (机体坐标系)              odom (里程计坐标系)
    ┌───────────────┐                   ┌─────────────────────┐
    │       ↑ X     │                   │                     │
    │       │       │    坐标变换        │    机器人轨迹       │
    │    ←──┼──→    │  ───────────→     │    ○──○──○──○       │
    │     Y │       │  base_link→odom   │                     │
    │       ↓       │                   │    启动位置 ●       │
    └───────────────┘                   └─────────────────────┘

- base_link: 网络输出轨迹的坐标系 (局部坐标，当前位置为原点)
- odom: 控制器工作坐标系 (里程计坐标系，从启动位置累积)
- odom 就是你的"世界坐标系"，不需要建图
"""

# 一致性检查配置
CONSISTENCY_CONFIG = {
    'kappa_thresh': 0.5,              # 曲率一致性阈值
    'v_dir_thresh': 0.8,              # 速度方向一致性阈值
    'temporal_smooth_thresh': 0.5,    # 时序平滑度阈值
    # 注意: low_speed_thresh 统一使用 trajectory.low_speed_thresh
    # 一致性检查器会从 trajectory 配置中读取此值
    # 这样可以确保轨迹速度计算和一致性检查使用相同的阈值
    'alpha_min': 0.1,                 # α 最小值
    'max_curvature': 10.0,            # 曲率计算的最大值限制 (1/m)
    'temporal_window_size': 10,       # 时序平滑度计算的滑动窗口大小
    'weights': {
        'kappa': 1.0,                 # 曲率权重
        'velocity': 1.5,              # 速度方向权重
        'temporal': 0.8,              # 时序平滑权重
    },
}

# 坐标变换配置
#
# 坐标系说明 (不需要建图/定位):
# - base_link: 机体坐标系，原点在机器人中心，X轴朝前
# - odom: 里程计坐标系，从启动位置开始累积 (这就是你的"世界坐标系")
#
# 数据流:
# 网络输出 (base_link, 局部) → 坐标变换 → 控制器 (odom) → 控制输出
#
TRANSFORM_CONFIG = {
    'target_frame': 'odom',           # 目标坐标系 (里程计坐标系，控制器工作坐标系)
    'source_frame': 'base_link',      # 源坐标系 (网络输出轨迹的坐标系，局部坐标)
    'fallback_duration_limit_ms': 500,   # 降级持续限制 (ms)
    'fallback_critical_limit_ms': 1000,  # 临界降级限制 (ms)
    'tf2_timeout_ms': 10,             # TF2 超时 (ms)
    'drift_estimation_enabled': False,    # 漂移估计开关
    'recovery_correction_enabled': True, # 恢复校正开关
    'drift_rate': 0.01,               # 漂移率 (米/秒)
    'max_drift_dt': 0.5,              # 漂移估计最大时间间隔 (秒)
    'drift_correction_thresh': 0.01,  # 漂移校正阈值 (米/弧度)
    # 坐标系验证
    # 期望的源坐标系列表:
    # - base_link: 标准机体坐标系 (网络输出的局部轨迹)
    # - base_link_0: 推理时刻冻结的机体坐标系 (同 base_link)
    # - '': 空字符串，使用默认 source_frame
    # - odom: 已经在里程计坐标系的轨迹 (不需要变换，直接使用)
    'expected_source_frames': ['base_link', 'base_link_0', '', 'odom'],
    'warn_unexpected_frame': True,    # 对非期望坐标系发出警告
}

# 平滑过渡配置
TRANSITION_CONFIG = {
    'type': 'exponential',            # 过渡类型: 'exponential' 或 'linear'
    'tau': 0.1,                       # 指数过渡时间常数 (秒)
    'max_duration': 0.5,              # 最大过渡时长 (秒)
    'completion_threshold': 0.95,     # 过渡完成阈值
    'duration': 0.2,                  # 线性过渡时长 (秒)
}

# 备份控制器配置 (Pure Pursuit)
BACKUP_CONFIG = {
    'lookahead_dist': 1.0,            # 前瞻距离 (m)
    'min_lookahead': 0.5,             # 最小前瞻距离 (m)
    'max_lookahead': 3.0,             # 最大前瞻距离 (m)
    'lookahead_ratio': 0.5,           # 前瞻比例
    'kp_z': 1.0,                      # Z 方向增益
    'kp_heading': 1.5,                # 航向增益
    'heading_mode': 'follow_velocity',  # 航向模式
    'dt': 0.02,                       # 时间步长 (秒)
    
    # Pure Pursuit 控制参数
    'heading_error_thresh': 1.047,    # 航向误差阈值 (rad, ~60°)
    'pure_pursuit_angle_thresh': 1.047,  # Pure Pursuit 模式角度阈值
    'heading_control_angle_thresh': 1.571,  # 航向控制模式角度阈值 (~90°)
    'max_curvature': 5.0,             # 最大曲率限制 (1/m)
    'min_turn_speed': 0.1,            # 阿克曼车辆最小转向速度 (m/s)
    'default_speed_ratio': 0.5,       # 无 soft 速度时的默认速度比例
    
    # 低速过渡参数
    'low_speed_transition_factor': 0.5,  # 低速过渡区域因子
    
    # 曲率速度限制阈值
    'curvature_speed_limit_thresh': 0.1,  # 曲率阈值 (1/m)
    
    # 距离阈值
    'min_distance_thresh': 0.1,       # 最小距离阈值 (m)
    
    # 角速度变化率限制
    'omega_rate_limit': None,         # None 表示使用 alpha_max * dt
}

# 模块配置验证规则
MODULES_VALIDATION_RULES = {
    # 一致性配置
    'consistency.alpha_min': (0.0, 1.0, '最小 alpha 值'),
    'consistency.kappa_thresh': (0.0, 10.0, '曲率一致性阈值'),
    'consistency.v_dir_thresh': (0.0, 1.0, '速度方向一致性阈值'),
    'consistency.weights.kappa': (0.0, None, '曲率一致性权重'),
    'consistency.weights.velocity': (0.0, None, '速度方向一致性权重'),
    'consistency.weights.temporal': (0.0, None, '时序平滑度权重'),
    # 坐标变换配置
    'transform.fallback_duration_limit_ms': (0, 10000, 'TF2 降级持续限制 (ms)'),
    'transform.fallback_critical_limit_ms': (0, 30000, 'TF2 临界降级限制 (ms)'),
    'transform.tf2_timeout_ms': (1, 1000, 'TF2 查询超时 (ms)'),
    'transform.drift_rate': (0.0, 1.0, '漂移率 (米/秒)'),
    'transform.max_drift_dt': (0.01, 5.0, '漂移估计最大时间间隔 (秒)'),
    'transform.drift_correction_thresh': (0.0, 1.0, '漂移校正阈值 (米/弧度)'),
    # 过渡配置
    'transition.tau': (0.001, 10.0, '过渡时间常数 (秒)'),
    'transition.max_duration': (0.01, 10.0, '最大过渡时长 (秒)'),
    'transition.completion_threshold': (0.5, 1.0, '过渡完成阈值'),
    # 备份控制器配置
    'backup.lookahead_dist': (0.1, 10.0, '前瞻距离 (m)'),
    'backup.min_lookahead': (0.01, 5.0, '最小前瞻距离 (m)'),
    'backup.max_lookahead': (0.5, 20.0, '最大前瞻距离 (m)'),
    'backup.kp_heading': (0.1, 10.0, '航向控制增益'),
}
