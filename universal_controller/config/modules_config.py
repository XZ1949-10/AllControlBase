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
#
# 注意: low_speed_thresh 统一定义在 trajectory_config.py 中
# 一致性检查器通过 trajectory.low_speed_thresh 获取此值
# 这确保了轨迹速度计算和一致性检查使用相同的阈值
#
CONSISTENCY_CONFIG = {
    'kappa_thresh': 0.5,              # 曲率一致性阈值
    'v_dir_thresh': 0.8,              # 速度方向一致性阈值
    'temporal_smooth_thresh': 0.5,    # 时序平滑度阈值
    'alpha_min': 0.1,                 # α 最小值
    'max_curvature': 10.0,            # 曲率计算的最大值限制 (1/m)
    'temporal_window_size': 10,       # 时序平滑度计算的滑动窗口大小
    'invalid_data_confidence': 0.5,   # 数据无效时的保守置信度
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
# - base_footprint: TurtleBot 等平台使用的机体坐标系 (与 base_link 类似)
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
    'timeout_ms': 10,                 # TF2 查询超时 (ms)，与 YAML tf.timeout_ms 保持一致
    'drift_estimation_enabled': False,    # 漂移估计开关
    'recovery_correction_enabled': True, # 恢复校正开关
    'max_accumulated_drift': 1.0,     # 漂移累积上限 (米) - 超过后停止累积
    'drift_rate': 0.01,               # 漂移率 (米/秒)
    'drift_velocity_factor': 0.1,     # 速度漂移因子 (每 1 m/s 增加的漂移率比例)
    'max_drift_dt': 0.5,              # 漂移估计最大时间间隔 (秒)
    'drift_correction_thresh': 0.01,  # 漂移校正阈值 (米/弧度)
    # 坐标系验证
    # 期望的源坐标系列表:
    # - base_link: 标准机体坐标系 (网络输出的局部轨迹)
    # - base_footprint: TurtleBot 等平台使用的机体坐标系
    # - base_link_0: 推理时刻冻结的机体坐标系 (同 base_link)
    # - '': 空字符串，使用默认 source_frame
    # - odom: 轨迹已在里程计坐标系中 (无需变换，直接使用)
    #         适用于规划器直接输出全局坐标的场景
    'expected_source_frames': ['base_link', 'base_footprint', 'base_link_0', '', 'odom'],
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
#
# 控制模式切换逻辑说明:
# =====================
# Pure Pursuit 根据目标点相对于车辆的角度选择不同的控制策略:
#
#   目标点角度 (相对于车辆前进方向):
#   ┌─────────────────────────────────────────────────────────────┐
#   │  0° ~ pure_pursuit_angle_thresh (60°)  → Pure Pursuit 曲率控制
#   │  60° ~ heading_control_angle_thresh (90°) → 混合过渡区域
#   │  90° ~ 180° → 航向误差控制 (先转向再前进)
#   └─────────────────────────────────────────────────────────────┘
#
#   航向误差处理:
#   ┌─────────────────────────────────────────────────────────────┐
#   │  heading_error < heading_error_thresh (60°) → 边走边转
#   │  heading_error >= heading_error_thresh (60°) → 停止前进，原地转向
#   └─────────────────────────────────────────────────────────────┘
#
# 共享参数说明:
# =============
# 以下参数同时被 Pure Pursuit 备份控制器和 MPC Fallback 求解器使用:
# - max_curvature: 最大曲率限制
# - min_distance_thresh: 最小距离阈值
# - min_turn_speed: 阿克曼车辆最小转向速度
# - default_speed_ratio: 无 soft 速度时的默认速度比例
# 这确保了两种备份模式的行为一致性。
#
BACKUP_CONFIG = {
    # 前瞻距离参数
    'lookahead_dist': 1.0,            # 前瞻距离 (m)
    'min_lookahead': 0.5,             # 最小前瞻距离 (m)
    'max_lookahead': 3.0,             # 最大前瞻距离 (m)
    'lookahead_ratio': 0.5,           # 前瞻比例 (速度越快，前瞻越远)
    
    # 控制增益
    'kp_z': 1.0,                      # Z 方向增益 (3D 平台用)
    'kp_heading': 1.5,                # 航向增益
    
    # 注意: 时间步长统一使用 system.ctrl_freq 计算 (dt = 1/ctrl_freq)
    
    # 航向模式
    'heading_mode': 'follow_velocity',  # 航向模式: follow_velocity, fixed, target_point, manual
    
    # Pure Pursuit 控制参数
    'heading_error_thresh': 1.047,    # 航向误差阈值 (rad, ~60°) - 超过此值停止前进
    'pure_pursuit_angle_thresh': 1.047,  # Pure Pursuit 模式角度阈值 (rad, ~60°)
    'heading_control_angle_thresh': 1.571,  # 航向控制模式角度阈值 (rad, ~90°)
    'max_curvature': 5.0,             # 最大曲率限制 (1/m)
    'min_turn_speed': 0.1,            # 阿克曼车辆最小转向速度 (m/s)
    'default_speed_ratio': 0.5,       # 无 soft 速度时的默认速度比例
    
    # 低速过渡参数
    'low_speed_transition_factor': 0.5,  # 低速过渡区域因子
    'curvature_speed_limit_thresh': 0.1,  # 曲率速度限制阈值 (1/m)
    
    # 距离阈值
    'min_distance_thresh': 0.1,       # 最小距离阈值 (m)
    
    # 角速度变化率限制
    # null/None: 自动计算为 alpha_max * dt，确保与角加速度限制一致
    # 正数: 使用指定值 (rad/s per step)
    'omega_rate_limit': None,
    
    # 正后方处理参数 (防止目标点在正后方时的角速度跳变)
    # 当 heading_error 接近 ±π 时，arctan2 可能在 +π 和 -π 之间跳变
    # 这些参数用于检测和处理这种情况
    'rear_angle_thresh': 2.827,       # 正后方检测阈值 (rad, ~162°, 0.9*π)
    'rear_direction_min_thresh': 0.05,  # 正后方转向判断最小阈值 (m)
    'default_turn_direction': 'left',   # 正后方默认转向方向 ("left" 或 "right")
}

# 模块配置验证规则
MODULES_VALIDATION_RULES = {
    # 一致性配置
    'consistency.alpha_min': (0.0, 1.0, '最小 alpha 值'),
    'consistency.kappa_thresh': (0.0, 10.0, '曲率一致性阈值'),
    'consistency.v_dir_thresh': (0.0, 1.0, '速度方向一致性阈值'),
    'consistency.temporal_smooth_thresh': (0.0, 10.0, '时序平滑度阈值'),
    'consistency.max_curvature': (0.1, 100.0, '曲率计算最大值限制 (1/m)'),
    'consistency.temporal_window_size': (2, 100, '时序平滑度滑动窗口大小'),
    'consistency.invalid_data_confidence': (0.0, 1.0, '数据无效时的保守置信度'),
    'consistency.weights.kappa': (0.0, None, '曲率一致性权重'),
    'consistency.weights.velocity': (0.0, None, '速度方向一致性权重'),
    'consistency.weights.temporal': (0.0, None, '时序平滑度权重'),
    # 坐标变换配置
    'transform.fallback_duration_limit_ms': (0, 10000, 'TF2 降级持续限制 (ms)'),
    'transform.fallback_critical_limit_ms': (0, 30000, 'TF2 临界降级限制 (ms)'),
    'transform.timeout_ms': (1, 1000, 'TF2 查询超时 (ms)'),
    'transform.drift_rate': (0.0, 1.0, '漂移率 (米/秒)'),
    'transform.drift_velocity_factor': (0.0, 1.0, '速度漂移因子'),
    'transform.max_drift_dt': (0.01, 5.0, '漂移估计最大时间间隔 (秒)'),
    'transform.drift_correction_thresh': (0.0, 1.0, '漂移校正阈值 (米/弧度)'),
    # 过渡配置
    'transition.tau': (0.001, 10.0, '过渡时间常数 (秒)'),
    'transition.max_duration': (0.01, 10.0, '最大过渡时长 (秒)'),
    'transition.completion_threshold': (0.5, 1.0, '过渡完成阈值'),
    'transition.duration': (0.01, 10.0, '线性过渡时长 (秒)'),
    # 备份控制器配置
    'backup.lookahead_dist': (0.1, 10.0, '前瞻距离 (m)'),
    'backup.min_lookahead': (0.01, 5.0, '最小前瞻距离 (m)'),
    'backup.max_lookahead': (0.5, 20.0, '最大前瞻距离 (m)'),
    'backup.lookahead_ratio': (0.0, 5.0, '前瞻比例'),
    'backup.kp_z': (0.0, 10.0, 'Z 方向增益'),
    'backup.kp_heading': (0.1, 10.0, '航向控制增益'),
    'backup.heading_error_thresh': (0.1, 3.14, '航向误差阈值 (rad)'),
    'backup.pure_pursuit_angle_thresh': (0.1, 3.14, 'Pure Pursuit 模式角度阈值 (rad)'),
    'backup.heading_control_angle_thresh': (0.1, 3.14, '航向控制模式角度阈值 (rad)'),
    'backup.max_curvature': (0.1, 20.0, '最大曲率限制 (1/m)'),
    'backup.min_turn_speed': (0.0, 1.0, '阿克曼车辆最小转向速度 (m/s)'),
    'backup.default_speed_ratio': (0.0, 1.0, '无 soft 速度时的默认速度比例'),
    'backup.min_distance_thresh': (0.001, 1.0, '最小距离阈值 (m)'),
    'backup.low_speed_transition_factor': (0.0, 1.0, '低速过渡区域因子'),
    'backup.curvature_speed_limit_thresh': (0.0, 10.0, '曲率速度限制阈值 (1/m)'),
    'backup.rear_angle_thresh': (1.57, 3.14, '正后方检测阈值 (rad)'),
    'backup.rear_direction_min_thresh': (0.0, 1.0, '正后方转向判断最小阈值 (m)'),
}
