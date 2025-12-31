"""轨迹配置

轨迹数据处理相关的配置参数：
- 默认时间步长
- 轨迹点数限制
- 速度计算阈值
- 轨迹验证参数
- 轨迹适配器参数 (ROS 消息转换)

坐标系配置说明:
===============
坐标系配置统一在 modules_config.py 的 TRANSFORM_CONFIG 中定义:
- transform.source_frame: 输入坐标系 (网络输出的局部坐标系，如 base_link)
- transform.target_frame: 输出坐标系 (控制器工作坐标系，如 odom)

这样设计的原因:
1. 避免配置重复和不一致
2. 坐标变换是 transform 模块的职责
3. 轨迹配置专注于轨迹数据本身的参数
"""

# 轨迹配置
TRAJECTORY_CONFIG = {
    # 时间参数
    'default_dt_sec': 0.1,            # 默认时间步长 (秒)
    'min_dt_sec': 0.01,               # 最小时间步长 (秒)
    'max_dt_sec': 1.0,                # 最大时间步长 (秒)
    
    # 轨迹点数限制
    'min_points': 2,                  # 最小轨迹点数
    'max_points': 100,                # 最大轨迹点数
    
    # 速度计算参数
    # 此值是低速阈值的唯一定义点
    # consistency 模块会自动从这里读取，确保一致性
    'low_speed_thresh': 0.1,          # 低速阈值 (m/s)，用于角速度计算和一致性检查
    
    # 轨迹验证参数
    'max_point_distance': 10.0,       # 相邻点最大距离 (m)
    'max_coord': 100.0,               # 最大合理坐标值 (米)，超出视为无效
    
    # 速度填充参数 (轨迹适配器使用)
    'velocity_decay_threshold': 0.1,  # 速度衰减填充阈值 (m/s)
                                      # 当速度点数少于位置点数时，如果最后速度大于此阈值，
                                      # 使用线性衰减填充；否则使用零填充
    
    # 置信度参数
    'min_confidence': 0.0,            # 最小置信度 (用于 clip)
    'max_confidence': 1.0,            # 最大置信度 (用于 clip)
    'default_confidence': 0.9,        # 默认置信度
}

# 轨迹配置验证规则
TRAJECTORY_VALIDATION_RULES = {
    'trajectory.default_dt_sec': (0.001, 1.0, '默认时间步长 (秒)'),
    'trajectory.min_dt_sec': (0.001, 1.0, '最小时间步长 (秒)'),
    'trajectory.max_dt_sec': (0.01, 10.0, '最大时间步长 (秒)'),
    'trajectory.low_speed_thresh': (0.0, 10.0, '低速阈值 (m/s)'),
    'trajectory.min_points': (1, 100, '最小轨迹点数'),
    'trajectory.max_points': (2, 1000, '最大轨迹点数'),
    'trajectory.max_point_distance': (0.1, 100.0, '相邻点最大距离 (m)'),
    'trajectory.max_coord': (1.0, 10000.0, '最大合理坐标值 (米)'),
    'trajectory.velocity_decay_threshold': (0.0, 10.0, '速度衰减填充阈值 (m/s)'),
    'trajectory.min_confidence': (0.0, 1.0, '最小置信度'),
    'trajectory.max_confidence': (0.0, 1.0, '最大置信度'),
    'trajectory.default_confidence': (0.0, 1.0, '默认置信度'),
}

# 注意: 轨迹验证参数已整合到 core/data_types.py 的 TrajectoryDefaults 类中
# TrajectoryDefaults.configure(config) 会同时配置默认值和验证参数
