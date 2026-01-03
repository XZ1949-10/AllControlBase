"""轨迹配置

轨迹数据处理相关的配置参数：
- 默认时间步长
- 轨迹点数限制
- 速度计算阈值
- 轨迹适配器参数 (ROS 消息转换)

配置使用说明:
===============
所有模块应通过 TrajectoryConfig 实例获取轨迹配置，而不是直接从此文件读取。
TrajectoryConfig 是轨迹配置的数据容器。

使用方式:
    from universal_controller.core.data_types import TrajectoryConfig
    from universal_controller.core.constants import TRAJECTORY_MAX_COORD
    
    # 初始化配置
    traj_config = TrajectoryConfig.from_dict(config)
    
    # 读取配置
    dt_sec = traj_config.dt_sec
    
    # 读取常量 (从 constants.py)
    max_coord = TRAJECTORY_MAX_COORD

坐标系配置说明:
===============
坐标系配置统一在 transform_config.py 的 TRANSFORM_CONFIG 中定义:
- transform.source_frame: 输入坐标系 (网络输出的局部坐标系，如 base_link)
- transform.target_frame: 输出坐标系 (控制器工作坐标系，如 odom)

这样设计的原因:
1. 避免配置重复和不一致
2. 坐标变换是 transform 模块的职责
3. 轨迹配置专注于轨迹数据本身的参数

注意:
=====
以下参数是数值稳定性/物理合理性常量，已移至 core/constants.py:
- TRAJECTORY_MIN_DT_SEC: 最小时间步长 (0.01s)
- TRAJECTORY_MAX_DT_SEC: 最大时间步长 (1.0s)
- TRAJECTORY_MAX_COORD: 最大合理坐标值 (100m)
- CONFIDENCE_MIN, CONFIDENCE_MAX: 置信度边界 [0, 1]

这些参数不应由用户配置。
"""

# 轨迹配置
TRAJECTORY_CONFIG = {
    # 时间参数
    'default_dt_sec': 0.1,            # 默认时间步长 (秒)
    # 注意: min_dt_sec 和 max_dt_sec 已移至 constants.py
    # (TRAJECTORY_MIN_DT_SEC, TRAJECTORY_MAX_DT_SEC)
    
    # 轨迹点数限制
    'min_points': 2,                  # 最小轨迹点数
    'max_points': 100,                # 最大轨迹点数
    
    # 速度计算参数
    # 此值是低速阈值的唯一定义点
    # consistency 模块会自动从这里读取，确保一致性
    'low_speed_thresh': 0.1,          # 低速阈值 (m/s)，用于角速度计算和一致性检查
    
    # 轨迹验证参数
    'max_point_distance': 10.0,       # 相邻点最大距离 (m)
    # 注意: max_coord 已移至 constants.py (TRAJECTORY_MAX_COORD)
    
    # 速度填充参数 (轨迹适配器使用)
    'velocity_decay_threshold': 0.1,  # 速度衰减填充阈值 (m/s)
                                      # 当速度点数少于位置点数时，如果最后速度大于此阈值，
                                      # 使用线性衰减填充；否则使用零填充
    
    # 置信度参数
    # 注意: min_confidence 和 max_confidence 是数学定义 [0, 1]，
    # 使用 core/constants.py 中的 CONFIDENCE_MIN, CONFIDENCE_MAX
    'default_confidence': 0.9,        # 默认置信度
}

# 轨迹配置验证规则
# 注意: min_dt_sec, max_dt_sec, max_coord 已移至 constants.py，不再需要验证
TRAJECTORY_VALIDATION_RULES = {
    'trajectory.default_dt_sec': (0.001, 1.0, '默认时间步长 (秒)'),
    'trajectory.low_speed_thresh': (0.0, 10.0, '低速阈值 (m/s)'),
    'trajectory.min_points': (1, 100, '最小轨迹点数'),
    'trajectory.max_points': (2, 1000, '最大轨迹点数'),
    'trajectory.max_point_distance': (0.1, 100.0, '相邻点最大距离 (m)'),
    'trajectory.velocity_decay_threshold': (0.0, 10.0, '速度衰减填充阈值 (m/s)'),
    'trajectory.default_confidence': (0.0, 1.0, '默认置信度'),
}

# 注意: 轨迹配置参数现在由 core/data_types.py 的 TrajectoryConfig 类处理
# Common configuration is passed via TrajectoryConfig.from_dict(config)
# 所有模块应通过 TrajectoryConfig 实例获取配置，而不是直接依赖全局状态
