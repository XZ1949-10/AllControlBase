"""一致性检查配置

轨迹一致性检查器的配置参数：
- 曲率一致性阈值
- 速度方向一致性阈值
- 时序平滑度阈值
- 权重配置

注意: 
=====
- low_speed_thresh 统一定义在 trajectory_config.py 中
  一致性检查器通过 trajectory.low_speed_thresh 获取此值
- invalid_data_confidence 已移至 constants.py (CONSISTENCY_INVALID_DATA_CONFIDENCE)
  这是安全策略常量，不应由用户配置
"""

# 一致性检查配置
CONSISTENCY_CONFIG = {
    'kappa_thresh': 0.5,              # 曲率一致性阈值
    'v_dir_thresh': 0.8,              # 速度方向一致性阈值
    'temporal_smooth_thresh': 0.5,    # 时序平滑度阈值
    'alpha_min': 0.1,                 # α 最小值
    'min_value_for_log': 1e-10,       # 对数计算的最小数值 (防止下溢)
    'max_curvature': 10.0,            # 曲率计算的最大值限制 (1/m)
    'temporal_window_size': 10,       # 时序平滑度计算的滑动窗口大小
    # 注意: invalid_data_confidence 已移至 constants.py (CONSISTENCY_INVALID_DATA_CONFIDENCE)
    'weights': {
        'kappa': 1.0,                 # 曲率权重
        'velocity': 1.5,              # 速度方向权重
        'temporal': 0.8,              # 时序平滑权重
    },
}

# 一致性配置验证规则
# 注意: invalid_data_confidence 已移至 constants.py，不再需要验证
CONSISTENCY_VALIDATION_RULES = {
    'consistency.alpha_min': (0.0, 1.0, '最小 alpha 值'),
    'consistency.min_value_for_log': (1e-20, 1e-3, '对数计算最小数值'),
    'consistency.kappa_thresh': (0.0, 10.0, '曲率一致性阈值'),
    'consistency.v_dir_thresh': (0.0, 1.0, '速度方向一致性阈值'),
    'consistency.temporal_smooth_thresh': (0.0, 10.0, '时序平滑度阈值'),
    'consistency.max_curvature': (0.1, 100.0, '曲率计算最大值限制 (1/m)'),
    'consistency.temporal_window_size': (2, 100, '时序平滑度滑动窗口大小'),
    'consistency.weights.kappa': (0.0, None, '曲率一致性权重'),
    'consistency.weights.velocity': (0.0, None, '速度方向一致性权重'),
    'consistency.weights.temporal': (0.0, None, '时序平滑度权重'),
}

__all__ = [
    'CONSISTENCY_CONFIG',
    'CONSISTENCY_VALIDATION_RULES',
]
