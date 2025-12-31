"""备份控制器配置

Pure Pursuit 备份控制器的配置参数：
- 前瞻距离参数
- 控制增益
- 航向控制参数

控制模式切换逻辑说明:
=====================
Pure Pursuit 根据目标点相对于车辆的角度选择不同的控制策略。
角度阈值定义在 core/constants.py 中（几何常量，不可配置）：
- PURE_PURSUIT_ANGLE_THRESH (π/3 ≈ 60°): Pure Pursuit 曲率控制
- HEADING_CONTROL_ANGLE_THRESH (π/2 = 90°): 航向误差控制
- REAR_ANGLE_THRESH (~162°): 正后方检测

这些角度阈值是基于控制理论的几何常量：
- 60° 是 Pure Pursuit 算法的经典切换角度
- 90° 是正交方向，几何意义明确
- 162° 接近 180° 但留有余量，防止边界震荡

  目标点角度 (相对于车辆前进方向):
  ┌─────────────────────────────────────────────────────────────┐
  │  0° ~ 60°  → Pure Pursuit 曲率控制
  │  60° ~ 90° → 混合过渡区域
  │  90° ~ 180° → 航向误差控制 (先转向再前进)
  └─────────────────────────────────────────────────────────────┘

共享参数说明:
=============
以下参数同时被 Pure Pursuit 备份控制器和 MPC Fallback 求解器使用:
- max_curvature: 最大曲率限制
- min_distance_thresh: 最小距离阈值
- min_turn_speed: 阿克曼车辆最小转向速度
- default_speed_ratio: 无 soft 速度时的默认速度比例
"""

# 备份控制器配置 (Pure Pursuit)
BACKUP_CONFIG = {
    # 前瞻距离参数
    'lookahead_dist': 1.0,            # 前瞻距离 (m)
    'min_lookahead': 0.5,             # 最小前瞻距离 (m)
    'max_lookahead': 3.0,             # 最大前瞻距离 (m)
    'lookahead_ratio': 0.5,           # 前瞻比例 (速度越快，前瞻越远)
    
    # 控制增益
    'kp_z': 1.0,                      # Z 方向增益 (3D 平台用)
    'kp_heading': 1.5,                # 航向增益
    
    # 航向模式
    'heading_mode': 'follow_velocity',  # 航向模式: follow_velocity, fixed, target_point, manual
    
    # Pure Pursuit 控制参数
    # 注意: 角度阈值 (PURE_PURSUIT_ANGLE_THRESH, HEADING_CONTROL_ANGLE_THRESH, REAR_ANGLE_THRESH)
    # 是基于控制理论的几何常量，定义在 core/constants.py 中，不可配置
    'heading_error_thresh': 1.047,    # 航向误差阈值 (rad, ~60°) - 超过此值停止前进
    'max_curvature': 5.0,             # 最大曲率限制 (1/m)
    'min_turn_speed': 0.1,            # 阿克曼车辆最小转向速度 (m/s)
    'default_speed_ratio': 0.5,       # 无 soft 速度时的默认速度比例
    
    # 低速过渡参数
    'low_speed_transition_factor': 0.5,  # 低速过渡区域因子
    'curvature_speed_limit_thresh': 0.1,  # 曲率速度限制阈值 (1/m)
    
    # 距离阈值
    'min_distance_thresh': 0.1,       # 最小距离阈值 (m)
    
    # 角速度变化率限制
    # -1 或 None: 自动计算为 alpha_max * dt
    # 正数: 使用指定值 (rad/s per step)
    'omega_rate_limit': -1,
    
    # 正后方处理参数
    # 注意: REAR_ANGLE_THRESH 是几何常量，定义在 core/constants.py 中
    'rear_direction_min_thresh': 0.05,  # 正后方转向判断最小阈值 (m)
    'default_turn_direction': 'left',   # 正后方默认转向方向
}

# 备份控制器配置验证规则
BACKUP_VALIDATION_RULES = {
    'backup.lookahead_dist': (0.1, 10.0, '前瞻距离 (m)'),
    'backup.min_lookahead': (0.01, 5.0, '最小前瞻距离 (m)'),
    'backup.max_lookahead': (0.5, 20.0, '最大前瞻距离 (m)'),
    'backup.lookahead_ratio': (0.0, 5.0, '前瞻比例'),
    'backup.kp_z': (0.0, 10.0, 'Z 方向增益'),
    'backup.kp_heading': (0.1, 10.0, '航向控制增益'),
    'backup.heading_error_thresh': (0.1, 3.14, '航向误差阈值 (rad)'),
    'backup.max_curvature': (0.1, 20.0, '最大曲率限制 (1/m)'),
    'backup.min_turn_speed': (0.0, 1.0, '阿克曼车辆最小转向速度 (m/s)'),
    'backup.default_speed_ratio': (0.0, 1.0, '无 soft 速度时的默认速度比例'),
    'backup.min_distance_thresh': (0.001, 1.0, '最小距离阈值 (m)'),
    'backup.low_speed_transition_factor': (0.0, 1.0, '低速过渡区域因子'),
    'backup.curvature_speed_limit_thresh': (0.0, 10.0, '曲率速度限制阈值 (1/m)'),
    'backup.rear_direction_min_thresh': (0.0, 1.0, '正后方转向判断最小阈值 (m)'),
    # 注意: 角度阈值 (PURE_PURSUIT_ANGLE_THRESH 等) 是几何常量，定义在 constants.py 中，不需要验证
}

__all__ = [
    'BACKUP_CONFIG',
    'BACKUP_VALIDATION_RULES',
]
