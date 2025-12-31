"""系统基础配置

包含控制器的基础系统参数：
- 控制频率
- 重力加速度
- 暂停检测阈值
- 诊断话题配置
- 超时配置
- 跟踪质量评估配置 (Dashboard)
"""

# 系统配置
SYSTEM_CONFIG = {
    'ctrl_freq': 50,              # 控制频率 (Hz)
    'platform': 'differential',   # 默认平台类型
    'gravity': 9.81,              # 重力加速度 (m/s²)
    # 控制循环暂停检测
    'long_pause_threshold': 0.5,  # 长时间暂停检测阈值 (秒)
    'ekf_reset_threshold': 2.0,   # EKF 重置阈值 (秒)
}

# 超时配置 (Watchdog)
# 注意: 
# - 0 或负数表示禁用该数据源的超时检测
# - 如果配置了正数超时但数据源不存在，系统会立即报告超时
# - 对于可选的数据源（如 IMU），如果硬件不存在，应配置为 -1 禁用
# - 安全停止延迟 = traj_timeout_ms + traj_grace_ms
# - IMU 默认禁用 (-1)，因为大多数地面平台没有 IMU，有 IMU 的平台需显式启用
# - absolute_startup_timeout_ms: 从创建监控器开始的绝对超时，
#   如果超过此时间仍未收到任何数据，将报告超时。设为 <= 0 表示禁用。
#   默认值 -1 表示禁用（向后兼容），如需启用建议设为 startup_grace_ms * 2
WATCHDOG_CONFIG = {
    'odom_timeout_ms': 500,       # 里程计超时 (ms)，<=0 禁用
    'traj_timeout_ms': 1000,      # 轨迹超时 (ms)，<=0 禁用
    'traj_grace_ms': 500,         # 轨迹宽限期 (ms)
    'imu_timeout_ms': -1,         # IMU 超时 (ms)，<=0 禁用 (默认禁用)
    'startup_grace_ms': 5000,     # 启动宽限期 (ms)，期间不检测超时
    'absolute_startup_timeout_ms': -1,  # 绝对启动超时 (ms)，<=0 禁用
}

# 诊断配置
# 注意: 话题名称 (topic, cmd_topic) 是 ROS 层特有配置，
# 在 controller_ros/config/ 中定义，不在核心库中定义
DIAGNOSTICS_CONFIG = {
    'publish_rate': 10,                  # 诊断发布降频率 (每 N 次控制循环发布一次)
    # 错误处理配置
    'max_consecutive_errors_detail': 10, # 详细记录错误的最大次数
    'error_summary_interval': 50,        # 摘要日志间隔（每 N 次错误记录一次）
    'max_error_count': 1000,             # 错误计数上限（防止整数溢出）
}

# =============================================================================
# 跟踪质量评估配置 (Dashboard 显示用)
# 这些参数主要用于 Dashboard 显示，不影响控制器行为
# =============================================================================
TRACKING_CONFIG = {
    # 误差阈值 (超过此值评分为 0)
    'lateral_thresh': 0.3,        # 横向误差阈值 (m)
    'longitudinal_thresh': 0.5,   # 纵向误差阈值 (m)
    'heading_thresh': 0.5,        # 航向误差阈值 (rad, ~28.6°)
    'prediction_thresh': 0.5,     # 预测误差阈值 (m)
    
    # 质量评分权重 (总和应为 1.0)
    'weights': {
        'lateral': 0.4,           # 横向误差权重
        'longitudinal': 0.4,      # 纵向误差权重
        'heading': 0.2,           # 航向误差权重
    },
    
    # 评级阈值 (百分比)
    'rating': {
        'excellent': 90,          # >= 90% 为优秀
        'good': 70,               # >= 70% 为良好
        'fair': 50,               # >= 50% 为一般，< 50% 为较差
    },
}

# 系统配置验证规则
# 注意: 超时配置允许 <=0 表示禁用，所以最小值设为 None (无下限)
SYSTEM_VALIDATION_RULES = {
    'system.ctrl_freq': (1, 1000, '控制频率 (Hz)'),
    'system.gravity': (0.1, 20.0, '重力加速度 (m/s²)'),
    'system.long_pause_threshold': (0.01, 10.0, '长时间暂停检测阈值 (秒)'),
    'system.ekf_reset_threshold': (0.1, 60.0, 'EKF 重置阈值 (秒)'),
    'watchdog.odom_timeout_ms': (None, 10000, '里程计超时 (ms)，<=0 禁用'),
    'watchdog.traj_timeout_ms': (None, 10000, '轨迹超时 (ms)，<=0 禁用'),
    'watchdog.imu_timeout_ms': (None, 10000, 'IMU 超时 (ms)，<=0 禁用'),
    'watchdog.absolute_startup_timeout_ms': (None, 60000, '绝对启动超时 (ms)，<=0 禁用'),
    'diagnostics.publish_rate': (1, 100, '诊断发布降频率'),
    # 跟踪质量评估配置验证
    'tracking.lateral_thresh': (0.01, 10.0, '横向误差阈值 (m)'),
    'tracking.longitudinal_thresh': (0.01, 10.0, '纵向误差阈值 (m)'),
    'tracking.heading_thresh': (0.01, 3.14, '航向误差阈值 (rad)'),
}
