"""系统基础配置

包含控制器的基础系统参数：
- 控制频率
- 重力加速度
- 暂停检测阈值
- 诊断话题配置
- 超时配置
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
WATCHDOG_CONFIG = {
    'odom_timeout_ms': 200,       # 里程计超时 (ms)，<=0 禁用
    'traj_timeout_ms': 200,       # 轨迹超时 (ms)，<=0 禁用
    'traj_grace_ms': 100,         # 轨迹宽限期 (ms)
    'imu_timeout_ms': 100,        # IMU 超时 (ms)，<=0 禁用 (无 IMU 时设为 -1)
    'startup_grace_ms': 1000,     # 启动宽限期 (ms)，期间不检测超时
}

# 诊断配置
DIAGNOSTICS_CONFIG = {
    'topic': '/controller/diagnostics',  # 诊断话题名称
    'cmd_topic': '/cmd_unified',         # 控制命令话题名称
    'publish_rate': 10,                  # 诊断发布降频率 (每 N 次控制循环发布一次)
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
    'diagnostics.publish_rate': (1, 100, '诊断发布降频率'),
}
