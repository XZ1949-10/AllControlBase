"""EKF 配置

扩展卡尔曼滤波器的配置参数：
- 测量噪声
- 过程噪声
- 自适应参数
- 异常检测参数
"""

EKF_CONFIG = {
    # 航向备选配置
    'use_odom_orientation_fallback': True,      # 是否使用里程计航向作为备选
    'theta_covariance_fallback_thresh': 0.5,    # 航向协方差回退阈值
    'imu_motion_compensation': False,           # IMU 运动补偿
    
    # 自适应参数
    'adaptive': {
        'base_slip_thresh': 2.0,           # 打滑检测基础阈值 (m/s²)
        'slip_velocity_factor': 0.5,       # 速度相关因子
        'slip_covariance_scale': 10.0,     # 打滑时协方差放大倍数
        'stationary_covariance_scale': 0.1,  # 静止时协方差缩小倍数
        'stationary_thresh': 0.05,         # 静止检测阈值 (m/s)
        'slip_probability_k_factor': 5.0,  # 打滑概率 sigmoid 斜率因子
        'slip_history_window': 20,         # 打滑概率计算滑动窗口大小
    },
    
    # IMU 相关参数
    'max_tilt_angle': 1.047,               # IMU 姿态角有效性检查阈值 (rad, ~60°)
    'accel_freshness_thresh': 0.1,         # 加速度数据新鲜度阈值 (秒)
    
    # Jacobian 计算参数
    'min_velocity_for_jacobian': 0.01,     # Jacobian 计算的最小速度阈值 (m/s)
    
    # 测量噪声
    'measurement_noise': {
        'odom_position': 0.01,             # 里程计位置噪声
        'odom_velocity': 0.1,              # 里程计速度噪声
        'odom_orientation': 0.01,          # 里程计航向噪声
        'odom_angular_velocity': 0.05,     # 里程计角速度噪声
        'imu_accel': 0.5,                  # IMU 加速度噪声
        'imu_gyro': 0.01,                  # IMU 陀螺仪噪声
    },
    
    # 过程噪声
    'process_noise': {
        'position': 0.001,                 # 位置过程噪声
        'velocity': 0.1,                   # 速度过程噪声
        'orientation': 0.01,               # 航向过程噪声
        'angular_velocity': 0.1,           # 角速度过程噪声
        'imu_bias': 0.0001,                # IMU 偏置过程噪声
    },
    
    # 异常检测参数
    'anomaly_detection': {
        'drift_thresh': 0.1,               # IMU 漂移检测阈值 (rad/s)
        'jump_thresh': 0.5,                # 位置跳变检测阈值 (m)
        'covariance_explosion_thresh': 1000.0,  # 协方差爆炸检测阈值
        'innovation_anomaly_thresh': 10.0,      # 创新度异常检测阈值
    },
    
    # 协方差参数
    'covariance': {
        'min_eigenvalue': 1e-6,            # 最小特征值 (保证正定)
        'initial_value': 0.1,              # 初始协方差对角线值
    },
}

# EKF 配置验证规则
EKF_VALIDATION_RULES = {
    # 测量噪声 (必须为正数)
    'ekf.measurement_noise.odom_position': (1e-9, 10.0, 'Odom 位置测量噪声'),
    'ekf.measurement_noise.odom_velocity': (1e-9, 10.0, 'Odom 速度测量噪声'),
    'ekf.measurement_noise.odom_orientation': (1e-9, 10.0, 'Odom 航向测量噪声'),
    'ekf.measurement_noise.odom_angular_velocity': (1e-9, 10.0, 'Odom 角速度测量噪声'),
    'ekf.measurement_noise.imu_accel': (1e-9, 10.0, 'IMU 加速度测量噪声'),
    'ekf.measurement_noise.imu_gyro': (1e-9, 10.0, 'IMU 陀螺仪测量噪声'),
    # 过程噪声 (必须为正数)
    'ekf.process_noise.position': (1e-9, 1.0, '位置过程噪声'),
    'ekf.process_noise.velocity': (1e-9, 10.0, '速度过程噪声'),
    'ekf.process_noise.orientation': (1e-9, 1.0, '航向过程噪声'),
    'ekf.process_noise.angular_velocity': (1e-9, 10.0, '角速度过程噪声'),
    'ekf.process_noise.imu_bias': (1e-12, 0.1, 'IMU 偏置过程噪声'),
    # 异常检测阈值
    'ekf.anomaly_detection.drift_thresh': (0.001, 1.0, 'IMU 漂移检测阈值 (rad/s)'),
    'ekf.anomaly_detection.jump_thresh': (0.01, 10.0, '位置跳变检测阈值 (m)'),
    'ekf.anomaly_detection.covariance_explosion_thresh': (10.0, 100000.0, '协方差爆炸检测阈值'),
    'ekf.anomaly_detection.innovation_anomaly_thresh': (1.0, 100.0, '创新度异常检测阈值'),
}
