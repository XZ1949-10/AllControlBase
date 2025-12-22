"""默认配置

本模块定义了通用控制器的所有默认配置参数，包括：
- 平台配置：支持阿克曼、差速、全向、四旋翼等平台类型
- 系统配置：控制频率、重力加速度等基础参数
- MPC 配置：模型预测控制器的预测时域、权重、求解器参数
- 安全配置：速度限制、加速度限制、状态机参数
- EKF 配置：扩展卡尔曼滤波器的噪声参数、自适应参数
- 其他模块配置：一致性检查、坐标变换、备份控制器等
"""
from typing import Dict, Any
from ..core.enums import PlatformType


# =============================================================================
# 平台配置
# =============================================================================
# 定义不同机器人平台的运动学特性和控制维度
# 状态向量维度定义: [px, py, pz, vx, vy, vz, theta, omega]
#                   [0,  1,  2,  3,  4,  5,  6,     7    ]
# =============================================================================
PLATFORM_CONFIG = {
    # 阿克曼转向车辆 (如汽车)
    "ackermann": {
        "type": PlatformType.ACKERMANN,
        "active_dims": [0, 1, 3, 6],      # 活跃状态维度: px, py, vx, theta
        "control_dims": [3, 7],            # 控制维度: vx (纵向速度), omega (角速度)
        "constraints": {
            "pz": 0,           # z 位置约束为 0 (地面车辆)
            "vy": 0,           # 横向速度约束为 0 (非完整约束)
            "vz": 0,           # z 速度约束为 0
            "curvature": True  # 启用曲率约束 (转向角限制)
        },
        "velocity_heading_coupled": True,  # 速度方向与航向耦合 (车头朝向即运动方向)
        "output_type": "differential",     # 输出类型: 差速 (v, omega)
        "output_frame": "base_link"        # 输出坐标系: 机体坐标系
    },
    # 差速驱动车辆 (如两轮机器人)
    "differential": {
        "type": PlatformType.DIFFERENTIAL,
        "active_dims": [0, 1, 3, 6, 7],    # 活跃状态维度: px, py, vx, theta, omega
        "control_dims": [3, 7],            # 控制维度: vx, omega
        "constraints": {
            "pz": 0,            # z 位置约束为 0
            "vy": 0,            # 横向速度约束为 0 (非完整约束)
            "vz": 0,            # z 速度约束为 0
            "curvature": False  # 无曲率约束 (可原地旋转)
        },
        "velocity_heading_coupled": True,  # 速度方向与航向耦合
        "output_type": "differential",     # 输出类型: 差速
        "output_frame": "base_link"        # 输出坐标系: 机体坐标系
    },
    # 全向移动平台 (如麦克纳姆轮)
    "omni": {
        "type": PlatformType.OMNI,
        "active_dims": [0, 1, 3, 4, 6, 7],  # 活跃状态维度: px, py, vx, vy, theta, omega
        "control_dims": [3, 4, 7],          # 控制维度: vx, vy, omega
        "constraints": {
            "pz": 0,  # z 位置约束为 0
            "vz": 0   # z 速度约束为 0
        },
        "velocity_heading_coupled": False,  # 速度方向与航向解耦 (可横向移动)
        "output_type": "omni",              # 输出类型: 全向
        "output_frame": "world"             # 输出坐标系: 世界坐标系
    },
    # 四旋翼无人机
    "quadrotor": {
        "type": PlatformType.QUADROTOR,
        "active_dims": [0, 1, 2, 3, 4, 5, 6, 7],  # 全部状态维度都活跃
        "control_dims": [3, 4, 5, 7],              # 控制维度: vx, vy, vz, omega (yaw rate)
        "constraints": {},                         # 无运动学约束 (可自由飞行)
        "attitude_interface": True,                # 使用姿态接口 (输出期望姿态角)
        "velocity_heading_coupled": False,         # 速度方向与航向解耦
        "output_type": "3d",                       # 输出类型: 三维
        "output_frame": "world"                    # 输出坐标系: 世界坐标系
    }
}


DEFAULT_CONFIG = {
    'system': {
        'ctrl_freq': 50,
        'platform': 'differential',
        'gravity': 9.81,  # 重力加速度 (m/s²)，用于 EKF 和姿态控制
        # 控制循环暂停检测
        'long_pause_threshold': 0.5,  # 长时间暂停检测阈值 (秒)，超过此值使用多步预测
        'ekf_reset_threshold': 2.0,   # EKF 重置阈值 (秒)，超过此值重置 EKF
    },
    'mpc': {
        'horizon': 20,
        'horizon_degraded': 10,
        'dt': 0.02,
        'weights': {
            'position': 10.0,
            'velocity': 1.0,
            'heading': 5.0,
            'control_v': 0.1,
            'control_omega': 0.1,
        },
        'health_monitor': {
            'time_warning_thresh_ms': 8,
            'time_critical_thresh_ms': 15,
            'time_recovery_thresh_ms': 6,
            'condition_number_thresh': 1e8,
            'condition_number_recovery': 1e5,
            'kkt_residual_thresh': 1e-3,
            'consecutive_warning_limit': 3,
            'consecutive_recovery_limit': 5,
            'recovery_multiplier': 2.0,  # 备选恢复条件的倍数
            'consecutive_good_for_decay': 2,  # 连续良好次数达到此值后开始衰减超时计数
            'timeout_decay_rate': 2,  # 超时计数衰减速率
        },
        # Fallback 求解器参数
        'fallback': {
            'lookahead_steps': 3,  # 前瞻步数
            'heading_kp': 1.5,  # 航向控制增益
            'max_curvature': 5.0,  # 最大曲率限制 (1/m)
            'min_distance_thresh': 0.1,  # 最小距离阈值 (m)，用于避免除零
            'min_turn_speed': 0.1,  # 阿克曼车辆最小转向速度 (m/s)
            'default_speed_ratio': 0.5,  # 无 soft 速度时的默认速度比例
        },
        # ACADOS 求解器参数
        'solver': {
            'nlp_max_iter': 50,  # NLP 求解器最大迭代次数
            'qp_solver': 'PARTIAL_CONDENSING_HPIPM',  # QP 求解器类型
            'integrator_type': 'ERK',  # 积分器类型
            'nlp_solver_type': 'SQP_RTI',  # NLP 求解器类型
        },
    },
    'watchdog': {
        'odom_timeout_ms': 200,
        'traj_timeout_ms': 200,
        'traj_grace_ms': 100,
        'imu_timeout_ms': 100,
        'startup_grace_ms': 1000,
    },
    'diagnostics': {
        'topic': '/controller/diagnostics',  # 诊断话题名称
        'cmd_topic': '/cmd_unified',  # 控制命令话题名称
    },
    'consistency': {
        'kappa_thresh': 0.5,
        'v_dir_thresh': 0.8,
        'temporal_smooth_thresh': 0.5,
        'low_speed_thresh': 0.1,
        'alpha_min': 0.1,
        'max_curvature': 10.0,  # 曲率计算的最大值限制 (1/m)
        'temporal_window_size': 10,  # 时序平滑度计算的滑动窗口大小
        'weights': {
            'kappa': 1.0,
            'velocity': 1.5,
            'temporal': 0.8,
        },
    },
    'safety': {
        'v_stop_thresh': 0.05,
        'vz_stop_thresh': 0.1,
        'stopping_timeout': 5.0,
        'emergency_decel': 3.0,
        'velocity_margin': 1.1,
        'accel_margin': 1.5,
        # 加速度滤波参数
        'accel_filter_window': 3,  # 滑动窗口大小
        'accel_filter_alpha': 0.3,  # 低通滤波系数
        'accel_filter_warmup_alpha': 0.5,  # 滤波器预热期间的系数（加速收敛）
        'accel_filter_warmup_period': 3,  # 滤波器预热期长度（通常等于窗口大小）
        'min_dt_for_accel': 0.001,  # 加速度计算的最小时间间隔 (秒)
        'max_dt_for_accel': 1.0,  # 加速度计算的最大时间间隔 (秒)
        'state_machine': {
            'alpha_recovery_thresh': 5,
            'alpha_recovery_value': 0.3,
            'alpha_disable_thresh': 0.1,
            'mpc_recovery_thresh': 5,
            'mpc_fail_window_size': 10,  # MPC 失败检测滑动窗口大小
            'mpc_fail_thresh': 3,  # 窗口内失败次数阈值
            'mpc_fail_ratio_thresh': 0.5,  # 失败率阈值
            'mpc_recovery_history_min': 3,  # MPC 恢复检测最小历史记录数
            'mpc_recovery_recent_count': 5,  # MPC 恢复检测最近检查次数
            'mpc_recovery_tolerance': 1,  # MPC 恢复检测容错次数 (最近 N 次中允许失败的次数)
            'mpc_recovery_success_ratio': 0.8,  # MPC 恢复检测成功率阈值
        },
    },
    'transform': {
        'target_frame': 'odom',
        'source_frame': 'odom',  # 轨迹和状态都在 odom 坐标系，无需变换
        'fallback_duration_limit_ms': 500,
        'fallback_critical_limit_ms': 1000,
        'tf2_timeout_ms': 10,
        'drift_estimation_enabled': False,  # 无外部定位，禁用漂移估计
        'recovery_correction_enabled': False,  # 无外部定位，禁用恢复校正
        'drift_rate': 0.01,
        'max_drift_dt': 0.5,  # 漂移估计最大时间间隔 (秒)
        'drift_correction_thresh': 0.01,  # 漂移校正阈值 (米/弧度)
    },
    'transition': {
        'type': 'exponential',
        'tau': 0.1,
        'max_duration': 0.5,
        'completion_threshold': 0.95,
        'duration': 0.2,
    },
    'backup': {
        'lookahead_dist': 1.0,
        'min_lookahead': 0.5,
        'max_lookahead': 3.0,
        'lookahead_ratio': 0.5,
        'kp_z': 1.0,
        'kp_heading': 1.5,
        'heading_mode': 'follow_velocity',
        'dt': 0.02,
        # Pure Pursuit 控制参数
        'heading_error_thresh': 1.047,  # 航向误差阈值 (rad, ~60°)
        'pure_pursuit_angle_thresh': 1.047,  # Pure Pursuit 模式角度阈值 (rad, ~60°)
        'heading_control_angle_thresh': 1.571,  # 航向控制模式角度阈值 (rad, ~90°)
        'max_curvature': 5.0,  # 最大曲率限制 (1/m)
        'min_turn_speed': 0.1,  # 阿克曼车辆最小转向速度 (m/s)
        'default_speed_ratio': 0.5,  # 无 soft 速度时的默认速度比例 (相对于 v_max)
        # 低速过渡参数
        'low_speed_transition_factor': 0.5,  # 低速过渡区域因子 (相对于 v_low_thresh)
        # 曲率速度限制阈值
        'curvature_speed_limit_thresh': 0.1,  # 曲率阈值，超过此值时限制速度 (1/m)
        # 距离阈值
        'min_distance_thresh': 0.1,  # 最小距离阈值，用于判断是否到达目标 (m)
        # 角速度变化率限制 (用于防止目标点在正后方时的跳变)
        # 默认值 None 表示使用 alpha_max * dt
        'omega_rate_limit': None,
    },
    'constraints': {
        'v_max': 2.0,
        'v_min': 0.0,
        'omega_max': 2.0,
        'omega_max_low': 1.0,
        'v_low_thresh': 0.1,
        'a_max': 1.5,
        'az_max': 1.0,
        'alpha_max': 3.0,
        'vx_max': 1.5,
        'vx_min': -1.5,
        'vy_max': 1.5,
        'vy_min': -1.5,
        'vz_max': 2.0,
    },
    'ekf': {
        'use_odom_orientation_fallback': True,
        'theta_covariance_fallback_thresh': 0.5,
        'imu_motion_compensation': False,
        'adaptive': {
            'base_slip_thresh': 2.0,
            'slip_velocity_factor': 0.5,
            'slip_covariance_scale': 10.0,
            'stationary_covariance_scale': 0.1,
            'stationary_thresh': 0.05,
            'slip_probability_k_factor': 5.0,  # 打滑概率 sigmoid 函数的斜率因子
            'slip_history_window': 20,  # 打滑概率计算的滑动窗口大小
        },
        # IMU 相关参数
        'max_tilt_angle': 1.047,  # IMU 姿态角有效性检查阈值 (rad, ~60°)
        'accel_freshness_thresh': 0.1,  # 加速度数据新鲜度阈值 (秒)
        # Jacobian 计算参数
        'min_velocity_for_jacobian': 0.01,  # Jacobian 计算的最小速度阈值 (m/s)
        'measurement_noise': {
            'odom_position': 0.01,
            'odom_velocity': 0.1,
            'imu_accel': 0.5,
            'imu_gyro': 0.01,
        },
        'process_noise': {
            'position': 0.001,
            'velocity': 0.1,
            'orientation': 0.01,
            'angular_velocity': 0.1,
            'imu_bias': 0.0001,
        },
        'anomaly_detection': {
            'drift_thresh': 0.1,
            'jump_thresh': 0.5,
        },
        'covariance': {
            'min_eigenvalue': 1e-6,
            'initial_value': 0.1,  # 初始协方差对角线值
        },
    },
    # F14: 无人机姿态控制配置
    'attitude': {
        'mass': 1.5,  # kg
        # 注意: 重力加速度统一使用 system.gravity，此处已废弃
        # 'gravity': 9.81,  # 已废弃，请使用 system.gravity
        # F14.2: 姿态角速度限制
        'roll_rate_max': 3.0,  # rad/s
        'pitch_rate_max': 3.0,  # rad/s
        'yaw_rate_max': 2.0,  # rad/s
        # 姿态角限制
        'roll_max': 0.5,  # rad (~30 deg)
        'pitch_max': 0.5,  # rad (~30 deg)
        # 速度控制增益
        'kp_vx': 0.5,
        'kp_vy': 0.5,
        'kp_vz': 1.0,
        # F14.3: 悬停 yaw 漂移补偿
        'hover_yaw_compensation': True,
        'hover_speed_thresh': 0.1,  # m/s (水平速度阈值)
        'hover_vz_thresh': 0.05,  # m/s (垂直速度阈值，更严格)
        'yaw_drift_rate': 0.001,  # rad/s
        # 悬停检测滞后参数
        'hover_enter_factor': 1.0,  # 进入悬停的阈值因子
        'hover_exit_factor': 1.5,  # 退出悬停的阈值因子
        'hover_cmd_exit_factor': 2.0,  # 命令速度退出悬停的阈值因子
        'hover_debounce_time': 0.1,  # 悬停状态切换去抖动时间 (秒)
        # F14.4: 位置-姿态解耦
        'position_attitude_decoupled': False,
        # 推力限制 (归一化到悬停推力)
        'thrust_min': 0.1,  # 最小推力 (防止自由落体)
        'thrust_max': 2.0,  # 最大推力
        'thrust_rate_max': 2.0,  # 推力变化率限制 (每秒)
        'min_thrust_factor': 0.1,  # 最小推力加速度因子 (相对于重力)
        'attitude_factor_min': 0.1,  # 姿态角饱和后推力重计算的最小因子
        'invert_pitch_sign': True,  # pitch 符号反转 (True: 正 pitch = 前向加速度)
        'dt': 0.02,
    },
}


def get_config_value(config: Dict[str, Any], key_path: str, default: Any = None) -> Any:
    """从配置字典中获取值，支持点分隔的路径"""
    keys = key_path.split('.')
    value = config
    for key in keys:
        if isinstance(value, dict) and key in value:
            value = value[key]
        else:
            default_value = DEFAULT_CONFIG
            for k in keys:
                if isinstance(default_value, dict) and k in default_value:
                    default_value = default_value[k]
                else:
                    return default
            return default_value
    return value


# =============================================================================
# 配置验证
# =============================================================================

class ConfigValidationError(ValueError):
    """配置验证错误"""
    pass


# 配置验证规则定义
# 格式: 'key_path': (min_value, max_value, description)
# None 表示无限制
CONFIG_VALIDATION_RULES = {
    # 系统配置
    'system.ctrl_freq': (1, 1000, '控制频率 (Hz)'),
    'system.gravity': (0.1, 20.0, '重力加速度 (m/s²)'),
    'system.long_pause_threshold': (0.01, 10.0, '长时间暂停检测阈值 (秒)'),
    'system.ekf_reset_threshold': (0.1, 60.0, 'EKF 重置阈值 (秒)'),
    
    # MPC 配置
    'mpc.horizon': (1, 100, 'MPC 预测时域'),
    'mpc.horizon_degraded': (1, 100, 'MPC 降级预测时域'),
    'mpc.dt': (0.001, 1.0, 'MPC 时间步长 (秒)'),
    'mpc.fallback.min_distance_thresh': (0.001, 1.0, 'Fallback 最小距离阈值 (m)'),
    'mpc.fallback.min_turn_speed': (0.0, 1.0, 'Fallback 最小转向速度 (m/s)'),
    'mpc.fallback.default_speed_ratio': (0.0, 1.0, 'Fallback 默认速度比例'),
    
    # MPC 权重配置 (必须为正数)
    'mpc.weights.position': (0.0, None, 'MPC 位置权重'),
    'mpc.weights.velocity': (0.0, None, 'MPC 速度权重'),
    'mpc.weights.heading': (0.0, None, 'MPC 航向权重'),
    'mpc.weights.control_v': (0.0, None, 'MPC 速度控制权重'),
    'mpc.weights.control_omega': (0.0, None, 'MPC 角速度控制权重'),
    
    # 约束配置
    'constraints.v_max': (0.01, 100.0, '最大速度 (m/s)'),
    'constraints.omega_max': (0.01, 50.0, '最大角速度 (rad/s)'),
    'constraints.a_max': (0.01, 50.0, '最大加速度 (m/s²)'),
    'constraints.alpha_max': (0.01, 100.0, '最大角加速度 (rad/s²)'),
    
    # 超时配置
    'watchdog.odom_timeout_ms': (1, 10000, '里程计超时 (ms)'),
    'watchdog.traj_timeout_ms': (1, 10000, '轨迹超时 (ms)'),
    'watchdog.imu_timeout_ms': (1, 10000, 'IMU 超时 (ms)'),
    
    # 安全配置
    'safety.v_stop_thresh': (0.0, 1.0, '停止速度阈值 (m/s)'),
    'safety.stopping_timeout': (0.1, 60.0, '停止超时 (秒)'),
    'safety.velocity_margin': (1.0, 2.0, '速度裕度'),
    'safety.accel_margin': (1.0, 3.0, '加速度裕度'),
    'safety.accel_filter_window': (1, 20, '加速度滤波窗口大小'),
    'safety.accel_filter_alpha': (0.0, 1.0, '加速度滤波系数'),
    'safety.accel_filter_warmup_period': (1, 20, '滤波器预热期长度'),
    
    # 状态机配置
    'safety.state_machine.mpc_recovery_success_ratio': (0.0, 1.0, 'MPC 恢复成功率阈值'),
    
    # 一致性配置
    'consistency.alpha_min': (0.0, 1.0, '最小 alpha 值'),
    'consistency.kappa_thresh': (0.0, 10.0, '曲率一致性阈值'),
    'consistency.v_dir_thresh': (0.0, 1.0, '速度方向一致性阈值'),
    
    # 一致性权重配置 (必须为非负数)
    'consistency.weights.kappa': (0.0, None, '曲率一致性权重'),
    'consistency.weights.velocity': (0.0, None, '速度方向一致性权重'),
    'consistency.weights.temporal': (0.0, None, '时序平滑度权重'),
    
    # 过渡配置
    'transition.tau': (0.001, 10.0, '过渡时间常数 (秒)'),
    'transition.max_duration': (0.01, 10.0, '最大过渡时长 (秒)'),
    'transition.completion_threshold': (0.5, 1.0, '过渡完成阈值'),
    
    # 备份控制器配置
    'backup.lookahead_dist': (0.1, 10.0, '前瞻距离 (m)'),
    'backup.min_lookahead': (0.01, 5.0, '最小前瞻距离 (m)'),
    'backup.max_lookahead': (0.5, 20.0, '最大前瞻距离 (m)'),
    'backup.kp_heading': (0.1, 10.0, '航向控制增益'),
    
    # 姿态控制配置
    'attitude.mass': (0.01, 100.0, '质量 (kg)'),
    'attitude.roll_max': (0.01, 1.57, '最大滚转角 (rad)'),
    'attitude.pitch_max': (0.01, 1.57, '最大俯仰角 (rad)'),
    'attitude.thrust_min': (0.01, 1.0, '最小推力'),
    'attitude.thrust_max': (1.0, 10.0, '最大推力'),
    
    # EKF 测量噪声配置 (必须为正数)
    'ekf.measurement_noise.odom_position': (1e-9, 10.0, 'Odom 位置测量噪声'),
    'ekf.measurement_noise.odom_velocity': (1e-9, 10.0, 'Odom 速度测量噪声'),
    'ekf.measurement_noise.imu_accel': (1e-9, 10.0, 'IMU 加速度测量噪声'),
    'ekf.measurement_noise.imu_gyro': (1e-9, 10.0, 'IMU 陀螺仪测量噪声'),
    
    # EKF 过程噪声配置 (必须为正数)
    'ekf.process_noise.position': (1e-9, 1.0, '位置过程噪声'),
    'ekf.process_noise.velocity': (1e-9, 10.0, '速度过程噪声'),
    'ekf.process_noise.orientation': (1e-9, 1.0, '航向过程噪声'),
    'ekf.process_noise.angular_velocity': (1e-9, 10.0, '角速度过程噪声'),
    'ekf.process_noise.imu_bias': (1e-12, 0.1, 'IMU 偏置过程噪声'),
}


def validate_config(config: Dict[str, Any], raise_on_error: bool = True) -> list:
    """
    验证配置参数
    
    Args:
        config: 配置字典
        raise_on_error: 是否在发现错误时抛出异常
    
    Returns:
        错误列表，每个元素为 (key_path, error_message)
    
    Raises:
        ConfigValidationError: 当 raise_on_error=True 且发现错误时
    
    Example:
        >>> config = DEFAULT_CONFIG.copy()
        >>> config['mpc']['horizon'] = -1
        >>> errors = validate_config(config, raise_on_error=False)
        >>> print(errors)
        [('mpc.horizon', 'MPC 预测时域 值 -1 超出范围 [1, 100]')]
    """
    errors = []
    
    for key_path, (min_val, max_val, description) in CONFIG_VALIDATION_RULES.items():
        value = get_config_value(config, key_path)
        
        if value is None:
            continue  # 使用默认值，跳过验证
        
        # 类型检查
        if not isinstance(value, (int, float)):
            errors.append((key_path, f'{description} 类型错误，期望数值，实际为 {type(value).__name__}'))
            continue
        
        # 范围检查
        if min_val is not None and value < min_val:
            errors.append((key_path, f'{description} 值 {value} 小于最小值 {min_val}'))
        elif max_val is not None and value > max_val:
            errors.append((key_path, f'{description} 值 {value} 大于最大值 {max_val}'))
    
    # 逻辑一致性检查
    logical_errors = _validate_logical_consistency(config)
    errors.extend(logical_errors)
    
    if errors and raise_on_error:
        error_messages = '\n'.join([f'  - {key}: {msg}' for key, msg in errors])
        raise ConfigValidationError(f'配置验证失败:\n{error_messages}')
    
    return errors


def _validate_logical_consistency(config: Dict[str, Any]) -> list:
    """
    验证配置的逻辑一致性
    
    检查配置参数之间的逻辑关系，例如：
    - min_lookahead < max_lookahead
    - horizon_degraded <= horizon
    - v_min <= v_max
    """
    errors = []
    
    def _is_numeric(value) -> bool:
        """检查值是否为数值类型"""
        return isinstance(value, (int, float)) and not isinstance(value, bool)
    
    # MPC horizon 一致性
    horizon = get_config_value(config, 'mpc.horizon')
    horizon_degraded = get_config_value(config, 'mpc.horizon_degraded')
    if _is_numeric(horizon) and _is_numeric(horizon_degraded):
        if horizon_degraded > horizon:
            errors.append(('mpc.horizon_degraded', 
                          f'降级 horizon ({horizon_degraded}) 不应大于正常 horizon ({horizon})'))
    
    # 前瞻距离一致性
    min_lookahead = get_config_value(config, 'backup.min_lookahead')
    max_lookahead = get_config_value(config, 'backup.max_lookahead')
    if _is_numeric(min_lookahead) and _is_numeric(max_lookahead):
        if min_lookahead > max_lookahead:
            errors.append(('backup.min_lookahead', 
                          f'最小前瞻距离 ({min_lookahead}) 不应大于最大前瞻距离 ({max_lookahead})'))
    
    # 速度约束一致性
    v_min = get_config_value(config, 'constraints.v_min')
    v_max = get_config_value(config, 'constraints.v_max')
    if _is_numeric(v_min) and _is_numeric(v_max):
        if v_min > v_max:
            errors.append(('constraints.v_min', 
                          f'最小速度 ({v_min}) 不应大于最大速度 ({v_max})'))
    
    # 推力约束一致性
    thrust_min = get_config_value(config, 'attitude.thrust_min')
    thrust_max = get_config_value(config, 'attitude.thrust_max')
    if _is_numeric(thrust_min) and _is_numeric(thrust_max):
        if thrust_min > thrust_max:
            errors.append(('attitude.thrust_min', 
                          f'最小推力 ({thrust_min}) 不应大于最大推力 ({thrust_max})'))
    
    # 超时一致性
    traj_timeout = get_config_value(config, 'watchdog.traj_timeout_ms')
    traj_grace = get_config_value(config, 'watchdog.traj_grace_ms')
    if _is_numeric(traj_timeout) and _is_numeric(traj_grace):
        if traj_grace > traj_timeout:
            errors.append(('watchdog.traj_grace_ms', 
                          f'轨迹宽限期 ({traj_grace}ms) 不应大于轨迹超时 ({traj_timeout}ms)'))
    
    return errors
