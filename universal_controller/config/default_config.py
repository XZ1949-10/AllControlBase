"""默认配置"""
from typing import Dict, Any
from ..core.enums import PlatformType


PLATFORM_CONFIG = {
    "ackermann": {
        "type": PlatformType.ACKERMANN,
        "active_dims": [0, 1, 3, 6],
        "control_dims": [3, 7],
        "constraints": {"pz": 0, "vy": 0, "vz": 0, "curvature": True},
        "velocity_heading_coupled": True,
        "output_type": "differential",
        "output_frame": "base_link"
    },
    "differential": {
        "type": PlatformType.DIFFERENTIAL,
        "active_dims": [0, 1, 3, 6, 7],
        "control_dims": [3, 7],
        "constraints": {"pz": 0, "vy": 0, "vz": 0, "curvature": False},
        "velocity_heading_coupled": True,
        "output_type": "differential",
        "output_frame": "base_link"
    },
    "omni": {
        "type": PlatformType.OMNI,
        "active_dims": [0, 1, 3, 4, 6, 7],
        "control_dims": [3, 4, 7],
        "constraints": {"pz": 0, "vz": 0},
        "velocity_heading_coupled": False,
        "output_type": "omni",
        "output_frame": "world"
    },
    "quadrotor": {
        "type": PlatformType.QUADROTOR,
        "active_dims": [0, 1, 2, 3, 4, 5, 6, 7],
        "control_dims": [3, 4, 5, 7],
        "constraints": {},
        "attitude_interface": True,
        "velocity_heading_coupled": False,
        "output_type": "3d",
        "output_frame": "world"
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
        },
        # Fallback 求解器参数
        'fallback': {
            'lookahead_steps': 3,  # 前瞻步数
            'heading_kp': 1.5,  # 航向控制增益
            'max_curvature': 5.0,  # 最大曲率限制 (1/m)
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
