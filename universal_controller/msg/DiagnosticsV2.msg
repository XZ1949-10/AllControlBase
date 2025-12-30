# DiagnosticsV2.msg
# 控制器诊断消息 v2
# 
# 用于发布控制器的完整诊断信息，包括:
# - 控制器状态
# - MPC 健康状态
# - 一致性指标
# - 状态估计器健康
# - 跟踪误差
# - 坐标变换状态
# - 超时状态
# - 控制命令

std_msgs/Header header

# 控制器状态 (ControllerState 枚举值)
# 0=INIT, 1=NORMAL, 2=SOFT_DISABLED, 3=MPC_DEGRADED, 
# 4=BACKUP_ACTIVE, 5=STOPPING, 6=STOPPED
uint8 state

# MPC 状态
bool mpc_success
float32 mpc_solve_time_ms
bool backup_active

# MPC 健康状态
float32 mpc_health_kkt_residual
float32 mpc_health_condition_number
uint16 mpc_health_consecutive_near_timeout
bool mpc_health_degradation_warning
bool mpc_health_can_recover

# 一致性指标
float32 consistency_curvature
float32 consistency_velocity_dir
float32 consistency_temporal
float32 consistency_alpha_soft
bool consistency_data_valid

# 状态估计器健康
float32 estimator_covariance_norm
float32 estimator_innovation_norm
float32 estimator_slip_probability
bool estimator_imu_drift_detected
float32[3] estimator_imu_bias
bool estimator_imu_available

# 跟踪误差
float32 tracking_lateral_error
float32 tracking_longitudinal_error
float32 tracking_heading_error
float32 tracking_prediction_error

# 坐标变换状态
bool transform_tf2_available
bool transform_tf2_injected
float32 transform_fallback_duration_ms
float32 transform_accumulated_drift
string transform_source_frame
string transform_target_frame
string transform_error_message

# 超时状态
bool timeout_odom
bool timeout_traj
bool timeout_traj_grace_exceeded
bool timeout_imu
float32 timeout_last_odom_age_ms
float32 timeout_last_traj_age_ms
float32 timeout_last_imu_age_ms
bool timeout_in_startup_grace

# 控制命令
float32 cmd_vx
float32 cmd_vy
float32 cmd_vz
float32 cmd_omega
string cmd_frame_id

# 过渡进度 [0, 1]
float32 transition_progress
