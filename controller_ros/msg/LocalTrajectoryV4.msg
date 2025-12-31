# LocalTrajectoryV4.msg
# 局部轨迹消息 v4
#
# 支持两种模式:
# - Hard 模式: 只有位置点，速度从位置差分计算
# - Soft 模式: 位置点 + 显式速度，支持软约束
#
# 速度数组格式说明:
# =================
# velocities_flat 是一个扁平化的速度数组，每 4 个元素表示一个轨迹点的速度:
#   [vx_0, vy_0, vz_0, wz_0, vx_1, vy_1, vz_1, wz_1, ...]
#
# 其中:
#   - vx, vy, vz: 线速度分量 (m/s)
#   - wz: 角速度 (rad/s)
#
# 长度约束:
#   - Soft 模式: len(velocities_flat) == len(points) * 4
#   - Hard 模式: len(velocities_flat) == 0
#
# 如果速度点数少于位置点数，适配器会自动填充:
#   - MODE_STOP/MODE_EMERGENCY: 使用零速度填充
#   - 其他模式: 使用衰减速度填充

std_msgs/Header header

# 轨迹模式 (与 universal_controller.core.enums.TrajectoryMode 对应)
# 0=MODE_TRACK (跟踪模式)
# 1=MODE_STOP (停止模式)
# 2=MODE_HOVER (悬停/保持模式)
# 3=MODE_EMERGENCY (紧急模式)
uint8 mode

# 轨迹点 (3D 位置)
# 坐标系由 header.frame_id 指定，通常为 base_link (局部坐标)
geometry_msgs/Point[] points

# 速度数组 (Soft 模式使用)
# 格式: [vx_0, vy_0, vz_0, wz_0, vx_1, vy_1, vz_1, wz_1, ...]
# 长度: len(points) * 4 (Soft 模式) 或 0 (Hard 模式)
float32[] velocities_flat

# 轨迹点时间间隔 (秒)
# 应与 MPC 时间步长 (mpc.dt) 一致
float32 dt_sec

# 轨迹置信度 [0, 1]
# 0 = 完全不可信，1 = 完全可信
float32 confidence

# 是否启用 Soft 约束
# true: 使用 velocities_flat 中的速度
# false: 从位置差分计算速度
bool soft_enabled
