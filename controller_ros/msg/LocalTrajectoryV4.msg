# LocalTrajectoryV4.msg
# 局部轨迹消息 v4
#
# 支持两种模式:
# - Hard 模式: 只有位置点，速度从位置差分计算
# - Soft 模式: 位置点 + 显式速度，支持软约束

std_msgs/Header header

# 轨迹模式
# 0=MODE_TRACK (跟踪模式)
# 1=MODE_STOP (停止模式)
# 2=MODE_HOLD (保持模式)
uint8 mode

# 轨迹点 (3D 位置)
geometry_msgs/Point[] points

# 速度数组 (Soft 模式使用)
# 每个元素: [vx, vy, vz, wz]
# 长度应与 points 相同 * 4，或为空 (Hard 模式)
float32[] velocities_flat

# 轨迹点时间间隔 (秒)
float32 dt_sec

# 轨迹置信度 [0, 1]
float32 confidence

# 是否启用 Soft 约束
bool soft_enabled
