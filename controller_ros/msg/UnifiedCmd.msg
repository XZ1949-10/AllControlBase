# UnifiedCmd.msg
# 统一控制命令消息
#
# 支持多种平台类型:
# - 差速车: 使用 vx, omega
# - 全向车: 使用 vx, vy, omega
# - 无人机: 使用 vx, vy, vz, omega
#
# 坐标系说明 (通过 header.frame_id 指定):
# - "base_link": 机体坐标系 (差速车/阿克曼车)
# - "world" 或 "odom": 世界/里程计坐标系 (全向车/无人机)

std_msgs/Header header

# 线速度 (m/s)
float32 vx      # 前向速度 (差速车机体系) 或 X 方向速度 (全向/无人机世界系)
float32 vy      # Y 方向速度 (全向/无人机)
float32 vz      # Z 方向速度 (无人机)

# 角速度 (rad/s)
float32 omega   # 偏航角速度

# 命令是否有效
bool success

# 求解时间 (ms)
float32 solve_time_ms
