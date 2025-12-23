# AttitudeCmd.msg
# 姿态命令消息 (仅四旋翼平台)
#
# 用于无人机姿态内环控制，将速度命令转换为姿态命令。
# 对应 universal_controller.core.data_types.AttitudeCommand

std_msgs/Header header

# 姿态角 (rad)
float32 roll                # 滚转角
float32 pitch               # 俯仰角
float32 yaw                 # 偏航角

# 推力 (归一化到悬停推力，1.0 = 悬停)
float32 thrust

# 航向模式
# 0 = FOLLOW_VELOCITY (朝向运动方向)
# 1 = FIXED (保持固定航向)
# 2 = TARGET_POINT (朝向目标点)
# 3 = MANUAL (外部指定航向)
uint8 yaw_mode

# 是否处于悬停状态
bool is_hovering
