"""状态向量索引定义"""
from enum import IntEnum

class StateIdx(IntEnum):
    """EKF 状态向量索引"""
    X = 0           # 位置 X
    Y = 1           # 位置 Y
    Z = 2           # 位置 Z
    VX = 3          # 速度 X (机体)
    VY = 4          # 速度 Y (机体)
    VZ = 5          # 速度 Z (机体)
    YAW = 6         # 航向角 (Yaw)
    YAW_RATE = 7    # 角速度 (Yaw Rate)
    ACCEL_BIAS_X = 8    # 加速度计偏置 X
    ACCEL_BIAS_Y = 9    # 加速度计偏置 Y
    ACCEL_BIAS_Z = 10   # 加速度计偏置 Z

class MPCStateIdx(IntEnum):
    """MPC 状态向量索引 (自动同步 StateIdx)"""
    X = StateIdx.X
    Y = StateIdx.Y
    Z = StateIdx.Z
    VX = StateIdx.VX
    VY = StateIdx.VY
    VZ = StateIdx.VZ
    THETA = StateIdx.YAW
    OMEGA = StateIdx.YAW_RATE

class MPCInputIdx(IntEnum):
    """MPC 控制输入索引"""
    ACCEL_X = 0
    ACCEL_Y = 1
    ACCEL_Z = 2
    ALPHA = 3

class MPCSlices:
    """MPC 矩阵切片索引常量"""
    # YRef / State indices
    POS = slice(0, 3)     # x, y, z
    VEL = slice(3, 6)     # vx, vy, vz
    POSE_VEL = slice(0, 7) # x, y, z, vx, vy, vz, theta (for parameters)
    
    # Cost matrix blocks -> slice(start, end)
    # yref structure: [px, py, pz, vx, vy, vz, theta, omega, ax, ay, az, alpha]
    REF_POS = slice(0, 3)
    REF_VEL = slice(3, 6)
    REF_THETA = 6
    REF_OMEGA = 7
    REF_CONTROLS = slice(8, 12)
    
    # Cost weights
    WEIGHT_POS = slice(0, 3)
    WEIGHT_VEL = slice(3, 6)
    WEIGHT_HEADING = 6
    
    # Input constraints
    CONST_ACCEL = slice(0, 3)
    CONST_ALPHA = 3
    
    # State constraints
    CONST_STATE_VEL = slice(3, 6) # vx, vy, vz
