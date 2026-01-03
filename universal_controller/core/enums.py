"""枚举定义"""
from enum import IntEnum


class ControllerState(IntEnum):
    """控制器状态枚举"""
    INIT = 0
    NORMAL = 1
    SOFT_DISABLED = 2
    MPC_DEGRADED = 3
    BACKUP_ACTIVE = 4
    STOPPING = 5
    STOPPED = 6


class TrajectoryMode(IntEnum):
    """轨迹模式枚举"""
    MODE_TRACK = 0
    MODE_STOP = 1
    MODE_HOVER = 2
    MODE_EMERGENCY = 3


class TransformStatus(IntEnum):
    """坐标变换状态"""
    ESTIMATOR_DIRECT = 0
    TF2_OK = 1
    FALLBACK_OK = 2
    FALLBACK_WARNING = 3
    FALLBACK_CRITICAL = 4
    
    def is_critical(self) -> bool:
        return self == TransformStatus.FALLBACK_CRITICAL
    
    def is_fallback(self) -> bool:
        return self not in (TransformStatus.TF2_OK, TransformStatus.ESTIMATOR_DIRECT)


class HeadingMode(IntEnum):
    """全向车航向控制模式"""
    FOLLOW_VELOCITY = 0      # 朝向运动方向（默认）
    FIXED = 1                # 保持固定航向
    TARGET_POINT = 2         # 朝向目标点
    MANUAL = 3               # 外部指定航向


class PlatformType(IntEnum):
    """平台类型枚举"""
    ACKERMANN = 0
    DIFFERENTIAL = 1
    OMNI = 2
    QUADROTOR = 3
