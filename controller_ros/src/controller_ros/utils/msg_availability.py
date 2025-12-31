"""
消息可用性检查

统一管理 ROS 自定义消息的可用性检查，避免在多处重复检查。

使用方法:
    from controller_ros.utils.msg_availability import (
        CUSTOM_MSGS_AVAILABLE, 
        get_msg_import_error,
        LocalTrajectoryV4,
        UnifiedCmd,
        DiagnosticsV2,
        AttitudeCmd,
    )
    
    if not CUSTOM_MSGS_AVAILABLE:
        raise RuntimeError(f"Custom messages not available: {get_msg_import_error()}")
"""
import logging

logger = logging.getLogger(__name__)

# 消息可用性状态
CUSTOM_MSGS_AVAILABLE = False
_MSG_IMPORT_ERROR = None

# 消息类型引用
LocalTrajectoryV4 = None
UnifiedCmd = None
DiagnosticsV2 = None
AttitudeCmd = None

# 尝试导入所有自定义消息
try:
    from controller_ros.msg import (
        LocalTrajectoryV4 as _LocalTrajectoryV4,
        UnifiedCmd as _UnifiedCmd,
        DiagnosticsV2 as _DiagnosticsV2,
        AttitudeCmd as _AttitudeCmd,
    )
    LocalTrajectoryV4 = _LocalTrajectoryV4
    UnifiedCmd = _UnifiedCmd
    DiagnosticsV2 = _DiagnosticsV2
    AttitudeCmd = _AttitudeCmd
    CUSTOM_MSGS_AVAILABLE = True
    logger.debug("Custom messages loaded successfully")
except ImportError as e:
    _MSG_IMPORT_ERROR = str(e)
    logger.warning(f"Custom messages not available: {e}")


def get_msg_import_error() -> str:
    """获取消息导入错误信息"""
    return _MSG_IMPORT_ERROR or "Unknown error"


def check_msgs_available(raise_error: bool = False) -> bool:
    """
    检查自定义消息是否可用
    
    Args:
        raise_error: 如果为 True，消息不可用时抛出 RuntimeError
    
    Returns:
        消息是否可用
    
    Raises:
        RuntimeError: 当 raise_error=True 且消息不可用时
    """
    if not CUSTOM_MSGS_AVAILABLE and raise_error:
        raise RuntimeError(
            f"Custom messages not available: {get_msg_import_error()}. "
            f"Please build the package with 'catkin_make' or 'catkin build'."
        )
    return CUSTOM_MSGS_AVAILABLE
