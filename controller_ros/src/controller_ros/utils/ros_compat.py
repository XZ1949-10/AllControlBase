"""
ROS 兼容层

支持 ROS1 (Noetic) 和 ROS2 (Humble) 双版本运行。

使用方法:
    from controller_ros.utils.ros_compat import (
        ROS_VERSION, ROS_AVAILABLE, TF2_AVAILABLE,
        get_time_sec, ros_time_to_sec, sec_to_ros_time
    )
"""
import time
import logging
from typing import Any, Callable, Optional

logger = logging.getLogger(__name__)

# ============================================================================
# ROS 版本检测
# ============================================================================

ROS_VERSION = 0  # 0=无, 1=ROS1, 2=ROS2
ROS_AVAILABLE = False
TF2_AVAILABLE = False

# 尝试检测 ROS2
try:
    import rclpy
    from rclpy.node import Node
    ROS_VERSION = 2
    ROS_AVAILABLE = True
    logger.debug("ROS2 detected")
except ImportError:
    pass

# 尝试检测 ROS1
if ROS_VERSION == 0:
    try:
        import rospy
        ROS_VERSION = 1
        ROS_AVAILABLE = True
        logger.debug("ROS1 detected")
    except ImportError:
        pass

# 检测 TF2
if ROS_VERSION == 2:
    try:
        import tf2_ros
        from tf2_ros.buffer import Buffer
        from tf2_ros.transform_listener import TransformListener
        TF2_AVAILABLE = True
    except ImportError:
        TF2_AVAILABLE = False
elif ROS_VERSION == 1:
    try:
        import tf2_ros
        TF2_AVAILABLE = True
    except ImportError:
        TF2_AVAILABLE = False


# ============================================================================
# 时间工具 (统一接口)
# ============================================================================

def get_time_sec(node=None) -> float:
    """
    获取当前 ROS 时间（秒）- 统一接口
    
    支持仿真时间模式。当时钟返回 0 时（仿真时间未初始化），回退到系统时间。
    
    Args:
        node: ROS2 节点实例 (ROS2 必需，ROS1 可选)
    
    Returns:
        当前时间（秒）
    
    Note:
        - ROS1: 不需要 node 参数，直接使用 rospy.Time.now()
        - ROS2: 需要 node 参数来获取节点时钟。如果 node 为 None，回退到系统时间。
    """
    if ROS_VERSION == 1:
        try:
            import rospy
            ros_time = rospy.Time.now().to_sec()
            # 仿真时间模式下可能为 0
            return ros_time if ros_time > 0 else time.time()
        except Exception:
            return time.time()
    elif ROS_VERSION == 2:
        if node is not None:
            try:
                clock_time = node.get_clock().now().nanoseconds * 1e-9
                # 仿真时间模式下可能为 0
                return clock_time if clock_time > 0 else time.time()
            except Exception:
                return time.time()
        else:
            # ROS2 需要节点实例来获取时钟，回退到系统时间
            # 这在某些场景下是预期行为（如单元测试），不记录警告
            return time.time()
    return time.time()


# 保留旧函数名作为别名，保持向后兼容
def get_current_time() -> float:
    """
    获取当前 ROS 时间（秒）
    
    .. deprecated:: 1.0.0
        此函数已废弃，请使用 :func:`get_time_sec` 替代。
        将在未来版本中移除。
    
    Returns:
        当前时间（秒）
    """
    import warnings
    warnings.warn(
        "get_current_time() is deprecated, use get_time_sec() instead. "
        "This function will be removed in a future version.",
        DeprecationWarning,
        stacklevel=2
    )
    return get_time_sec(None)


def get_monotonic_time() -> float:
    """获取单调时钟时间（秒），用于计算时间间隔"""
    return time.monotonic()


def ros_time_to_sec(stamp) -> float:
    """
    将 ROS 时间戳转换为秒
    
    支持 ROS1 (secs/nsecs 或 to_sec()) 和 ROS2 (sec/nanosec) 格式
    也支持测试环境中的 Mock 对象
    """
    if ROS_VERSION == 1:
        # ROS1: rospy.Time 有 to_sec() 方法
        if hasattr(stamp, 'to_sec'):
            return stamp.to_sec()
        # 或者直接访问属性
        if hasattr(stamp, 'secs') and hasattr(stamp, 'nsecs'):
            return stamp.secs + stamp.nsecs * 1e-9
    elif ROS_VERSION == 2:
        # ROS2: builtin_interfaces/Time
        if hasattr(stamp, 'sec') and hasattr(stamp, 'nanosec'):
            return stamp.sec + stamp.nanosec * 1e-9
    
    # 非 ROS 环境或 Mock 对象：尝试通用属性访问
    # 支持 ROS2 风格 (sec/nanosec)
    if hasattr(stamp, 'sec') and hasattr(stamp, 'nanosec'):
        return stamp.sec + stamp.nanosec * 1e-9
    # 支持 ROS1 风格 (secs/nsecs)
    if hasattr(stamp, 'secs') and hasattr(stamp, 'nsecs'):
        return stamp.secs + stamp.nsecs * 1e-9
    # 支持 to_sec() 方法
    if hasattr(stamp, 'to_sec'):
        return stamp.to_sec()
    
    # 最后尝试直接转换
    return float(stamp)


def sec_to_ros_time(sec: float):
    """
    将秒转换为 ROS 时间戳
    
    支持 ROS1 和 ROS2
    """
    if ROS_VERSION == 1:
        import rospy
        return rospy.Time.from_sec(sec)
    elif ROS_VERSION == 2:
        from builtin_interfaces.msg import Time
        t = Time()
        t.sec = int(sec)
        t.nanosec = int((sec - t.sec) * 1e9)
        return t
    return sec


# ============================================================================
# 日志工具
# ============================================================================

def _setup_logging_for_ros():
    """配置 logging 以便在 ROS 环境下正确输出"""
    if ROS_VERSION == 1:
        # ROS1: 配置 logging 使用 rospy 的日志系统
        import rospy
        
        class RospyHandler(logging.Handler):
            """将 Python logging 转发到 rospy"""
            def emit(self, record):
                try:
                    msg = self.format(record)
                    if record.levelno >= logging.ERROR:
                        rospy.logerr(msg)
                    elif record.levelno >= logging.WARNING:
                        rospy.logwarn(msg)
                    elif record.levelno >= logging.INFO:
                        rospy.loginfo(msg)
                    else:
                        rospy.logdebug(msg)
                except Exception:
                    self.handleError(record)
        
        # 为 controller_ros 模块配置 handler
        ros_handler = RospyHandler()
        ros_handler.setFormatter(logging.Formatter('%(name)s: %(message)s'))
        
        for module_name in ['controller_ros', 'universal_controller']:
            module_logger = logging.getLogger(module_name)
            # 避免重复添加 handler
            if not any(isinstance(h, RospyHandler) for h in module_logger.handlers):
                module_logger.addHandler(ros_handler)
                module_logger.setLevel(logging.DEBUG)


# 在模块加载时配置日志
if ROS_VERSION == 1:
    try:
        _setup_logging_for_ros()
    except Exception:
        pass  # 如果 rospy 未初始化，跳过配置


def log_info(msg: str):
    """记录信息日志"""
    if ROS_VERSION == 1:
        import rospy
        rospy.loginfo(msg)
    elif ROS_VERSION == 2:
        logger.info(msg)
    else:
        print(f"[INFO] {msg}")


def log_warn(msg: str):
    """记录警告日志"""
    if ROS_VERSION == 1:
        import rospy
        rospy.logwarn(msg)
    elif ROS_VERSION == 2:
        logger.warning(msg)
    else:
        print(f"[WARN] {msg}")


def log_error(msg: str):
    """记录错误日志"""
    if ROS_VERSION == 1:
        import rospy
        rospy.logerr(msg)
    elif ROS_VERSION == 2:
        logger.error(msg)
    else:
        print(f"[ERROR] {msg}")


def log_warn_throttle(period: float, msg: str):
    """节流警告日志"""
    if ROS_VERSION == 1:
        import rospy
        rospy.logwarn_throttle(period, msg)
    elif ROS_VERSION == 2:
        # ROS2 简单节流实现
        _log_throttle('warn', period, msg)
    else:
        print(f"[WARN] {msg}")


# 节流日志的状态存储 - 使用线程安全的实现
import threading
from collections import OrderedDict

_throttle_lock = threading.Lock()
_throttle_last_time: OrderedDict = OrderedDict()
_THROTTLE_CACHE_MAX_SIZE = 100  # 最大缓存条目数
_THROTTLE_CACHE_EXPIRE_SEC = 300.0  # 缓存条目过期时间（秒）
_throttle_last_cleanup = 0.0  # 上次清理时间
_THROTTLE_CLEANUP_INTERVAL = 60.0  # 清理间隔（秒）


def _log_throttle(level: str, period: float, msg: str):
    """
    简单的节流日志实现（线程安全）
    
    使用 OrderedDict 实现 LRU 缓存，自动清理最旧的条目。
    定期清理过期条目，避免长时间运行时内存泄漏。
    
    注意:
    - 此函数主要用于非 ROS 环境的回退方案
    - ROS1 节点应使用 rospy.logwarn_throttle
    - ROS2 节点应使用 logger.warn(..., throttle_duration_sec=...)
    
    Args:
        level: 日志级别 ('info', 'warn', 'error')
        period: 节流周期（秒）
        msg: 日志消息
    """
    import time
    global _throttle_last_cleanup
    
    # 使用 (level, msg) 元组作为键，避免哈希碰撞
    # 对于长消息，使用哈希以节省内存
    if len(msg) > 200:
        import hashlib
        key = (level, hashlib.sha256(msg.encode()).hexdigest())
    else:
        key = (level, msg)
    
    now = time.monotonic()
    
    should_log = False
    
    with _throttle_lock:
        # 定期清理过期条目
        if now - _throttle_last_cleanup > _THROTTLE_CLEANUP_INTERVAL:
            _cleanup_expired_entries(now)
            _throttle_last_cleanup = now
        
        last_time = _throttle_last_time.get(key, 0)
        if now - last_time >= period:
            # 更新时间戳并移动到末尾（最近使用）
            _throttle_last_time[key] = now
            _throttle_last_time.move_to_end(key)
            
            # LRU 缓存清理：当缓存过大时，删除最旧的条目
            while len(_throttle_last_time) > _THROTTLE_CACHE_MAX_SIZE:
                _throttle_last_time.popitem(last=False)  # 删除最旧的（第一个）
            
            should_log = True
    
    # 在锁外执行日志操作，避免持锁时间过长
    if should_log:
        if level == 'info':
            logger.info(msg)
        elif level == 'warn':
            logger.warning(msg)
        elif level == 'error':
            logger.error(msg)


def _cleanup_expired_entries(now: float) -> None:
    """
    清理过期的节流缓存条目
    
    Args:
        now: 当前时间（单调时钟）
    
    Note:
        此函数应在持有 _throttle_lock 的情况下调用
    """
    expired_keys = []
    for key, last_time in _throttle_last_time.items():
        if now - last_time > _THROTTLE_CACHE_EXPIRE_SEC:
            expired_keys.append(key)
    
    for key in expired_keys:
        del _throttle_last_time[key]


# ============================================================================
# TF2 兼容层
# ============================================================================

class TF2Compat:
    """TF2 兼容层，支持 ROS1 和 ROS2"""
    
    def __init__(self, node=None):
        """
        初始化 TF2 兼容层
        
        Args:
            node: ROS2 节点实例 (ROS1 不需要)
        """
        self._node = node
        self._buffer = None
        self._listener = None
        self._initialized = False
        
        self._initialize()
    
    def _initialize(self):
        """初始化 TF2"""
        if not TF2_AVAILABLE:
            logger.warning("TF2 not available")
            return
        
        try:
            if ROS_VERSION == 1:
                import tf2_ros
                self._buffer = tf2_ros.Buffer()
                self._listener = tf2_ros.TransformListener(self._buffer)
            elif ROS_VERSION == 2:
                from tf2_ros.buffer import Buffer
                from tf2_ros.transform_listener import TransformListener
                self._buffer = Buffer()
                self._listener = TransformListener(self._buffer, self._node)
            
            self._initialized = True
            logger.info("TF2 initialized")
        except Exception as e:
            logger.error(f"TF2 initialization failed: {e}")
            self._initialized = False
    
    def lookup_transform(self, target_frame: str, source_frame: str,
                        time=None, timeout_sec: float = 0.01) -> Optional[dict]:
        """
        查询坐标变换
        
        Args:
            target_frame: 目标坐标系
            source_frame: 源坐标系
            time: 时间戳，可以是:
                - None: 使用最新可用的变换 (rospy.Time(0))
                - float: 秒数，会被转换为 ROS Time
                - rospy.Time / rclpy.time.Time: 直接使用
            timeout_sec: 超时时间 (秒)
        
        Returns:
            {'translation': (x, y, z), 'rotation': (x, y, z, w)} 或 None
        """
        if not self._initialized or self._buffer is None:
            return None
        
        try:
            if ROS_VERSION == 1:
                import rospy
                import tf2_ros
                
                # 处理时间参数
                if time is None:
                    ros_time = rospy.Time(0)  # 最新可用的变换
                elif isinstance(time, (int, float)):
                    # float 秒数 -> 使用 Time(0) 获取最新变换
                    # 注意：TF2 lookup 通常使用 Time(0) 获取最新变换更可靠
                    # 因为精确时间戳可能不在 buffer 中
                    ros_time = rospy.Time(0)
                else:
                    ros_time = time  # 假设已经是 rospy.Time
                
                transform = self._buffer.lookup_transform(
                    target_frame, source_frame, ros_time,
                    timeout=rospy.Duration(timeout_sec)
                )
                
                return {
                    'translation': (
                        transform.transform.translation.x,
                        transform.transform.translation.y,
                        transform.transform.translation.z
                    ),
                    'rotation': (
                        transform.transform.rotation.x,
                        transform.transform.rotation.y,
                        transform.transform.rotation.z,
                        transform.transform.rotation.w
                    )
                }
            
            elif ROS_VERSION == 2:
                import tf2_ros
                from rclpy.time import Time
                from rclpy.duration import Duration
                
                # 处理时间参数
                if time is None:
                    ros_time = Time()  # 最新可用的变换
                elif isinstance(time, (int, float)):
                    # float 秒数 -> 使用 Time() 获取最新变换
                    ros_time = Time()
                else:
                    ros_time = time  # 假设已经是 rclpy.time.Time
                
                transform = self._buffer.lookup_transform(
                    target_frame, source_frame, ros_time,
                    timeout=Duration(seconds=timeout_sec)
                )
                
                return {
                    'translation': (
                        transform.transform.translation.x,
                        transform.transform.translation.y,
                        transform.transform.translation.z
                    ),
                    'rotation': (
                        transform.transform.rotation.x,
                        transform.transform.rotation.y,
                        transform.transform.rotation.z,
                        transform.transform.rotation.w
                    )
                }
        
        except Exception as e:
            logger.debug(f"TF lookup failed: {source_frame} -> {target_frame}: {e}")
            return None
        
        return None
    
    def can_transform(self, target_frame: str, source_frame: str,
                     time=None, timeout_sec: float = 0.01) -> bool:
        """检查是否可以进行坐标变换"""
        if not self._initialized or self._buffer is None:
            return False
        
        try:
            if ROS_VERSION == 1:
                import rospy
                # 处理时间参数
                if time is None or isinstance(time, (int, float)):
                    ros_time = rospy.Time(0)
                else:
                    ros_time = time
                return self._buffer.can_transform(
                    target_frame, source_frame, ros_time,
                    timeout=rospy.Duration(timeout_sec)
                )
            elif ROS_VERSION == 2:
                from rclpy.time import Time
                from rclpy.duration import Duration
                # 处理时间参数
                if time is None or isinstance(time, (int, float)):
                    ros_time = Time()
                else:
                    ros_time = time
                return self._buffer.can_transform(
                    target_frame, source_frame, ros_time,
                    timeout=Duration(seconds=timeout_sec)
                )
        except Exception:
            return False
        
        return False
    
    @property
    def is_initialized(self) -> bool:
        return self._initialized
    
    @property
    def buffer(self):
        return self._buffer
