"""
TF2 桥接层

管理 TF2 Buffer 和 Listener，提供坐标变换查询。
"""
from typing import Optional
import logging

logger = logging.getLogger(__name__)


class TFBridge:
    """
    TF2 桥接层
    
    职责:
    - 管理 TF2 Buffer 和 Listener
    - 提供坐标变换查询
    - 将变换注入到 universal_controller
    """
    
    def __init__(self, node):
        """
        初始化 TF 桥接
        
        Args:
            node: ROS2 节点实例
        """
        self._node = node
        self._buffer = None
        self._listener = None
        self._initialized = False
        
        self._initialize()
    
    def _initialize(self):
        """初始化 TF2"""
        try:
            import tf2_ros
            from tf2_ros.buffer import Buffer
            from tf2_ros.transform_listener import TransformListener
            
            self._buffer = Buffer()
            self._listener = TransformListener(self._buffer, self._node)
            self._initialized = True
            logger.info("TF2 bridge initialized")
        except ImportError as e:
            logger.warning(f"TF2 not available: {e}")
            self._initialized = False
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
            time: 时间戳 (None 表示最新)
            timeout_sec: 超时时间 (秒)
        
        Returns:
            变换字典 {'translation': (x, y, z), 'rotation': (x, y, z, w)} 或 None
        """
        if not self._initialized or self._buffer is None:
            return None
        
        try:
            import tf2_ros
            from rclpy.time import Time
            from rclpy.duration import Duration
            
            if time is None:
                time = Time()
            
            transform = self._buffer.lookup_transform(
                target_frame, source_frame, time,
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
        except tf2_ros.LookupException:
            logger.debug(f"TF lookup failed: {source_frame} -> {target_frame}")
            return None
        except tf2_ros.ExtrapolationException:
            logger.debug(f"TF extrapolation failed: {source_frame} -> {target_frame}")
            return None
        except Exception as e:
            logger.warning(f"TF lookup error: {e}")
            return None
    
    def can_transform(self, target_frame: str, source_frame: str,
                     time=None, timeout_sec: float = 0.01) -> bool:
        """检查是否可以进行坐标变换"""
        if not self._initialized or self._buffer is None:
            return False
        
        try:
            from rclpy.time import Time
            from rclpy.duration import Duration
            
            if time is None:
                time = Time()
            
            return self._buffer.can_transform(
                target_frame, source_frame, time,
                timeout=Duration(seconds=timeout_sec)
            )
        except Exception:
            return False
    
    @property
    def is_initialized(self) -> bool:
        """TF2 是否已初始化"""
        return self._initialized
    
    @property
    def buffer(self):
        """获取 TF2 Buffer (用于高级操作)"""
        return self._buffer
