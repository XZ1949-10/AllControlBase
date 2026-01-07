"""
TF2 桥接层

管理 TF2 Buffer 和 Listener，提供坐标变换查询。
支持 ROS1 和 ROS2。

注意:
    TF2 注入逻辑已移至 TF2InjectionManager，本类仅负责 TF2 查询。
"""
from typing import Optional
import logging

from ..utils.ros_compat import (
    ROS_VERSION, TF2_AVAILABLE, TF2Compat
)

logger = logging.getLogger(__name__)


class TFBridge:
    """
    TF2 桥接层
    
    职责:
    - 管理 TF2 Buffer 和 Listener
    - 提供坐标变换查询接口
    
    注意:
        TF2 注入逻辑已移至 TF2InjectionManager。
        本类仅作为 TF2Compat 的桥接层，提供统一的接口。
    
    支持 ROS1 和 ROS2。
    """
    
    def __init__(self, node=None):
        """
        初始化 TF 桥接
        
        Args:
            node: ROS2 节点实例 (ROS1 可传 None)
        """
        self._node = node
        self._tf2_compat = TF2Compat(node)
    
    def lookup_transform(self, target_frame: str, source_frame: str,
                        time=None, timeout_sec: float = 0.0) -> Optional[dict]:
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
        return self._tf2_compat.lookup_transform(
            target_frame, source_frame, time, timeout_sec
        )
    
    def can_transform(self, target_frame: str, source_frame: str,
                     time=None, timeout_sec: float = 0.0) -> bool:
        """检查是否可以进行坐标变换"""
        return self._tf2_compat.can_transform(
            target_frame, source_frame, time, timeout_sec
        )
    
    @property
    def is_initialized(self) -> bool:
        """TF2 是否已初始化"""
        return self._tf2_compat.is_initialized
    
    @property
    def buffer(self):
        """获取 TF2 Buffer (用于高级操作)"""
        return self._tf2_compat.buffer

    def shutdown(self) -> None:
        """
        关闭 TF 桥接，释放资源
        
        支持重复调用，第二次及之后的调用会被忽略。
        异常安全：即使 _tf2_compat.shutdown() 失败，引用也会被清空。
        """
        if self._tf2_compat is None:
            # 已经关闭，直接返回
            return
        try:
            self._tf2_compat.shutdown()
        finally:
            self._tf2_compat = None
            self._node = None
