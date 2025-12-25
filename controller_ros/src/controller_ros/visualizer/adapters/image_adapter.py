"""
图像消息适配器

ROS 消息: sensor_msgs/Image
可视化模型: numpy.ndarray (BGR 格式)
"""
from typing import Any, Optional
import numpy as np


class ImageAdapter:
    """
    图像消息适配器
    
    将 ROS sensor_msgs/Image 转换为 numpy 数组。
    支持多种图像编码格式。
    """
    
    # 支持的编码格式
    SUPPORTED_ENCODINGS = {
        'rgb8': ('uint8', 3),
        'rgba8': ('uint8', 4),
        'bgr8': ('uint8', 3),
        'bgra8': ('uint8', 4),
        'mono8': ('uint8', 1),
        'mono16': ('uint16', 1),
        '16UC1': ('uint16', 1),
        '32FC1': ('float32', 1),
    }
    
    def __init__(self, target_encoding: str = 'bgr8'):
        """
        初始化图像适配器
        
        Args:
            target_encoding: 目标编码格式 (默认 bgr8，适合 OpenCV)
        """
        self._target_encoding = target_encoding
        self._cv_bridge = None
        self._cv_bridge_available = False
        
        # 尝试导入 cv_bridge
        try:
            from cv_bridge import CvBridge
            self._cv_bridge = CvBridge()
            self._cv_bridge_available = True
        except ImportError:
            pass
    
    def to_model(self, ros_msg: Any) -> Optional[np.ndarray]:
        """
        ROS Image → numpy.ndarray
        
        Args:
            ros_msg: sensor_msgs/Image 消息
        
        Returns:
            BGR 格式的 numpy 数组，失败返回 None
        """
        if ros_msg is None:
            return None
        
        # 优先使用 cv_bridge
        if self._cv_bridge_available:
            return self._convert_with_cv_bridge(ros_msg)
        
        # 回退到手动转换
        return self._convert_manual(ros_msg)
    
    def _convert_with_cv_bridge(self, ros_msg: Any) -> Optional[np.ndarray]:
        """使用 cv_bridge 转换"""
        try:
            cv_image = self._cv_bridge.imgmsg_to_cv2(
                ros_msg, 
                desired_encoding=self._target_encoding
            )
            return cv_image
        except Exception as e:
            # 记录错误以便调试
            import logging
            logging.getLogger(__name__).debug(f"cv_bridge conversion failed: {e}")
            return None
    
    def _convert_manual(self, ros_msg: Any) -> Optional[np.ndarray]:
        """手动转换 (不依赖 cv_bridge)"""
        try:
            encoding = ros_msg.encoding
            if encoding not in self.SUPPORTED_ENCODINGS:
                return None
            
            dtype, channels = self.SUPPORTED_ENCODINGS[encoding]
            
            # 解析图像数据
            if channels == 1:
                shape = (ros_msg.height, ros_msg.width)
            else:
                shape = (ros_msg.height, ros_msg.width, channels)
            
            img = np.frombuffer(ros_msg.data, dtype=dtype).reshape(shape)
            
            # 转换为 BGR
            if encoding == 'rgb8':
                img = img[:, :, ::-1].copy()  # RGB -> BGR
            elif encoding == 'rgba8':
                img = img[:, :, [2, 1, 0]].copy()  # RGBA -> BGR
            elif encoding == 'mono8':
                img = np.stack([img, img, img], axis=-1)  # Mono -> BGR
            elif encoding == 'mono16' or encoding == '16UC1':
                # 归一化到 0-255
                img = (img / 256).astype(np.uint8)
                img = np.stack([img, img, img], axis=-1)
            
            return img
        except Exception:
            return None
    
    @property
    def cv_bridge_available(self) -> bool:
        """cv_bridge 是否可用"""
        return self._cv_bridge_available
