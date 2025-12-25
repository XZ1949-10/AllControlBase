"""
单应性变换模块

提供地面坐标到图像坐标的单应性变换功能。
用于在相机图像上叠加显示轨迹点。
"""
from typing import Optional, Tuple, List
import os
import logging

import numpy as np

logger = logging.getLogger(__name__)


class HomographyTransform:
    """
    单应性变换器
    
    将地面坐标 (base_link 坐标系) 投影到图像像素坐标。
    
    使用方法:
        transform = HomographyTransform()
        if transform.load_calibration('/path/to/calib.yaml'):
            u, v = transform.project(x=1.5, y=0.0)  # 前方 1.5m
    """
    
    def __init__(self):
        self._H: Optional[np.ndarray] = None  # 单应性矩阵 (3x3)
        self._H_inv: Optional[np.ndarray] = None  # 逆矩阵
        self._calib_file: str = ''
        self._ground_points: List[List[float]] = []
        self._image_points: List[List[float]] = []
    
    @property
    def is_calibrated(self) -> bool:
        """是否已标定"""
        return self._H is not None
    
    @property
    def calibration_file(self) -> str:
        """标定文件路径"""
        return self._calib_file
    
    def load_calibration(self, calib_file: str) -> bool:
        """
        加载标定文件
        
        Args:
            calib_file: YAML 标定文件路径
            
        Returns:
            是否加载成功
        """
        if not os.path.exists(calib_file):
            logger.warning(f"Calibration file not found: {calib_file}")
            return False
        
        try:
            import yaml
            with open(calib_file, 'r') as f:
                data = yaml.safe_load(f)
            
            self._H = np.array(data['homography_matrix'], dtype=np.float64)
            self._ground_points = data.get('ground_points', [])
            self._image_points = data.get('image_points', [])
            self._calib_file = calib_file
            
            # 计算逆矩阵 (用于图像坐标 -> 地面坐标)
            try:
                self._H_inv = np.linalg.inv(self._H)
            except np.linalg.LinAlgError:
                logger.warning("Homography matrix is singular, inverse not available")
                self._H_inv = None
            
            logger.info(f"Loaded calibration from: {calib_file}")
            return True
            
        except Exception as e:
            logger.error(f"Failed to load calibration: {e}")
            return False
    
    def project(self, x: float, y: float) -> Optional[Tuple[int, int]]:
        """
        将地面坐标投影到图像坐标
        
        Args:
            x: 地面 X 坐标 (base_link 坐标系, 米, 正方向为前)
            y: 地面 Y 坐标 (base_link 坐标系, 米, 正方向为左)
            
        Returns:
            (u, v) 图像像素坐标，或 None 如果投影失败
        """
        if self._H is None:
            return None
        
        # 输入验证：检查 NaN 和 Inf
        if not np.isfinite(x) or not np.isfinite(y):
            logger.debug(f"Invalid input coordinates: x={x}, y={y}")
            return None
        
        # 齐次坐标
        pt = np.array([x, y, 1.0], dtype=np.float64)
        
        # 投影
        result = self._H @ pt
        
        # 归一化
        if abs(result[2]) < 1e-10:
            return None
        
        u = result[0] / result[2]
        v = result[1] / result[2]
        
        # 输出验证：检查结果是否有效
        if not np.isfinite(u) or not np.isfinite(v):
            logger.debug(f"Projection resulted in invalid coordinates: u={u}, v={v}")
            return None
        
        return (int(round(u)), int(round(v)))
    
    def project_points(self, points: List[Tuple[float, float]]) -> List[Optional[Tuple[int, int]]]:
        """
        批量投影地面坐标到图像坐标
        
        Args:
            points: 地面坐标列表 [(x1, y1), (x2, y2), ...]
            
        Returns:
            图像坐标列表，投影失败的点为 None
        """
        return [self.project(x, y) for x, y in points]
    
    def unproject(self, u: int, v: int) -> Optional[Tuple[float, float]]:
        """
        将图像坐标反投影到地面坐标 (假设 z=0)
        
        Args:
            u: 图像 X 坐标 (像素)
            v: 图像 Y 坐标 (像素)
            
        Returns:
            (x, y) 地面坐标，或 None 如果反投影失败
        """
        if self._H_inv is None:
            return None
        
        # 输入验证：检查 NaN 和 Inf（虽然 int 类型通常不会有，但防御性编程）
        try:
            u_float = float(u)
            v_float = float(v)
            if not np.isfinite(u_float) or not np.isfinite(v_float):
                return None
        except (TypeError, ValueError):
            return None
        
        # 齐次坐标
        pt = np.array([u_float, v_float, 1.0], dtype=np.float64)
        
        # 反投影
        result = self._H_inv @ pt
        
        # 归一化
        if abs(result[2]) < 1e-10:
            return None
        
        x = result[0] / result[2]
        y = result[1] / result[2]
        
        # 输出验证
        if not np.isfinite(x) or not np.isfinite(y):
            return None
        
        return (x, y)
    
    def compute_reprojection_error(self) -> float:
        """
        计算重投影误差 (像素)
        
        Returns:
            平均重投影误差，如果无法计算返回 inf
        """
        if self._H is None or not self._ground_points or not self._image_points:
            return float('inf')
        
        total_error = 0.0
        count = 0
        
        for gp, ip in zip(self._ground_points, self._image_points):
            projected = self.project(gp[0], gp[1])
            if projected:
                error = np.sqrt((projected[0] - ip[0])**2 + (projected[1] - ip[1])**2)
                total_error += error
                count += 1
        
        return total_error / count if count > 0 else float('inf')
