"""
鲁棒坐标变换模块
"""
from typing import Dict, Any, Tuple, Optional, List
import time
import numpy as np
import logging
from threading import Lock

from ..core.interfaces import ICoordinateTransformer
from ..core.data_types import Trajectory, Point3D, EstimatorOutput
from ..core.enums import TransformStatus
from ..core.ros_compat import (
    TF2_AVAILABLE, Duration, Time, 
    transform_pose, get_transform_matrix, apply_transform_to_trajectory
)

logger = logging.getLogger(__name__)


class RobustCoordinateTransformer(ICoordinateTransformer):
    """
    带降级时限和恢复校正的坐标变换器
    
    主要功能:
    1. 将局部坐标系(base_link)的轨迹变换到世界坐标系(odom)
    2. 如果 TF2 可用，使用 TF2 进行变换
    3. 如果 TF2 不可用，使用 fallback_state (状态估计器) 进行降级处理
    4. 补偿网络推理延迟
    """
    
    def __init__(self, config: Dict[str, Any]):
        transform_config = config.get('transform', {})
        
        self.source_frame = transform_config.get('source_frame', 'base_link')
        self.default_target_frame = transform_config.get('target_frame', 'odom')
        self.timeout_sec = transform_config.get('timeout_sec', 0.05)
        
        # 降级阈值配置
        self.fallback_duration_limit = transform_config.get('fallback_duration_limit_ms', 500) / 1000.0
        self.fallback_critical_limit = transform_config.get('fallback_critical_limit_ms', 1000) / 1000.0
        self.warn_unexpected_frame = transform_config.get('warn_unexpected_frame', True)
        
        # 状态记录
        self._last_status = TransformStatus.TF2_OK
        self.fallback_start_time: Optional[float] = None
        self._tf2_lookup_callback = None
        self._tf2_available = TF2_AVAILABLE
        
        # 记录已警告过的坐标系，避免日志刷屏
        self._warned_frames = set()
        
        # 用于缓存上一次成功的变换，用于极短时间的平滑（可选）
        self._last_successful_transform = None
        self._lock = Lock()

    def set_tf2_lookup_callback(self, callback):
        """注入 TF2 查找回调"""
        self._tf2_lookup_callback = callback
        if callback:
            self._tf2_available = True

    def set_tf2_available(self, available: bool):
        """手动设置 TF2 可用性 (用于测试)"""
        self._tf2_available = available

    def transform_trajectory(self, traj: Trajectory, target_frame: str, 
                            target_time: float,
                            fallback_state: Optional[EstimatorOutput] = None) -> Tuple[Trajectory, TransformStatus]:
        """
        变换轨迹坐标系
        
        流程:
        1. 检查输入轨迹坐标系
        2. 尝试获取 TF 变换
        3. 如果 TF 失败，使用 fallback_state
        
        Args:
            traj: 输入轨迹
            target_frame: 目标坐标系 (通常是 'odom')
            target_time: 目标时间戳 (用于查询 TF)
            fallback_state: 备用的状态估计 (当 TF 失败时使用)
            
        Returns:
            (变换后的轨迹, 状态码)
        """
        if not traj.points:
            return traj, TransformStatus.TF2_OK
            
        source_frame = traj.header.frame_id
        if not source_frame:
            source_frame = self.source_frame
            
        # 如果坐标系已经一致，直接返回
        if source_frame == target_frame:
            return traj, TransformStatus.TF2_OK
            
        # 简单的坐标系验证警告
        if self.warn_unexpected_frame and source_frame != self.source_frame:
            if source_frame not in self._warned_frames:
                logger.warning(f"Unexpected trajectory frame: {source_frame} (expected {self.source_frame})")
                self._warned_frames.add(source_frame)

        # 尝试使用 TF2 获取变换
        transform_matrix = None
        tf2_success = False
        
        try:
            if self._tf2_available and self._tf2_lookup_callback:
                # 使用注入的回调查询 TF
                # 注意：traj.header.stamp 是轨迹生成的时刻，通常包含延迟补偿
                # 我们应该查询这个时刻的变换，或者 target_time
                lookup_time = traj.header.stamp if traj.header.stamp > 0 else target_time
                
                transform_data = self._tf2_lookup_callback(
                    target_frame, source_frame, lookup_time, self.timeout_sec
                )
                
                if transform_data:
                    # 获取变换矩阵
                    translation = transform_data['translation']
                    rotation = transform_data['rotation']
                    transform_matrix = get_transform_matrix(translation, rotation)
                    tf2_success = True
                    
                    # 成功获取 TF，重置降级状态
                    if self.fallback_start_time is not None:
                        logger.info(f"TF2 service recovered after {time.time() - self.fallback_start_time:.3f}s")
                        self.fallback_start_time = None
                    self._last_status = TransformStatus.TF2_OK
                    
        except Exception as e:
            logger.debug(f"TF2 lookup failed: {e}")
            tf2_success = False

        # 如果 TF2 成功，执行变换
        if tf2_success and transform_matrix is not None:
            new_traj = apply_transform_to_trajectory(traj, transform_matrix, target_frame)
            return new_traj, TransformStatus.TF2_OK
            
        # 如果 TF2 失败，进入降级处理
        return self._handle_fallback(traj, target_time, target_frame, fallback_state)

    def _handle_fallback(self, traj: Trajectory, target_time: float,
                        target_frame: str,
                        fallback_state: Optional[EstimatorOutput] = None) -> Tuple[Trajectory, TransformStatus]:
        """
        处理降级情况
        
        策略:
        1. 记录降级开始时间
        2. 检查 Frame 兼容性 (仅支持定义的 source_frame)
        3. 检查是否有 fallback_state (EstimatorOutput)
        4. 如果有，直接使用 fallback_state 的位置和航向作为变换基准
        5. 如果没有，只能报错并返回原轨迹
        """
        # 验证 Frame 是否匹配估计器
        source_frame = traj.header.frame_id or self.source_frame
        if source_frame != self.source_frame and source_frame != "" and self.source_frame != "":
             # simple inequality check might be safe enough if we assume normalization
            if source_frame != self.source_frame:
                logger.error(f"RobustFallback: Source frame '{source_frame}' mismatch with configured '{self.source_frame}'. Cannot use estimator fallback.")
                return traj, TransformStatus.FALLBACK_CRITICAL

        current_time = time.time()
        
        # 首次进入降级模式
        if self.fallback_start_time is None:
            self.fallback_start_time = current_time
            logger.warning(f"TF2 unavailable. Entering fallback mode using estimator state.")
            
        fallback_duration = current_time - self.fallback_start_time
        
        # 确定状态级别
        if fallback_duration > self.fallback_critical_limit:
            status = TransformStatus.FALLBACK_CRITICAL
        elif fallback_duration > self.fallback_duration_limit:
            status = TransformStatus.FALLBACK_WARNING
        else:
            status = TransformStatus.FALLBACK_OK
            
        self._last_status = status
        
        # 如果有状态估计器数据，使用它进行变换
        if fallback_state is not None:
            # 假设 fallback_state 是在 target_frame (odom) 下的 state
            # state: [x, y, z, vx, vy, vz, theta, omega]
            # 我们需要构建 base_link -> odom 的变换
            # 即机器人在 odom 下的位姿
            
            est_pos = fallback_state.state[0:3]
            est_theta = fallback_state.state[6]
            
            # 由于这是 2D 平面假设居多，我们主要关注 x, y, theta
            # 构建变换矩阵 (2D 旋转 + 平移)
            # T = [ R  p ]
            #     [ 0  1 ]
            
            cos_t = np.cos(est_theta)
            sin_t = np.sin(est_theta)
            
            # 构建 4x4 矩阵
            matrix = np.eye(4)
            matrix[0, 0] = cos_t
            matrix[0, 1] = -sin_t
            matrix[0, 3] = est_pos[0]
            
            matrix[1, 0] = sin_t
            matrix[1, 1] = cos_t
            matrix[1, 3] = est_pos[1]
            
            matrix[2, 3] = est_pos[2] # Z
            
            # 应用变换
            new_traj = apply_transform_to_trajectory(traj, matrix, target_frame)
            return new_traj, status
            
        else:
            # 严重错误：既没有 TF2 也没有 Estimator
            if fallback_duration > 1.0: # 稍微节流日志
                logger.error("TF2 fallback active but no state estimator available. Unable to transform trajectory correctly.")
            return traj, TransformStatus.FALLBACK_CRITICAL

    def get_status(self) -> Dict[str, Any]:
        """获取状态信息"""
        fallback_duration = 0.0
        if self.fallback_start_time is not None:
            fallback_duration = time.time() - self.fallback_start_time
            
        return {
            'status': self._last_status.name,
            'fallback_active': self.fallback_start_time is not None,
            'fallback_duration_ms': fallback_duration * 1000,
            'tf2_available': self._tf2_available
        }

    def reset(self) -> None:
        """重置状态"""
        self.fallback_start_time = None
        self._last_status = TransformStatus.TF2_OK
        self._warned_frames.clear()
        self._stored_transforms = {}

    def set_transform(self, target_frame: str, source_frame: str,
                      x: float, y: float, z: float, theta: float,
                      stamp: Optional[float] = None) -> None:
        """
        存储静态变换 (用于测试和简单场景)
        
        这个方法用于在没有真实 TF2 系统时存储变换。
        变换会通过内部 lookup 回调返回。
        
        Args:
            target_frame: 目标坐标系
            source_frame: 源坐标系
            x, y, z: 平移
            theta: 航向角 (yaw)
            stamp: 时间戳 (可选)
        """
        if not hasattr(self, '_stored_transforms'):
            self._stored_transforms = {}
        
        key = (target_frame, source_frame)
        self._stored_transforms[key] = {
            'translation': {'x': x, 'y': y, 'z': z},
            'rotation': self._euler_to_quaternion(0, 0, theta),
            'stamp': stamp or time.time()
        }
        
        # 如果没有回调，设置内部回调
        if self._tf2_lookup_callback is None:
            self._tf2_lookup_callback = self._internal_lookup
            self._tf2_available = True

    def _euler_to_quaternion(self, roll: float, pitch: float, yaw: float) -> dict:
        """欧拉角转四元数"""
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)
        
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        return {'x': x, 'y': y, 'z': z, 'w': w}

    def _internal_lookup(self, target_frame: str, source_frame: str, 
                         lookup_time: float, timeout: float) -> Optional[dict]:
        """内部变换查找"""
        if not hasattr(self, '_stored_transforms'):
            return None
        
        key = (target_frame, source_frame)
        if key in self._stored_transforms:
            return self._stored_transforms[key]
        
        # 尝试反向查找
        reverse_key = (source_frame, target_frame)
        if reverse_key in self._stored_transforms:
            t = self._stored_transforms[reverse_key]
            # 简化的反向变换 (仅适用于 2D)
            q = t['rotation']
            theta = 2 * np.arctan2(q['z'], q['w'])
            cos_t, sin_t = np.cos(theta), np.sin(theta)
            tx, ty = t['translation']['x'], t['translation']['y']
            
            # 反向平移: -R^T * t
            inv_x = -(cos_t * tx + sin_t * ty)
            inv_y = -(-sin_t * tx + cos_t * ty)
            
            return {
                'translation': {'x': inv_x, 'y': inv_y, 'z': -t['translation']['z']},
                'rotation': {'x': -q['x'], 'y': -q['y'], 'z': -q['z'], 'w': q['w']},
                'stamp': t['stamp']
            }
        
        return None

    # 为了保持接口兼容性，保留的方法名（但不再用于变换逻辑）
    def transform_velocity(self, velocity, heading):
        # 速度变换通常是在 Trajectory 变换中一并处理的
        pass
