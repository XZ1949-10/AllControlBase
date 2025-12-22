"""加权一致性分析器"""
from typing import Dict, Any, Tuple, List
from collections import deque
import numpy as np

from ..core.interfaces import IConsistencyChecker
from ..core.data_types import Trajectory, ConsistencyResult, Point3D


class WeightedConsistencyAnalyzer(IConsistencyChecker):
    """使用加权几何平均的一致性分析器"""
    
    def __init__(self, config: Dict[str, Any]):
        consistency_config = config.get('consistency', config)
        
        self.kappa_thresh = consistency_config.get('kappa_thresh', 0.5)
        self.v_dir_thresh = consistency_config.get('v_dir_thresh', 0.8)
        self.temporal_smooth_thresh = consistency_config.get('temporal_smooth_thresh', 0.5)
        self.low_speed_thresh = consistency_config.get('low_speed_thresh', 0.1)
        self.temporal_window: deque = deque(maxlen=10)
        
        weights = consistency_config.get('weights', {})
        self.w_kappa = weights.get('kappa', 1.0)
        self.w_velocity = weights.get('velocity', 1.5)
        self.w_temporal = weights.get('temporal', 0.8)
        self.alpha_min = consistency_config.get('alpha_min', 0.1)
    
    def compute(self, trajectory: Trajectory) -> ConsistencyResult:
        if not trajectory.soft_enabled or trajectory.velocities is None:
            return ConsistencyResult(
                alpha=0.0, kappa_consistency=1.0, v_dir_consistency=1.0,
                temporal_smooth=1.0, should_disable_soft=True, data_valid=True
            )
        
        hard_velocities = trajectory.get_hard_velocities()
        soft_velocities = trajectory.velocities
        
        # 计算各维度一致性
        kappa_hard, kappa_hard_valid = self._compute_curvature_from_points(trajectory.points)
        kappa_soft, kappa_soft_valid = self._compute_curvature_from_velocities(
            soft_velocities, trajectory.dt_sec)
        
        if kappa_hard_valid and kappa_soft_valid:
            kappa_consistency = 1.0 - min(abs(kappa_hard - kappa_soft) / self.kappa_thresh, 1.0)
            kappa_valid = True
        else:
            # 数据无效时使用中性值，但标记为无效
            kappa_consistency = 1.0
            kappa_valid = False
        
        v_dir_consistency, v_dir_valid = self._compute_velocity_direction_consistency(
            hard_velocities, soft_velocities)
        temporal_smooth, temporal_valid = self._compute_temporal_smoothness(soft_velocities)
        
        # 数据有效性：只有当所有指标都有效时才认为数据有效
        data_valid = kappa_valid and v_dir_valid and temporal_valid
        
        # 加权几何平均
        # 当某个指标数据无效时，使用调整后的权重
        # 无效指标使用 1.0（中性值），不影响其他指标的贡献
        effective_kappa = kappa_consistency if kappa_valid else 1.0
        effective_v_dir = v_dir_consistency if v_dir_valid else 1.0
        effective_temporal = temporal_smooth if temporal_valid else 1.0
        
        # 计算有效权重总和
        effective_w_kappa = self.w_kappa if kappa_valid else 0.0
        effective_w_velocity = self.w_velocity if v_dir_valid else 0.0
        effective_w_temporal = self.w_temporal if temporal_valid else 0.0
        total_effective_weight = effective_w_kappa + effective_w_velocity + effective_w_temporal
        
        # 如果没有有效数据，返回保守的 alpha 值
        if total_effective_weight < 1e-6:
            return ConsistencyResult(
                alpha=0.5 * trajectory.confidence,  # 保守估计
                kappa_consistency=kappa_consistency,
                v_dir_consistency=v_dir_consistency,
                temporal_smooth=temporal_smooth,
                should_disable_soft=True,  # 数据不足时禁用 soft
                data_valid=False
            )
        
        # 使用有效权重计算 alpha
        alpha = (
            (max(effective_kappa, 1e-6) ** effective_w_kappa) *
            (max(effective_v_dir, 1e-6) ** effective_w_velocity) *
            (max(effective_temporal, 1e-6) ** effective_w_temporal)
        ) ** (1.0 / total_effective_weight) * trajectory.confidence
        
        return ConsistencyResult(
            alpha=alpha, kappa_consistency=kappa_consistency,
            v_dir_consistency=v_dir_consistency, temporal_smooth=temporal_smooth,
            should_disable_soft=(alpha < self.alpha_min),
            data_valid=data_valid
        )
    
    def _compute_velocity_direction_consistency(self, v_hard: np.ndarray, 
                                                 v_soft: np.ndarray) -> Tuple[float, bool]:
        if len(v_hard) == 0 or len(v_soft) == 0:
            return 0.5, False
        
        min_len = min(len(v_hard), len(v_soft))
        v_hard = v_hard[:min_len, :2]
        v_soft = v_soft[:min_len, :2]
        dot_product = np.sum(v_hard * v_soft, axis=1)
        norms = np.linalg.norm(v_hard, axis=1) * np.linalg.norm(v_soft, axis=1)
        valid_mask = norms > 1e-6
        
        if not np.any(valid_mask):
            return 0.5, False
        
        cos_angles = dot_product[valid_mask] / norms[valid_mask]
        avg_cos = np.mean(cos_angles)
        if avg_cos >= self.v_dir_thresh:
            return 1.0, True
        return max((avg_cos + 1.0) / (self.v_dir_thresh + 1.0), 0.0), True
    
    def _compute_temporal_smoothness(self, velocities: np.ndarray) -> Tuple[float, bool]:
        """
        计算时序平滑度
        
        通过比较连续帧的速度变化来评估平滑度。
        
        Returns:
            (smoothness, data_valid): 平滑度值和数据有效性标志
            - 当数据充足时，返回计算的平滑度和 True
            - 当数据不足时，返回 1.0 和 False，表示数据无效
              调用者应该在 alpha 计算中忽略此指标
        """
        if len(velocities) > 0:
            self.temporal_window.append(velocities[0].copy())
        
        if len(self.temporal_window) >= 2:
            diffs = np.diff(np.array(list(self.temporal_window)), axis=0)
            smoothness = 1.0 - min(np.mean(np.abs(diffs)) / self.temporal_smooth_thresh, 1.0)
            return smoothness, True
        
        # 数据不足时返回 1.0（中性值）和 False（无效标志）
        # 调用者会根据 data_valid=False 来调整权重
        return 1.0, False
    
    def _compute_curvature_from_points(self, points: List[Point3D]) -> Tuple[float, bool]:
        """
        从轨迹点计算曲率
        
        使用三点法计算曲率: κ = 2 * |cross(v1, v2)| / (|v1| * |v2| * |p2-p0|)
        
        数值稳定性考虑:
        - 当点距离过近时，返回无效
        - 当分母接近零时，返回无效
        - 限制最大曲率值
        
        Returns:
            (curvature, valid): 曲率值和有效性标志
        """
        if len(points) < 3:
            return 0.0, False
        
        p0 = np.array([points[0].x, points[0].y])
        p1 = np.array([points[1].x, points[1].y])
        p2 = np.array([points[2].x, points[2].y])
        v1, v2 = p1 - p0, p2 - p1
        l1, l2 = np.linalg.norm(v1), np.linalg.norm(v2)
        
        # 检查向量长度
        MIN_SEGMENT_LENGTH = 1e-6
        if l1 < MIN_SEGMENT_LENGTH or l2 < MIN_SEGMENT_LENGTH:
            return 0.0, False
        
        cross = v1[0] * v2[1] - v1[1] * v2[0]
        l12 = np.linalg.norm(p2 - p0)
        
        if l12 < MIN_SEGMENT_LENGTH:
            return 0.0, False
        
        # 计算分母并检查数值稳定性
        denominator = l1 * l2 * l12
        MIN_DENOMINATOR = 1e-9
        if denominator < MIN_DENOMINATOR:
            # 分母过小，可能导致数值不稳定
            return 0.0, False
        
        curvature = 2.0 * abs(cross) / denominator
        
        # 限制曲率最大值 (对应最小转弯半径 0.1m)
        MAX_CURVATURE = 10.0
        curvature = min(curvature, MAX_CURVATURE)
        
        return curvature, True
    
    def _compute_curvature_from_velocities(self, velocities: np.ndarray, dt: float) -> Tuple[float, bool]:
        """
        从速度向量计算曲率
        
        使用公式: κ = |v × a| / |v|³
        其中 v 是速度向量，a 是加速度向量
        
        数值稳定性考虑:
        - 当速度过低时，返回无效
        - 当分母接近零时，返回无效
        - 限制最大曲率值
        
        Returns:
            (curvature, valid): 曲率值和有效性标志
        """
        if len(velocities) < 2:
            return 0.0, False
        
        if dt <= 0:
            return 0.0, False
        
        v = velocities[0, :2]
        v_next = velocities[1, :2]
        a = (v_next - v) / dt
        v_norm = np.linalg.norm(v)
        
        # 低速时曲率计算不可靠
        if v_norm < self.low_speed_thresh:
            return 0.0, False
        
        cross = v[0] * a[1] - v[1] * a[0]
        
        # 计算分母并检查数值稳定性
        denominator = v_norm ** 3
        MIN_DENOMINATOR = 1e-9
        if denominator < MIN_DENOMINATOR:
            return 0.0, False
        
        curvature = abs(cross) / denominator
        
        # 限制曲率最大值 (对应最小转弯半径 0.1m)
        MAX_CURVATURE = 10.0
        curvature = min(curvature, MAX_CURVATURE)
        
        return curvature, True
    
    def reset(self) -> None:
        self.temporal_window.clear()
