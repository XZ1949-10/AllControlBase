"""加权一致性分析器"""
from typing import Dict, Any, Tuple, List
from collections import deque
import numpy as np
import logging

from ..core.interfaces import IConsistencyChecker
from ..core.data_types import Trajectory, ConsistencyResult, Point3D
from ..core.constants import (
    EPSILON, 
    MIN_SEGMENT_LENGTH, 
    MIN_DENOMINATOR, 
    MIN_RELATIVE_CROSS,
    CONSISTENCY_INVALID_DATA_CONFIDENCE,
)

logger = logging.getLogger(__name__)


class WeightedConsistencyAnalyzer(IConsistencyChecker):
    """使用加权几何平均的一致性分析器"""
    
    # 数值稳定性常量 - 使用统一定义
    # 这些是基于 IEEE 754 双精度浮点数特性选择的内部常量
    # 从 core.constants 导入，确保全局一致性
    
    def __init__(self, config: Dict[str, Any]):
        consistency_config = config.get('consistency', config)
        trajectory_config = config.get('trajectory', {})
        
        self.kappa_thresh = consistency_config.get('kappa_thresh', 0.5)
        self.v_dir_thresh = consistency_config.get('v_dir_thresh', 0.8)
        self.temporal_smooth_thresh = consistency_config.get('temporal_smooth_thresh', 0.5)
        
        # low_speed_thresh 统一从 trajectory 配置读取
        # 这确保了轨迹速度计算和一致性检查使用相同的阈值
        self.low_speed_thresh = trajectory_config.get('low_speed_thresh', 0.1)
        
        self.max_curvature = consistency_config.get('max_curvature', 10.0)
        temporal_window_size = consistency_config.get('temporal_window_size', 10)
        self.temporal_window: deque = deque(maxlen=temporal_window_size)
        
        weights = consistency_config.get('weights', {})
        self.w_kappa = weights.get('kappa', 1.0)
        self.w_velocity = weights.get('velocity', 1.5)
        self.w_temporal = weights.get('temporal', 0.8)
        self.alpha_min = consistency_config.get('alpha_min', 0.1)
        self.min_value_for_log = consistency_config.get('min_value_for_log', 1e-10)
        
        # 数据无效时的保守 confidence 值 - 使用常量
        # 与 TrajectoryDefaults.default_confidence (0.9) 不同，这是运行时异常的保守处理
        self.invalid_data_confidence = CONSISTENCY_INVALID_DATA_CONFIDENCE
    
    def compute(self, trajectory: Trajectory) -> ConsistencyResult:
        if not trajectory.soft_enabled or trajectory.velocities is None:
            # soft_enabled=False 时，使用 hard velocities（从轨迹点计算的速度）
            # alpha=1.0 表示完全信任 hard velocities
            return ConsistencyResult(
                alpha=1.0, kappa_consistency=1.0, v_dir_consistency=1.0,
                temporal_smooth=1.0, should_disable_soft=True, data_valid=True
            )
        
        # 预先验证 confidence，确保后续计算的数值稳定性
        confidence = trajectory.confidence
        if confidence is None or not np.isfinite(confidence):
            logger.warning(f"Invalid trajectory confidence: {confidence}, using default {self.invalid_data_confidence}")
            confidence = self.invalid_data_confidence
        else:
            confidence = np.clip(confidence, 0.0, 1.0)
        
        hard_velocities = trajectory.get_hard_velocities()
        soft_velocities = trajectory.velocities
        
        # 检查输入数据是否包含 NaN/Inf
        if np.any(~np.isfinite(soft_velocities)):
            logger.warning("NaN/Inf detected in soft velocities, disabling soft mode")
            return ConsistencyResult(
                alpha=self.invalid_data_confidence, kappa_consistency=0.0, v_dir_consistency=0.0,
                temporal_smooth=0.0, should_disable_soft=True, data_valid=False
            )
        
        if np.any(~np.isfinite(hard_velocities)):
            logger.warning("NaN/Inf detected in hard velocities, using conservative alpha")
            return ConsistencyResult(
                alpha=self.invalid_data_confidence, kappa_consistency=0.0, v_dir_consistency=0.0,
                temporal_smooth=0.0, should_disable_soft=True, data_valid=False
            )
        
        # 计算各维度一致性
        # 返回值: (value, is_sufficient) - is_sufficient 表示数据是否充足
        kappa_hard, kappa_hard_sufficient = self._compute_curvature_from_points(trajectory.points)
        kappa_soft, kappa_soft_sufficient = self._compute_curvature_from_velocities(
            soft_velocities, trajectory.dt_sec)
        
        if kappa_hard_sufficient and kappa_soft_sufficient:
            kappa_consistency = 1.0 - min(abs(kappa_hard - kappa_soft) / self.kappa_thresh, 1.0)
            kappa_sufficient = True
        else:
            # 数据不足时使用中性值
            kappa_consistency = 1.0
            kappa_sufficient = False
        
        v_dir_consistency, v_dir_sufficient = self._compute_velocity_direction_consistency(
            hard_velocities, soft_velocities)
        temporal_smooth, temporal_sufficient = self._compute_temporal_smoothness(soft_velocities)
        
        # 加权几何平均
        # 当某个指标数据不足时，使用调整后的权重
        # 不足的指标使用 1.0（中性值），不影响其他指标的贡献
        effective_kappa = kappa_consistency if kappa_sufficient else 1.0
        effective_v_dir = v_dir_consistency if v_dir_sufficient else 1.0
        effective_temporal = temporal_smooth if temporal_sufficient else 1.0
        
        # 计算有效权重总和
        effective_w_kappa = self.w_kappa if kappa_sufficient else 0.0
        effective_w_velocity = self.w_velocity if v_dir_sufficient else 0.0
        effective_w_temporal = self.w_temporal if temporal_sufficient else 0.0
        total_effective_weight = effective_w_kappa + effective_w_velocity + effective_w_temporal
        
        # 如果没有任何有效数据，返回保守的 alpha 值
        if total_effective_weight < EPSILON:
            return ConsistencyResult(
                alpha=self.invalid_data_confidence * confidence,  # 保守估计
                kappa_consistency=kappa_consistency,
                v_dir_consistency=v_dir_consistency,
                temporal_smooth=temporal_smooth,
                should_disable_soft=True,  # 数据不足时禁用 soft
                # data_valid=True: 数据不足不等于数据无效
                # 只有当数据包含 NaN 或异常值时才设为 False
                data_valid=True
            )
        
        # 使用对数空间计算加权几何平均，避免数值下溢
        # 几何平均公式: (x1^w1 * x2^w2 * x3^w3)^(1/W) = exp((w1*log(x1) + w2*log(x2) + w3*log(x3)) / W)
        # 
        # 数值稳定性说明:
        # - 直接计算 (1e-6)^1.5 ≈ 3.16e-9 可能导致下溢
        # - 对数空间: log(1e-6) * 1.5 = -13.8 * 1.5 = -20.7，然后 exp(-20.7) ≈ 1e-9
        # - 对数空间计算更稳定，避免中间结果下溢
        # 
        # 下界选择 1e-10 (可配置):
        # - log(1e-10) = -23，乘以最大权重 1.5 后为 -34.5
        # - exp(-34.5) ≈ 1e-15，仍在 float64 精度范围内
        
        log_alpha = (
            effective_w_kappa * np.log(max(effective_kappa, self.min_value_for_log)) +
            effective_w_velocity * np.log(max(effective_v_dir, self.min_value_for_log)) +
            effective_w_temporal * np.log(max(effective_temporal, self.min_value_for_log))
        ) / total_effective_weight
        
        alpha = np.exp(log_alpha) * confidence
        
        # data_valid: 只有当数据包含 NaN 或异常值时才设为 False
        # 数据不足（如启动期间）不影响 data_valid
        data_valid = True
        
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
        valid_mask = norms > EPSILON
        
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
        - 当叉积接近零时（共线情况），返回 0 曲率
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
        
        # 检查向量长度 (使用统一常量)
        if l1 < MIN_SEGMENT_LENGTH or l2 < MIN_SEGMENT_LENGTH:
            return 0.0, False
        
        cross = v1[0] * v2[1] - v1[1] * v2[0]
        l12 = np.linalg.norm(p2 - p0)
        
        if l12 < MIN_SEGMENT_LENGTH:
            return 0.0, False
        
        # 计算分母并检查数值稳定性 (使用统一常量)
        denominator = l1 * l2 * l12
        if denominator < MIN_DENOMINATOR:
            # 分母过小，可能导致数值不稳定
            return 0.0, False
        
        # 检查叉积是否接近零（共线情况）
        # 使用相对阈值：叉积相对于分母的比例
        # 当 |cross| / denominator < MIN_RELATIVE_CROSS 时，认为是共线 (使用统一常量)
        abs_cross = abs(cross)
        if abs_cross < MIN_RELATIVE_CROSS * denominator:
            # 几乎共线，曲率接近 0
            return 0.0, True
        
        curvature = 2.0 * abs_cross / denominator
        
        # 限制曲率最大值 (使用配置值)
        curvature = min(curvature, self.max_curvature)
        
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
        
        # 计算分母并检查数值稳定性 (使用统一常量)
        denominator = v_norm ** 3
        if denominator < MIN_DENOMINATOR:
            return 0.0, False
        
        curvature = abs(cross) / denominator
        
        # 限制曲率最大值 (使用配置值)
        curvature = min(curvature, self.max_curvature)
        
        return curvature, True
    
    def reset(self) -> None:
        self.temporal_window.clear()
