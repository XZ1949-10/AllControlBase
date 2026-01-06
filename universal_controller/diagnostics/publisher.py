"""
诊断发布器

负责诊断数据的收集、格式化和通过回调发布。
从 ControllerManager 中抽离，实现单一职责原则。

职责说明：
- DiagnosticsPublisher: 构建诊断数据并通过回调机制传递
- ROS 消息发布由 controller_ros/io/publishers.py 中的 PublisherManager 负责
- 这种设计使 universal_controller 保持 ROS 无关性

注意：
- 话题名称 (diagnostics_topic, cmd_topic) 是 ROS 层特有配置
- 核心库不应该知道 ROS 话题的存在
- 这些参数仅用于日志记录，实际发布由 ROS 层处理
"""
from typing import Dict, Any, Optional, Callable, List
import numpy as np
import logging
import threading

from ..core.data_types import (
    DiagnosticsV2, Header, ControlOutput, EstimatorOutput, 
    TimeoutStatus, ConsistencyResult, MPCHealthStatus
)
from ..core.enums import ControllerState

logger = logging.getLogger(__name__)


class DiagnosticsPublisher:
    """
    诊断发布器
    
    负责：
    - 收集各模块的诊断数据
    - 构建 DiagnosticsV2 数据结构
    - 通过回调函数传递诊断数据
    - 维护发布历史
    
    设计说明：
    - 本类不直接进行 ROS 发布，保持 universal_controller 的 ROS 无关性
    - ROS 消息发布由 controller_ros/io/publishers.py 中的 PublisherManager 负责
    - 通过回调机制将诊断数据传递给 ROS 层
    
    使用示例:
        publisher = DiagnosticsPublisher()
        publisher.add_callback(my_callback)
        publisher.publish(...)
    """
    
    def __init__(self, publish_rate: float = 10.0):
        """
        初始化诊断发布器
        
        Args:
            publish_rate: 诊断发布频率 (Hz)，默认 10Hz
        """
        # 回调函数（线程安全）
        # Now passes DiagnosticsV2 object instead of Dict
        self._callbacks: List[Callable[[Any], None]] = []
        self._callbacks_lock = threading.Lock()
        
        # 回调失败计数（用于自动移除持续失败的回调）
        self._callback_fail_counts: Dict[int, int] = {}  # callback id -> fail count
        self._callback_max_failures = 5  # 连续失败超过此次数后移除回调
        
        # 发布历史
        self._last_published: Optional[DiagnosticsV2] = None
        
        # 节流控制
        self._min_interval = 1.0 / publish_rate if publish_rate > 0 else 0.0
        self._last_publish_time = 0.0
        
        # 之前使用对象复用 (Object Pooling) 导致了脏读风险
        # 现在每次发布都创建新对象，以确保线程安全和数据的不可变性
        # Python 的 GC 对这种小对象的处理非常高效，性能影响可忽略不计
    
    def add_callback(self, callback: Callable[['DiagnosticsV2'], None]) -> None:
        """
        添加诊断回调函数（线程安全）
        
        Args:
            callback: 回调函数，签名为 callback(diagnostics: DiagnosticsV2) -> None
        
        Note:
            如果回调连续失败超过 5 次，会被自动移除以防止日志泛滥
        """
        with self._callbacks_lock:
            if callback not in self._callbacks:
                self._callbacks.append(callback)
                self._callback_fail_counts[id(callback)] = 0
    
    def remove_callback(self, callback: Callable[[Any], None]) -> None:
        """移除诊断回调函数（线程安全）"""
        with self._callbacks_lock:
            if callback in self._callbacks:
                self._callbacks.remove(callback)
                self._callback_fail_counts.pop(id(callback), None)
    
    def clear_callbacks(self) -> None:
        """清除所有回调函数（线程安全）"""
        with self._callbacks_lock:
            self._callbacks.clear()
            self._callback_fail_counts.clear()
    
    def publish(self,
                current_time: float,
                state: ControllerState,
                cmd: ControlOutput,
                state_output: Optional[EstimatorOutput],
                consistency: Optional[ConsistencyResult],
                mpc_health: Optional[MPCHealthStatus],
                timeout_status: TimeoutStatus,
                transform_status: Dict[str, Any],
                tracking_error: Optional[Dict[str, float]],
                transition_progress: float,
                tf2_critical: bool,
                safety_check_passed: bool = True,
                emergency_stop: bool = False,
                tracking_quality: Optional[Dict[str, Any]] = None,
                force: bool = False) -> None:
        """
        发布诊断数据
        
        Args:
            current_time: 当前时间戳
            state: 控制器状态
            cmd: 控制命令
            state_output: 状态估计器输出
            consistency: 一致性检查结果
            mpc_health: MPC 健康状态
            timeout_status: 超时状态
            transform_status: 坐标变换状态
            tracking_error: 跟踪误差
            transition_progress: 过渡进度
            tf2_critical: TF2 是否临界
            safety_check_passed: 安全检查是否通过
            emergency_stop: 是否处于紧急停止状态
            tracking_quality: 跟踪质量评估结果
            force: 强制发布，忽略节流
        """
        # 节流检查
        if not force and (current_time - self._last_publish_time) < self._min_interval:
            return

        self._last_publish_time = current_time

        diag = self._build_diagnostics(
            current_time, state, cmd, state_output, consistency,
            mpc_health, timeout_status, transform_status,
            tracking_error, transition_progress, tf2_critical,
            safety_check_passed, emergency_stop, tracking_quality
        )
        
        # Pass the object directly, do NOT convert to dict
        self._last_published = diag
        
        # 调用所有回调（线程安全：复制列表后迭代）
        with self._callbacks_lock:
            if not self._callbacks:
                return
            callbacks_copy = list(self._callbacks)
        
        callbacks_to_remove = []
        for callback in callbacks_copy:
            try:
                # Pass the object
                callback(diag)
                # 成功时重置失败计数
                with self._callbacks_lock:
                    self._callback_fail_counts[id(callback)] = 0
            except Exception as e:
                callback_id = id(callback)
                with self._callbacks_lock:
                    fail_count = self._callback_fail_counts.get(callback_id, 0) + 1
                    self._callback_fail_counts[callback_id] = fail_count
                
                if fail_count >= self._callback_max_failures:
                    logger.warning(
                        f"Callback {callback} failed {fail_count} times consecutively, removing it. "
                        f"Last error: {e}"
                    )
                    callbacks_to_remove.append(callback)
                elif fail_count == 1:
                    # 首次失败记录警告
                    logger.warning(f"Callback error: {e}")
                else:
                    # 后续失败使用 debug 级别
                    logger.debug(f"Callback error (fail #{fail_count}): {e}")
        
        # 移除持续失败的回调
        if callbacks_to_remove:
            with self._callbacks_lock:
                for callback in callbacks_to_remove:
                    if callback in self._callbacks:
                        self._callbacks.remove(callback)
                        self._callback_fail_counts.pop(id(callback), None)
    
    def get_last_published(self) -> Optional[DiagnosticsV2]:
        """获取最后发布的诊断数据"""
        return self._last_published
    
    def _build_diagnostics(self,
                          current_time: float,
                          state: ControllerState,
                          cmd: ControlOutput,
                          state_output: Optional[EstimatorOutput],
                          consistency: Optional[ConsistencyResult],
                          mpc_health: Optional[MPCHealthStatus],
                          timeout_status: TimeoutStatus,
                          transform_status: Dict[str, Any],
                          tracking_error: Optional[Dict[str, float]],
                          transition_progress: float,
                          tf2_critical: bool,
                          safety_check_passed: bool = True,
                          emergency_stop: bool = False,
                          tracking_quality: Optional[Dict[str, Any]] = None) -> DiagnosticsV2:
        """构建 DiagnosticsV2 消息"""
        """构建 DiagnosticsV2 消息"""
        d = DiagnosticsV2()
        
        # Header
        d.header.stamp = current_time
        d.header.frame_id = ''
        
        d.state = int(state)
        d.mpc_success = cmd.success if cmd else False
        d.mpc_solve_time_ms = cmd.solve_time_ms if cmd else 0.0
        d.backup_active = state == ControllerState.BACKUP_ACTIVE
        
        # MPC 健康状态
        if mpc_health:
            d.mpc_health_kkt_residual = mpc_health.kkt_residual
            d.mpc_health_condition_number = mpc_health.condition_number
            d.mpc_health_consecutive_near_timeout = mpc_health.consecutive_near_timeout
            d.mpc_health_degradation_warning = mpc_health.degradation_warning
            d.mpc_health_can_recover = mpc_health.can_recover
        else:
            d.mpc_health_kkt_residual = 0.0
            d.mpc_health_condition_number = 1.0
            d.mpc_health_consecutive_near_timeout = 0
            d.mpc_health_degradation_warning = False
            d.mpc_health_can_recover = False
            
        # 一致性指标
        if consistency:
            d.consistency_curvature = consistency.kappa_consistency
            d.consistency_velocity_dir = consistency.v_dir_consistency
            d.consistency_temporal = consistency.temporal_smooth
            d.consistency_alpha_soft = consistency.alpha
            d.consistency_data_valid = consistency.data_valid
        else:
            d.consistency_curvature = 1.0
            d.consistency_velocity_dir = 1.0
            d.consistency_temporal = 1.0
            d.consistency_alpha_soft = 0.0
            d.consistency_data_valid = True
            
        # 状态估计器健康
        if state_output:
            d.estimator_covariance_norm = state_output.covariance_norm
            d.estimator_innovation_norm = state_output.innovation_norm
            d.estimator_slip_probability = state_output.slip_probability
            d.estimator_imu_drift_detected = state_output.imu_drift_detected
            d.estimator_imu_bias = state_output.imu_bias
            d.estimator_imu_available = state_output.imu_available
        else:
            d.estimator_covariance_norm = 0.0
            d.estimator_innovation_norm = 0.0
            d.estimator_slip_probability = 0.0
            d.estimator_imu_drift_detected = False
            d.estimator_imu_bias = np.zeros(3)
            d.estimator_imu_available = True
            
        # 跟踪误差
        if tracking_error:
            d.tracking_lateral_error = tracking_error.get('lateral_error', 0.0)
            d.tracking_longitudinal_error = tracking_error.get('longitudinal_error', 0.0)
            d.tracking_heading_error = tracking_error.get('heading_error', 0.0)
            d.tracking_prediction_error = tracking_error.get('prediction_error', float('nan'))
        else:
            d.tracking_lateral_error = 0.0
            d.tracking_longitudinal_error = 0.0
            d.tracking_heading_error = 0.0
            d.tracking_prediction_error = float('nan')
            
        # 跟踪质量
        if tracking_quality:
            d.tracking_quality_score = tracking_quality.get('overall_score', 0.0)
            d.tracking_quality_rating = tracking_quality.get('rating', 'unknown')
        else:
            d.tracking_quality_score = 0.0
            d.tracking_quality_rating = 'unknown'
            
        # 坐标变换状态
        d.transform_tf2_available = not tf2_critical
        d.transform_tf2_injected = transform_status.get('tf2_injected', False)
        d.transform_fallback_duration_ms = transform_status.get('fallback_duration_ms', 0.0)
        d.transform_accumulated_drift = transform_status.get('accumulated_drift', 0.0)
        d.transform_source_frame = transform_status.get('source_frame', '')
        d.transform_target_frame = transform_status.get('target_frame', '')
        d.transform_error_message = transform_status.get('error_message', '')
            
        # 超时状态
        d.timeout_odom = timeout_status.odom_timeout
        d.timeout_traj = timeout_status.traj_timeout
        d.timeout_traj_grace_exceeded = timeout_status.traj_grace_exceeded
        d.timeout_imu = timeout_status.imu_timeout
        d.timeout_last_odom_age_ms = timeout_status.last_odom_age_ms
        d.timeout_last_traj_age_ms = timeout_status.last_traj_age_ms
        d.timeout_last_imu_age_ms = timeout_status.last_imu_age_ms
        d.timeout_in_startup_grace = timeout_status.in_startup_grace
            
        # 控制命令
        if cmd:
            d.cmd_vx = cmd.vx
            d.cmd_vy = cmd.vy
            d.cmd_vz = cmd.vz
            d.cmd_omega = cmd.omega
            d.cmd_frame_id = cmd.frame_id
        else:
            d.cmd_vx = 0.0
            d.cmd_vy = 0.0
            d.cmd_vz = 0.0
            d.cmd_omega = 0.0
            d.cmd_frame_id = ''
            
        # 其他状态
        d.transition_progress = transition_progress
        d.safety_check_passed = safety_check_passed
        d.emergency_stop = emergency_stop
        
        return d
