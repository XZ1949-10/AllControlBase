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
    
    def __init__(self):
        """
        初始化诊断发布器
        
        注意: 话题名称配置已移至 ROS 层，核心库不再处理话题相关配置
        """
        # 回调函数（线程安全）
        self._callbacks: List[Callable[[Dict[str, Any]], None]] = []
        self._callbacks_lock = threading.Lock()
        
        # 回调失败计数（用于自动移除持续失败的回调）
        self._callback_fail_counts: Dict[int, int] = {}  # callback id -> fail count
        self._callback_max_failures = 5  # 连续失败超过此次数后移除回调
        
        # 发布历史
        self._last_published: Optional[Dict[str, Any]] = None
    
    def add_callback(self, callback: Callable[[Dict[str, Any]], None]) -> None:
        """
        添加诊断回调函数（线程安全）
        
        Args:
            callback: 回调函数，签名为 callback(diagnostics: Dict[str, Any]) -> None
        
        Note:
            如果回调连续失败超过 5 次，会被自动移除以防止日志泛滥
        """
        with self._callbacks_lock:
            if callback not in self._callbacks:
                self._callbacks.append(callback)
                self._callback_fail_counts[id(callback)] = 0
    
    def remove_callback(self, callback: Callable[[Dict[str, Any]], None]) -> None:
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
                tracking_quality: Optional[Dict[str, Any]] = None) -> None:
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
        """
        diag = self._build_diagnostics(
            current_time, state, cmd, state_output, consistency,
            mpc_health, timeout_status, transform_status,
            tracking_error, transition_progress, tf2_critical,
            safety_check_passed, emergency_stop, tracking_quality
        )
        
        diag_dict = diag.to_ros_msg()
        self._last_published = diag_dict
        
        # 调用所有回调（线程安全：复制列表后迭代）
        with self._callbacks_lock:
            callbacks_copy = list(self._callbacks)
        
        callbacks_to_remove = []
        for callback in callbacks_copy:
            try:
                callback(diag_dict)
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
    
    def get_last_published(self) -> Optional[Dict[str, Any]]:
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
        return DiagnosticsV2(
            header=Header(stamp=current_time, frame_id=''),
            state=int(state),
            mpc_success=cmd.success if cmd else False,
            mpc_solve_time_ms=cmd.solve_time_ms if cmd else 0.0,
            backup_active=state == ControllerState.BACKUP_ACTIVE,
            
            # MPC 健康状态
            mpc_health_kkt_residual=mpc_health.kkt_residual if mpc_health else 0.0,
            mpc_health_condition_number=mpc_health.condition_number if mpc_health else 1.0,
            mpc_health_consecutive_near_timeout=mpc_health.consecutive_near_timeout if mpc_health else 0,
            mpc_health_degradation_warning=mpc_health.degradation_warning if mpc_health else False,
            mpc_health_can_recover=mpc_health.can_recover if mpc_health else False,
            
            # 一致性指标
            consistency_curvature=consistency.kappa_consistency if consistency else 1.0,
            consistency_velocity_dir=consistency.v_dir_consistency if consistency else 1.0,
            consistency_temporal=consistency.temporal_smooth if consistency else 1.0,
            consistency_alpha_soft=consistency.alpha if consistency else 0.0,
            consistency_data_valid=consistency.data_valid if consistency else True,
            
            # 状态估计器健康
            estimator_covariance_norm=state_output.covariance_norm if state_output else 0.0,
            estimator_innovation_norm=state_output.innovation_norm if state_output else 0.0,
            estimator_slip_probability=state_output.slip_probability if state_output else 0.0,
            estimator_imu_drift_detected=state_output.imu_drift_detected if state_output else False,
            estimator_imu_bias=state_output.imu_bias if state_output else np.zeros(3),
            estimator_imu_available=state_output.imu_available if state_output else True,
            
            # 跟踪误差
            tracking_lateral_error=tracking_error.get('lateral_error', 0.0) if tracking_error else 0.0,
            tracking_longitudinal_error=tracking_error.get('longitudinal_error', 0.0) if tracking_error else 0.0,
            tracking_heading_error=tracking_error.get('heading_error', 0.0) if tracking_error else 0.0,
            tracking_prediction_error=tracking_error.get('prediction_error', 0.0) if tracking_error else 0.0,
            
            # 跟踪质量
            tracking_quality_score=tracking_quality.get('overall_score', 0.0) if tracking_quality else 0.0,
            tracking_quality_rating=tracking_quality.get('rating', 'unknown') if tracking_quality else 'unknown',
            
            # 坐标变换状态
            transform_tf2_available=not tf2_critical,
            transform_tf2_injected=transform_status.get('tf2_injected', False),
            transform_fallback_duration_ms=transform_status.get('fallback_duration_ms', 0.0),
            transform_accumulated_drift=transform_status.get('accumulated_drift', 0.0),
            transform_source_frame=transform_status.get('source_frame', ''),
            transform_target_frame=transform_status.get('target_frame', ''),
            transform_error_message=transform_status.get('error_message', ''),
            
            # 超时状态
            timeout_odom=timeout_status.odom_timeout,
            timeout_traj=timeout_status.traj_timeout,
            timeout_traj_grace_exceeded=timeout_status.traj_grace_exceeded,
            timeout_imu=timeout_status.imu_timeout,
            timeout_last_odom_age_ms=timeout_status.last_odom_age_ms,
            timeout_last_traj_age_ms=timeout_status.last_traj_age_ms,
            timeout_last_imu_age_ms=timeout_status.last_imu_age_ms,
            timeout_in_startup_grace=timeout_status.in_startup_grace,
            
            # 控制命令
            cmd_vx=cmd.vx if cmd else 0.0,
            cmd_vy=cmd.vy if cmd else 0.0,
            cmd_vz=cmd.vz if cmd else 0.0,
            cmd_omega=cmd.omega if cmd else 0.0,
            cmd_frame_id=cmd.frame_id if cmd else '',
            
            # 过渡进度
            transition_progress=transition_progress,
            
            # 安全状态
            safety_check_passed=safety_check_passed,
            emergency_stop=emergency_stop
        )
