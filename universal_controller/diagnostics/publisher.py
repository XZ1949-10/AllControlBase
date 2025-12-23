"""
诊断发布器

负责诊断数据的收集、格式化和发布。
从 ControllerManager 中抽离，实现单一职责原则。
"""
from typing import Dict, Any, Optional, Callable, List
import json
import numpy as np
import logging
import threading

from ..core.data_types import (
    DiagnosticsV2, Header, ControlOutput, EstimatorOutput, 
    TimeoutStatus, ConsistencyResult, MPCHealthStatus
)
from ..core.enums import ControllerState
from ..core.ros_compat import ROS_AVAILABLE

logger = logging.getLogger(__name__)


def _numpy_json_encoder(obj: Any) -> Any:
    """
    自定义 JSON 编码器处理 numpy 类型
    
    用于将包含 numpy 类型的字典序列化为 JSON。
    
    Args:
        obj: 需要编码的对象
    
    Returns:
        JSON 可序列化的对象
    
    Raises:
        TypeError: 如果对象无法序列化
    """
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    elif isinstance(obj, (np.integer,)):
        return int(obj)
    elif isinstance(obj, (np.floating,)):
        return float(obj)
    elif isinstance(obj, np.bool_):
        return bool(obj)
    elif isinstance(obj, (np.complex64, np.complex128)):
        return {'real': float(obj.real), 'imag': float(obj.imag)}
    raise TypeError(f"Object of type {type(obj).__name__} is not JSON serializable")


class DiagnosticsPublisher:
    """
    诊断发布器
    
    负责：
    - 收集各模块的诊断数据
    - 构建 DiagnosticsV2 消息
    - 通过 ROS 话题或回调函数发布
    - 维护发布历史
    
    使用示例:
        publisher = DiagnosticsPublisher()
        publisher.set_callback(my_callback)
        publisher.publish(...)
    """
    
    def __init__(self, diagnostics_topic: str = '/controller/diagnostics',
                 cmd_topic: str = '/cmd_unified'):
        """
        初始化诊断发布器
        
        Args:
            diagnostics_topic: 诊断话题名称
            cmd_topic: 控制命令话题名称
        """
        self._diagnostics_topic = diagnostics_topic
        self._cmd_topic = cmd_topic
        
        # 回调函数（线程安全）
        self._callbacks: List[Callable[[Dict[str, Any]], None]] = []
        self._callbacks_lock = threading.Lock()
        
        # ROS Publishers
        self._diagnostics_pub: Optional[Any] = None
        self._cmd_pub: Optional[Any] = None
        
        # 发布历史
        self._last_published: Optional[Dict[str, Any]] = None
        
        # 发布失败警告标志（避免重复警告）
        self._cmd_publish_warned: bool = False
        self._diag_publish_warned: bool = False
        
        # 初始化 ROS Publishers
        self._init_ros_publishers()
    
    @property
    def cmd_topic(self) -> str:
        """获取控制命令话题名称"""
        return self._cmd_topic
    
    @property
    def diagnostics_topic(self) -> str:
        """获取诊断话题名称"""
        return self._diagnostics_topic
    
    def _init_ros_publishers(self) -> None:
        """初始化 ROS Publishers
        
        注意: 在 ROS 环境下，诊断发布由 controller_ros 的 controller_node.py 处理，
        使用 DiagnosticsV2 消息类型。这里不再创建发布器，避免话题类型冲突。
        
        如果需要在非 ROS 环境下使用 ROS 发布功能，可以通过 enable_ros_publish=True 参数启用。
        """
        # 在 ROS 环境下，不自动创建发布器
        # 诊断发布由 controller_ros/controller_node.py 处理 (使用 DiagnosticsV2)
        # 这里只使用回调机制传递数据
        self._diagnostics_pub = None
        self._cmd_pub = None
        
        logger.debug("DiagnosticsPublisher: ROS publishers disabled (handled by controller_node)")
    
    def add_callback(self, callback: Callable[[Dict[str, Any]], None]) -> None:
        """
        添加诊断回调函数（线程安全）
        
        Args:
            callback: 回调函数，签名为 callback(diagnostics: Dict[str, Any]) -> None
        """
        with self._callbacks_lock:
            if callback not in self._callbacks:
                self._callbacks.append(callback)
    
    def remove_callback(self, callback: Callable[[Dict[str, Any]], None]) -> None:
        """移除诊断回调函数（线程安全）"""
        with self._callbacks_lock:
            if callback in self._callbacks:
                self._callbacks.remove(callback)
    
    def clear_callbacks(self) -> None:
        """清除所有回调函数（线程安全）"""
        with self._callbacks_lock:
            self._callbacks.clear()
    
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
                tf2_critical: bool) -> None:
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
        """
        diag = self._build_diagnostics(
            current_time, state, cmd, state_output, consistency,
            mpc_health, timeout_status, transform_status,
            tracking_error, transition_progress, tf2_critical
        )
        
        diag_dict = diag.to_ros_msg()
        self._last_published = diag_dict
        
        # 调用所有回调（线程安全：复制列表后迭代）
        with self._callbacks_lock:
            callbacks_copy = list(self._callbacks)
        
        for callback in callbacks_copy:
            try:
                callback(diag_dict)
            except Exception as e:
                logger.warning(f"Callback error: {e}")
        
        # ROS 发布
        self._publish_to_ros(diag_dict)
    
    def publish_command(self, cmd: ControlOutput) -> None:
        """
        发布控制命令到 ROS 话题
        
        Args:
            cmd: 控制命令
        """
        if self._cmd_pub is None:
            return
        
        try:
            from geometry_msgs.msg import Twist
            
            twist = Twist()
            twist.linear.x = cmd.vx
            twist.linear.y = cmd.vy
            twist.linear.z = cmd.vz
            twist.angular.z = cmd.omega
            
            self._cmd_pub.publish(twist)
        except Exception as e:
            # 首次失败记录 warning，后续使用 debug 避免日志泛滥
            if not self._cmd_publish_warned:
                logger.warning(f"Command publish failed (subsequent failures will be debug): {e}")
                self._cmd_publish_warned = True
            else:
                logger.debug(f"Command publish failed: {e}")
    
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
                          tf2_critical: bool) -> DiagnosticsV2:
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
            mpc_health_degradation_warning=mpc_health.warning if mpc_health else False,
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
            
            # 坐标变换状态
            transform_tf2_available=not tf2_critical,
            transform_tf2_injected=transform_status.get('tf2_injected', False),
            transform_fallback_duration_ms=transform_status.get('fallback_duration_ms', 0.0),
            transform_accumulated_drift=transform_status.get('accumulated_drift', 0.0),
            
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
            transition_progress=transition_progress
        )
    
    def _publish_to_ros(self, diag_dict: Dict[str, Any]) -> None:
        """发布到 ROS 话题"""
        if self._diagnostics_pub is None:
            return
        
        try:
            from std_msgs.msg import String
            
            msg = String()
            msg.data = json.dumps(diag_dict, default=_numpy_json_encoder)
            self._diagnostics_pub.publish(msg)
        except Exception as e:
            # 首次失败记录 warning，后续使用 debug 避免日志泛滥
            if not self._diag_publish_warned:
                logger.warning(f"Diagnostics publish failed (subsequent failures will be debug): {e}")
                self._diag_publish_warned = True
            else:
                logger.debug(f"Diagnostics publish failed: {e}")
