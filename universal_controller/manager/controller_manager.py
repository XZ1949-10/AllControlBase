"""控制器管理器"""
from typing import Dict, Any, Optional
import numpy as np
import time

from ..core.interfaces import (
    IStateEstimator, ITrajectoryTracker, IConsistencyChecker,
    ISafetyMonitor, ISmoothTransition, ICoordinateTransformer, IAttitudeController
)
from ..core.data_types import (
    Trajectory, ControlOutput, ConsistencyResult, EstimatorOutput,
    TimeoutStatus, DiagnosticsV2, Header, Odometry, Imu, AttitudeCommand
)
from ..core.enums import ControllerState, PlatformType
from ..config.default_config import PLATFORM_CONFIG, DEFAULT_CONFIG
from ..safety.timeout_monitor import TimeoutMonitor
from ..safety.state_machine import StateMachine
from ..health.mpc_health_monitor import MPCHealthMonitor


def _get_monotonic_time() -> float:
    """
    获取单调时钟时间（秒）
    
    使用 time.monotonic() 而非 time.time()，避免系统时间跳变
    （如 NTP 同步）导致的时间间隔计算错误。
    """
    return time.monotonic()


class ControllerManager:
    """控制器管理器"""
    
    def __init__(self, config: Dict[str, Any]):
        self.config = config
        self.ctrl_freq = config.get('system', {}).get('ctrl_freq', 50)
        self.dt = 1.0 / self.ctrl_freq
        
        platform_name = config.get('system', {}).get('platform', 'differential')
        self.platform_config = PLATFORM_CONFIG.get(platform_name, PLATFORM_CONFIG['differential'])
        self.default_frame_id = self.platform_config.get('output_frame', 'base_link')
        self.transform_target_frame = config.get('transform', {}).get('target_frame', 'odom')
        
        # 检查是否为无人机平台
        self.is_quadrotor = self.platform_config.get('type') == PlatformType.QUADROTOR
        
        mpc_config = config.get('mpc', {})
        self.horizon_normal = mpc_config.get('horizon', 20)
        self.horizon_degraded = mpc_config.get('horizon_degraded', 10)
        self._current_horizon = self.horizon_normal
        
        # 组件
        self.state_estimator: Optional[IStateEstimator] = None
        self.mpc_tracker: Optional[ITrajectoryTracker] = None
        self.backup_tracker: Optional[ITrajectoryTracker] = None
        self.consistency_checker: Optional[IConsistencyChecker] = None
        self.state_machine: Optional[StateMachine] = None
        self.smooth_transition: Optional[ISmoothTransition] = None
        self.coord_transformer: Optional[ICoordinateTransformer] = None
        self.safety_monitor: Optional[ISafetyMonitor] = None
        self.mpc_health_monitor: Optional[MPCHealthMonitor] = None
        self.attitude_controller: Optional[IAttitudeController] = None  # F14.1: 姿态控制器
        self.timeout_monitor = TimeoutMonitor(config)
        
        # 状态
        self._last_state = ControllerState.INIT
        self._safety_failed = False
        self._last_diagnostics: Dict[str, Any] = {}
        self._last_mpc_health = None
        self._last_consistency: Optional[ConsistencyResult] = None
        self._last_mpc_cmd: Optional[ControlOutput] = None
        self._last_backup_cmd: Optional[ControlOutput] = None
        self._last_tracking_error: Optional[Dict[str, float]] = None
        self._last_published_diagnostics: Optional[Dict[str, Any]] = None
        self._last_attitude_cmd: Optional[AttitudeCommand] = None  # F14: 姿态命令
        self._last_update_time: Optional[float] = None  # 用于计算实际时间间隔
        
        # 诊断发布
        self._diagnostics_callback: Optional[callable] = None  # 诊断回调函数
        self._diagnostics_pub: Optional[Any] = None  # ROS Publisher (如果可用)
        self._cmd_pub: Optional[Any] = None  # 控制命令 Publisher
        
        # 尝试初始化 ROS Publisher
        self._init_ros_publishers()
    
    def _init_ros_publishers(self) -> None:
        """
        初始化 ROS Publishers
        
        在 ROS 环境下创建:
        - /controller/diagnostics: 诊断消息
        - /cmd_unified: 统一控制命令
        
        非 ROS 环境下静默跳过
        """
        from ..core.ros_compat import ROS_AVAILABLE
        
        if not ROS_AVAILABLE:
            return
        
        try:
            import rospy
            from std_msgs.msg import String
            
            # 使用 String 消息类型发布 JSON 格式的诊断数据
            # 实际部署时可替换为自定义消息类型
            self._diagnostics_pub = rospy.Publisher(
                '/controller/diagnostics',
                String,
                queue_size=1
            )
            
            # 控制命令发布器 (使用 geometry_msgs/Twist 或自定义消息)
            from geometry_msgs.msg import Twist
            self._cmd_pub = rospy.Publisher(
                '/cmd_unified',
                Twist,
                queue_size=1
            )
            
            print("[ControllerManager] ROS publishers initialized")
        except Exception as e:
            print(f"[ControllerManager] ROS publisher init failed: {e}")
            self._diagnostics_pub = None
            self._cmd_pub = None
    
    def set_diagnostics_callback(self, callback: callable) -> None:
        """
        设置诊断回调函数
        
        回调函数签名: callback(diagnostics: Dict[str, Any]) -> None
        
        用于非 ROS 环境下获取诊断数据，或用于日志记录、监控等
        """
        self._diagnostics_callback = callback
    
    def publish_command(self, cmd: ControlOutput) -> None:
        """
        发布控制命令到 ROS 话题
        
        在 ROS 环境下发布到 /cmd_unified
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
            pass  # 静默处理发布失败
    
    def get_last_published_diagnostics(self) -> Optional[Dict[str, Any]]:
        """获取最后发布的诊断数据"""
        return self._last_published_diagnostics
    
    def initialize_components(self,
                             state_estimator: Optional[IStateEstimator] = None,
                             mpc_tracker: Optional[ITrajectoryTracker] = None,
                             backup_tracker: Optional[ITrajectoryTracker] = None,
                             consistency_checker: Optional[IConsistencyChecker] = None,
                             safety_monitor: Optional[ISafetyMonitor] = None,
                             smooth_transition: Optional[ISmoothTransition] = None,
                             coord_transformer: Optional[ICoordinateTransformer] = None,
                             mpc_health_monitor: Optional[MPCHealthMonitor] = None,
                             attitude_controller: Optional[IAttitudeController] = None) -> None:
        """组件注入方法"""
        if state_estimator:
            self.state_estimator = state_estimator
        if mpc_tracker:
            self.mpc_tracker = mpc_tracker
        if backup_tracker:
            self.backup_tracker = backup_tracker
        if consistency_checker:
            self.consistency_checker = consistency_checker
        if safety_monitor:
            self.safety_monitor = safety_monitor
        if smooth_transition:
            self.smooth_transition = smooth_transition
        if coord_transformer:
            self.coord_transformer = coord_transformer
        if mpc_health_monitor:
            self.mpc_health_monitor = mpc_health_monitor
        if attitude_controller:
            self.attitude_controller = attitude_controller
        
        # 延迟绑定
        if self.coord_transformer and self.state_estimator:
            self.coord_transformer.set_state_estimator(self.state_estimator)
        
        # 初始化状态机
        self.state_machine = StateMachine(self.config)
    
    def initialize_default_components(self) -> None:
        """使用默认组件初始化"""
        from ..estimator.adaptive_ekf import AdaptiveEKFEstimator
        from ..tracker.pure_pursuit import PurePursuitController
        from ..tracker.mpc_controller import MPCController
        from ..consistency.weighted_analyzer import WeightedConsistencyAnalyzer
        from ..safety.safety_monitor import BasicSafetyMonitor
        from ..transition.smooth_transition import ExponentialSmoothTransition
        from ..transform.robust_transformer import RobustCoordinateTransformer
        
        # 基础组件
        components = {
            'state_estimator': AdaptiveEKFEstimator(self.config),
            'mpc_tracker': MPCController(self.config, self.platform_config),
            'backup_tracker': PurePursuitController(self.config, self.platform_config),
            'consistency_checker': WeightedConsistencyAnalyzer(self.config),
            'safety_monitor': BasicSafetyMonitor(self.config, self.platform_config),
            'smooth_transition': ExponentialSmoothTransition(self.config),
            'coord_transformer': RobustCoordinateTransformer(self.config),
            'mpc_health_monitor': MPCHealthMonitor(self.config)
        }
        
        # F14: 无人机平台添加姿态控制器
        if self.is_quadrotor:
            from ..tracker.attitude_controller import QuadrotorAttitudeController
            components['attitude_controller'] = QuadrotorAttitudeController(self.config)
        
        self.initialize_components(**components)
    
    def _on_state_changed(self, old_state: ControllerState, new_state: ControllerState) -> None:
        """状态变化回调"""
        print(f"Controller state changed: {old_state.name} -> {new_state.name}")
        
        # MPC horizon 动态调整
        if new_state == ControllerState.MPC_DEGRADED:
            if self._current_horizon != self.horizon_degraded:
                self._current_horizon = self.horizon_degraded
                if self.mpc_tracker:
                    self.mpc_tracker.set_horizon(self.horizon_degraded)
                print(f"MPC horizon reduced to {self.horizon_degraded}")
        elif new_state == ControllerState.NORMAL and old_state == ControllerState.MPC_DEGRADED:
            if self._current_horizon != self.horizon_normal:
                self._current_horizon = self.horizon_normal
                if self.mpc_tracker:
                    self.mpc_tracker.set_horizon(self.horizon_normal)
                print(f"MPC horizon restored to {self.horizon_normal}")
        
        # 平滑过渡
        if (old_state in [ControllerState.NORMAL, ControllerState.SOFT_DISABLED, ControllerState.MPC_DEGRADED] and
            new_state == ControllerState.BACKUP_ACTIVE):
            if self.smooth_transition and self._last_mpc_cmd is not None:
                self.smooth_transition.start_transition(self._last_mpc_cmd)
        
        elif (old_state == ControllerState.BACKUP_ACTIVE and
              new_state in [ControllerState.NORMAL, ControllerState.SOFT_DISABLED, ControllerState.MPC_DEGRADED]):
            if self.smooth_transition and self._last_backup_cmd is not None:
                self.smooth_transition.start_transition(self._last_backup_cmd)

    
    def update(self, odom: Odometry, trajectory: Trajectory, 
               imu: Optional[Imu] = None) -> ControlOutput:
        """主控制循环"""
        # 使用单调时钟，避免系统时间跳变影响
        current_time = _get_monotonic_time()
        
        # 计算实际时间间隔，用于 EKF 预测
        # 长时间暂停检测阈值
        LONG_PAUSE_THRESHOLD = 0.5  # 500ms
        EKF_RESET_THRESHOLD = 2.0   # 2s
        
        # 标记是否需要跳过预测（EKF 刚重置后）
        skip_prediction = False
        
        if self._last_update_time is not None:
            actual_dt = current_time - self._last_update_time
            
            # 检测长时间暂停
            if actual_dt > EKF_RESET_THRESHOLD:
                # 超过 2 秒的暂停，重置 EKF 以避免累积误差
                print(f"[ControllerManager] Long pause detected ({actual_dt:.2f}s), resetting EKF")
                if self.state_estimator:
                    self.state_estimator.reset()
                # 重置后跳过本次预测，只进行观测更新
                # 这样可以让 EKF 从观测数据重新初始化状态
                skip_prediction = True
                actual_dt = self.dt  # 使用默认值（用于后续计算）
            elif actual_dt > LONG_PAUSE_THRESHOLD:
                # 500ms - 2s 的暂停，使用多步预测
                # 将长时间间隔分解为多个小步骤，提高预测精度
                print(f"[ControllerManager] Medium pause detected ({actual_dt:.2f}s), using multi-step prediction")
                num_steps = int(actual_dt / self.dt)
                if self.state_estimator and num_steps > 1:
                    for _ in range(num_steps - 1):
                        self.state_estimator.predict(self.dt)
                actual_dt = actual_dt - (num_steps - 1) * self.dt
            
            # 限制时间间隔在合理范围内，避免异常值
            # 最小 1ms，最大 500ms（长暂停已经处理）
            actual_dt = np.clip(actual_dt, 0.001, LONG_PAUSE_THRESHOLD)
        else:
            actual_dt = self.dt  # 首次调用使用默认值
        self._last_update_time = current_time
        
        # 1. 更新超时监控
        self.timeout_monitor.update_odom(odom.header.stamp)
        self.timeout_monitor.update_trajectory(trajectory.header.stamp)
        if imu is not None:
            self.timeout_monitor.update_imu(imu.header.stamp)
        timeout_status = self.timeout_monitor.check()  # 不传入时间，使用内部单调时钟
        
        # 2. 状态估计
        state_output: Optional[EstimatorOutput] = None
        if self.state_estimator:
            # EKF 标准流程: 先预测，再更新
            # 如果 EKF 刚重置，跳过预测步骤
            if not skip_prediction:
                self.state_estimator.predict(actual_dt)
            
            # 更新步骤: 先更新 odom，再更新 IMU
            self.state_estimator.update_odom(odom)
            if timeout_status.imu_timeout:
                self.state_estimator.set_imu_available(False)
            elif imu:
                self.state_estimator.set_imu_available(True)
                self.state_estimator.update_imu(imu)
            state_output = self.state_estimator.get_state()
            state = state_output.state
        else:
            state = np.zeros(8)
        
        # 3. 一致性检查
        if self.consistency_checker:
            consistency = self.consistency_checker.compute(trajectory)
        else:
            consistency = ConsistencyResult(0, 1, 1, 1, True, True)
        self._last_consistency = consistency
        
        # 4. 坐标变换
        if self.coord_transformer:
            transformed_traj, tf_status = self.coord_transformer.transform_trajectory(
                trajectory, self.transform_target_frame, current_time)
            tf2_critical = tf_status.is_critical()
        else:
            transformed_traj = trajectory
            tf2_critical = False
        
        # 5. MPC 计算
        mpc_cmd = None
        if self.mpc_tracker and self._last_state not in [ControllerState.BACKUP_ACTIVE, 
                                                          ControllerState.STOPPING,
                                                          ControllerState.STOPPED]:
            mpc_cmd = self.mpc_tracker.compute(state, transformed_traj, consistency)
            if mpc_cmd.success:
                self._last_mpc_cmd = mpc_cmd.copy()
        
        # MPC 健康监控
        if self.mpc_health_monitor and mpc_cmd is not None:
            mpc_health = self.mpc_health_monitor.update(
                mpc_cmd.solve_time_ms,
                mpc_cmd.health_metrics.get('kkt_residual', 0.0),
                mpc_cmd.health_metrics.get('condition_number', 1.0)
            )
        else:
            mpc_health = None
        self._last_mpc_health = mpc_health
        
        # 6. 构建诊断信息
        # 注意: safety_failed 使用上一周期的值，避免循环依赖
        diagnostics = {
            'alpha': consistency.alpha,
            'data_valid': consistency.data_valid,
            'mpc_health': mpc_health,
            'mpc_success': mpc_cmd.success if mpc_cmd else False,
            'odom_timeout': timeout_status.odom_timeout,
            'traj_timeout_exceeded': timeout_status.traj_grace_exceeded,
            'v_horizontal': np.sqrt(state[3]**2 + state[4]**2),
            'vz': state[5],
            'has_valid_data': len(trajectory.points) > 0,
            'tf2_critical': tf2_critical,
            'safety_failed': self._safety_failed,
            'current_state': self._last_state,  # 传递当前状态给安全监控器
        }
        self._last_diagnostics = diagnostics
        
        # 7. 选择控制器输出 (在状态机更新之前，基于当前状态)
        if self._last_state in [ControllerState.BACKUP_ACTIVE, ControllerState.STOPPING]:
            if self.backup_tracker:
                cmd = self.backup_tracker.compute(state, transformed_traj, consistency)
                self._last_backup_cmd = cmd.copy()
            else:
                cmd = ControlOutput(vx=0, vy=0, vz=0, omega=0, frame_id=self.default_frame_id)
        else:
            cmd = mpc_cmd if mpc_cmd else ControlOutput(vx=0, vy=0, vz=0, omega=0, 
                                                        frame_id=self.default_frame_id)
        
        # 计算跟踪误差
        self._last_tracking_error = self._compute_tracking_error(state, transformed_traj)
        
        # 8. 安全检查 (在状态机更新之前)
        safety_triggered = False
        if self.safety_monitor:
            safety_decision = self.safety_monitor.check(state, cmd, diagnostics)
            if not safety_decision.safe:
                self._safety_failed = True
                safety_triggered = True
                if safety_decision.limited_cmd is not None:
                    cmd = safety_decision.limited_cmd
                # 更新诊断信息中的 safety_failed 标志
                diagnostics['safety_failed'] = True
            else:
                self._safety_failed = False
        
        # 9. 更新状态机 (只调用一次，包含安全检查结果)
        if self.state_machine:
            new_state = self.state_machine.update(diagnostics)
            if new_state != self._last_state:
                self._on_state_changed(self._last_state, new_state)
                self._last_state = new_state
        
        # 10. 平滑过渡
        if self.smooth_transition and not self.smooth_transition.is_complete():
            cmd = self.smooth_transition.get_blended_output(cmd, current_time)
        
        # 11. 发布诊断 (使用 time.time() 作为时间戳，因为这是给外部使用的)
        wall_time = time.time()
        self._publish_diagnostics(wall_time, cmd, state_output, timeout_status, tf2_critical)
        
        return cmd
    
    def _publish_diagnostics(self, current_time: float, cmd: ControlOutput,
                            state_output: Optional[EstimatorOutput],
                            timeout_status: TimeoutStatus,
                            tf2_critical: bool) -> None:
        """
        发布诊断消息
        
        支持两种发布模式:
        1. ROS 环境: 通过 ROS Publisher 发布到 /controller/diagnostics
        2. 非 ROS 环境: 保存到 _last_published_diagnostics，可通过回调获取
        """
        transform_status = self.coord_transformer.get_status() if self.coord_transformer else {
            'fallback_duration_ms': 0.0, 'accumulated_drift': 0.0
        }
        
        diag = DiagnosticsV2(
            header=Header(stamp=current_time, frame_id=''),
            state=int(self._last_state),
            mpc_success=self._last_diagnostics.get('mpc_success', False),
            mpc_solve_time_ms=cmd.solve_time_ms if cmd else 0.0,
            backup_active=self._last_state == ControllerState.BACKUP_ACTIVE,
            
            mpc_health_kkt_residual=self._last_mpc_health.kkt_residual if self._last_mpc_health else 0.0,
            mpc_health_condition_number=self._last_mpc_health.condition_number if self._last_mpc_health else 1.0,
            mpc_health_consecutive_near_timeout=self._last_mpc_health.consecutive_near_timeout if self._last_mpc_health else 0,
            mpc_health_degradation_warning=self._last_mpc_health.warning if self._last_mpc_health else False,
            mpc_health_can_recover=self._last_mpc_health.can_recover if self._last_mpc_health else False,
            
            consistency_curvature=self._last_consistency.kappa_consistency if self._last_consistency else 1.0,
            consistency_velocity_dir=self._last_consistency.v_dir_consistency if self._last_consistency else 1.0,
            consistency_temporal=self._last_consistency.temporal_smooth if self._last_consistency else 1.0,
            consistency_alpha_soft=self._last_consistency.alpha if self._last_consistency else 0.0,
            consistency_data_valid=self._last_consistency.data_valid if self._last_consistency else True,
            
            estimator_covariance_norm=state_output.covariance_norm if state_output else 0.0,
            estimator_innovation_norm=state_output.innovation_norm if state_output else 0.0,
            estimator_slip_probability=state_output.slip_probability if state_output else 0.0,
            estimator_imu_drift_detected=state_output.imu_drift_detected if state_output else False,
            estimator_imu_bias=state_output.imu_bias if state_output else np.zeros(3),
            estimator_imu_available=state_output.imu_available if state_output else True,
            
            tracking_lateral_error=self._last_tracking_error.get('lateral_error', 0.0) if self._last_tracking_error else 0.0,
            tracking_longitudinal_error=self._last_tracking_error.get('longitudinal_error', 0.0) if self._last_tracking_error else 0.0,
            tracking_heading_error=self._last_tracking_error.get('heading_error', 0.0) if self._last_tracking_error else 0.0,
            tracking_prediction_error=self._last_tracking_error.get('prediction_error', 0.0) if self._last_tracking_error else 0.0,
            
            transform_tf2_available=not tf2_critical,
            transform_fallback_duration_ms=transform_status['fallback_duration_ms'],
            transform_accumulated_drift=transform_status['accumulated_drift'],
            
            timeout_odom=timeout_status.odom_timeout,
            timeout_traj=timeout_status.traj_timeout,
            timeout_traj_grace_exceeded=timeout_status.traj_grace_exceeded,
            timeout_imu=timeout_status.imu_timeout,
            timeout_last_odom_age_ms=timeout_status.last_odom_age_ms,
            timeout_last_traj_age_ms=timeout_status.last_traj_age_ms,
            timeout_last_imu_age_ms=timeout_status.last_imu_age_ms,
            timeout_in_startup_grace=timeout_status.in_startup_grace,
            
            cmd_vx=cmd.vx,
            cmd_vy=cmd.vy,
            cmd_vz=cmd.vz,
            cmd_omega=cmd.omega,
            cmd_frame_id=cmd.frame_id,
            
            transition_progress=self.smooth_transition.get_progress() if self.smooth_transition else 0.0
        )
        
        # 转换为字典格式
        diag_dict = diag.to_ros_msg()
        self._last_published_diagnostics = diag_dict
        
        # 调用诊断回调 (如果已注册)
        if self._diagnostics_callback is not None:
            try:
                self._diagnostics_callback(diag_dict)
            except Exception as e:
                print(f"[ControllerManager] Diagnostics callback error: {e}")
        
        # ROS 环境下发布到话题
        if self._diagnostics_pub is not None:
            try:
                import json
                from std_msgs.msg import String
                
                # 自定义 JSON 编码器处理 numpy 类型
                def numpy_encoder(obj):
                    if isinstance(obj, np.ndarray):
                        return obj.tolist()
                    elif isinstance(obj, (np.integer, np.floating)):
                        return float(obj)
                    elif isinstance(obj, np.bool_):
                        return bool(obj)
                    return str(obj)
                
                msg = String()
                msg.data = json.dumps(diag_dict, default=numpy_encoder)
                self._diagnostics_pub.publish(msg)
            except Exception as e:
                # 非 ROS 环境或发布失败，静默处理
                pass
    
    def _compute_tracking_error(self, state: np.ndarray, trajectory: Trajectory) -> Dict[str, float]:
        """计算跟踪误差"""
        if len(trajectory.points) < 2:
            return {'lateral_error': 0.0, 'longitudinal_error': 0.0, 
                    'heading_error': 0.0, 'prediction_error': 0.0}
        
        px, py = state[0], state[1]
        theta = state[6]
        
        min_dist = float('inf')
        closest_idx = 0
        for i, p in enumerate(trajectory.points):
            dist = np.sqrt((p.x - px)**2 + (p.y - py)**2)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        
        if closest_idx < len(trajectory.points) - 1:
            p0 = trajectory.points[closest_idx]
            p1 = trajectory.points[closest_idx + 1]
        elif closest_idx > 0:
            p0 = trajectory.points[closest_idx - 1]
            p1 = trajectory.points[closest_idx]
        else:
            return {'lateral_error': min_dist, 'longitudinal_error': 0.0,
                    'heading_error': 0.0, 'prediction_error': 0.0}
        
        dx = p1.x - p0.x
        dy = p1.y - p0.y
        traj_length = np.sqrt(dx**2 + dy**2)
        
        if traj_length < 1e-6:
            return {'lateral_error': min_dist, 'longitudinal_error': 0.0,
                    'heading_error': 0.0, 'prediction_error': 0.0}
        
        tx, ty = dx / traj_length, dy / traj_length
        ex = px - p0.x
        ey = py - p0.y
        
        longitudinal_error = ex * tx + ey * ty
        lateral_error = -ex * ty + ey * tx
        
        traj_heading = np.arctan2(dy, dx)
        heading_error = theta - traj_heading
        heading_error = np.arctan2(np.sin(heading_error), np.cos(heading_error))
        
        return {
            'lateral_error': lateral_error,
            'longitudinal_error': longitudinal_error,
            'heading_error': heading_error,
            'prediction_error': 0.0
        }
    
    def get_timeout_status(self) -> TimeoutStatus:
        return self.timeout_monitor.check(time.time())
    
    def get_state(self) -> ControllerState:
        return self._last_state
    
    def get_diagnostics(self) -> Dict[str, Any]:
        return self._last_diagnostics.copy()
    
    def reset(self) -> None:
        if self.state_estimator: self.state_estimator.reset()
        if self.consistency_checker: self.consistency_checker.reset()
        if self.state_machine: self.state_machine.reset()
        if self.safety_monitor: self.safety_monitor.reset()
        if self.mpc_health_monitor: self.mpc_health_monitor.reset()
        if self.attitude_controller: self.attitude_controller.reset()  # F14: 重置姿态控制器
        self.timeout_monitor.reset()
        self._last_state = ControllerState.INIT
        self._safety_failed = False
        self._last_mpc_cmd = None
        self._last_backup_cmd = None
        self._last_tracking_error = None
        self._last_consistency = None
        self._last_mpc_health = None
        self._last_attitude_cmd = None
        self._last_update_time = None  # 重置时间跟踪
        self._current_horizon = self.horizon_normal
        if self.mpc_tracker:
            self.mpc_tracker.set_horizon(self.horizon_normal)
    
    def shutdown(self) -> None:
        if self.backup_tracker: self.backup_tracker.shutdown()
        if self.mpc_tracker: self.mpc_tracker.shutdown()
    
    # F14: 无人机姿态控制接口
    def compute_attitude_command(self, velocity_cmd: ControlOutput, 
                                 current_state: np.ndarray,
                                 yaw_mode: str = 'velocity') -> Optional[AttitudeCommand]:
        """
        计算姿态命令 (F14.1)
        
        仅在无人机平台且姿态控制器可用时有效
        
        Args:
            velocity_cmd: 速度命令
            current_state: 当前状态
            yaw_mode: 航向模式 ('velocity', 'fixed', 'manual')
        
        Returns:
            AttitudeCommand 或 None (非无人机平台)
        """
        if not self.is_quadrotor or self.attitude_controller is None:
            return None
        
        attitude_cmd = self.attitude_controller.compute_attitude(
            velocity_cmd, current_state, yaw_mode)
        self._last_attitude_cmd = attitude_cmd
        return attitude_cmd
    
    def get_last_attitude_command(self) -> Optional[AttitudeCommand]:
        """获取最后的姿态命令"""
        return self._last_attitude_cmd
    
    def set_hover_yaw(self, yaw: float) -> None:
        """设置悬停时的目标航向 (F14.3)"""
        if self.attitude_controller is not None:
            self.attitude_controller.set_hover_yaw(yaw)
