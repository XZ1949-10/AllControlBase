"""
控制器管理器

负责协调所有控制器组件，执行主控制循环。

坐标系说明 (不需要建图/定位):
==================================

    base_link (机体坐标系)              odom (里程计坐标系)
    ┌───────────────┐                   ┌─────────────────────┐
    │       ↑ X     │                   │                     │
    │       │       │    坐标变换        │    机器人轨迹       │
    │    ←──┼──→    │  ───────────→     │    ○──○──○──○       │
    │     Y │       │  base_link→odom   │                     │
    │       ↓       │                   │    启动位置 ●       │
    └───────────────┘                   └─────────────────────┘
    
    - 原点在机器人中心                   - 从启动位置开始累积
    - X轴朝前                            - 会有漂移（正常）
    - 随机器人移动                       - 不需要建图/定位

数据流:
    网络输出轨迹 (base_link, 局部坐标)
        ↓
    坐标变换 (base_link → odom)
        ↓
    控制器计算 (在 odom 坐标系下)
        ↓
    控制输出:
        - 差速车/阿克曼车: base_link (vx, omega)
        - 全向车/四旋翼: odom (vx, vy, omega)

注意:
    - odom 就是你的"世界坐标系"，不需要建图
    - 网络输出的轨迹应该设置 frame_id='base_link'
    - 如果轨迹已经在 odom 坐标系，会跳过变换直接使用
"""
from typing import Dict, Any, Optional
import numpy as np
import time
import logging

from ..core.interfaces import (
    IStateEstimator, ITrajectoryTracker, IConsistencyChecker,
    ISafetyMonitor, ISmoothTransition, ICoordinateTransformer, IAttitudeController
)
from ..core.data_types import (
    Trajectory, ControlOutput, ConsistencyResult, EstimatorOutput,
    TimeoutStatus, DiagnosticsV2, Header, Odometry, Imu, AttitudeCommand
)
from ..core.enums import ControllerState, PlatformType
from ..core.diagnostics_input import DiagnosticsInput
from ..core.ros_compat import get_monotonic_time, normalize_angle
from ..config.default_config import PLATFORM_CONFIG, DEFAULT_CONFIG
from ..safety.timeout_monitor import TimeoutMonitor
from ..safety.state_machine import StateMachine
from ..health.mpc_health_monitor import MPCHealthMonitor
from ..diagnostics.publisher import DiagnosticsPublisher

logger = logging.getLogger(__name__)


class ControllerManager:
    """
    控制器管理器
    
    负责协调所有控制器组件，执行主控制循环。
    
    线程安全性说明:
        - update() 方法不是线程安全的，应该在单个线程中调用
        - 如果需要在多线程环境中使用，调用者需要自行加锁
        - set_diagnostics_callback() 是线程安全的
    
    使用示例:
        manager = ControllerManager(config)
        manager.initialize_default_components()
        
        # 在控制循环中调用
        cmd = manager.update(odom, trajectory, imu)
    """
    
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
        self._last_diagnostics: Optional[DiagnosticsInput] = None
        self._last_mpc_health = None
        self._last_consistency: Optional[ConsistencyResult] = None
        self._last_mpc_cmd: Optional[ControlOutput] = None
        self._last_backup_cmd: Optional[ControlOutput] = None
        self._last_tracking_error: Optional[Dict[str, float]] = None
        self._last_published_diagnostics: Optional[Dict[str, Any]] = None
        self._last_attitude_cmd: Optional[AttitudeCommand] = None  # F14: 姿态命令
        self._last_update_time: Optional[float] = None  # 用于计算实际时间间隔
        
        # 诊断发布器（职责分离）
        diagnostics_config = config.get('diagnostics', {})
        self._diagnostics_publisher = DiagnosticsPublisher(
            diagnostics_topic=diagnostics_config.get('topic', '/controller/diagnostics'),
            cmd_topic=diagnostics_config.get('cmd_topic', '/cmd_unified')
        )
        
        # 尝试初始化 ROS Publisher (向后兼容)
        self._cmd_pub: Optional[Any] = None
        self._init_ros_publishers()
    
    def _init_ros_publishers(self) -> None:
        """
        初始化 ROS Publishers (已禁用)
        
        注意: 在 ROS 环境下，命令发布由 controller_ros/controller_node.py 处理，
        使用 UnifiedCmd 消息类型。这里不再创建发布器，避免话题类型冲突。
        
        controller_node.py 会调用 _publish_cmd() 方法发布 UnifiedCmd 消息。
        """
        # 禁用内部 ROS 发布器，由 controller_node.py 处理
        self._cmd_pub = None
    
    def set_diagnostics_callback(self, callback: callable) -> None:
        """
        设置诊断回调函数
        
        回调函数签名: callback(diagnostics: Dict[str, Any]) -> None
        
        用于非 ROS 环境下获取诊断数据，或用于日志记录、监控等
        """
        self._diagnostics_publisher.add_callback(callback)
    
    def get_last_published_diagnostics(self) -> Optional[Dict[str, Any]]:
        """获取最后发布的诊断数据"""
        return self._diagnostics_publisher.get_last_published()
    
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
        from ..core.data_types import TrajectoryDefaults
        
        # 初始化轨迹默认配置
        TrajectoryDefaults.configure(self.config)
        
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
        logger.info(f"Controller state changed: {old_state.name} -> {new_state.name}")
        
        # MPC horizon 动态调整
        if new_state == ControllerState.MPC_DEGRADED:
            if self._current_horizon != self.horizon_degraded:
                self._current_horizon = self.horizon_degraded
                if self.mpc_tracker:
                    self.mpc_tracker.set_horizon(self.horizon_degraded)
                logger.info(f"MPC horizon reduced to {self.horizon_degraded}")
        elif new_state == ControllerState.NORMAL and old_state == ControllerState.MPC_DEGRADED:
            if self._current_horizon != self.horizon_normal:
                self._current_horizon = self.horizon_normal
                if self.mpc_tracker:
                    self.mpc_tracker.set_horizon(self.horizon_normal)
                logger.info(f"MPC horizon restored to {self.horizon_normal}")
        
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
        """
        主控制循环
        
        控制流程:
        1. 时间管理和暂停检测
        2. 超时监控
        3. 状态估计 (EKF)
        4. 一致性检查
        5. 坐标变换
        6. MPC 计算和健康监控
        7. 控制器选择
        8. 安全检查
        9. 状态机更新
        10. 平滑过渡
        11. 诊断发布
        """
        current_time = get_monotonic_time()
        
        # 1. 时间管理
        actual_dt, skip_prediction = self._compute_time_step(current_time)
        
        # 2. 超时监控
        timeout_status = self._update_timeout_monitor(imu is not None)
        
        # 3. 状态估计
        state, state_output = self._update_state_estimation(
            odom, imu, timeout_status, actual_dt, skip_prediction)
        
        # 4. 一致性检查
        consistency = self._compute_consistency(trajectory)
        
        # 5. 坐标变换
        transformed_traj, tf2_critical = self._transform_trajectory(trajectory, current_time)
        
        # 6. MPC 计算和健康监控
        mpc_cmd, mpc_health = self._compute_mpc(state, transformed_traj, consistency)
        
        # 7. 构建诊断信息
        diagnostics = self._build_diagnostics(
            consistency, mpc_health, mpc_cmd, timeout_status, 
            state, trajectory, tf2_critical)
        
        # 8. 选择控制器输出
        cmd = self._select_controller_output(
            state, transformed_traj, consistency, mpc_cmd)
        
        # 9. 计算跟踪误差
        self._last_tracking_error = self._compute_tracking_error(state, transformed_traj)
        
        # 10. 安全检查
        cmd = self._apply_safety_check(state, cmd, diagnostics)
        
        # 11. 状态机更新
        self._update_state_machine(diagnostics)
        
        # 12. 平滑过渡
        cmd = self._apply_smooth_transition(cmd, current_time)
        
        # 13. 发布诊断
        self._publish_diagnostics(
            state_output, timeout_status, tf2_critical, cmd)
        
        return cmd
    
    def _compute_time_step(self, current_time: float) -> tuple:
        """
        计算时间步长，处理暂停检测
        
        Returns:
            (actual_dt, skip_prediction): 实际时间步长和是否跳过预测
        """
        long_pause_threshold = self.config.get('system', {}).get('long_pause_threshold', 0.5)
        ekf_reset_threshold = self.config.get('system', {}).get('ekf_reset_threshold', 2.0)
        skip_prediction = False
        
        if self._last_update_time is not None:
            actual_dt = current_time - self._last_update_time
            
            if actual_dt > ekf_reset_threshold:
                logger.warning(f"Long pause detected ({actual_dt:.2f}s), resetting EKF")
                if self.state_estimator:
                    self.state_estimator.reset()
                skip_prediction = True
                actual_dt = self.dt
            elif actual_dt > long_pause_threshold:
                logger.info(f"Medium pause detected ({actual_dt:.2f}s), using multi-step prediction")
                num_steps = int(actual_dt / self.dt)
                if self.state_estimator and num_steps > 1:
                    for _ in range(num_steps - 1):
                        self.state_estimator.predict(self.dt)
                actual_dt = actual_dt - (num_steps - 1) * self.dt
            
            actual_dt = np.clip(actual_dt, 0.001, long_pause_threshold)
        else:
            actual_dt = self.dt
        
        self._last_update_time = current_time
        return actual_dt, skip_prediction
    
    def _update_timeout_monitor(self, has_imu: bool) -> TimeoutStatus:
        """
        更新超时监控
        
        Args:
            has_imu: 是否有 IMU 数据
        
        Note:
            超时检测基于数据接收时间（单调时钟），而非消息时间戳。
            这确保了超时检测不受系统时间跳变影响。
            
            重要：update_odom/trajectory/imu 应该在数据实际接收时调用，
            而不是在控制循环中调用。这里只调用 check() 检查超时状态。
        """
        # 注意：不再在这里调用 update_xxx()，这些应该在数据接收时调用
        # 保留这些调用是为了向后兼容，但实际上应该由外部调用者负责
        # 在 ROS 节点中，应该在回调函数中调用 notify_xxx_received()
        return self.timeout_monitor.check()
    
    def _update_state_estimation(self, odom: Odometry, imu: Optional[Imu],
                                 timeout_status: TimeoutStatus, actual_dt: float,
                                 skip_prediction: bool) -> tuple:
        """
        更新状态估计
        
        Returns:
            (state, state_output): 状态向量和完整估计输出
        """
        state_output: Optional[EstimatorOutput] = None
        
        if self.state_estimator:
            if not skip_prediction:
                self.state_estimator.predict(actual_dt)
            
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
        
        return state, state_output
    
    def _compute_consistency(self, trajectory: Trajectory) -> ConsistencyResult:
        """计算一致性"""
        if self.consistency_checker:
            consistency = self.consistency_checker.compute(trajectory)
        else:
            consistency = ConsistencyResult(0, 1, 1, 1, True, True)
        self._last_consistency = consistency
        return consistency
    
    def _transform_trajectory(self, trajectory: Trajectory, 
                              current_time: float) -> tuple:
        """
        坐标变换
        
        将网络输出的局部轨迹变换到控制器工作坐标系。
        
        坐标系说明:
        - 输入轨迹: base_link (局部坐标系，当前位置为原点)
        - 输出轨迹: odom (世界坐标系)
        
        Returns:
            (transformed_traj, tf2_critical): 变换后轨迹和是否临界
        """
        if self.coord_transformer:
            transformed_traj, tf_status = self.coord_transformer.transform_trajectory(
                trajectory, self.transform_target_frame, current_time)
            tf2_critical = tf_status.is_critical()
        else:
            # 无坐标变换器时，假设轨迹已经在正确的坐标系
            # 这种情况下，轨迹应该已经是世界坐标系
            transformed_traj = trajectory
            tf2_critical = False
            
            # 如果轨迹声明在局部坐标系但没有变换器，发出警告
            if trajectory.header.frame_id in ['base_link', 'base_link_0']:
                logger.warning(
                    f"Trajectory in local frame '{trajectory.header.frame_id}' "
                    f"but no coordinate transformer configured. "
                    f"Control may be incorrect."
                )
        
        return transformed_traj, tf2_critical
    
    def _compute_mpc(self, state: np.ndarray, trajectory: Trajectory,
                     consistency: ConsistencyResult) -> tuple:
        """
        MPC 计算和健康监控
        
        Returns:
            (mpc_cmd, mpc_health): MPC 命令和健康状态
        """
        mpc_cmd = None
        mpc_health = None
        
        if self.mpc_tracker and self._last_state not in [
            ControllerState.BACKUP_ACTIVE, 
            ControllerState.STOPPING,
            ControllerState.STOPPED
        ]:
            mpc_cmd = self.mpc_tracker.compute(state, trajectory, consistency)
            if mpc_cmd.success:
                self._last_mpc_cmd = mpc_cmd.copy()
        
        if self.mpc_health_monitor and mpc_cmd is not None:
            mpc_health = self.mpc_health_monitor.update(
                mpc_cmd.solve_time_ms,
                mpc_cmd.health_metrics.get('kkt_residual', 0.0),
                mpc_cmd.health_metrics.get('condition_number', 1.0)
            )
        
        self._last_mpc_health = mpc_health
        return mpc_cmd, mpc_health
    
    def _build_diagnostics(self, consistency: ConsistencyResult,
                          mpc_health: Any, mpc_cmd: Optional[ControlOutput],
                          timeout_status: TimeoutStatus, state: np.ndarray,
                          trajectory: Trajectory, tf2_critical: bool) -> DiagnosticsInput:
        """构建诊断信息"""
        diagnostics = DiagnosticsInput(
            alpha=consistency.alpha,
            data_valid=consistency.data_valid,
            mpc_health=mpc_health,
            mpc_success=mpc_cmd.success if mpc_cmd else False,
            odom_timeout=timeout_status.odom_timeout,
            traj_timeout_exceeded=timeout_status.traj_grace_exceeded,
            v_horizontal=np.sqrt(state[3]**2 + state[4]**2),
            vz=state[5],
            has_valid_data=len(trajectory.points) > 0,
            tf2_critical=tf2_critical,
            safety_failed=self._safety_failed,
            current_state=self._last_state,
        )
        self._last_diagnostics = diagnostics
        return diagnostics
    
    def _select_controller_output(self, state: np.ndarray, trajectory: Trajectory,
                                  consistency: ConsistencyResult,
                                  mpc_cmd: Optional[ControlOutput]) -> ControlOutput:
        """选择控制器输出"""
        if self._last_state in [ControllerState.BACKUP_ACTIVE, ControllerState.STOPPING]:
            if self.backup_tracker:
                cmd = self.backup_tracker.compute(state, trajectory, consistency)
                self._last_backup_cmd = cmd.copy()
            else:
                cmd = ControlOutput(vx=0, vy=0, vz=0, omega=0, frame_id=self.default_frame_id)
        else:
            cmd = mpc_cmd if mpc_cmd else ControlOutput(
                vx=0, vy=0, vz=0, omega=0, frame_id=self.default_frame_id)
        return cmd
    
    def _apply_safety_check(self, state: np.ndarray, cmd: ControlOutput,
                           diagnostics: DiagnosticsInput) -> ControlOutput:
        """应用安全检查"""
        if self.safety_monitor:
            safety_decision = self.safety_monitor.check(state, cmd, diagnostics)
            if not safety_decision.safe:
                self._safety_failed = True
                if safety_decision.limited_cmd is not None:
                    cmd = safety_decision.limited_cmd
                diagnostics.safety_failed = True
            else:
                self._safety_failed = False
        return cmd
    
    def _update_state_machine(self, diagnostics: DiagnosticsInput) -> None:
        """更新状态机"""
        if self.state_machine:
            new_state = self.state_machine.update(diagnostics)
            if new_state != self._last_state:
                self._on_state_changed(self._last_state, new_state)
                self._last_state = new_state
    
    def _apply_smooth_transition(self, cmd: ControlOutput, 
                                 current_time: float) -> ControlOutput:
        """应用平滑过渡"""
        if self.smooth_transition and not self.smooth_transition.is_complete():
            cmd = self.smooth_transition.get_blended_output(cmd, current_time)
        return cmd
    
    def _publish_diagnostics(self, state_output: Optional[EstimatorOutput],
                            timeout_status: TimeoutStatus, tf2_critical: bool,
                            cmd: ControlOutput) -> None:
        """发布诊断信息"""
        wall_time = time.time()
        transform_status = self.coord_transformer.get_status() if self.coord_transformer else {
            'fallback_duration_ms': 0.0, 'accumulated_drift': 0.0
        }
        self._diagnostics_publisher.publish(
            current_time=wall_time,
            state=self._last_state,
            cmd=cmd,
            state_output=state_output,
            consistency=self._last_consistency,
            mpc_health=self._last_mpc_health,
            timeout_status=timeout_status,
            transform_status=transform_status,
            tracking_error=self._last_tracking_error,
            transition_progress=self.smooth_transition.get_progress() if self.smooth_transition else 0.0,
            tf2_critical=tf2_critical
        )
    
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
        heading_error = normalize_angle(heading_error)
        
        return {
            'lateral_error': lateral_error,
            'longitudinal_error': longitudinal_error,
            'heading_error': heading_error,
            'prediction_error': 0.0
        }
    
    def get_timeout_status(self) -> TimeoutStatus:
        """获取超时状态"""
        return self.timeout_monitor.check()
    
    def get_state(self) -> ControllerState:
        return self._last_state
    
    def get_diagnostics(self) -> Dict[str, Any]:
        if self._last_diagnostics is None:
            return {}
        return self._last_diagnostics.to_dict()
    
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
