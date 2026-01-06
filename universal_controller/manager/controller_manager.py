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
    ISafetyMonitor, ISmoothTransition, ICoordinateTransformer, IControlProcessor
)
from ..core.data_types import (
    Trajectory, ControlOutput, ConsistencyResult, EstimatorOutput,
    TimeoutStatus, DiagnosticsV2, Header, Odometry, Imu, AttitudeCommand,
    TrajectoryConfig
)
from ..core.enums import ControllerState, PlatformType
from ..core.diagnostics_input import DiagnosticsInput
from ..core.ros_compat import get_monotonic_time, normalize_angle
from ..core.constants import EPSILON, MIN_SEGMENT_LENGTH
from ..config.default_config import PLATFORM_CONFIG, DEFAULT_CONFIG
from ..config.validation import validate_logical_consistency, ConfigValidationError
from ..config.utils import deep_update
from ..safety.timeout_monitor import TimeoutMonitor
from ..safety.state_machine import StateMachine
from ..health.mpc_health_monitor import MPCHealthMonitor
from ..diagnostics.publisher import DiagnosticsPublisher
from ..estimator.adaptive_ekf import AdaptiveEKFEstimator
from ..tracker.pure_pursuit import PurePursuitController
from ..tracker.mpc_controller import MPCController
from ..consistency.weighted_analyzer import WeightedConsistencyAnalyzer
from ..safety.safety_monitor import BasicSafetyMonitor
from ..transition.smooth_transition import ExponentialSmoothTransition, LinearSmoothTransition
from ..transform.robust_transformer import RobustCoordinateTransformer
from ..processors.attitude_processor import AttitudeProcessor

logger = logging.getLogger(__name__)


class ControllerManager:
    """
    控制器管理器
    
    负责协调所有控制器组件，执行主控制循环。
    
    线程安全性说明:
        - update() 方法不是线程安全的，应该在单个线程中调用
        - 如果需要在多线程环境中使用，调用者需要自行加锁
        - set_diagnostics_callback() 是线程安全的
    
    配置验证:
        - 默认在初始化时进行配置验证（可通过 validate_config=False 禁用）
        - 验证失败会记录警告但不会阻止初始化
        - 使用 strict_mode=True 可在验证失败时抛出异常
    
    使用示例:
        manager = ControllerManager(config)
        manager.initialize_default_components()
        
        # 在控制循环中调用
        cmd = manager.update(odom, trajectory, imu)
    """
    
    def __init__(self, config: Dict[str, Any], validate_config: bool = True, 
                 strict_mode: bool = False, profile: Optional[str] = None):
        """
        初始化控制器管理器
        
        Args:
            config: 配置字典（如果提供了 profile，则此参数为基础配置或覆盖项）
            validate_config: 是否在初始化时验证配置（默认 True）
            strict_mode: 严格模式，验证失败时抛出异常（默认 False）
            profile: 配置预设名称 ("balanced", "aggressive", "safe")
        """
        # 加载 Profile 配置
        if profile:
            from ..config.profiles import create_default_config
            profile_config = create_default_config(profile)
            # 简单的字典合并：config 覆盖 profile_config
            # 注意：这里只做浅层合并，实际使用可能需要递归合并
            # 使用深度合并，确保嵌套配置（如 MPC 权重）不丢失
            deep_update(profile_config, config) 
            self.config = profile_config
        else:
            self.config = config
        
        # 配置验证
        if validate_config:
            # 1. 结构验证 (Schema Validation) - 确保类型正确
            # 这通常是致命错误，因此默认抛出异常 (Crash Early)
            from .config_validator import ConfigValidator
            ConfigValidator.validate(self.config)
            
            # 2. 逻辑验证 (Logical Validation) - 确保数值合理
            self._validate_config(strict_mode)
        
        # 加载 TrajectoryConfig (不再使用全局 TrajectoryDefaults)
        self.trajectory_config = TrajectoryConfig.from_dict(self.config)

        self.ctrl_freq = self.config.get('system', {}).get('ctrl_freq', 50)
        self.dt = 1.0 / self.ctrl_freq

        platform_name = self.config.get('system', {}).get('platform', 'differential')
        self.platform_config = PLATFORM_CONFIG.get(platform_name, PLATFORM_CONFIG['differential'])
        self.default_frame_id = self.platform_config.get('output_frame', 'base_link')
        self.transform_target_frame = self.config.get('transform', {}).get('target_frame', 'odom')
        
        # 检查是否为无人机平台（使用 is_ground_vehicle 配置）
        self.is_quadrotor = not self.platform_config.get(
            'is_ground_vehicle', 
            self.platform_config.get('type') != PlatformType.QUADROTOR
        )
        
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
        self.processors: list[IControlProcessor] = []
        self.timeout_monitor = TimeoutMonitor(config)
        
        # 状态
        self._last_state = ControllerState.INIT
        self._safety_failed = False
        self._safety_check_passed = True  # 安全检查是否通过
        self._safety_check_passed = True  # 安全检查是否通过
        self._diagnostics_input = DiagnosticsInput()  # 复用诊断对象以减少 GC
        self._last_diagnostics: Optional[DiagnosticsInput] = None
        self._last_mpc_health = None
        self._last_consistency: Optional[ConsistencyResult] = None
        self._last_mpc_cmd: Optional[ControlOutput] = None
        self._last_backup_cmd: Optional[ControlOutput] = None
        self._last_tracking_error: Optional[Dict[str, float]] = None
        self._last_tracking_quality: Optional[Dict[str, Any]] = None  # 跟踪质量评估结果
        self._last_published_diagnostics: Optional[Dict[str, Any]] = None
        self._last_attitude_cmd: Optional[AttitudeCommand] = None  # F14: 姿态命令
        self._last_update_time: Optional[float] = None  # 用于计算实际时间间隔
        
        # 跟踪质量评估配置
        from ..config.system_config import TRACKING_CONFIG
        tracking_config = config.get('tracking', {})
        self._tracking_thresholds = {
            'lateral': tracking_config.get('lateral_thresh', TRACKING_CONFIG['lateral_thresh']),
            'longitudinal': tracking_config.get('longitudinal_thresh', TRACKING_CONFIG['longitudinal_thresh']),
            'heading': tracking_config.get('heading_thresh', TRACKING_CONFIG['heading_thresh']),
            'prediction': tracking_config.get('prediction_thresh', TRACKING_CONFIG['prediction_thresh']),
        }
        self._tracking_weights = tracking_config.get('weights', TRACKING_CONFIG['weights'].copy())
        self._tracking_rating = tracking_config.get('rating', TRACKING_CONFIG['rating'].copy())
        
        # 预测误差计算相关状态
        self._last_mpc_predicted_state: Optional[np.ndarray] = None
        
        # 诊断发布器（职责分离）
        # 读取配置中的发布频率，默认 10Hz
        diag_rate = self.config.get('system', {}).get('diagnostics_rate', 10.0)
        self._diagnostics_publisher = DiagnosticsPublisher(publish_rate=diag_rate)
        
        # 处理器故障计数器
        self._processor_failure_counts = {}
        self._processor_max_failures = 10
    
    def _validate_config(self, strict_mode: bool = False) -> None:
        """
        验证配置参数
        
        验证策略:
        - FATAL 级别错误: 始终阻止启动（如 v_max <= 0）
        - ERROR 级别错误: strict_mode=True 时阻止启动，否则只记录警告
        - WARNING 级别: 只记录日志，不阻止启动
        
        Args:
            strict_mode: 严格模式，ERROR 级别错误也会阻止启动
        
        Raises:
            ConfigValidationError: 当存在 FATAL 错误，或 strict_mode=True 且存在 ERROR 错误时
        """
        from ..config.validation import ValidationSeverity
        
        errors = validate_logical_consistency(self.config)
        
        if not errors:
            return
        
        # 按严重级别分类
        fatal_errors = [(k, m, s) for k, m, s in errors if s == ValidationSeverity.FATAL]
        error_errors = [(k, m, s) for k, m, s in errors if s == ValidationSeverity.ERROR]
        warning_errors = [(k, m, s) for k, m, s in errors if s == ValidationSeverity.WARNING]
        
        # 记录所有警告
        for key, msg, _ in warning_errors:
            logger.warning(f'配置警告 [{key}]: {msg}')
        
        # FATAL 错误始终阻止启动
        if fatal_errors:
            fatal_msgs = '\n'.join([f'  - [FATAL] {key}: {msg}' for key, msg, _ in fatal_errors])
            raise ConfigValidationError(f'配置存在致命错误，无法启动:\n{fatal_msgs}')
        
        # ERROR 错误根据 strict_mode 决定
        if error_errors:
            if strict_mode:
                error_msgs = '\n'.join([f'  - [ERROR] {key}: {msg}' for key, msg, _ in error_errors])
                raise ConfigValidationError(f'配置验证失败 (严格模式):\n{error_msgs}')
            else:
                # 非严格模式：记录警告但不阻止启动
                for key, msg, _ in error_errors:
                    logger.warning(f'配置问题 [{key}]: {msg} [ERROR]')
    

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
    
    # ==================== 数据接收通知接口 ====================
    # Refactored: notify_xxx methods removed to simplify interface.
    # Timeout detection is now handled by passing data_ages directly to update().

    
    def initialize_components(self,
                             state_estimator: Optional[IStateEstimator] = None,
                             mpc_tracker: Optional[ITrajectoryTracker] = None,
                             backup_tracker: Optional[ITrajectoryTracker] = None,
                             consistency_checker: Optional[IConsistencyChecker] = None,
                             safety_monitor: Optional[ISafetyMonitor] = None,
                             smooth_transition: Optional[ISmoothTransition] = None,
                             coord_transformer: Optional[ICoordinateTransformer] = None,
                             mpc_health_monitor: Optional[MPCHealthMonitor] = None,
                             processors: Optional[list[IControlProcessor]] = None) -> None:
        """
        组件注入方法
        
        
        依赖绑定:
        - 各组件独立，无显式相互依赖注入。
        - 运行时依赖（如 Transformer 需要 Estimator 状态）通过方法参数传递。
        """
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
        if processors:
            self.processors = processors
        

        
        # 初始化状态机（如果尚未初始化）
        if self.state_machine is None:
            self.state_machine = StateMachine(self.config)
    

    
    def _get_default_component_classes(self) -> Dict[str, Any]:
        """
        获取默认组件类
        
        子类可以重写此方法以注入自定义组件类，而无需重写整个初始化逻辑。
        """
        return {
            'estimator_cls': AdaptiveEKFEstimator,
            'mpc_cls': MPCController,
            'backup_cls': PurePursuitController,
            'consistency_cls': WeightedConsistencyAnalyzer,
            'safety_cls': BasicSafetyMonitor,
            'transformer_cls': RobustCoordinateTransformer,
            'smooth_linear_cls': LinearSmoothTransition,
            'smooth_exp_cls': ExponentialSmoothTransition,
            'attitude_processor_cls': AttitudeProcessor,
        }

    def initialize_default_components(self) -> None:
        """
        使用默认组件初始化
        
        这是一个便利方法，用于快速设置标准组件栈。
        如果需要自定义各个组件实例，请直接调用 initialize_components()。
        """
        from ..health.mpc_health_monitor import MPCHealthMonitor
        
        classes = self._get_default_component_classes()
        
        # 根据配置选择平滑过渡实现
        transition_type = self.config.get('transition', {}).get('type', 'exponential')
        if transition_type == 'linear':
            smooth_transition = classes['smooth_linear_cls'](self.config)
        else:
            smooth_transition = classes['smooth_exp_cls'](self.config)
        
        # 基础组件
        components = {
            'state_estimator': classes['estimator_cls'](self.config),
            'mpc_tracker': classes['mpc_cls'](self.config, self.platform_config),
            'backup_tracker': classes['backup_cls'](self.config, self.platform_config),
            'consistency_checker': classes['consistency_cls'](self.config),
            'safety_monitor': classes['safety_cls'](self.config, self.platform_config),
            'smooth_transition': smooth_transition,
            'coord_transformer': classes['transformer_cls'](self.config),
            'mpc_health_monitor': MPCHealthMonitor(self.config)
        }
        
        # Load processors
        proc_list = []
        if self.is_quadrotor:
            proc_list.append(classes['attitude_processor_cls'](self.config))
        components['processors'] = proc_list
        
        self.initialize_components(**components)
    
    def _on_state_changed(self, old_state: ControllerState, new_state: ControllerState) -> None:
        """状态变化回调"""
        logger.info(f"Controller state changed: {old_state.name} -> {new_state.name}")
        
        # MPC horizon 动态调整
        # 
        # 设计说明:
        # - _current_horizon 记录"期望的 horizon"，而非"实际的 horizon"
        # - 即使 set_horizon() 被节流，也要更新 _current_horizon
        # - 这确保了状态机的期望状态被正确记录
        # - MPC 控制器内部会在下次调用时检查并应用正确的 horizon
        #
        # 节流机制说明:
        # - set_horizon() 有节流机制防止频繁重新初始化 ACADOS 求解器
        # - 当被节流时返回 False，但这不影响状态机的期望状态
        # - 实际的 horizon 会在节流间隔过后自动同步
        if new_state == ControllerState.MPC_DEGRADED:
            target_horizon = self.horizon_degraded
            if self._current_horizon != target_horizon:
                self._current_horizon = target_horizon
                if self.mpc_tracker:
                    if self.mpc_tracker.set_horizon(target_horizon):
                        logger.info(f"MPC horizon reduced to {target_horizon}")
                    else:
                        logger.debug(f"MPC horizon change to {target_horizon} throttled, will sync later")
        elif new_state == ControllerState.NORMAL and old_state == ControllerState.MPC_DEGRADED:
            target_horizon = self.horizon_normal
            if self._current_horizon != target_horizon:
                self._current_horizon = target_horizon
                if self.mpc_tracker:
                    if self.mpc_tracker.set_horizon(target_horizon):
                        logger.info(f"MPC horizon restored to {target_horizon}")
                    else:
                        logger.debug(f"MPC horizon change to {target_horizon} throttled, will sync later")
        
        # 平滑过渡
        if (old_state in [ControllerState.NORMAL, ControllerState.SOFT_DISABLED, ControllerState.MPC_DEGRADED] and
            new_state == ControllerState.BACKUP_ACTIVE):
            if self.smooth_transition and self._last_mpc_cmd is not None:
                self.smooth_transition.start_transition(self._last_mpc_cmd)
        
        elif (old_state == ControllerState.BACKUP_ACTIVE and
              new_state in [ControllerState.NORMAL, ControllerState.SOFT_DISABLED, ControllerState.MPC_DEGRADED]):
            if self.smooth_transition and self._last_backup_cmd is not None:
                self.smooth_transition.start_transition(self._last_backup_cmd)

    
    def update(self, current_time: float, odom: Odometry, trajectory: Trajectory, 
               data_ages: Dict[str, float], imu: Optional[Imu] = None) -> ControlOutput:
        """
        主控制循环
        
        Args:
            current_time: 当前时间 (秒) - 应使用 ROS Time / Sim Time 以确保 TF 同步
            odom: 里程计数据
            trajectory: 轨迹数据
            data_ages: 数据年龄字典 {'odom': sec, 'trajectory': sec, 'imu': sec}
            imu: IMU 数据 (可选)
            
        Returns:
            ControlOutput: 控制输出
        """
        
        # 1. 时间管理
        actual_dt, skip_prediction = self._compute_time_step(current_time)
        
        # 1.5 轨迹验证 (现在需要显式调用，因为去除了全局默认值)
        trajectory.validate(self.trajectory_config)
        
        # 2. 超时监控
        # 直接使用传入的 data_ages 进行检查
        current_timeout_status = self.timeout_monitor.check(data_ages)
        self._last_timeout_status = current_timeout_status
        
        # 3. 状态估计
        state, state_output = self._update_state_estimation(
            odom, imu, current_timeout_status, actual_dt, current_time, skip_prediction)
        
        # 4. 一致性检查
        consistency = self._compute_consistency(trajectory)
        
        # 5. 坐标变换
        transformed_traj, tf2_critical = self._transform_trajectory(trajectory, current_time)
        
        # 6. MPC 计算和健康监控
        mpc_cmd, mpc_health = self._compute_mpc(state, transformed_traj, consistency)
        
        # 7. 构建诊断信息
        diagnostics = self._build_diagnostics(
            consistency, mpc_health, mpc_cmd, current_timeout_status, 
            state, trajectory, tf2_critical)
        
        # 8. 选择控制器输出
        cmd = self._select_controller_output(
            state, transformed_traj, consistency, mpc_cmd)
        
        # 9. 计算跟踪误差和质量评估
        self._update_tracking_metrics(state, transformed_traj)
        
        # 10. 安全检查
        cmd = self._apply_safety_check(state, cmd, diagnostics)
        
        # 11. 状态机更新
        self._update_state_machine(diagnostics)
        
        # 12. 平滑过渡
        cmd = self._apply_smooth_transition(cmd, current_time)
        
        # 13. 发布诊断
        self._publish_diagnostics(
            state_output, current_timeout_status, tf2_critical, cmd, current_time)
        
        # 14. 运行额外处理器 (Processors)
        self._run_processors(state, cmd)
        
        return cmd

    def _update_tracking_metrics(self, state: np.ndarray, transformed_traj: Trajectory) -> None:
        """更新跟踪误差和质量评估指标"""
        self._last_tracking_error = self._compute_tracking_error(state, transformed_traj)
        self._last_tracking_quality = self._compute_tracking_quality(self._last_tracking_error)

    def _run_processors(self, state: np.ndarray, cmd: ControlOutput) -> None:
        """运行额外处理器 (带性能监控)"""
        if not self.processors:
            return
            
        # 优化: 仅在 DEBUG 模式下启用耗时统计，避免生产环境下的 System Call 开销
        enable_profiling = logger.isEnabledFor(logging.DEBUG)

        # 将解耦的平台特定逻辑（如姿态控制）作为处理器运行
        for processor in self.processors:
            # 使用对象 ID 作为唯一键，避免同类实例冲突
            proc_id = id(processor)
            proc_name = processor.__class__.__name__
            
            # 检查是否因为连续失败而被禁用
            if self._processor_failure_counts.get(proc_id, 0) >= self._processor_max_failures:
                continue

            try:
                if enable_profiling:
                    proc_start = time.monotonic()
                    extra_result = processor.compute_extra(state, cmd)
                    proc_duration = (time.monotonic() - proc_start) * 1000
                    
                    # 性能告警：处理器耗时超过 5ms
                    if proc_duration > 5.0:
                        logger.warning(
                            f"Processor {proc_name} took too long: {proc_duration:.2f}ms. "
                            f"This may affect control loop stability."
                        )
                else:
                    # 生产模式: 直接调用，无计时开销
                    extra_result = processor.compute_extra(state, cmd)
                
                # 成功运行，重置失败计数
                if self._processor_failure_counts.get(proc_id, 0) > 0:
                    self._processor_failure_counts[proc_id] = 0
                
                if extra_result:
                    cmd.extras.update(extra_result)
            except Exception as e:
                # 累加失败计数
                self._processor_failure_counts[proc_id] = self._processor_failure_counts.get(proc_id, 0) + 1
                fail_count = self._processor_failure_counts[proc_id]
                
                if fail_count >= self._processor_max_failures:
                    logger.error(f"Processor {proc_name} (id={proc_id}) failed {fail_count} times. Disabling it. Last error: {e}")
                else:
                    logger.error(f"Processor {proc_name} (id={proc_id}) failed (count {fail_count}): {e}")

    def request_stop(self) -> bool:
        """
        请求控制器进入停止状态
        
        代理到状态机的 request_stop 方法
        
        Returns:
            bool: 成功请求返回 True，否则返回 False
        """
        if self.state_machine:
            return self.state_machine.request_stop()
        return False

    def get_timeout_status(self) -> TimeoutStatus:
        """获取当前超时状态 (供外部查询)"""
        if hasattr(self, '_last_timeout_status'):
            return self._last_timeout_status
        # 如果尚未运行 update，返回默认超时状态
        return self.timeout_monitor.check({})
    
    # 辅助接口
    def set_hover_yaw(self, yaw: float) -> None:
        for p in self.processors:
            if hasattr(p, 'set_hover_yaw'):
                p.set_hover_yaw(yaw)

    def get_attitude_rate_limits(self) -> Optional[Dict[str, Any]]:
        for p in self.processors:
            if hasattr(p, 'get_attitude_rate_limits'):
                return p.get_attitude_rate_limits()
        return None        

    
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
                # 限制最大补算步数，防止死循环 (例如 System Time Step = 20ms, Pause = 60s -> 3000 steps)
                # 如果暂停太久，与其卡死 CPU 补算，不如只补算最近的一段，或者重置
                MAX_CATCHUP_STEPS = 20  # Max 0.2s - 0.5s of catchup
                
                num_steps = int(actual_dt / self.dt)
                if num_steps > MAX_CATCHUP_STEPS:
                    logger.warning(f"Pause ({actual_dt:.2f}s) exceeds catchup limit. Resetting EKF to current state.")
                    # 强制重置 EKF 到当前状态，避免巨大的 Innovation 导致发散
                    if self.state_estimator:
                        self.state_estimator.reset()
                    
                    # 标记跳过预测，actual_dt 设为正常帧率，让 estimator 重新锁定 odom
                    skip_prediction = True
                    actual_dt = self.dt
                
                if not skip_prediction and self.state_estimator and num_steps > 1:
                    for _ in range(num_steps - 1):
                        self.state_estimator.predict(self.dt)
                actual_dt = actual_dt - (num_steps - 1) * self.dt
            
            actual_dt = np.clip(actual_dt, 0.001, long_pause_threshold)
        else:
            actual_dt = self.dt
        
        self._last_update_time = current_time
        return actual_dt, skip_prediction
    

    
    def _update_state_estimation(self, odom: Odometry, imu: Optional[Imu],
                                 timeout_status: TimeoutStatus, actual_dt: float,
                                 current_time: float,
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
            
            self.state_estimator.update_odom(odom, current_time)
            
            if timeout_status.imu_timeout:
                self.state_estimator.set_imu_available(False)
            elif imu:
                self.state_estimator.set_imu_available(True)
                self.state_estimator.update_imu(imu, current_time)
            
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
        # 优化: 仅变换当前 Horizon 所需的轨迹片段，避免全量变换浪费算力
        # MPC 需要 horizon 个点，预留一定的余量 (BUFFER) 以应对求解器可能的超前访问或 Lookahead
        # Buffer set to 10 points
        slice_len = self._current_horizon + 10
        if len(trajectory.points) > slice_len:
             # 高性能切片: 使用 Trajectory.get_slice() 获取切片
             # 这不仅更高效（利用 Numpy View），而且更安全（自动复制所有字段）
             traj_to_transform = trajectory.get_slice(0, slice_len)
             
        else:
             traj_to_transform = trajectory

        if self.coord_transformer:
            # 准备 fallback_state
            fallback_state = None
            if self.state_estimator:
                fallback_state = self.state_estimator.get_state()
            
            transformed_traj, tf_status = self.coord_transformer.transform_trajectory(
                traj_to_transform, self.transform_target_frame, current_time,
                fallback_state=fallback_state)
            tf2_critical = tf_status.is_critical()
        else:
            # 无坐标变换器时，假设轨迹已经在正确的坐标系
            # 这种情况下，轨迹应该已经是世界坐标系
            transformed_traj = traj_to_transform
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
                # mpc_cmd 是从 compute() 返回的新对象，无需 copy
                self._last_mpc_cmd = mpc_cmd
        
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
        """构建诊断信息 (复用对象)"""
        d = self._diagnostics_input
        d.alpha = consistency.alpha
        d.data_valid = consistency.data_valid
        d.mpc_health = mpc_health
        d.mpc_success = mpc_cmd.success if mpc_cmd else False
        d.odom_timeout = timeout_status.odom_timeout
        d.traj_timeout_exceeded = timeout_status.traj_grace_exceeded
        d.v_horizontal = np.sqrt(state[3]**2 + state[4]**2)
        d.vz = state[5]
        d.has_valid_data = len(trajectory.points) > 0
        d.tf2_critical = tf2_critical
        d.safety_failed = self._safety_failed
        d.current_state = self._last_state
        
        self._last_diagnostics = d
        return d
    
    def _select_controller_output(self, state: np.ndarray, trajectory: Trajectory,
                                  consistency: ConsistencyResult,
                                  mpc_cmd: Optional[ControlOutput]) -> ControlOutput:
        """选择控制器输出"""
        # STOPPING 状态：输出零速度，让机器人停下来
        if self._last_state == ControllerState.STOPPING:
            # 直接输出零速度，不再跟踪轨迹
            return ControlOutput(vx=0, vy=0, vz=0, omega=0, frame_id=self.default_frame_id)
        
        # STOPPED 状态：完全停止
        elif self._last_state == ControllerState.STOPPED:
            return ControlOutput(vx=0, vy=0, vz=0, omega=0, frame_id=self.default_frame_id)
        
        # BACKUP_ACTIVE 状态：使用备用控制器
        elif self._last_state == ControllerState.BACKUP_ACTIVE:
            if self.backup_tracker:
                cmd = self.backup_tracker.compute(state, trajectory, consistency)
                self._last_backup_cmd = cmd.copy()
            else:
                cmd = ControlOutput(vx=0, vy=0, vz=0, omega=0, frame_id=self.default_frame_id)
            return cmd
        
        # 其他状态：优先使用 MPC 输出
        else:
            if mpc_cmd and mpc_cmd.success:
                return mpc_cmd
            
            # MPC 失败或未运行：尝试使用备份控制器
            # 这是关键的架构变更：由 Manager 统一处理降级
            if self.backup_tracker:
                if mpc_cmd:  # 如果有失败的 MPC 命令，记录原因
                    logger.warning(f"MPC failed, falling back to backup controller. "
                                 f"Reason: {mpc_cmd.health_metrics.get('error_type', 'unknown')}")
                
                cmd = self.backup_tracker.compute(state, trajectory, consistency)
                self._last_backup_cmd = cmd.copy()
                
                # 标记为 fallback 来源，方便诊断
                if cmd.health_metrics is None:
                    cmd.health_metrics = {}
                cmd.health_metrics['source'] = 'backup_fallback'
                return cmd
            
            # 彻底失败：停车
            logger.error("MPC failed and no backup controller available. Stopping.")
            return ControlOutput(
                vx=0, vy=0, vz=0, omega=0, frame_id=self.default_frame_id, success=False)
    
    def _apply_safety_check(self, state: np.ndarray, cmd: ControlOutput,
                           diagnostics: DiagnosticsInput) -> ControlOutput:
        """应用安全检查"""
        if self.safety_monitor:
            safety_decision = self.safety_monitor.check(state, cmd, diagnostics)
            if not safety_decision.safe:
                self._safety_failed = True
                self._safety_check_passed = False
                if safety_decision.limited_cmd is not None:
                    cmd = safety_decision.limited_cmd
                diagnostics.safety_failed = True
            else:
                self._safety_failed = False
                self._safety_check_passed = True
        else:
            self._safety_check_passed = True
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
                            cmd: ControlOutput, current_time: float) -> None:
        """发布诊断信息"""
        transform_status = self.coord_transformer.get_status() if self.coord_transformer else {
            'fallback_duration_ms': 0.0, 'accumulated_drift': 0.0
        }
        # 检查是否有外部紧急停止请求
        emergency_stop = self.state_machine.is_stop_requested() if self.state_machine else False
        
        self._diagnostics_publisher.publish(
            current_time=current_time,
            state=self._last_state,
            cmd=cmd,
            state_output=state_output,
            consistency=self._last_consistency,
            mpc_health=self._last_mpc_health,
            timeout_status=timeout_status,
            transform_status=transform_status,
            tracking_error=self._last_tracking_error,
            transition_progress=self.smooth_transition.get_progress() if self.smooth_transition else 0.0,
            tf2_critical=tf2_critical,
            safety_check_passed=self._safety_check_passed,
            emergency_stop=emergency_stop,
            tracking_quality=self._last_tracking_quality
        )
    
    def _compute_tracking_error(self, state: np.ndarray, trajectory: Trajectory) -> Dict[str, float]:
        """
        计算跟踪误差
        
        包括:
        - lateral_error: 横向误差绝对值 (垂直于轨迹方向)
        - longitudinal_error: 纵向误差绝对值 (沿轨迹方向)
        - heading_error: 航向误差绝对值
        - prediction_error: 预测误差 (上一次 MPC 预测状态与当前实际状态的差异)
        
        注意:
        - 所有误差均为绝对值，用于诊断和质量评估
        - prediction_error 为 NaN 表示无预测数据（如使用 fallback 求解器时）
        
        边界情况处理:
        - 空轨迹 (0 点): 返回零误差，无法计算有意义的跟踪误差
        - 单点轨迹 (1 点): 计算到该点的距离作为误差，航向误差为指向该点的方向差
        - 正常轨迹 (≥2 点): 使用完整的横向/纵向误差分解
        """
        px, py = state[0], state[1]
        theta = state[6]
        
        # 计算预测误差 (在任何情况下都需要更新)
        prediction_error = float('nan')
        if self._last_mpc_predicted_state is not None:
            pred_px, pred_py = self._last_mpc_predicted_state[0], self._last_mpc_predicted_state[1]
            prediction_error = np.sqrt((px - pred_px)**2 + (py - pred_py)**2)
        
        # 更新预测状态
        if self.mpc_tracker is not None:
            self._last_mpc_predicted_state = self.mpc_tracker.get_predicted_next_state()
        else:
            self._last_mpc_predicted_state = None
        
        # 获取点矩阵 (兼容 numpy 数组和 Point3D 列表)
        points_matrix = trajectory.get_points_matrix()
        num_points = len(points_matrix)
        
        # 空轨迹: 无法计算有意义的误差
        if num_points == 0:
            return {'lateral_error': 0.0, 'longitudinal_error': 0.0, 
                    'heading_error': 0.0, 'prediction_error': prediction_error}
        
        # 单点轨迹: 计算到该点的距离和航向误差
        if num_points == 1:
            target_x, target_y = points_matrix[0, 0], points_matrix[0, 1]
            dx = target_x - px
            dy = target_y - py
            dist = np.sqrt(dx**2 + dy**2)
            
            # 航向误差: 当前航向与指向目标点方向的差值
            if dist > EPSILON:
                target_heading = np.arctan2(dy, dx)
                heading_error = abs(normalize_angle(theta - target_heading))
            else:
                heading_error = 0.0
            
            # 单点轨迹无法区分横向/纵向，将距离作为横向误差
            return {'lateral_error': dist, 'longitudinal_error': 0.0, 
                    'heading_error': heading_error, 'prediction_error': prediction_error}
        
        # 正常轨迹 (≥2 点): 完整的横向/纵向误差分解
        # 向量化计算最近点
        dists = np.sqrt((points_matrix[:, 0] - px)**2 + (points_matrix[:, 1] - py)**2)
        closest_idx = np.argmin(dists)
        min_dist = dists[closest_idx]
        
        # 确定用于计算误差的线段
        if closest_idx < num_points - 1:
            p0_x, p0_y = points_matrix[closest_idx, 0], points_matrix[closest_idx, 1]
            p1_x, p1_y = points_matrix[closest_idx + 1, 0], points_matrix[closest_idx + 1, 1]
        else:
            # closest_idx 是最后一个点，使用前一个线段
            p0_x, p0_y = points_matrix[closest_idx - 1, 0], points_matrix[closest_idx - 1, 1]
            p1_x, p1_y = points_matrix[closest_idx, 0], points_matrix[closest_idx, 1]
        
        dx = p1_x - p0_x
        dy = p1_y - p0_y
        traj_length = np.sqrt(dx**2 + dy**2)
        
        if traj_length < MIN_SEGMENT_LENGTH:
            return {'lateral_error': min_dist, 'longitudinal_error': 0.0,
                    'heading_error': 0.0, 'prediction_error': prediction_error}
        
        tx, ty = dx / traj_length, dy / traj_length
        ex = px - p0_x
        ey = py - p0_y
        
        longitudinal_error = abs(ex * tx + ey * ty)
        lateral_error = abs(-ex * ty + ey * tx)
        
        traj_heading = np.arctan2(dy, dx)
        heading_error = abs(normalize_angle(theta - traj_heading))
        
        return {
            'lateral_error': lateral_error,
            'longitudinal_error': longitudinal_error,
            'heading_error': heading_error,
            'prediction_error': prediction_error
        }
    
    def _compute_tracking_quality(self, tracking_error: Dict[str, float]) -> Dict[str, Any]:
        """
        计算跟踪质量评分
        
        使用 tracking 配置中的阈值和权重计算质量评分。
        
        评分算法:
        1. 对每个误差分量，计算归一化评分: score = max(0, 1 - error/threshold) * 100
        2. 使用配置的权重计算加权平均分
        3. 根据评级阈值确定质量等级
        
        Args:
            tracking_error: 跟踪误差字典，包含 lateral_error, longitudinal_error, 
                           heading_error, prediction_error
        
        Returns:
            质量评估结果字典:
            - scores: 各分量评分 (0-100)
            - overall_score: 加权总分 (0-100)
            - rating: 质量等级 ('excellent', 'good', 'fair', 'poor')
            - thresholds: 使用的阈值配置
        """
        scores = {}
        
        # 计算各分量评分
        for key in ['lateral', 'longitudinal', 'heading']:
            error_key = f'{key}_error'
            if error_key in tracking_error:
                error = tracking_error[error_key]
                threshold = self._tracking_thresholds.get(key, 1.0)
                if threshold > 0:
                    # 归一化评分: 误差为0时100分，误差>=阈值时0分
                    score = max(0.0, 1.0 - error / threshold) * 100
                else:
                    score = 100.0 if error == 0 else 0.0
                scores[key] = score
            else:
                scores[key] = 100.0  # 无数据时默认满分
        
        # 预测误差评分 (可选，不参与加权平均)
        prediction_error = tracking_error.get('prediction_error', float('nan'))
        if np.isfinite(prediction_error):
            threshold = self._tracking_thresholds.get('prediction', 0.5)
            if threshold > 0:
                scores['prediction'] = max(0.0, 1.0 - prediction_error / threshold) * 100
            else:
                scores['prediction'] = 100.0 if prediction_error == 0 else 0.0
        else:
            scores['prediction'] = float('nan')
        
        # 计算加权总分 (只使用 lateral, longitudinal, heading)
        total_weight = 0.0
        weighted_sum = 0.0
        for key in ['lateral', 'longitudinal', 'heading']:
            weight = self._tracking_weights.get(key, 0.0)
            if key in scores:
                weighted_sum += scores[key] * weight
                total_weight += weight
        
        if total_weight > 0:
            overall_score = weighted_sum / total_weight
        else:
            overall_score = 0.0
        
        # 确定质量等级
        if overall_score >= self._tracking_rating.get('excellent', 90):
            rating = 'excellent'
        elif overall_score >= self._tracking_rating.get('good', 70):
            rating = 'good'
        elif overall_score >= self._tracking_rating.get('fair', 50):
            rating = 'fair'
        else:
            rating = 'poor'
        
        return {
            'scores': scores,
            'overall_score': overall_score,
            'rating': rating,
            'thresholds': self._tracking_thresholds.copy(),
            'weights': self._tracking_weights.copy(),
        }
    
    def get_tracking_quality(self) -> Optional[Dict[str, Any]]:
        """获取最新的跟踪质量评估结果"""
        return self._last_tracking_quality
    
    def get_state(self) -> ControllerState:
        return self._last_state
    
    def get_diagnostics(self) -> Dict[str, Any]:
        if self._last_diagnostics is None:
            return {}
        return self._last_diagnostics.to_dict()
    
    def reset(self) -> None:
        """
        重置控制器管理器状态
        
        重置所有组件的内部状态，但保留资源。
        可以继续调用 update() 方法。
        """
        if self.state_estimator: self.state_estimator.reset()
        if self.consistency_checker: self.consistency_checker.reset()
        if self.state_machine: self.state_machine.reset()
        if self.safety_monitor: self.safety_monitor.reset()
        if self.mpc_health_monitor: self.mpc_health_monitor.reset()
        # 重置所有处理器
        for processor in self.processors:
            if hasattr(processor, 'reset'):
                processor.reset()
        if self.smooth_transition: self.smooth_transition.reset()
        if self.coord_transformer: self.coord_transformer.reset()
        # 重置轨迹跟踪器内部状态（不释放资源）
        if self.mpc_tracker: self.mpc_tracker.reset()
        if self.backup_tracker: self.backup_tracker.reset()
        self.timeout_monitor.reset()
        self._last_state = ControllerState.INIT
        self._safety_failed = False
        self._safety_check_passed = True
        self._last_mpc_cmd = None
        self._last_backup_cmd = None
        self._last_tracking_error = None
        self._last_tracking_quality = None
        self._last_consistency = None
        self._last_mpc_health = None
        self._last_attitude_cmd = None
        self._last_update_time = None  # 重置时间跟踪
        # 重置 notify 跟踪状态
        self._current_horizon = self.horizon_normal
        self._last_mpc_predicted_state = None  # 重置预测状态
        if self.mpc_tracker:
            self.mpc_tracker.set_horizon(self.horizon_normal)
    
    def shutdown(self) -> None:
        """
        关闭控制器管理器并释放所有资源
        
        按正确顺序关闭所有组件：
        1. 先关闭依赖其他组件的组件
        2. 再关闭被依赖的组件
        3. 最后清理诊断发布器
        
        调用后管理器不应再使用。
        """
        logger.info("Shutting down ControllerManager...")
        
        # 1. 关闭轨迹跟踪器（可能有外部资源如 ACADOS）
        if self.backup_tracker:
            try:
                self.backup_tracker.shutdown()
            except Exception as e:
                logger.warning(f"Error shutting down backup_tracker: {e}")
            self.backup_tracker = None
        
        if self.mpc_tracker:
            try:
                self.mpc_tracker.shutdown()
            except Exception as e:
                logger.warning(f"Error shutting down mpc_tracker: {e}")
            self.mpc_tracker = None
        
        # 2. 关闭坐标变换器（依赖 state_estimator）
        if self.coord_transformer:
            try:
                self.coord_transformer.shutdown()
            except Exception as e:
                logger.warning(f"Error shutting down coord_transformer: {e}")
            self.coord_transformer = None
        
        # 3. 关闭姿态控制器
        # 3. 关闭附加处理器
        for processor in self.processors:
            try:
                if hasattr(processor, 'shutdown'):
                    processor.shutdown()
            except Exception as e:
                logger.warning(f"Error shutting down processor {processor}: {e}")
        self.processors.clear()
        
        # 4. 关闭状态估计器
        if self.state_estimator:
            try:
                self.state_estimator.shutdown()
            except Exception as e:
                logger.warning(f"Error shutting down state_estimator: {e}")
            self.state_estimator = None
        
        # 5. 关闭其他组件（无外部资源，但调用 shutdown 以保持一致性）
        if self.consistency_checker:
            try:
                self.consistency_checker.shutdown()
            except Exception as e:
                logger.warning(f"Error shutting down consistency_checker: {e}")
            self.consistency_checker = None
        
        if self.safety_monitor:
            try:
                self.safety_monitor.shutdown()
            except Exception as e:
                logger.warning(f"Error shutting down safety_monitor: {e}")
            self.safety_monitor = None
        
        if self.smooth_transition:
            try:
                self.smooth_transition.shutdown()
            except Exception as e:
                logger.warning(f"Error shutting down smooth_transition: {e}")
            self.smooth_transition = None
        
        # 6. 清理非接口组件
        if self.mpc_health_monitor:
            self.mpc_health_monitor.reset()
            self.mpc_health_monitor = None
        
        if self.state_machine:
            self.state_machine.reset()
            self.state_machine = None
        
        # 7. 清理诊断发布器回调
        self._diagnostics_publisher.clear_callbacks()
        
        # 8. 重置内部状态
        self._last_state = ControllerState.STOPPED
        self._last_diagnostics = None
        self._last_mpc_health = None
        self._last_consistency = None
        self._last_mpc_cmd = None
        self._last_backup_cmd = None
        self._last_tracking_error = None
        self._last_tracking_quality = None
        self._last_attitude_cmd = None
        self._last_update_time = None
        
        logger.info("ControllerManager shutdown complete")
    

