#!/usr/bin/env python3
"""
ROS 数据源适配器 - 订阅 ROS 话题获取诊断数据

支持 ROS1 和 ROS2，通过订阅 /controller/diagnostics 话题获取数据，
转换为统一的 DisplayData 模型供 Dashboard 使用。

架构说明：
---------
此模块从 universal_controller/dashboard/ros_data_source.py 迁移而来。
迁移原因：ROS 数据源依赖 controller_ros.msg，应该放在 controller_ros 层。

用法:
    from controller_ros.dashboard import ROSDashboardDataSource
    from universal_controller.dashboard import DashboardWindow
    
    # ROS1
    data_source = ROSDashboardDataSource(config)
    
    # ROS2
    data_source = ROSDashboardDataSource(config, node=my_node)
    
    # 在 Dashboard 中使用
    window = DashboardWindow(data_source)
"""

import time
import math
import threading
from typing import Dict, Any, Optional
from collections import deque

# 导入 Dashboard 数据模型（从 universal_controller）
from universal_controller.dashboard.models import (
    DisplayData, EnvironmentStatus, PlatformConfig, ControllerStatus,
    MPCHealthStatus, ConsistencyStatus, TimeoutStatus, TrackingStatus,
    EstimatorStatus, TransformStatus, ControlCommand, TrajectoryData,
    StatisticsData, SafetyStatus, ControllerStateEnum, DataAvailability
)

# 导入配置（从 universal_controller）
from universal_controller.config import DEFAULT_CONFIG

# 检测 ROS 版本
ROS_VERSION = 0
try:
    import rospy
    ROS_VERSION = 1
except ImportError:
    pass

if ROS_VERSION == 0:
    try:
        import rclpy
        ROS_VERSION = 2
    except ImportError:
        pass

# 尝试检测 ACADOS
try:
    from acados_template import AcadosOcp
    ACADOS_AVAILABLE = True
except ImportError:
    ACADOS_AVAILABLE = False

# 数据过期判断的最小阈值 (毫秒)
# 即使诊断发布周期很短，也至少等待这么长时间才认为数据过期
MIN_DATA_STALE_THRESHOLD_MS = 1000.0

# 平台名称映射
PLATFORM_NAMES = {
    'differential': '差速车',
    'ackermann': '阿克曼',
    'omni': '全向车',
    'quadrotor': '四旋翼',
}

# 状态名称映射
STATE_INFO = {
    -1: ('UNKNOWN', '未知状态'),  # 未初始化或无数据
    0: ('INIT', '初始化'),
    1: ('NORMAL', '正常运行'),
    2: ('SOFT_DISABLED', 'Soft禁用'),
    3: ('MPC_DEGRADED', 'MPC降级'),
    4: ('BACKUP_ACTIVE', '备用激活'),
    5: ('STOPPING', '停车中'),
    6: ('STOPPED', '已停车'),
}


class ROSDashboardDataSource:
    """
    ROS Dashboard 数据源 - 通过订阅 ROS 话题获取数据
    
    与 DashboardDataSource 接口兼容，但数据来源是 ROS 话题而非直接访问 ControllerManager。
    
    订阅话题:
    - /controller/diagnostics (DiagnosticsV2) - 诊断信息
    - /controller/state (Int32) - 控制器状态 (可选，用于更高频率的状态更新)
    
    参数来源:
    - ROS 参数服务器 (ROS1) 或节点参数 (ROS2)
    """

    def __init__(self, config: Dict[str, Any] = None, node=None):
        """
        初始化 ROS 数据源
        
        Args:
            config: 配置字典 (如果为 None，将从 ROS 参数加载)
            node: ROS2 节点实例 (ROS2 必需，ROS1 忽略)
        """
        self._node = node
        self._config = config or {}
        self._start_time = time.time()
        
        # 线程安全锁
        self._lock = threading.Lock()
        
        # 最新的诊断数据
        self._last_diagnostics: Dict[str, Any] = {}
        self._last_diag_time: float = 0.0
        self._diagnostics_received = False

        # 统计数据
        self._total_cycles = 0
        self._state_counts = {i: 0 for i in range(7)}
        self._mpc_success_count = 0
        self._backup_switch_count = 0
        self._safety_limit_count = 0
        self._tf2_fallback_count = 0
        self._soft_disable_count = 0
        
        # 状态变化跟踪（用于统计计数）
        self._last_state: int = 0  # 上一次的控制器状态
        self._last_safety_check_passed: bool = True  # 上一次安全检查是否通过
        self._last_tf2_fallback_active: bool = False  # 上一次 TF2 是否处于降级状态

        # 历史数据
        self._history_size = 500
        self._solve_time_history = deque(maxlen=self._history_size)
        self._lateral_error_history = deque(maxlen=self._history_size)
        self._alpha_history = deque(maxlen=self._history_size)

        # 周期时间统计
        self._cycle_times = deque(maxlen=100)
        self._last_update_time = time.time()
        
        # ROS 环境状态
        self._ros_available = ROS_VERSION > 0
        self._tf2_available = False
        self._tf2_injected = False
        
        # 初始化 ROS 订阅
        self._init_ros_subscriptions()
        
        # 如果没有提供配置，从 ROS 参数加载
        if not self._config:
            self._load_ros_params()

    def _init_ros_subscriptions(self):
        """初始化 ROS 订阅"""
        if ROS_VERSION == 1:
            self._init_ros1_subscriptions()
        elif ROS_VERSION == 2:
            self._init_ros2_subscriptions()
        else:
            print("[ROSDashboardDataSource] Warning: ROS not available, using mock data")

    def _init_ros1_subscriptions(self):
        """初始化 ROS1 订阅"""
        try:
            # 直接导入本包的消息类型（不再是跨层依赖）
            from controller_ros.msg import DiagnosticsV2
            from std_msgs.msg import Int32
            
            # 订阅诊断话题
            self._diag_sub = rospy.Subscriber(
                '/controller/diagnostics',
                DiagnosticsV2,
                self._diagnostics_callback_ros1,
                queue_size=1
            )

            # 订阅状态话题 (更高频率)
            self._state_sub = rospy.Subscriber(
                '/controller/state',
                Int32,
                self._state_callback_ros1,
                queue_size=1
            )
            
            rospy.loginfo("[ROSDashboardDataSource] Subscribed to /controller/diagnostics and /controller/state")
            rospy.loginfo("[ROSDashboardDataSource] Waiting for diagnostics data...")
            
        except ImportError as e:
            rospy.logwarn(f"[ROSDashboardDataSource] Failed to import ROS messages: {e}")
            rospy.logwarn("[ROSDashboardDataSource] Make sure controller_ros package is built and sourced")
        except Exception as e:
            rospy.logerr(f"[ROSDashboardDataSource] Failed to create subscriptions: {e}")

    def _init_ros2_subscriptions(self):
        """初始化 ROS2 订阅"""
        if self._node is None:
            print("[ROSDashboardDataSource] Error: ROS2 node not provided")
            return
            
        try:
            # 直接导入本包的消息类型
            from controller_ros.msg import DiagnosticsV2
            from std_msgs.msg import Int32
            
            # 订阅诊断话题
            self._diag_sub = self._node.create_subscription(
                DiagnosticsV2,
                '/controller/diagnostics',
                self._diagnostics_callback_ros2,
                10
            )
            
            # 订阅状态话题
            self._state_sub = self._node.create_subscription(
                Int32,
                '/controller/state',
                self._state_callback_ros2,
                10
            )
            
            self._node.get_logger().info("[ROSDashboardDataSource] Subscribed to diagnostics topics")
            
        except ImportError as e:
            self._node.get_logger().warn(f"[ROSDashboardDataSource] Failed to import ROS messages: {e}")
        except Exception as e:
            self._node.get_logger().error(f"[ROSDashboardDataSource] Failed to create subscriptions: {e}")

    def _diagnostics_callback_ros1(self, msg):
        """ROS1 诊断消息回调"""
        with self._lock:
            self._last_diagnostics = self._convert_diagnostics_msg(msg)
            self._last_diag_time = time.time()
            self._diagnostics_received = True
            
            # 更新 TF2 状态
            self._tf2_available = msg.transform_tf2_available
            self._tf2_injected = msg.transform_tf2_injected

    def _diagnostics_callback_ros2(self, msg):
        """ROS2 诊断消息回调"""
        with self._lock:
            self._last_diagnostics = self._convert_diagnostics_msg(msg)
            self._last_diag_time = time.time()
            self._diagnostics_received = True
            
            # 更新 TF2 状态
            self._tf2_available = msg.transform_tf2_available
            self._tf2_injected = msg.transform_tf2_injected

    def _state_callback_ros1(self, msg):
        """ROS1 状态消息回调"""
        with self._lock:
            if 'state' not in self._last_diagnostics:
                self._last_diagnostics['state'] = msg.data

    def _state_callback_ros2(self, msg):
        """ROS2 状态消息回调"""
        with self._lock:
            if 'state' not in self._last_diagnostics:
                self._last_diagnostics['state'] = msg.data

    def _convert_diagnostics_msg(self, msg) -> Dict[str, Any]:
        """将 DiagnosticsV2 消息转换为字典格式"""
        return {
            'state': msg.state,
            'mpc_success': msg.mpc_success,
            'mpc_solve_time_ms': msg.mpc_solve_time_ms,
            'backup_active': msg.backup_active,
            'mpc_health': {
                'kkt_residual': msg.mpc_health_kkt_residual,
                'condition_number': msg.mpc_health_condition_number,
                'consecutive_near_timeout': msg.mpc_health_consecutive_near_timeout,
                'degradation_warning': msg.mpc_health_degradation_warning,
                'can_recover': msg.mpc_health_can_recover,
            },
            'consistency': {
                'curvature': msg.consistency_curvature,
                'velocity_dir': msg.consistency_velocity_dir,
                'temporal': msg.consistency_temporal,
                'alpha_soft': msg.consistency_alpha_soft,
                'data_valid': msg.consistency_data_valid,
            },
            'estimator_health': {
                'covariance_norm': msg.estimator_covariance_norm,
                'innovation_norm': msg.estimator_innovation_norm,
                'slip_probability': msg.estimator_slip_probability,
                'imu_drift_detected': msg.estimator_imu_drift_detected,
                'imu_bias': list(msg.estimator_imu_bias),
                'imu_available': msg.estimator_imu_available,
            },
            'tracking': {
                'lateral_error': msg.tracking_lateral_error,
                'longitudinal_error': msg.tracking_longitudinal_error,
                'heading_error': msg.tracking_heading_error,
                'prediction_error': msg.tracking_prediction_error,
            },
            'transform': {
                'tf2_available': msg.transform_tf2_available,
                'tf2_injected': msg.transform_tf2_injected,
                'fallback_duration_ms': msg.transform_fallback_duration_ms,
                'accumulated_drift': msg.transform_accumulated_drift,
            },
            'timeout': {
                'odom_timeout': msg.timeout_odom,
                'traj_timeout': msg.timeout_traj,
                'traj_grace_exceeded': msg.timeout_traj_grace_exceeded,
                'imu_timeout': msg.timeout_imu,
                'last_odom_age_ms': msg.timeout_last_odom_age_ms,
                'last_traj_age_ms': msg.timeout_last_traj_age_ms,
                'last_imu_age_ms': msg.timeout_last_imu_age_ms,
                'in_startup_grace': msg.timeout_in_startup_grace,
            },
            'cmd': {
                'vx': msg.cmd_vx,
                'vy': msg.cmd_vy,
                'vz': msg.cmd_vz,
                'omega': msg.cmd_omega,
                'frame_id': msg.cmd_frame_id,
            },
            'transition_progress': msg.transition_progress,
            'error_message': msg.error_message,
            'consecutive_errors': msg.consecutive_errors,
            'safety_check_passed': msg.safety_check_passed,
            'emergency_stop': msg.emergency_stop,
        }

    def _load_ros_params(self):
        """从 ROS 参数服务器加载配置"""
        if ROS_VERSION == 1:
            self._load_ros1_params()
        elif ROS_VERSION == 2:
            self._load_ros2_params()

    def _load_ros1_params(self):
        """从 ROS1 参数服务器加载配置"""
        try:
            import copy
            
            # 深拷贝默认配置
            self._config = copy.deepcopy(DEFAULT_CONFIG)
            
            # Dashboard 需要的配置路径列表
            dashboard_params = [
                'system/platform', 'system/ctrl_freq',
                'mpc/horizon', 'mpc/horizon_degraded', 'mpc/dt',
                'constraints/v_max', 'constraints/v_min', 'constraints/omega_max',
                'constraints/a_max', 'constraints/az_max', 'constraints/alpha_max',
                'consistency/kappa_thresh', 'consistency/v_dir_thresh',
                'consistency/temporal_smooth_thresh', 'consistency/alpha_min',
                'watchdog/odom_timeout_ms', 'watchdog/traj_timeout_ms',
                'watchdog/traj_grace_ms', 'watchdog/imu_timeout_ms',
                'diagnostics/publish_rate',
                'ekf/use_odom_orientation_fallback', 'ekf/imu_motion_compensation',
                'transform/recovery_correction_enabled',
                'tracking/lateral_thresh', 'tracking/longitudinal_thresh',
                'safety/v_stop_thresh', 'safety/stopping_timeout',
            ]
            
            def set_nested(d, path, value):
                keys = path.split('/')
                for key in keys[:-1]:
                    if key not in d:
                        d[key] = {}
                    d = d[key]
                d[keys[-1]] = value
            
            def get_nested(d, path, default=None):
                keys = path.split('/')
                for key in keys:
                    if not isinstance(d, dict) or key not in d:
                        return default
                    d = d[key]
                return d
            
            for param_path in dashboard_params:
                default_value = get_nested(self._config, param_path)
                ros_value = rospy.get_param(param_path, default_value)
                set_nested(self._config, param_path, ros_value)
            
            # transform 配置（坐标系名称统一从 transform 读取）
            self._config.setdefault('transform', {})
            self._config['transform']['target_frame'] = rospy.get_param('transform/target_frame', 'odom')
            self._config['transform']['source_frame'] = rospy.get_param('transform/source_frame', 'base_link')
            
            rospy.loginfo(f"[ROSDashboardDataSource] Loaded config: platform={self._config['system']['platform']}")
        except Exception as e:
            rospy.logwarn(f"[ROSDashboardDataSource] Failed to load ROS params: {e}")

    def _load_ros2_params(self):
        """从 ROS2 参数加载配置"""
        if self._node is None:
            return
        try:
            import copy
            self._config = copy.deepcopy(DEFAULT_CONFIG)
            self._node.get_logger().info("[ROSDashboardDataSource] Using default config for ROS2")
        except Exception as e:
            self._node.get_logger().warn(f"[ROSDashboardDataSource] Failed to load params: {e}")

    def get_display_data(self) -> DisplayData:
        """
        获取统一的显示数据
        
        这是面板获取数据的唯一入口，与 DashboardDataSource 接口兼容。
        """
        with self._lock:
            raw_diagnostics = self._last_diagnostics.copy()
            diag_time = self._last_diag_time
        
        self._update_statistics(raw_diagnostics)

        data = DisplayData()
        data.availability = self._build_availability(raw_diagnostics, diag_time)
        data.environment = self._build_environment_status()
        data.platform = self._build_platform_config()
        data.controller = self._build_controller_status(raw_diagnostics)
        data.mpc_health = self._build_mpc_health(raw_diagnostics)
        data.consistency = self._build_consistency(raw_diagnostics)
        data.timeout = self._build_timeout_status(raw_diagnostics)
        data.tracking = self._build_tracking_status(raw_diagnostics)
        data.estimator = self._build_estimator_status(raw_diagnostics)
        data.transform = self._build_transform_status(raw_diagnostics)
        data.safety = self._build_safety_status(raw_diagnostics)
        data.command = self._build_control_command(raw_diagnostics)
        data.trajectory = TrajectoryData()  # ROS 模式暂不支持轨迹可视化
        data.statistics = self._build_statistics()
        data.version = 'v3.17.12'
        data.transition_progress = raw_diagnostics.get('transition_progress', 1.0)

        return data

    def _build_availability(self, diagnostics: Dict[str, Any], diag_time: float) -> DataAvailability:
        """构建数据可用性状态"""
        has_diag = bool(diagnostics) and self._diagnostics_received
        current_time = time.time()
        data_age_ms = (current_time - diag_time) * 1000 if diag_time > 0 else float('inf')
        
        diag_publish_rate = self._config.get('diagnostics', {}).get('publish_rate', 5)
        ctrl_freq = self._config.get('system', {}).get('ctrl_freq', 20)
        diag_period_ms = (1000.0 / ctrl_freq) * diag_publish_rate
        data_stale_threshold_ms = max(diag_period_ms * 3, MIN_DATA_STALE_THRESHOLD_MS)
        data_stale = data_age_ms > data_stale_threshold_ms
        
        mpc_health = diagnostics.get('mpc_health', {})
        consistency = diagnostics.get('consistency', {})
        tracking = diagnostics.get('tracking', {})
        estimator = diagnostics.get('estimator_health', {})
        transform = diagnostics.get('transform', {})
        timeout = diagnostics.get('timeout', {})
        
        return DataAvailability(
            diagnostics_available=has_diag and not data_stale,
            trajectory_available=False,
            position_available=False,
            odom_available=has_diag and not data_stale and not timeout.get('odom_timeout', False),
            imu_data_available=has_diag and not data_stale and estimator.get('imu_available', False),
            mpc_data_available=has_diag and not data_stale and isinstance(mpc_health, dict) and bool(mpc_health),
            consistency_data_available=has_diag and not data_stale and isinstance(consistency, dict) and bool(consistency),
            tracking_data_available=has_diag and not data_stale and isinstance(tracking, dict) and bool(tracking),
            estimator_data_available=has_diag and not data_stale and isinstance(estimator, dict) and bool(estimator),
            transform_data_available=has_diag and not data_stale and isinstance(transform, dict) and bool(transform),
            last_update_time=diag_time,
            data_age_ms=data_age_ms,
        )

    def _update_statistics(self, diagnostics: Dict[str, Any]):
        """更新统计数据
        
        统计计数器说明:
        - backup_switch_count: 切换到 BACKUP_ACTIVE 状态的次数
        - safety_limit_count: 安全检查失败的次数（从通过变为失败）
        - tf2_fallback_count: TF2 降级激活的次数（从正常变为降级）
        - soft_disable_count: 切换到 SOFT_DISABLED 状态的次数
        """
        current_time = time.time()
        cycle_time = current_time - self._last_update_time
        self._cycle_times.append(cycle_time * 1000)
        self._last_update_time = current_time
        self._total_cycles += 1

        if diagnostics:
            state = diagnostics.get('state', 0)
            if 0 <= state < 7:
                self._state_counts[state] += 1
            
            # 状态变化检测：backup_switch_count
            # 当状态从非 BACKUP_ACTIVE 变为 BACKUP_ACTIVE (state=4) 时计数
            if state == 4 and self._last_state != 4:
                self._backup_switch_count += 1
            
            # 状态变化检测：soft_disable_count
            # 当状态从非 SOFT_DISABLED 变为 SOFT_DISABLED (state=2) 时计数
            if state == 2 and self._last_state != 2:
                self._soft_disable_count += 1
            
            # 更新上一次状态
            self._last_state = state
            
            # 安全检查失败计数
            # 当安全检查从通过变为失败时计数
            safety_check_passed = diagnostics.get('safety_check_passed', True)
            if not safety_check_passed and self._last_safety_check_passed:
                self._safety_limit_count += 1
            self._last_safety_check_passed = safety_check_passed
            
            # TF2 降级计数
            # 当 TF2 从正常变为降级时计数
            transform = diagnostics.get('transform', {})
            if isinstance(transform, dict):
                fallback_ms = transform.get('fallback_duration_ms', 0)
                tf2_fallback_active = fallback_ms > 0
                if tf2_fallback_active and not self._last_tf2_fallback_active:
                    self._tf2_fallback_count += 1
                self._last_tf2_fallback_active = tf2_fallback_active
            
            if diagnostics.get('mpc_success', False):
                self._mpc_success_count += 1
            self._solve_time_history.append(diagnostics.get('mpc_solve_time_ms', 0))
            tracking = diagnostics.get('tracking', {})
            if isinstance(tracking, dict):
                self._lateral_error_history.append(tracking.get('lateral_error', 0))
            consistency = diagnostics.get('consistency', {})
            if isinstance(consistency, dict):
                self._alpha_history.append(consistency.get('alpha_soft', 0))

    def _build_environment_status(self) -> EnvironmentStatus:
        """构建环境状态"""
        with self._lock:
            estimator = self._last_diagnostics.get('estimator_health', {})
            imu_available = estimator.get('imu_available', False) if isinstance(estimator, dict) else False
        
        return EnvironmentStatus(
            ros_available=self._ros_available,
            tf2_available=self._tf2_available,
            acados_available=ACADOS_AVAILABLE,
            imu_available=imu_available,
            is_mock_mode=False,
        )

    def _build_platform_config(self) -> PlatformConfig:
        """构建平台配置"""
        platform = self._config.get('system', {}).get('platform', 'differential')
        return PlatformConfig(
            platform=platform,
            platform_display=PLATFORM_NAMES.get(platform, platform),
            ctrl_freq=self._config.get('system', {}).get('ctrl_freq', 50),
            mpc_horizon=self._config.get('mpc', {}).get('horizon', 20),
            mpc_horizon_degraded=self._config.get('mpc', {}).get('horizon_degraded', 10),
            mpc_dt=self._config.get('mpc', {}).get('dt', 0.1),  # 默认值与 mpc_config.py 一致
        )

    def _build_controller_status(self, diag: Dict) -> ControllerStatus:
        """构建控制器状态"""
        diagnostics_received = getattr(self, '_diagnostics_received', False)
        if not diag or not diagnostics_received:
            state = -1
        else:
            state = diag.get('state', -1)
        
        state_name, state_desc = STATE_INFO.get(state, ('UNKNOWN', f'未知({state})'))
        consistency = diag.get('consistency', {})
        alpha = consistency.get('alpha_soft', 0) if isinstance(consistency, dict) else 0

        try:
            state_enum = ControllerStateEnum(state) if 0 <= state <= 6 else ControllerStateEnum.INIT
        except ValueError:
            state_enum = ControllerStateEnum.INIT

        return ControllerStatus(
            state=state_enum,
            state_name=state_name,
            state_desc=state_desc,
            mpc_success=diag.get('mpc_success', False),
            backup_active=diag.get('backup_active', False),
            current_controller='Backup' if diag.get('backup_active', False) else 'MPC',
            soft_head_enabled=alpha > 0.1,
            alpha_soft=alpha,
        )

    def _build_mpc_health(self, diag: Dict) -> MPCHealthStatus:
        """构建 MPC 健康状态"""
        health = diag.get('mpc_health', {})
        if not isinstance(health, dict):
            health = {}
        return MPCHealthStatus(
            kkt_residual=health.get('kkt_residual', 0),
            condition_number=health.get('condition_number', 0),
            solve_time_ms=diag.get('mpc_solve_time_ms', 0),
            consecutive_near_timeout=health.get('consecutive_near_timeout', 0),
            degradation_warning=health.get('degradation_warning', False),
            can_recover=health.get('can_recover', True),
            healthy=diag.get('mpc_success', False) and not health.get('degradation_warning', False),
        )

    def _build_consistency(self, diag: Dict) -> ConsistencyStatus:
        """构建一致性状态"""
        cons = diag.get('consistency', {})
        if not isinstance(cons, dict):
            cons = {}
        return ConsistencyStatus(
            curvature=cons.get('curvature', 0),
            velocity_dir=cons.get('velocity_dir', 0),
            temporal=cons.get('temporal', 0),
            alpha_soft=cons.get('alpha_soft', 0),
            data_valid=cons.get('data_valid', True),
        )

    def _build_timeout_status(self, diag: Dict) -> TimeoutStatus:
        """构建超时状态"""
        timeout = diag.get('timeout', {})
        if not isinstance(timeout, dict):
            timeout = {}
        return TimeoutStatus(
            odom_timeout=timeout.get('odom_timeout', False),
            traj_timeout=timeout.get('traj_timeout', False),
            traj_grace_exceeded=timeout.get('traj_grace_exceeded', False),
            imu_timeout=timeout.get('imu_timeout', False),
            last_odom_age_ms=timeout.get('last_odom_age_ms', 0),
            last_traj_age_ms=timeout.get('last_traj_age_ms', 0),
            last_imu_age_ms=timeout.get('last_imu_age_ms', 0),
            in_startup_grace=timeout.get('in_startup_grace', False),
        )

    def _build_tracking_status(self, diag: Dict) -> TrackingStatus:
        """构建跟踪状态
        
        注意: prediction_error 可能为 NaN（表示无预测数据，如使用 fallback 求解器时）
        Dashboard 显示时应检查 NaN 并显示 "N/A" 或类似提示
        """
        tracking = diag.get('tracking', {})
        if not isinstance(tracking, dict):
            tracking = {}
        
        # 获取预测误差，保留 NaN 值（表示无数据）
        prediction_error = tracking.get('prediction_error', float('nan'))
        
        return TrackingStatus(
            lateral_error=tracking.get('lateral_error', 0),
            longitudinal_error=tracking.get('longitudinal_error', 0),
            heading_error=tracking.get('heading_error', 0),
            prediction_error=prediction_error,
        )

    def _build_estimator_status(self, diag: Dict) -> EstimatorStatus:
        """构建状态估计器状态
        
        功能开关说明 (与 DashboardDataSource 保持一致):
        - ekf_enabled: EKF 始终启用（核心组件）
        - slip_detection_enabled: 打滑检测是否启用，基于 adaptive.base_slip_thresh > 0
        - drift_correction_enabled: 漂移校正是否启用，从 transform 配置读取
        - heading_fallback_enabled: 航向备选是否启用，从 ekf 配置读取
        """
        est = diag.get('estimator_health', {})
        if not isinstance(est, dict):
            est = {}
        bias = est.get('imu_bias', [0, 0, 0])
        if not isinstance(bias, (list, tuple)) or len(bias) < 3:
            bias = [0, 0, 0]

        ekf_config = self._config.get('ekf', {})
        transform_config = self._config.get('transform', {})
        adaptive_config = ekf_config.get('adaptive', {})
        
        # EKF 始终启用（核心组件）
        ekf_enabled = True
        
        # 打滑检测：检查打滑检测阈值是否配置且大于 0
        slip_detection_enabled = adaptive_config.get('base_slip_thresh', 0) > 0
        
        # 漂移校正：从 transform 配置读取
        drift_correction_enabled = transform_config.get('recovery_correction_enabled', True)
        
        # 航向备选：从 ekf 配置读取
        heading_fallback_enabled = ekf_config.get('use_odom_orientation_fallback', True)
        
        return EstimatorStatus(
            covariance_norm=est.get('covariance_norm', 0),
            innovation_norm=est.get('innovation_norm', 0),
            slip_probability=est.get('slip_probability', 0),
            imu_drift_detected=est.get('imu_drift_detected', False),
            imu_bias=(bias[0], bias[1], bias[2]),
            imu_available=est.get('imu_available', False),
            ekf_enabled=ekf_enabled,
            slip_detection_enabled=slip_detection_enabled,
            drift_correction_enabled=drift_correction_enabled,
            heading_fallback_enabled=heading_fallback_enabled,
        )

    def _build_transform_status(self, diag: Dict) -> TransformStatus:
        """构建坐标变换状态"""
        transform = diag.get('transform', {})
        if not isinstance(transform, dict):
            transform = {}
        fallback_ms = transform.get('fallback_duration_ms', 0)
        transform_config = self._config.get('transform', {})
        
        return TransformStatus(
            tf2_available=self._tf2_available,
            fallback_active=fallback_ms > 0 or not self._tf2_injected,
            fallback_duration_ms=fallback_ms,
            accumulated_drift=transform.get('accumulated_drift', 0),
            target_frame=transform_config.get('target_frame', 'odom'),
            output_frame=transform_config.get('source_frame', 'base_link'),
        )

    def _build_safety_status(self, diag: Dict) -> SafetyStatus:
        """构建安全状态"""
        cmd = diag.get('cmd', {})
        if not isinstance(cmd, dict):
            cmd = {}
        vx = cmd.get('vx', 0)
        vy = cmd.get('vy', 0)
        current_v = math.sqrt(vx ** 2 + vy ** 2)

        return SafetyStatus(
            v_max=self._config.get('constraints', {}).get('v_max', 2.0),
            omega_max=self._config.get('constraints', {}).get('omega_max', 2.0),
            a_max=self._config.get('constraints', {}).get('a_max', 1.5),
            current_v=current_v,
            current_omega=abs(cmd.get('omega', 0)),
            low_speed_protection_active=current_v < 0.1,
            safety_check_passed=diag.get('safety_check_passed', True),
            emergency_stop=diag.get('emergency_stop', False),
        )

    def _build_control_command(self, diag: Dict) -> ControlCommand:
        """构建控制命令"""
        cmd = diag.get('cmd', {})
        if not isinstance(cmd, dict):
            cmd = {}
        return ControlCommand(
            vx=cmd.get('vx', 0),
            vy=cmd.get('vy', 0),
            vz=cmd.get('vz', 0),
            omega=cmd.get('omega', 0),
            frame_id=cmd.get('frame_id', 'base_link'),
        )

    def _build_statistics(self) -> StatisticsData:
        """构建统计数据"""
        elapsed = time.time() - self._start_time
        hours = int(elapsed // 3600)
        minutes = int((elapsed % 3600) // 60)
        seconds = int(elapsed % 60)
        avg_cycle = sum(self._cycle_times) / len(self._cycle_times) if self._cycle_times else 0
        actual_freq = 1000 / avg_cycle if avg_cycle > 0 else 0

        return StatisticsData(
            elapsed_time=elapsed,
            elapsed_time_str=f'{hours:02d}:{minutes:02d}:{seconds:02d}',
            total_cycles=self._total_cycles,
            actual_freq=actual_freq,
            avg_cycle_ms=avg_cycle,
            max_cycle_ms=max(self._cycle_times) if self._cycle_times else 0,
            min_cycle_ms=min(self._cycle_times) if self._cycle_times else 0,
            mpc_success_rate=(self._mpc_success_count / self._total_cycles * 100
                             if self._total_cycles > 0 else 0),
            state_counts=self._state_counts.copy(),
            backup_switch_count=self._backup_switch_count,
            safety_limit_count=self._safety_limit_count,
            tf2_fallback_count=self._tf2_fallback_count,
            soft_disable_count=self._soft_disable_count,
        )

    def get_history(self) -> Dict[str, Any]:
        """获取历史数据 (用于曲线图)"""
        return {
            'solve_time': list(self._solve_time_history),
            'lateral_error': list(self._lateral_error_history),
            'alpha': list(self._alpha_history),
        }

    def is_connected(self) -> bool:
        """检查是否已连接到控制器"""
        return self._diagnostics_received

    def get_last_update_age(self) -> float:
        """获取最后一次更新的时间间隔（秒）"""
        if self._last_diag_time == 0:
            return float('inf')
        return time.time() - self._last_diag_time
