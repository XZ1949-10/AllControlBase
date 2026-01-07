"""
控制器节点基类

封装 ROS1 和 ROS2 节点的共享逻辑，避免代码重复。

功能:
- 控制器初始化和生命周期管理
- 数据管理和控制循环
- TF2 集成
- 紧急停止处理
- 姿态控制接口 (四旋翼平台)

生命周期说明：
- 实现 ILifecycleComponent 接口（通过 LifecycleMixin）
- initialize(): 初始化所有组件（由 _initialize() 调用）
- reset(): 重置控制器和数据管理器
- shutdown(): 关闭所有组件，释放资源
"""
from abc import ABC, abstractmethod
from typing import Dict, Any, Optional, Callable
import logging
import time
import threading

import numpy as np

from universal_controller.config.default_config import DEFAULT_CONFIG, PLATFORM_CONFIG
from universal_controller.core.data_types import (
    Odometry, Imu, Trajectory, ControlOutput, AttitudeCommand, TimeoutStatus
)
from universal_controller.core.enums import ControllerState, PlatformType

from ..bridge import ControllerBridge
from ..io.data_manager import DataManager
from ..utils import ErrorHandler, TF2InjectionManager
from ..lifecycle import LifecycleMixin, LifecycleState, HealthChecker

logger = logging.getLogger(__name__)


class ControllerNodeBase(LifecycleMixin, ABC):
    """
    Controller Node Base
    """
    
    def __init__(self):
        """Init"""
        LifecycleMixin.__init__(self)
        
        self._shutting_down = threading.Event()
        self._control_lock = threading.RLock()
        
        self._params: Dict[str, Any] = {}
        self._topics: Dict[str, str] = {}
        self._default_frame_id: str = 'base_link'
        self._platform_type: str = 'differential'
        
        self._controller_bridge: Optional[ControllerBridge] = None
        self._data_manager: Optional[DataManager] = None
        self._tf_bridge: Any = None  # TFBridge, varies by ROS version
        
        # TF2 注入管理器 - 管理 TF2 回调注入和重试逻辑
        self._tf2_injection_manager: Optional[TF2InjectionManager] = None
        
        self._health_checker: Optional[HealthChecker] = None
        
        # ROS 版本特定的管理器和订阅器
        # 类型为 Any 因为 ROS1 和 ROS2 有不同的实现类
        self._publishers: Optional[Any] = None   # PublisherManager (ROS2) 或 ROS1PublisherManager
        self._services: Optional[Any] = None     # ServiceManager (ROS2) 或 ROS1ServiceManager
        self._timer: Optional[Any] = None        # rclpy.Timer (ROS2) 或 rospy.Timer (ROS1)
        self._control_timer: Optional[Any] = None
        self._odom_sub: Optional[Any] = None
        self._imu_sub: Optional[Any] = None
        self._traj_sub: Optional[Any] = None
        self._traj_msg_available: bool = False
        self._emergency_stop_sub: Optional[Any] = None
        
        self._waiting_for_data: bool = True
        self._error_handler: Optional[ErrorHandler] = None
        
        self._emergency_stop_requested: bool = False
        self._emergency_stop_time: Optional[float] = None
        
        self._is_quadrotor: bool = False
        self._last_attitude_cmd: Optional[AttitudeCommand] = None
        self._attitude_yaw_mode: int = 0

    def _initialize(self):
        """Initialize controller components"""
        if not LifecycleMixin.initialize(self):
            self._log_error("Controller node initialization failed")
    
    # ==================== LifecycleMixin Implementation ====================
    
    def _do_initialize(self) -> bool:
        self._platform_type = self._params.get('system', {}).get('platform', 'differential')
        platform_config = PLATFORM_CONFIG.get(self._platform_type, PLATFORM_CONFIG['differential'])
        self._default_frame_id = platform_config.get('output_frame', 'base_link')
        
        self._is_quadrotor = platform_config.get('type') == PlatformType.QUADROTOR
        
        diag_config = self._params.get('diagnostics', {})
        self._error_handler = ErrorHandler(
            log_error_func=self._log_error,
            log_warn_func=self._log_warn,
            config=diag_config
        )
        
        trajectory_config = self._params.get('trajectory', {})
        clock_config = self._params.get('clock', {})
        self._data_manager = DataManager(
            get_time_func=self._get_time,
            on_clock_jump=self._on_clock_jump,
            trajectory_config=trajectory_config,
            clock_config=clock_config
        )
        
        self._controller_bridge = ControllerBridge.create(self._params)
        if self._controller_bridge is None:
            raise RuntimeError("Failed to create ControllerBridge")
        
        self._controller_bridge.set_diagnostics_callback(self._on_diagnostics)
        
        self._health_checker = HealthChecker()
        self._health_checker.register('controller_bridge', self._controller_bridge)
        self._health_checker.register('data_manager', self._data_manager)
        
        manager = self._controller_bridge.manager
        if manager is not None:
            if manager.state_estimator is not None:
                self._health_checker.register('state_estimator', manager.state_estimator)
            if manager.mpc_tracker is not None:
                self._health_checker.register('mpc_tracker', manager.mpc_tracker)
            if manager.backup_tracker is not None:
                self._health_checker.register('backup_tracker', manager.backup_tracker)
            if manager.safety_monitor is not None:
                self._health_checker.register('safety_monitor', manager.safety_monitor)
        
        return True
    
    def _do_shutdown(self) -> None:
        if self._controller_bridge is not None:
            self._controller_bridge.shutdown()
            self._controller_bridge = None
        
        if self._data_manager is not None:
            self._data_manager.shutdown()
            self._data_manager = None
        
        if self._health_checker is not None:
            self._health_checker = None
        
        # TF2InjectionManager 不需要显式 shutdown，只需清除引用
        self._tf2_injection_manager = None
        
        self._log_info('Controller node shutdown complete')
    
    def _do_reset(self) -> None:
        with self._control_lock:
            if self._controller_bridge is not None:
                self._controller_bridge.reset()
            if self._data_manager is not None:
                self._data_manager.reset()
            if self._error_handler is not None:
                self._error_handler.reset()
            
            # Reset TF2 injection manager retry state
            # Note: We do not need to re-inject TF2 callback here as the manager instance persists.
            # Only full re-initialization requires re-binding.
            # The injection state is preserved because the callback is still bound in Manager.
            if self._tf2_injection_manager is not None:
                self._tf2_injection_manager.reset()
            
            self._waiting_for_data = True
            # Note: Do NOT clear emergency stop state here for safety
            self._last_attitude_cmd = None
            
        self._log_info('Controller node reset complete (emergency stop state preserved)')
    
    def _get_health_details(self) -> Dict[str, Any]:
        details = {
            'platform': self._platform_type,
            'is_quadrotor': self._is_quadrotor,
            'waiting_for_data': self._waiting_for_data,
            'consecutive_errors': self._error_handler.consecutive_errors if self._error_handler else 0,
            'emergency_stop': self._emergency_stop_requested,
            'traj_msg_available': self._traj_msg_available,
        }
        
        if self._controller_bridge is not None:
            details['controller_bridge'] = self._controller_bridge.get_health_status()
        
        if self._data_manager is not None:
            details['data_manager'] = self._data_manager.get_health_status()
        
        # TF2 注入状态
        details['tf2_injected'] = self._is_tf2_injected
        if self._tf2_injection_manager is not None:
            details['tf2_injection'] = self._tf2_injection_manager.get_status()
        
        if self._health_checker is not None:
            details['health_summary'] = self._health_checker.get_summary()
        
        return details
    
    def _on_clock_jump(self, event) -> None:
        if event.is_backward:
            self._log_warn(
                f"Clock jumped backward by {abs(event.jump_delta):.3f}s. "
                f"Waiting for fresh sensor data before resuming control."
            )
            self._waiting_for_data = True
    
    def _inject_tf2_to_controller(self, blocking: bool = True):
        """
        Initial TF2 injection.
        
        创建 TF2InjectionManager（如果尚未创建）并执行注入。
        
        Args:
            blocking: 是否阻塞等待 TF buffer 预热
        """
        self._ensure_tf2_injection_manager()
        if self._tf2_injection_manager is not None:
            self._tf2_injection_manager.inject(blocking=blocking)
    
    def _try_tf2_reinjection(self):
        """
        Runtime reinjection attempt.
        
        在控制循环中调用，检查是否需要重试注入。
        使用指数退避策略避免频繁重试。
        """
        if self._tf2_injection_manager is not None:
            self._tf2_injection_manager.try_reinjection_if_needed()
    
    def _ensure_tf2_injection_manager(self):
        """
        确保 TF2InjectionManager 已创建
        
        延迟创建，因为在 __init__ 时 _tf_bridge 和 _controller_bridge 可能尚未设置。
        """
        if self._tf2_injection_manager is not None:
            return
        
        if self._tf_bridge is None or self._controller_bridge is None:
            return
        
        if self._controller_bridge.manager is None:
            return
        
        # 从配置中获取 transform 相关参数
        transform_config = self._params.get('transform', {})
        
        self._tf2_injection_manager = TF2InjectionManager(
            tf_bridge=self._tf_bridge,
            controller_manager=self._controller_bridge.manager,
            config=transform_config,
            transform_config=transform_config,
            log_info=self._log_info,
            log_warn=self._log_warn,
            get_time_func=self._get_time,
        )
    
    @property
    def _is_tf2_injected(self) -> bool:
        """
        检查 TF2 是否已注入
        
        提供向后兼容的属性访问。
        """
        if self._tf2_injection_manager is not None:
            return self._tf2_injection_manager.is_injected
        return False
    
    # ==================== 紧急停止处理 ====================
    
    def _handle_emergency_stop(self):
        """
        处理紧急停止请求
        
        当收到紧急停止信号时调用此方法。
        设计目标：非阻塞，最高优先级。
        
        策略:
        1. 设置原子标志位 _emergency_stop_requested
        2. 立即发送停止命令 (不锁)
        3. 尝试通知 bridge 切换状态 (尝试获取锁，失败则异步或者由主循环处理)
        """
        if not self._emergency_stop_requested:
            self._emergency_stop_requested = True
            self._emergency_stop_time = self._get_time()
            self._log_warn("Emergency stop requested! Sending immediate stop command.")
            
            # 立即发送停止命令 (不等待下一个控制周期, 且不使用任何锁)
            try:
                self._publish_stop_cmd()
            except Exception as e:
                self._log_error(f"Failed to publish emergency stop command: {e}")
            
            # 尝试请求控制器进入停止状态
            # 使用 non-blocking lock acquire。如果主循环正在忙 (holding lock)，
            # 我们不需要死等，因为主循环会在下一次迭代或者当前迭代结束时看到 _emergency_stop_requested 标志。
            # 如果能立即拿到锁，就顺手就把状态切换了。
            if self._control_lock.acquire(blocking=False):
                try:
                    if self._controller_bridge is not None:
                        self._controller_bridge.request_stop()
                finally:
                    self._control_lock.release()
            else:
                 self._log_warn("Control lock busy, deferred internal state switch (safety flag already set)")

    def _clear_emergency_stop(self):
        """
        清除紧急停止状态
        """
        self._emergency_stop_requested = False
        self._emergency_stop_time = None
        self._log_info("Emergency stop cleared")
    
    def _is_emergency_stopped(self) -> bool:
        """检查是否处于紧急停止状态"""
        return self._emergency_stop_requested
    
    # ==================== 姿态控制接口 ====================
    
    def _publish_extra_outputs(self, cmd: ControlOutput) -> None:
        """
        发布额外的控制输出
        """
        # 发布姿态命令 (四旋翼平台)
        if self._is_quadrotor and 'attitude_cmd' in cmd.extras:
            attitude_cmd = cmd.extras['attitude_cmd']
            if attitude_cmd is not None:
                self._last_attitude_cmd = attitude_cmd
                # 获取悬停状态 (如果处理器提供了的话)
                is_hovering = cmd.extras.get('is_hovering', False)
                
                self._publishers.publish_attitude_cmd(
                    attitude_cmd,
                    yaw_mode=self._attitude_yaw_mode if hasattr(self, '_attitude_yaw_mode') else 0,
                    is_hovering=is_hovering
                )
        
        # Publish MPC predicted trajectory if available
        if 'predicted_trajectory' in cmd.extras:
            predicted_traj = cmd.extras['predicted_trajectory']
            if predicted_traj and self._publishers is not None:
                # The MPC state is typically in the odom frame
                self._publishers.publish_predicted_path(predicted_traj, frame_id='odom')
    
    def _get_yaw_mode_string(self) -> str:
        """将航向模式整数转换为字符串"""
        mode_map = {
            0: 'velocity',
            1: 'fixed',
            2: 'target_point',
            3: 'manual',
        }
        return mode_map.get(self._attitude_yaw_mode, 'velocity')
    
    def _handle_set_hover_yaw(self, yaw: float) -> bool:
        """处理设置悬停航向请求"""
        if not self._is_quadrotor:
            self._log_warn("Set hover yaw is only available for quadrotor platform")
            return False
        
        if self._controller_bridge is None or self._controller_bridge.manager is None:
            return False
        
        self._controller_bridge.manager.set_hover_yaw(yaw)
        self._log_info(f"Hover yaw set to {yaw:.3f} rad")
        return True
    
    def _handle_get_attitude_rate_limits(self) -> Optional[Dict[str, float]]:
        """获取姿态角速度限制"""
        if not self._is_quadrotor:
            return None
        
        if self._controller_bridge is None or self._controller_bridge.manager is None:
            return None
        
        manager = self._controller_bridge.manager
        return manager.get_attitude_rate_limits()
    
    # ==================== 控制循环 ====================

    def _control_loop_core(self) -> Optional[ControlOutput]:
        """
        控制循环核心逻辑
        
        Returns:
            控制输出，如果无法执行控制则返回 None
        """
        # 0. 检查关闭状态
        if self._shutting_down.is_set():
            return None
        
        # 0.1 检查紧急停止 (无锁检查，最为快速)
        if self._emergency_stop_requested:
            self._publish_stop_cmd()
            return None
        
        # 0.2 检查轨迹消息类型
        if not self._traj_msg_available:
            self._log_warn_throttle(
                10.0, 
                "Trajectory message type not available! "
            )
            self._publish_stop_cmd()
            return None
        

        
        # 1. 获取最新数据 (Atomic Snapshot)
        snapshot = self._data_manager.get_control_snapshot()
        odom = snapshot['odom']
        imu = snapshot['imu']
        trajectory = snapshot['trajectory']
        data_ages = snapshot['ages']
        
        # 2. 检查数据有效性
        if odom is None or trajectory is None:
            if self._waiting_for_data:
                self._log_warn_throttle(5.0, "Waiting for odom and trajectory data...")
            return None
        
        if self._waiting_for_data:
            self._log_info("Data received, starting control")
            self._waiting_for_data = False
        
        # 3. 执行控制更新
        current_time = self._get_time()
        
        try:
            # 使用锁保护控制更新，但范围仅限于核心计算
            # 这样做的目的是确保 Reset 服务不会在计算中间修改状态
            # 但 Emergency Stop 可以在计算期间设置 flag，虽然无法中断计算，但可以拦截输出
            cmd = None
            with self._control_lock:
                # 再次快速检查，防止在等待锁的过程中状态改变
                # RACE CONDITION FIX: Explicitly check _waiting_for_data inside lock
                # If reset() happened while we were waiting for lock, _waiting_for_data will be True.
                if self._shutting_down.is_set() or self._emergency_stop_requested or self._waiting_for_data:
                    return None
                    
                cmd = self._controller_bridge.update(current_time, odom, trajectory, data_ages, imu)
            
            # [关键] 计算后再次检查紧急停止
            # 如果在 update 计算期间发生了急停，我们应该丢弃刚刚算出来的 cmd
            if self._emergency_stop_requested:
                self._publish_stop_cmd()
                # 尝试异步切换状态 (如果还没切换的话)
                # FIX: Ensure we transition to STOPPING if not already there.
                # Previously used magic number 3 which was MPC_DEGRADED, causing E-Stop failure in degraded mode.
                current_state = self._controller_bridge.get_state()
                if current_state not in (ControllerState.STOPPING, ControllerState.STOPPED):
                     self._controller_bridge.request_stop()
                return None

            if self._error_handler:
                self._error_handler.reset()
            
            # 4. 记录超时警告
            self._log_timeout_warnings()
            
            # 5. 发布额外输出
            self._publish_extra_outputs(cmd)
            
            # 6. 发布调试路径 (异步)
            self._publish_debug_path(trajectory)
            
            return cmd
        except Exception as e:
            self._handle_control_error(e)
            return None
    
    def _log_timeout_warnings(self) -> None:
        """
        记录超时警告
        
        从 ControllerManager 获取统一的超时状态，避免重复检测逻辑
        """
        if self._controller_bridge is None or self._controller_bridge.manager is None:
            return
        
        timeout_status = self._controller_bridge.manager.get_timeout_status()
        
        if timeout_status.odom_timeout:
            self._log_warn_throttle(
                1.0, 
                f"Odom timeout: age={timeout_status.last_odom_age_ms:.1f}ms"
            )
        
        if timeout_status.traj_timeout:
            if timeout_status.traj_grace_exceeded:
                self._log_warn_throttle(
                    1.0, 
                    f"Trajectory timeout (grace exceeded): age={timeout_status.last_traj_age_ms:.1f}ms"
                )
            else:
                self._log_warn_throttle(
                    2.0, 
                    f"Trajectory timeout (in grace period): age={timeout_status.last_traj_age_ms:.1f}ms"
                )
    
    def _handle_control_error(self, error: Exception):
        """
        处理控制更新错误 (委托给 ErrorHandler)
        """
        if self._error_handler is None:
            # Fallback if error handler not initialized (should not happen in normal run)
            self._log_error(f"Critical: Error handler not initialized. Original error: {error}")
            return

        # 1. 委托主要处理逻辑
        diag_base = self._error_handler.handle_control_error(
            error, 
            tf2_reinjection_callback=self._try_tf2_reinjection
        )
        
        # 2. 收集上下文信息 (Timing, TF, Estop)
        timeout_status = None
        if self._controller_bridge is not None and self._controller_bridge.manager is not None:
             timeout_status = self._controller_bridge.manager.get_timeout_status()
             
        tf2_status = {
            'tf2_available': getattr(self._tf_bridge, 'is_initialized', False) if self._tf_bridge else False,
            'tf2_injected': self._is_tf2_injected,
        }
        
        # 3. 丰富诊断信息
        full_diag = self._error_handler.enrich_diagnostics(
            diag_base,
            timeout_status=timeout_status,
            tf2_status=tf2_status,
            estop_status=self._emergency_stop_requested
        )
        
        # 4. 发布停止命令和错误诊断
        self._publish_stop_cmd()
        self._publish_diagnostics(full_diag, force=True)
    
    def _on_diagnostics(self, diag: Any):
        """诊断回调"""
        if isinstance(diag, dict):
            if 'transform' not in diag:
                diag['transform'] = {}
            diag['transform']['tf2_available'] = (
                getattr(self._tf_bridge, 'is_initialized', False) 
                if self._tf_bridge else False
            )
            diag['transform']['tf2_injected'] = self._is_tf2_injected
            
            # 添加紧急停止状态
            diag['emergency_stop'] = self._emergency_stop_requested
        else:
            # Assume it is a DiagnosticsV2 object
            diag.transform_tf2_available = (
                getattr(self._tf_bridge, 'is_initialized', False) 
                if self._tf_bridge else False
            )
            diag.transform_tf2_injected = self._is_tf2_injected
            diag.emergency_stop = self._emergency_stop_requested
        
        self._publish_diagnostics(diag)
    
    # ==================== 服务处理 ====================
    
    def _handle_reset(self):
        """处理重置请求"""
        # 使用 LifecycleMixin 的 reset 方法
        self.reset()
        self._log_info('Controller reset via service')
    
    def _handle_get_diagnostics(self) -> Optional[Dict[str, Any]]:
        """处理获取诊断请求"""
        if self._controller_bridge is not None:
            return self._controller_bridge.get_diagnostics()
        return None
    
    def _handle_set_state(self, target_state: int) -> bool:
        """
        处理设置状态请求
        
        支持的状态转换:
        - STOPPING: 请求停止 (安全操作，始终允许)
        - NORMAL: 从紧急停止恢复 (需要先清除紧急停止标志)
        
        其他状态转换出于安全考虑不允许外部直接设置。
        
        紧急停止恢复安全机制:
        - 恢复前会记录警告日志
        - 恢复后会有短暂的安全延迟（通过 _waiting_for_data 标志）
        - 需要收到新的传感器数据后才会恢复控制
        """
        if target_state == ControllerState.STOPPING.value:
            if self._controller_bridge is not None:
                success = self._controller_bridge.request_stop()
                if success:
                    self._log_info('Stop requested via service')
                return success
            return False
        elif target_state == ControllerState.NORMAL.value:
            # 允许从紧急停止恢复到正常状态
            if self._emergency_stop_requested:
                # 安全警告
                self._log_warn(
                    'Emergency stop recovery requested via set_state service. '
                    'Ensure the emergency condition has been resolved before resuming operation.'
                )
                self._clear_emergency_stop()
                # 设置等待数据标志，确保收到新数据后才恢复控制
                # 这提供了一个隐式的安全延迟
                self._waiting_for_data = True
                self._log_info(
                    'Emergency stop cleared. Waiting for fresh sensor data before resuming control.'
                )
                return True
            else:
                self._log_warn('Set state to NORMAL requested but not in emergency stop state')
                return False
        else:
            self._log_warn(
                f'Set state to {target_state} not allowed. '
                f'Only STOPPING ({ControllerState.STOPPING.value}) and '
                f'NORMAL ({ControllerState.NORMAL.value}) are supported for safety reasons.'
            )
            return False
    
    def shutdown(self):
        """
        关闭控制器
        
        设置关闭标志并调用 LifecycleMixin.shutdown() 执行实际关闭逻辑。
        关闭标志确保控制循环不会在关闭过程中继续发布消息。
        """
        # 设置关闭标志，阻止控制循环继续执行
        self._shutting_down.set()
        
        # 使用 LifecycleMixin 的 shutdown 方法
        LifecycleMixin.shutdown(self)
    
    # ==================== 抽象方法 ====================
    
    @abstractmethod
    def _get_time(self) -> float:
        """获取当前时间（秒）"""
        pass
    
    @abstractmethod
    def _log_info(self, msg: str):
        """记录信息日志"""
        pass
    
    @abstractmethod
    def _log_warn(self, msg: str):
        """记录警告日志"""
        pass
    
    @abstractmethod
    def _log_warn_throttle(self, period: float, msg: str):
        """记录节流警告日志"""
        pass
    
    @abstractmethod
    def _log_error(self, msg: str):
        """记录错误日志"""
        pass
    
    @abstractmethod
    def _publish_cmd(self, cmd: ControlOutput):
        """发布控制命令"""
        pass
    
    @abstractmethod
    def _publish_stop_cmd(self):
        """发布停止命令"""
        pass
    
    @abstractmethod
    def _publish_diagnostics(self, diag: Dict[str, Any], force: bool = False):
        """发布诊断信息"""
        pass
    
    def _publish_attitude_cmd(self, attitude_cmd: AttitudeCommand):
        """
        发布姿态命令 (四旋翼平台)
        
        子类可选实现。默认实现为空操作。
        """
        pass
    
    def _publish_debug_path(self, trajectory: Trajectory):
        """
        发布调试路径 (用于 RViz 可视化)
        
        子类可选实现。默认实现为空操作。
        将轨迹转换为 nav_msgs/Path 消息发布。
        
        Args:
            trajectory: UC 轨迹数据
        """
        pass
