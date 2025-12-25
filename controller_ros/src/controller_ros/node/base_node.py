"""
控制器节点基类

封装 ROS1 和 ROS2 节点的共享逻辑，避免代码重复。

功能:
- 控制器初始化和生命周期管理
- 数据管理和控制循环
- TF2 集成
- 紧急停止处理
- 姿态控制接口 (四旋翼平台)
"""
from abc import ABC, abstractmethod
from typing import Dict, Any, Optional, Callable
import logging
import time

import numpy as np

from universal_controller.config.default_config import DEFAULT_CONFIG, PLATFORM_CONFIG
from universal_controller.core.data_types import (
    Odometry, Imu, Trajectory, ControlOutput, AttitudeCommand
)
from universal_controller.core.enums import ControllerState, PlatformType

from ..bridge import ControllerBridge
from ..io.data_manager import DataManager
from ..utils import TimeSync, TF2InjectionManager

logger = logging.getLogger(__name__)


class ControllerNodeBase(ABC):
    """
    控制器节点基类
    
    封装 ROS1 和 ROS2 节点的共享逻辑：
    - 控制器初始化
    - 数据管理
    - 控制循环核心逻辑
    - TF2 注入
    - 诊断处理
    - 紧急停止处理
    - 姿态控制接口 (四旋翼平台)
    
    子类需要实现：
    - _create_ros_interfaces(): 创建 ROS 特定的接口
    - _get_time(): 获取当前 ROS 时间
    - _log_info/warn/error(): 日志方法
    - _publish_cmd(): 发布控制命令
    - _publish_stop_cmd(): 发布停止命令
    - _publish_diagnostics(): 发布诊断信息
    - _publish_attitude_cmd(): 发布姿态命令 (四旋翼平台)
    """
    
    def __init__(self):
        """初始化基类（子类应在调用 super().__init__() 前完成 ROS 节点初始化）"""
        # 配置属性（由子类在调用 _initialize() 前设置）
        self._params: Dict[str, Any] = {}
        self._topics: Dict[str, str] = {}
        self._default_frame_id: str = 'base_link'
        self._platform_type: str = 'differential'
        
        # 核心组件（由 _initialize() 创建）
        self._controller_bridge: Optional[ControllerBridge] = None
        self._data_manager: Optional[DataManager] = None
        self._time_sync: Optional[TimeSync] = None
        self._tf_bridge: Any = None
        self._tf2_injection_manager: Optional[TF2InjectionManager] = None
        
        # 状态
        self._waiting_for_data = True
        self._consecutive_errors = 0
        self._max_consecutive_errors = 10
        self._traj_msg_available = True
        
        # 紧急停止状态
        self._emergency_stop_requested = False
        self._emergency_stop_time: Optional[float] = None
        
        # 姿态控制状态 (四旋翼平台)
        self._is_quadrotor = False
        self._last_attitude_cmd: Optional[AttitudeCommand] = None
        self._attitude_yaw_mode: int = 0  # 0=FOLLOW_VELOCITY
    
    def _initialize(self):
        """
        初始化控制器组件
        
        子类应在加载参数后调用此方法。
        """
        # 1. 获取平台配置
        self._platform_type = self._params.get('system', {}).get('platform', 'differential')
        platform_config = PLATFORM_CONFIG.get(self._platform_type, PLATFORM_CONFIG['differential'])
        self._default_frame_id = platform_config.get('output_frame', 'base_link')
        
        # 检查是否为四旋翼平台
        self._is_quadrotor = platform_config.get('type') == PlatformType.QUADROTOR
        
        # 2. 创建数据管理器（带时钟跳变回调）
        self._data_manager = DataManager(
            get_time_func=self._get_time,
            on_clock_jump=self._on_clock_jump
        )
        
        # 3. 创建控制器桥接
        self._controller_bridge = ControllerBridge(self._params)
        
        # 4. 创建时间同步
        watchdog_config = self._params.get('watchdog', {})
        self._time_sync = TimeSync(
            max_odom_age_ms=watchdog_config.get('odom_timeout_ms', 500),
            max_traj_age_ms=watchdog_config.get('traj_timeout_ms', 2500),
            max_imu_age_ms=watchdog_config.get('imu_timeout_ms', -1)
        )
        
        # 5. 设置诊断回调
        self._controller_bridge.set_diagnostics_callback(self._on_diagnostics)
    
    def _notify_odom_received(self) -> None:
        """通知收到里程计数据（更新超时监控）"""
        if self._controller_bridge is not None:
            self._controller_bridge.notify_odom_received()
    
    def _notify_trajectory_received(self) -> None:
        """通知收到轨迹数据（更新超时监控）"""
        if self._controller_bridge is not None:
            self._controller_bridge.notify_trajectory_received()
    
    def _notify_imu_received(self) -> None:
        """通知收到 IMU 数据（更新超时监控）"""
        if self._controller_bridge is not None:
            self._controller_bridge.notify_imu_received()
    
    def _on_clock_jump(self, event) -> None:
        """
        时钟跳变回调
        
        Args:
            event: ClockJumpEvent 对象
        """
        if event.is_backward:
            self._log_warn(
                f"Clock jumped backward by {abs(event.jump_delta):.3f}s. "
                f"Waiting for fresh sensor data before resuming control."
            )
            # 可以在这里触发控制器重置或其他安全措施
            self._waiting_for_data = True
    
    def _inject_tf2_to_controller(self, blocking: bool = True):
        """
        将 TF2 注入到坐标变换器
        
        Args:
            blocking: 是否阻塞等待 TF2 buffer 预热
        """
        if self._tf_bridge is None:
            self._log_info("TF bridge is None, skipping TF2 injection")
            return
        
        if self._controller_bridge is None or self._controller_bridge.manager is None:
            self._log_warn("Controller bridge not ready, cannot inject TF2")
            return
        
        # 创建 TF2 注入管理器
        tf_config = self._params.get('tf', {})
        # 将控制频率传入 TF 配置，用于向后兼容 retry_interval_cycles
        tf_config_with_freq = tf_config.copy()
        tf_config_with_freq['ctrl_freq'] = self._params.get('system', {}).get('ctrl_freq', 50)
        
        self._tf2_injection_manager = TF2InjectionManager(
            tf_bridge=self._tf_bridge,
            controller_manager=self._controller_bridge.manager,
            config=tf_config_with_freq,
            log_info=self._log_info,
            log_warn=self._log_warn,
        )
        
        # 执行注入
        self._tf2_injection_manager.inject(blocking=blocking)
    
    def _try_tf2_reinjection(self):
        """尝试重新注入 TF2（运行时调用）"""
        if self._tf2_injection_manager is not None:
            self._tf2_injection_manager.try_reinjection_if_needed()
    
    # ==================== 紧急停止处理 ====================
    
    def _handle_emergency_stop(self):
        """
        处理紧急停止请求
        
        当收到紧急停止信号时调用此方法。
        """
        if not self._emergency_stop_requested:
            self._emergency_stop_requested = True
            self._emergency_stop_time = self._get_time()
            self._log_warn("Emergency stop requested!")
            
            # 请求控制器进入停止状态
            if self._controller_bridge is not None:
                self._controller_bridge.request_stop()
    
    def _clear_emergency_stop(self):
        """
        清除紧急停止状态
        
        通过 reset 服务调用时会清除紧急停止状态。
        """
        self._emergency_stop_requested = False
        self._emergency_stop_time = None
        self._log_info("Emergency stop cleared")
    
    def _is_emergency_stopped(self) -> bool:
        """检查是否处于紧急停止状态"""
        return self._emergency_stop_requested
    
    # ==================== 姿态控制接口 ====================
    
    def _try_publish_attitude_command(self, cmd: ControlOutput) -> None:
        """
        尝试获取状态并发布姿态命令 (仅四旋翼平台)
        
        安全地从状态估计器获取状态数组，处理各种边界情况。
        
        Args:
            cmd: 速度控制命令
        """
        try:
            # 显式检查 controller_bridge 是否存在
            if self._controller_bridge is None:
                return
            
            manager = self._controller_bridge.manager
            if manager is None:
                return
            
            state_estimator = getattr(manager, 'state_estimator', None)
            if state_estimator is None:
                return
            
            # 安全获取状态
            get_state_func = getattr(state_estimator, 'get_state', None)
            if get_state_func is None:
                return
            
            state_result = get_state_func()
            if state_result is None:
                return
            
            # 获取状态数组，支持多种返回格式
            state_array = None
            if hasattr(state_result, 'state'):
                state_array = state_result.state
            elif hasattr(state_result, 'x'):
                state_array = state_result.x
            elif isinstance(state_result, (list, tuple)):
                state_array = np.array(state_result)
            
            if state_array is None:
                return
            
            # 验证状态数组有效性
            if not isinstance(state_array, np.ndarray):
                try:
                    state_array = np.array(state_array)
                except (TypeError, ValueError):
                    return
            
            if state_array.size == 0 or np.any(np.isnan(state_array)):
                return
            
            self._compute_and_publish_attitude(cmd, state_array)
            
        except Exception as e:
            # 姿态命令发布失败不应影响主控制循环
            logger.debug(f"Failed to publish attitude command: {e}")
    
    def _compute_and_publish_attitude(self, cmd: ControlOutput, state_array) -> Optional[AttitudeCommand]:
        """
        计算并发布姿态命令 (仅四旋翼平台)
        
        Args:
            cmd: 速度控制命令
            state_array: 当前状态数组
        
        Returns:
            姿态命令，非四旋翼平台返回 None
        """
        if not self._is_quadrotor:
            return None
        
        if self._controller_bridge is None or self._controller_bridge.manager is None:
            return None
        
        manager = self._controller_bridge.manager
        
        # 使用 ControllerManager 的姿态控制接口
        attitude_cmd = manager.compute_attitude_command(
            cmd, state_array, 
            yaw_mode=self._get_yaw_mode_string()
        )
        
        if attitude_cmd is not None:
            self._last_attitude_cmd = attitude_cmd
            self._publish_attitude_cmd(attitude_cmd)
        
        return attitude_cmd
    
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
        """
        处理设置悬停航向请求
        
        Args:
            yaw: 目标航向 (rad)
        
        Returns:
            是否成功
        """
        if not self._is_quadrotor:
            self._log_warn("Set hover yaw is only available for quadrotor platform")
            return False
        
        if self._controller_bridge is None or self._controller_bridge.manager is None:
            return False
        
        self._controller_bridge.manager.set_hover_yaw(yaw)
        self._log_info(f"Hover yaw set to {yaw:.3f} rad")
        return True
    
    def _handle_get_attitude_rate_limits(self) -> Optional[Dict[str, float]]:
        """
        获取姿态角速度限制
        
        Returns:
            角速度限制字典，非四旋翼平台返回 None
        """
        if not self._is_quadrotor:
            return None
        
        if self._controller_bridge is None or self._controller_bridge.manager is None:
            return None
        
        manager = self._controller_bridge.manager
        if manager.attitude_controller is None:
            return None
        
        return manager.attitude_controller.get_attitude_rate_limits()
    
    # ==================== 控制循环 ====================
    
    def _control_loop_core(self) -> Optional[ControlOutput]:
        """
        控制循环核心逻辑
        
        Returns:
            控制输出，如果无法执行控制则返回 None
        """
        # 0. 检查紧急停止
        if self._emergency_stop_requested:
            self._publish_stop_cmd()
            return None
        
        # 0.1 检查轨迹消息类型是否可用
        if not self._traj_msg_available:
            self._log_warn_throttle(
                10.0, 
                "Trajectory message type not available! "
                "Controller cannot work. Please build the package with message generation."
            )
            self._publish_stop_cmd()
            return None
        
        # 0.2 尝试 TF2 重新注入（由 TF2InjectionManager 管理）
        self._try_tf2_reinjection()
        
        # 1. 获取最新数据
        odom = self._data_manager.get_latest_odom()
        imu = self._data_manager.get_latest_imu()
        trajectory = self._data_manager.get_latest_trajectory()
        
        # 2. 检查数据有效性
        if odom is None or trajectory is None:
            if self._waiting_for_data:
                self._log_warn_throttle(5.0, "Waiting for odom and trajectory data...")
            return None
        
        if self._waiting_for_data:
            self._log_info("Data received, starting control")
            self._waiting_for_data = False
        
        # 3. 检查数据新鲜度
        ages = self._data_manager.get_data_ages()
        timeouts = self._time_sync.check_freshness(ages)
        
        # 3.1 处理里程计超时
        if timeouts.get('odom_timeout', False):
            self._log_warn_throttle(
                1.0, 
                f"Odom timeout: age={ages.get('odom', 0)*1000:.1f}ms"
            )
        
        # 3.2 处理轨迹超时 - 这是安全关键的，需要特别处理
        if timeouts.get('traj_timeout', False):
            self._log_warn_throttle(
                1.0, 
                f"Trajectory timeout: age={ages.get('traj', 0)*1000:.1f}ms, "
                f"controller will use stale trajectory (safety handled by ControllerManager)"
            )
            # 注意：不在这里发布停止命令，因为 ControllerManager 内部的
            # TimeoutMonitor 会处理超时逻辑，包括宽限期和安全停止
        
        # 4. 执行控制更新
        try:
            cmd = self._controller_bridge.update(odom, trajectory, imu)
            self._consecutive_errors = 0
            
            # 5. 四旋翼平台：计算并发布姿态命令
            if self._is_quadrotor and self._controller_bridge.manager is not None:
                self._try_publish_attitude_command(cmd)
            
            return cmd
        except Exception as e:
            self._handle_control_error(e, timeouts)
            return None
    
    def _handle_control_error(self, error: Exception, timeouts: Dict[str, bool]):
        """处理控制更新错误"""
        self._consecutive_errors += 1
        
        # 限制连续错误计数的上限，避免无限增长
        # 上限设为 max_consecutive_errors 的 100 倍，足够记录长时间错误
        max_error_count = self._max_consecutive_errors * 100
        if self._consecutive_errors > max_error_count:
            self._consecutive_errors = max_error_count
        
        if self._consecutive_errors <= self._max_consecutive_errors:
            self._log_error(f'Controller update failed ({self._consecutive_errors}): {error}')
        elif self._consecutive_errors % 50 == 0:
            self._log_error(
                f'Controller update still failing ({self._consecutive_errors} consecutive errors): {error}'
            )
        
        self._publish_stop_cmd()
        
        error_diag = self._create_error_diagnostics(error, timeouts)
        self._publish_diagnostics(error_diag, force=True)
    
    def _create_error_diagnostics(self, error: Exception, 
                                   timeouts: Dict[str, bool]) -> Dict[str, Any]:
        """创建错误诊断信息"""
        ages = self._data_manager.get_data_ages()
        
        return {
            'state': 0,
            'mpc_success': False,
            'backup_active': False,
            'error_message': str(error),
            'consecutive_errors': self._consecutive_errors,
            'timeout': {
                'odom_timeout': timeouts.get('odom_timeout', False),
                'traj_timeout': timeouts.get('traj_timeout', False),
                'imu_timeout': timeouts.get('imu_timeout', False),
                'last_odom_age_ms': ages.get('odom', float('inf')) * 1000,
                'last_traj_age_ms': ages.get('traj', float('inf')) * 1000,
                'last_imu_age_ms': ages.get('imu', float('inf')) * 1000,
            },
            'cmd': {'vx': 0.0, 'vy': 0.0, 'vz': 0.0, 'omega': 0.0},
            'transform': {
                'tf2_available': getattr(self._tf_bridge, 'is_initialized', False) 
                                 if self._tf_bridge else False,
                'tf2_injected': self._is_tf2_injected,
            },
            'emergency_stop': self._emergency_stop_requested,
        }
    
    @property
    def _is_tf2_injected(self) -> bool:
        """检查 TF2 是否已注入"""
        if self._tf2_injection_manager is not None:
            return self._tf2_injection_manager.is_injected
        return False
    
    def _on_diagnostics(self, diag: Dict[str, Any]):
        """诊断回调"""
        if 'transform' not in diag:
            diag['transform'] = {}
        diag['transform']['tf2_available'] = (
            getattr(self._tf_bridge, 'is_initialized', False) 
            if self._tf_bridge else False
        )
        diag['transform']['tf2_injected'] = self._is_tf2_injected
        
        # 添加紧急停止状态
        diag['emergency_stop'] = self._emergency_stop_requested
        
        self._publish_diagnostics(diag)
    
    # ==================== 服务处理 ====================
    
    def _handle_reset(self):
        """处理重置请求"""
        if self._controller_bridge is not None:
            self._controller_bridge.reset()
        if self._data_manager is not None:
            self._data_manager.clear()
        if self._tf2_injection_manager is not None:
            self._tf2_injection_manager.reset()
        self._waiting_for_data = True
        self._consecutive_errors = 0
        self._clear_emergency_stop()
        self._last_attitude_cmd = None
        self._log_info('Controller reset')
    
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
                self._clear_emergency_stop()
                self._log_info('Emergency stop cleared via set_state service, resuming normal operation')
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
        """关闭控制器"""
        if self._controller_bridge is not None:
            self._controller_bridge.shutdown()
        self._log_info('Controller node shutdown')
    
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
