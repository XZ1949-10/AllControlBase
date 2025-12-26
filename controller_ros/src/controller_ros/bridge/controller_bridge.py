"""
控制器桥接层

封装 ControllerManager 调用，隔离 ROS 层与算法库。

生命周期说明：
- 实现 ILifecycle 接口（通过 LifecycleMixin）
- initialize(): 创建并初始化 ControllerManager
- reset(): 重置控制器状态
- shutdown(): 关闭控制器，释放资源
"""
from typing import Dict, Any, Optional
import logging

from universal_controller.manager.controller_manager import ControllerManager
from universal_controller.core.data_types import (
    Odometry, Imu, Trajectory, ControlOutput
)
from universal_controller.core.enums import ControllerState
from ..lifecycle import LifecycleMixin, LifecycleState

logger = logging.getLogger(__name__)


class ControllerBridge(LifecycleMixin):
    """
    控制器桥接层
    
    职责:
    - 封装 ControllerManager 调用
    - 管理控制器生命周期
    - 提供统一的更新接口
    - 隔离 ROS 层与算法库
    
    生命周期:
    - 实现 ILifecycle 接口（通过 LifecycleMixin）
    - initialize(): 创建并初始化 ControllerManager（默认在构造时自动调用）
    - reset(): 重置控制器状态
    - shutdown(): 关闭控制器，释放资源
    
    初始化行为:
    - 默认 auto_initialize=True，构造时自动初始化，失败时抛出 RuntimeError
    - 设置 auto_initialize=False 可延迟初始化，调用者需手动调用 initialize()
    """
    
    def __init__(self, config: Dict[str, Any], auto_initialize: bool = True):
        """
        初始化控制器桥接
        
        Args:
            config: 控制器配置字典，将传递给 ControllerManager
            auto_initialize: 是否自动初始化（默认 True）
                - True: 构造时自动初始化，失败时抛出 RuntimeError
                - False: 延迟初始化，调用者需手动调用 initialize()
        
        Raises:
            RuntimeError: 当 auto_initialize=True 且初始化失败时
        """
        # 初始化 LifecycleMixin
        super().__init__()
        
        self._config = config
        self._manager: Optional[ControllerManager] = None
        
        # 自动初始化
        if auto_initialize:
            if not self.initialize():
                # 获取更详细的错误信息
                error_msg = self._last_error if self._last_error else "Unknown error"
                raise RuntimeError(
                    f"Controller bridge initialization failed: {error_msg}. "
                    f"Check configuration and ensure all dependencies are available."
                )
    
    # ==================== LifecycleMixin 实现 ====================
    
    def _do_initialize(self) -> bool:
        """初始化控制器"""
        try:
            self._manager = ControllerManager(self._config)
            self._manager.initialize_default_components()
            logger.info("Controller bridge initialized successfully")
            return True
        except Exception as e:
            logger.error(f"Controller initialization failed: {e}")
            self._manager = None
            return False
    
    def _do_shutdown(self) -> None:
        """关闭控制器"""
        if self._manager is not None:
            self._manager.shutdown()
            self._manager = None
        logger.info("Controller bridge shutdown")
    
    def _do_reset(self) -> None:
        """重置控制器状态"""
        if self._manager is not None:
            self._manager.reset()
            logger.info("Controller bridge reset")
    
    def _get_health_details(self) -> Dict[str, Any]:
        """获取详细健康信息"""
        if self._manager is None:
            return {'manager_available': False}
        
        return {
            'manager_available': True,
            'state': self._manager.get_state().name,
            'timeout_status': {
                'odom_timeout': self._manager.timeout_monitor.odom_timeout_enabled,
                'traj_timeout': self._manager.timeout_monitor.traj_timeout_enabled,
            },
        }
    
    def update(self, odom: Odometry, trajectory: Trajectory,
               imu: Optional[Imu] = None) -> ControlOutput:
        """
        执行一次控制更新
        
        Args:
            odom: 里程计数据 (UC 格式)
            trajectory: 轨迹数据 (UC 格式)
            imu: IMU 数据 (UC 格式，可选)
        
        Returns:
            控制输出 (UC 格式)
        
        Raises:
            RuntimeError: 控制器未初始化或已关闭
        """
        if not self.is_running or self._manager is None:
            raise RuntimeError("Controller not initialized or already shutdown")
        
        return self._manager.update(odom, trajectory, imu)
    
    def get_state(self) -> ControllerState:
        """获取控制器状态"""
        if self._manager is None:
            return ControllerState.INIT
        return self._manager.get_state()
    
    def get_diagnostics(self) -> Optional[Dict[str, Any]]:
        """获取诊断信息"""
        if self._manager is None:
            return None
        return self._manager.get_last_published_diagnostics()
    
    def set_diagnostics_callback(self, callback: callable) -> None:
        """设置诊断回调函数"""
        if self._manager is not None:
            self._manager.set_diagnostics_callback(callback)
    
    def request_stop(self) -> bool:
        """
        请求控制器进入停止状态
        
        这是唯一允许的外部状态干预，用于紧急停止场景。
        使用 StateMachine 的官方 request_stop() 接口，状态转换会在
        下一次 update() 时由状态机统一处理。
        
        Returns:
            是否成功请求停止
        """
        if not self.is_running or self._manager is None:
            return False
        
        if self._manager.state_machine is None:
            return False
        
        # 使用状态机的官方接口请求停止
        return self._manager.state_machine.request_stop()
    
    def notify_odom_received(self) -> None:
        """通知收到里程计数据（用于超时监控）"""
        if self._manager is not None:
            self._manager.notify_odom_received()
    
    def notify_trajectory_received(self) -> None:
        """通知收到轨迹数据（用于超时监控）"""
        if self._manager is not None:
            self._manager.notify_trajectory_received()
    
    def notify_imu_received(self) -> None:
        """通知收到 IMU 数据（用于超时监控）"""
        if self._manager is not None:
            self._manager.notify_imu_received()
    
    @property
    def is_initialized(self) -> bool:
        """
        控制器是否已初始化
        
        .. deprecated:: 3.18
            使用 is_running 代替
        """
        import warnings
        warnings.warn(
            "is_initialized is deprecated, use is_running instead",
            DeprecationWarning,
            stacklevel=2
        )
        return self.is_running
    
    @property
    def manager(self) -> Optional[ControllerManager]:
        """获取底层 ControllerManager (用于高级操作)"""
        return self._manager
