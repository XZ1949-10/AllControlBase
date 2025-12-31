"""
控制器桥接层

封装 ControllerManager 调用，隔离 ROS 层与算法库。

生命周期说明：
- 实现 ILifecycleComponent 接口（通过 LifecycleMixin）
- initialize(): 创建并初始化 ControllerManager
- reset(): 重置控制器状态
- shutdown(): 关闭控制器，释放资源

使用方式：
- 推荐: 使用 create() 工厂方法，失败时返回 None
- 备选: 直接构造后调用 initialize()，需检查返回值
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
    - 实现 ILifecycleComponent 接口（通过 LifecycleMixin）
    - initialize(): 创建并初始化 ControllerManager
    - reset(): 重置控制器状态
    - shutdown(): 关闭控制器，释放资源
    
    使用示例:
        # 方式 1: 工厂方法 (推荐)
        bridge = ControllerBridge.create(config)
        if bridge is None:
            handle_error()
        
        # 方式 2: 显式初始化
        bridge = ControllerBridge(config)
        if not bridge.initialize():
            handle_error(bridge.last_error)
    """
    
    @classmethod
    def create(cls, config: Dict[str, Any]) -> Optional['ControllerBridge']:
        """
        工厂方法：创建并初始化 ControllerBridge
        
        Args:
            config: 控制器配置字典
        
        Returns:
            初始化成功的 ControllerBridge 实例，失败返回 None
        
        Example:
            bridge = ControllerBridge.create(config)
            if bridge is None:
                logger.error("Failed to create controller bridge")
                return
        """
        bridge = cls(config)
        if bridge.initialize():
            return bridge
        logger.error(f"ControllerBridge initialization failed: {bridge.last_error}")
        return None
    
    def __init__(self, config: Dict[str, Any]):
        """
        初始化控制器桥接（不执行实际初始化）
        
        Args:
            config: 控制器配置字典，将传递给 ControllerManager
        
        Note:
            构造后需调用 initialize() 或使用 create() 工厂方法
        """
        super().__init__()
        self._config = config
        self._manager: Optional[ControllerManager] = None
    
    # ==================== LifecycleMixin 实现 ====================
    
    def _do_initialize(self) -> bool:
        """
        初始化控制器
        
        Note:
            不捕获异常，让异常传播到 LifecycleMixin.initialize()，
            由其统一处理并保存完整错误信息到 _last_error。
            如果初始化失败，_manager 会在 _do_shutdown() 中被清理。
        """
        self._manager = ControllerManager(self._config)
        self._manager.initialize_default_components()
        logger.info("Controller bridge initialized successfully")
        return True
    
    def _do_shutdown(self) -> None:
        """关闭控制器"""
        if self._manager is not None:
            try:
                self._manager.shutdown()
            except Exception as e:
                logger.warning(f"Error during controller shutdown: {e}")
            finally:
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
    def manager(self) -> Optional[ControllerManager]:
        """获取底层 ControllerManager (用于高级操作)"""
        return self._manager
