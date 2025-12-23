"""
控制器桥接层

封装 ControllerManager 调用，隔离 ROS 层与算法库。
"""
from typing import Dict, Any, Optional
import logging

from universal_controller.manager.controller_manager import ControllerManager
from universal_controller.core.data_types import (
    Odometry, Imu, Trajectory, ControlOutput
)
from universal_controller.core.enums import ControllerState

logger = logging.getLogger(__name__)


class ControllerBridge:
    """
    控制器桥接层
    
    职责:
    - 封装 ControllerManager 调用
    - 管理控制器生命周期
    - 提供统一的更新接口
    - 隔离 ROS 层与算法库
    """
    
    def __init__(self, config: Dict[str, Any]):
        """
        初始化控制器桥接
        
        Args:
            config: 控制器配置字典，将传递给 ControllerManager
        """
        self._config = config
        self._manager: Optional[ControllerManager] = None
        self._initialized = False
        
        self._initialize()
    
    def _initialize(self):
        """初始化控制器"""
        try:
            self._manager = ControllerManager(self._config)
            self._manager.initialize_default_components()
            self._initialized = True
            logger.info("Controller bridge initialized successfully")
        except Exception as e:
            logger.error(f"Controller initialization failed: {e}")
            raise
    
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
            RuntimeError: 控制器未初始化
        """
        if not self._initialized or self._manager is None:
            raise RuntimeError("Controller not initialized")
        
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
    
    def reset(self):
        """重置控制器"""
        if self._manager is not None:
            self._manager.reset()
            logger.info("Controller reset")
    
    def request_stop(self) -> bool:
        """
        请求控制器进入停止状态
        
        这是唯一允许的外部状态干预，用于紧急停止场景。
        使用 StateMachine 的官方 request_stop() 接口，状态转换会在
        下一次 update() 时由状态机统一处理。
        
        Returns:
            是否成功请求停止
        """
        if not self._initialized or self._manager is None:
            return False
        
        if self._manager.state_machine is None:
            return False
        
        # 使用状态机的官方接口请求停止
        return self._manager.state_machine.request_stop()
    
    def shutdown(self):
        """关闭控制器"""
        if self._manager is not None:
            self._manager.shutdown()
            self._initialized = False
            logger.info("Controller shutdown")
    
    @property
    def is_initialized(self) -> bool:
        """控制器是否已初始化"""
        return self._initialized
    
    @property
    def manager(self) -> Optional[ControllerManager]:
        """获取底层 ControllerManager (用于高级操作)"""
        return self._manager
