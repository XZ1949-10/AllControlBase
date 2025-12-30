"""
生命周期管理 Mixin

提供 ILifecycleComponent 接口的完整实现，简化组件的生命周期管理。

v3.19 重构说明：
---------------
- LifecycleMixin 现在实现统一的 ILifecycleComponent 接口
- 提供完整的状态跟踪、错误处理、线程安全保护
- 子类只需实现 _do_xxx() 方法

使用方法：
---------
1. 继承 LifecycleMixin
2. 重写 _do_initialize(), _do_shutdown(), _do_reset() 方法
3. 可选：重写 _get_health_details() 提供详细健康信息

示例：
------
    class MyComponent(LifecycleMixin):
        def __init__(self):
            super().__init__()
            self._resource = None
        
        def _do_initialize(self) -> bool:
            self._resource = create_resource()
            return self._resource is not None
        
        def _do_shutdown(self) -> None:
            if self._resource:
                self._resource.close()
                self._resource = None
        
        def _do_reset(self) -> None:
            if self._resource:
                self._resource.reset()
        
        def _get_health_details(self) -> Dict[str, Any]:
            return {'resource_active': self._resource is not None}
"""

from typing import Dict, Any, Optional
import time
import logging
import threading

from .interfaces import ILifecycle, LifecycleState

logger = logging.getLogger(__name__)


class LifecycleMixin(ILifecycle):
    """
    生命周期管理 Mixin
    
    提供 ILifecycle 接口的默认实现，包括：
    - 状态跟踪 (lifecycle_state 属性)
    - 错误处理
    - 运行时间统计
    - 线程安全的状态转换
    
    子类需要实现：
    - _do_initialize(): 实际的初始化逻辑
    - _do_shutdown(): 实际的关闭逻辑
    - _do_reset(): 实际的重置逻辑
    
    可选重写：
    - _get_health_details(): 提供详细健康信息
    """
    
    def __init__(self):
        """初始化 Mixin"""
        self._lifecycle_state = LifecycleState.UNINITIALIZED
        self._lifecycle_lock = threading.RLock()
        self._start_time: Optional[float] = None
        self._last_error: Optional[str] = None
        self._component_name = self.__class__.__name__
    
    # ==================== ILifecycleComponent 实现 ====================
    
    def initialize(self) -> bool:
        """
        初始化组件
        
        线程安全，幂等。
        如果初始化失败，会调用 _do_shutdown() 清理可能已分配的资源。
        """
        with self._lifecycle_lock:
            # 幂等性：已经运行则直接返回成功
            if self._lifecycle_state == LifecycleState.RUNNING:
                logger.debug(f"{self._component_name}: Already initialized")
                return True
            
            # 已关闭则不能重新初始化
            if self._lifecycle_state == LifecycleState.SHUTDOWN:
                logger.warning(f"{self._component_name}: Cannot initialize after shutdown")
                return False
            
            try:
                logger.info(f"{self._component_name}: Initializing...")
                success = self._do_initialize()
                
                if success:
                    self._lifecycle_state = LifecycleState.RUNNING
                    self._start_time = time.monotonic()
                    self._last_error = None
                    logger.info(f"{self._component_name}: Initialized successfully")
                else:
                    self._lifecycle_state = LifecycleState.ERROR
                    self._last_error = "Initialization returned False"
                    logger.error(f"{self._component_name}: Initialization failed")
                    # 清理可能已分配的资源
                    self._safe_cleanup()
                
                return success
                
            except Exception as e:
                self._lifecycle_state = LifecycleState.ERROR
                self._last_error = str(e)
                logger.error(f"{self._component_name}: Initialization error: {e}")
                # 清理可能已分配的资源
                self._safe_cleanup()
                return False
    
    def _safe_cleanup(self) -> None:
        """
        安全清理资源
        
        在初始化失败时调用，清理可能已分配的资源。
        捕获所有异常，确保不会因清理失败而掩盖原始错误。
        """
        try:
            self._do_shutdown()
        except Exception as e:
            logger.warning(f"{self._component_name}: Cleanup error (ignored): {e}")
    
    def shutdown(self) -> None:
        """
        关闭组件
        
        线程安全，幂等。
        """
        with self._lifecycle_lock:
            # 幂等性：已经关闭则直接返回
            if self._lifecycle_state == LifecycleState.SHUTDOWN:
                logger.debug(f"{self._component_name}: Already shutdown")
                return
            
            try:
                logger.info(f"{self._component_name}: Shutting down...")
                self._do_shutdown()
                logger.info(f"{self._component_name}: Shutdown complete")
            except Exception as e:
                logger.error(f"{self._component_name}: Shutdown error: {e}")
            finally:
                # 无论是否出错，都标记为已关闭
                self._lifecycle_state = LifecycleState.SHUTDOWN
                self._start_time = None
    
    def reset(self) -> None:
        """
        重置组件状态
        
        线程安全。
        """
        with self._lifecycle_lock:
            if self._lifecycle_state == LifecycleState.SHUTDOWN:
                logger.warning(f"{self._component_name}: Cannot reset after shutdown")
                return
            
            try:
                logger.info(f"{self._component_name}: Resetting...")
                self._do_reset()
                
                # 如果之前是错误状态，重置后恢复为运行状态
                if self._lifecycle_state == LifecycleState.ERROR:
                    self._lifecycle_state = LifecycleState.RUNNING
                
                self._last_error = None
                logger.info(f"{self._component_name}: Reset complete")
                
            except Exception as e:
                self._lifecycle_state = LifecycleState.ERROR
                self._last_error = str(e)
                logger.error(f"{self._component_name}: Reset error: {e}")
    
    def get_health_status(self) -> Dict[str, Any]:
        """获取健康状态"""
        with self._lifecycle_lock:
            status = {
                'healthy': self._lifecycle_state == LifecycleState.RUNNING,
                'state': self._lifecycle_state.name,
                'component': self._component_name,
                'message': self._get_state_message(),
            }
            
            if self._last_error:
                status['last_error'] = self._last_error
            
            if self._start_time is not None:
                status['uptime_sec'] = time.monotonic() - self._start_time
            
            # 添加子类提供的详细信息
            try:
                details = self._get_health_details()
                if details:
                    status['details'] = details
            except Exception as e:
                status['details_error'] = str(e)
            
            return status
    
    @property
    def lifecycle_state(self) -> LifecycleState:
        """获取当前生命周期状态"""
        with self._lifecycle_lock:
            return self._lifecycle_state
    
    @property
    def is_running(self) -> bool:
        """
        组件是否正在运行
        
        便捷属性，等价于 lifecycle_state == LifecycleState.RUNNING
        """
        return self.lifecycle_state == LifecycleState.RUNNING
    
    # ==================== 子类需要实现的方法 ====================
    
    def _do_initialize(self) -> bool:
        """
        实际的初始化逻辑
        
        子类必须实现此方法。
        
        Returns:
            bool: 初始化是否成功
        """
        raise NotImplementedError(
            f"{self._component_name} must implement _do_initialize()"
        )
    
    def _do_shutdown(self) -> None:
        """
        实际的关闭逻辑
        
        子类必须实现此方法。
        """
        raise NotImplementedError(
            f"{self._component_name} must implement _do_shutdown()"
        )
    
    def _do_reset(self) -> None:
        """
        实际的重置逻辑
        
        子类必须实现此方法。
        """
        raise NotImplementedError(
            f"{self._component_name} must implement _do_reset()"
        )
    
    def _get_health_details(self) -> Dict[str, Any]:
        """
        获取详细健康信息
        
        子类可以重写此方法提供更多信息。
        
        Returns:
            Dict[str, Any]: 详细健康信息
        """
        return {}
    
    # ==================== 辅助方法 ====================
    
    def _get_state_message(self) -> str:
        """获取状态描述信息"""
        messages = {
            LifecycleState.UNINITIALIZED: "Component not initialized",
            LifecycleState.RUNNING: "Component running normally",
            LifecycleState.SHUTDOWN: "Component has been shutdown",
            LifecycleState.ERROR: f"Component in error state: {self._last_error or 'Unknown error'}",
        }
        return messages.get(self._lifecycle_state, "Unknown state")
    
    def _set_error(self, error_message: str) -> None:
        """
        设置错误状态
        
        供子类在运行时检测到错误时调用。
        
        Args:
            error_message: 错误描述
        """
        with self._lifecycle_lock:
            self._lifecycle_state = LifecycleState.ERROR
            self._last_error = error_message
            logger.error(f"{self._component_name}: {error_message}")
