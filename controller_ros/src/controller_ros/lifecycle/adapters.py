"""
生命周期适配器 (已废弃)

v3.19 重构说明：
---------------
统一接口后，不再需要适配器。所有组件都实现 ILifecycleComponent 接口，
HealthChecker 可以直接使用组件的 get_health_status() 方法。

本模块保留是为了向后兼容，但标记为废弃。
新代码应直接使用 ILifecycleComponent 接口。

迁移指南：
---------
旧代码:
    from controller_ros.lifecycle import LifecycleComponentAdapter
    checker.register('ekf', LifecycleComponentAdapter(ekf, 'ekf'))

新代码:
    checker.register('ekf', ekf)  # 直接注册，无需适配器
"""

from typing import Dict, Any, Optional, Callable
import time
import logging
import warnings

from .interfaces import ILifecycleComponent, LifecycleState

logger = logging.getLogger(__name__)


class LifecycleComponentAdapter(ILifecycleComponent):
    """
    ILifecycleComponent 适配器 (已废弃)
    
    .. deprecated:: 3.19
        统一接口后不再需要适配器。直接使用组件即可。
    
    保留此类是为了向后兼容。新代码应直接注册组件到 HealthChecker。
    """
    
    def __init__(self, component, name: str, 
                 health_check_func: Optional[Callable[[], Dict[str, Any]]] = None):
        warnings.warn(
            "LifecycleComponentAdapter is deprecated since v3.19. "
            "Register components directly to HealthChecker without adapter.",
            DeprecationWarning,
            stacklevel=2
        )
        
        self._component = component
        self._name = name
        self._health_check_func = health_check_func
        self._state = LifecycleState.RUNNING
        self._start_time = time.monotonic()
        self._last_error: Optional[str] = None
        self._is_shutdown = False
    
    def reset(self) -> None:
        if self._is_shutdown:
            return
        try:
            if hasattr(self._component, 'reset'):
                self._component.reset()
            self._last_error = None
            if self._state == LifecycleState.ERROR:
                self._state = LifecycleState.RUNNING
        except Exception as e:
            self._last_error = str(e)
            self._state = LifecycleState.ERROR
    
    def shutdown(self) -> None:
        if self._is_shutdown:
            return
        try:
            if hasattr(self._component, 'shutdown'):
                self._component.shutdown()
        except Exception as e:
            self._last_error = str(e)
        finally:
            self._state = LifecycleState.SHUTDOWN
            self._is_shutdown = True
    
    def initialize(self) -> bool:
        if self._is_shutdown:
            return False
        self._state = LifecycleState.RUNNING
        self._start_time = time.monotonic()
        return True
    
    def get_health_status(self) -> Dict[str, Any]:
        status = {
            'healthy': self._state == LifecycleState.RUNNING,
            'state': self._state.name,
            'component': self._name,
            'adapter': 'LifecycleComponentAdapter',
        }
        if self._last_error:
            status['last_error'] = self._last_error
        if self._start_time and not self._is_shutdown:
            status['uptime_sec'] = time.monotonic() - self._start_time
        if self._health_check_func:
            try:
                status['details'] = self._health_check_func()
            except Exception:
                pass
        if hasattr(self._component, 'get_health_metrics'):
            try:
                status.setdefault('details', {})['metrics'] = self._component.get_health_metrics()
            except Exception:
                pass
        return status
    
    @property
    def lifecycle_state(self) -> LifecycleState:
        return self._state
    
    @property
    def component(self):
        return self._component


class ControllerManagerAdapter(ILifecycleComponent):
    """ControllerManager 适配器 (已废弃)"""
    
    def __init__(self, manager, name: str = 'controller_manager'):
        warnings.warn(
            "ControllerManagerAdapter is deprecated since v3.19.",
            DeprecationWarning,
            stacklevel=2
        )
        self._manager = manager
        self._name = name
        self._state = LifecycleState.RUNNING
        self._start_time = time.monotonic()
        self._is_shutdown = False
    
    def reset(self) -> None:
        if not self._is_shutdown:
            self._manager.reset()
    
    def shutdown(self) -> None:
        if not self._is_shutdown:
            self._manager.shutdown()
            self._state = LifecycleState.SHUTDOWN
            self._is_shutdown = True
    
    def initialize(self) -> bool:
        return not self._is_shutdown
    
    def get_health_status(self) -> Dict[str, Any]:
        from universal_controller.core.enums import ControllerState
        controller_state = self._manager.get_state()
        unhealthy = {ControllerState.STOPPED, ControllerState.STOPPING}
        return {
            'healthy': self._state == LifecycleState.RUNNING and controller_state not in unhealthy,
            'state': self._state.name,
            'component': self._name,
            'adapter': 'ControllerManagerAdapter',
            'details': {'controller_state': controller_state.name},
        }
    
    @property
    def lifecycle_state(self) -> LifecycleState:
        return self._state
    
    @property
    def manager(self):
        return self._manager
