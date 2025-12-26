"""
生命周期管理模块

v3.19 重构：统一生命周期接口
"""

from .interfaces import ILifecycle, ILifecycleComponent, LifecycleState
from .mixins import LifecycleMixin
from .health_checker import HealthChecker
from .adapters import LifecycleComponentAdapter, ControllerManagerAdapter

__all__ = [
    'ILifecycle',
    'ILifecycleComponent',
    'LifecycleState',
    'LifecycleMixin',
    'HealthChecker',
    'LifecycleComponentAdapter',
    'ControllerManagerAdapter',
]
