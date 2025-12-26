"""
生命周期接口定义

v3.19 重构：从 universal_controller 导入统一接口
"""

# 从 universal_controller 导入统一接口
from universal_controller.core.interfaces import ILifecycleComponent, LifecycleState

# 向后兼容：ILifecycle 作为 ILifecycleComponent 的别名
ILifecycle = ILifecycleComponent

__all__ = ['ILifecycle', 'ILifecycleComponent', 'LifecycleState']
