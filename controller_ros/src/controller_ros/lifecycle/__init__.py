"""
生命周期管理模块

提供统一的生命周期接口和管理器，用于协调 controller_ros 中各组件的
初始化、运行、重置和关闭。

设计原则：
1. 统一接口：所有需要生命周期管理的组件实现 ILifecycle 接口
2. 分离关注：reset() 重置状态，shutdown() 释放资源
3. 健康检查：统一的健康状态查询接口
4. 非侵入式：现有组件可以逐步迁移，不强制要求

使用示例：
    from controller_ros.lifecycle import ILifecycle, LifecycleState, HealthChecker
    
    class MyComponent(ILifecycle):
        def initialize(self) -> bool:
            # 初始化逻辑
            return True
        
        def shutdown(self) -> None:
            # 清理资源
            pass
        
        def reset(self) -> None:
            # 重置状态
            pass
        
        def get_health_status(self) -> Dict[str, Any]:
            return {'healthy': True, 'message': 'OK'}
        
        @property
        def lifecycle_state(self) -> LifecycleState:
            return self._state
    
    # 使用健康检查器
    checker = HealthChecker()
    checker.register('my_component', my_component)
    print(checker.get_summary())
"""

from .interfaces import ILifecycle, LifecycleState
from .mixins import LifecycleMixin
from .health_checker import HealthChecker

__all__ = [
    'ILifecycle',
    'LifecycleState',
    'LifecycleMixin',
    'HealthChecker',
]
