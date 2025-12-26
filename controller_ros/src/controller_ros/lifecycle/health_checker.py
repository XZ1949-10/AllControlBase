"""
健康检查工具

提供统一的健康检查接口，用于监控多个组件的状态。

v3.19 重构说明：
---------------
统一接口后，所有组件都实现 ILifecycleComponent 接口：
- get_health_status() 返回 Dict 表示支持健康检查
- get_health_status() 返回 None 表示不支持（使用默认状态）

不再需要适配器，HealthChecker 直接调用组件的 get_health_status() 方法。

使用示例：
    from controller_ros.lifecycle import HealthChecker
    
    checker = HealthChecker()
    
    # 直接注册任何实现 ILifecycleComponent 的组件
    checker.register('controller', controller_bridge)
    checker.register('data_manager', data_manager)
    checker.register('ekf', ekf_estimator)  # 无需适配器
    
    # 获取所有组件的健康状态
    status = checker.check_all()
    
    # 检查是否所有组件都健康
    if checker.is_all_healthy():
        print("All components healthy")
"""

from typing import Dict, Any, List, Optional, Union
import logging
import time

from .interfaces import ILifecycleComponent, LifecycleState

logger = logging.getLogger(__name__)


class HealthChecker:
    """
    健康检查器
    
    职责:
    - 注册和管理多个组件
    - 提供统一的健康检查接口
    - 聚合健康状态报告
    
    支持的组件类型:
    - 实现 ILifecycleComponent 接口的组件（推荐）
    - 任意具有 reset() 方法的对象（向后兼容）
    """
    
    def __init__(self):
        """初始化健康检查器"""
        self._components: Dict[str, Any] = {}
        self._check_history: List[Dict[str, Any]] = []
        self._max_history = 100
    
    def register(self, name: str, component: Any) -> None:
        """
        注册组件
        
        支持任何实现 ILifecycleComponent 接口的组件。
        对于不支持健康检查的组件（get_health_status 返回 None），
        将使用默认的健康状态。
        
        Args:
            name: 组件名称
            component: 组件实例
        """
        # 检查是否实现了基本的生命周期方法
        has_reset = hasattr(component, 'reset') and callable(getattr(component, 'reset'))
        
        if not has_reset:
            logger.warning(
                f"Component '{name}' does not implement reset() method. "
                f"Health checks may not work correctly."
            )
        
        self._components[name] = component
        logger.debug(f"Registered component: {name}")
    
    def unregister(self, name: str) -> bool:
        """
        取消注册组件
        
        Args:
            name: 组件名称
        
        Returns:
            是否成功取消注册
        """
        if name in self._components:
            del self._components[name]
            logger.debug(f"Unregistered component: {name}")
            return True
        return False
    
    def check(self, name: str) -> Optional[Dict[str, Any]]:
        """
        检查单个组件的健康状态
        
        Args:
            name: 组件名称
        
        Returns:
            健康状态字典，如果组件不存在返回 None
        """
        component = self._components.get(name)
        if component is None:
            return None
        
        try:
            # 尝试调用 get_health_status
            if hasattr(component, 'get_health_status'):
                status = component.get_health_status()
                if status is not None:
                    # 确保有 component 字段
                    if 'component' not in status:
                        status['component'] = name
                    return status
            
            # 组件不支持健康检查，返回基于状态的默认值
            return self._get_default_health_status(name, component)
            
        except Exception as e:
            logger.exception(f"Health check failed for component '{name}'")
            return {
                'healthy': False,
                'state': 'ERROR',
                'component': name,
                'message': f'Health check failed: {e}',
            }
    
    def _get_default_health_status(self, name: str, component: Any) -> Dict[str, Any]:
        """
        获取默认健康状态
        
        对于不支持 get_health_status 的组件，基于其他信息推断状态。
        """
        # 检查是否有 lifecycle_state 属性
        if hasattr(component, 'lifecycle_state'):
            state = component.lifecycle_state
            if isinstance(state, LifecycleState):
                return {
                    'healthy': state == LifecycleState.RUNNING,
                    'state': state.name,
                    'component': name,
                    'message': f'Status inferred from lifecycle_state',
                }
        
        # 检查是否有 get_health_metrics 方法（如 MPCController）
        if hasattr(component, 'get_health_metrics'):
            try:
                metrics = component.get_health_metrics()
                return {
                    'healthy': True,
                    'state': 'RUNNING',
                    'component': name,
                    'message': 'Component has health metrics',
                    'details': {'metrics': metrics},
                }
            except Exception:
                pass
        
        # 默认假设健康
        return {
            'healthy': True,
            'state': 'UNKNOWN',
            'component': name,
            'message': 'Component does not support health check',
        }
    
    def check_all(self) -> Dict[str, Dict[str, Any]]:
        """
        检查所有组件的健康状态
        
        Returns:
            字典，键为组件名称，值为健康状态
        """
        result = {}
        for name in self._components:
            result[name] = self.check(name)
        
        # 记录历史
        self._record_history(result)
        
        return result
    
    def is_healthy(self, name: str) -> bool:
        """
        检查单个组件是否健康
        
        Args:
            name: 组件名称
        
        Returns:
            组件是否健康
        """
        status = self.check(name)
        if status is None:
            return False
        return status.get('healthy', False)
    
    def is_all_healthy(self) -> bool:
        """
        检查所有组件是否都健康
        
        Returns:
            所有组件是否都健康
        """
        for name in self._components:
            if not self.is_healthy(name):
                return False
        return True
    
    def get_unhealthy_components(self) -> List[str]:
        """
        获取不健康的组件列表
        
        Returns:
            不健康组件的名称列表
        """
        unhealthy = []
        for name in self._components:
            if not self.is_healthy(name):
                unhealthy.append(name)
        return unhealthy
    
    def get_summary(self) -> Dict[str, Any]:
        """
        获取健康状态摘要
        
        Returns:
            摘要字典，包含：
            - total: 组件总数
            - healthy: 健康组件数
            - unhealthy: 不健康组件数
            - all_healthy: 是否所有组件都健康
            - unhealthy_names: 不健康组件名称列表
            - timestamp: 检查时间戳
        """
        all_status = self.check_all()
        healthy_count = sum(1 for s in all_status.values() if s and s.get('healthy', False))
        unhealthy_names = [name for name, s in all_status.items() 
                          if not s or not s.get('healthy', False)]
        
        return {
            'total': len(self._components),
            'healthy': healthy_count,
            'unhealthy': len(self._components) - healthy_count,
            'all_healthy': healthy_count == len(self._components),
            'unhealthy_names': unhealthy_names,
            'timestamp': time.time(),
        }
    
    def _record_history(self, status: Dict[str, Dict[str, Any]]) -> None:
        """记录检查历史"""
        record = {
            'timestamp': time.time(),
            'status': status,
            'all_healthy': all(s.get('healthy', False) for s in status.values() if s),
        }
        self._check_history.append(record)
        
        # 限制历史记录数量
        if len(self._check_history) > self._max_history:
            self._check_history = self._check_history[-self._max_history:]
    
    def get_history(self, count: int = 10) -> List[Dict[str, Any]]:
        """
        获取最近的检查历史
        
        Args:
            count: 返回的记录数量
        
        Returns:
            检查历史列表
        """
        return self._check_history[-count:]
    
    @property
    def component_names(self) -> List[str]:
        """获取所有注册的组件名称"""
        return list(self._components.keys())
    
    @property
    def component_count(self) -> int:
        """获取注册的组件数量"""
        return len(self._components)
