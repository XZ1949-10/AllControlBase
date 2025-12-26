# 生命周期管理模块

## 概述

本模块提供统一的生命周期管理接口，用于协调 `controller_ros` 中各组件的初始化、运行、重置和关闭。

## v3.19 重构

### 统一接口设计

之前的设计有两套接口：
- `ILifecycleComponent` (universal_controller): 轻量级，只有 reset/shutdown
- `ILifecycle` (controller_ros): 完整版，包含 initialize/get_health_status

重构后统一为一套接口 `ILifecycleComponent`：

```python
class ILifecycleComponent(ABC):
    @abstractmethod
    def reset(self) -> None:
        """必须实现：重置内部状态"""
        pass
    
    def shutdown(self) -> None:
        """可选：释放资源（默认空实现）"""
        pass
    
    def initialize(self) -> bool:
        """可选：初始化（默认返回 True）"""
        return True
    
    def get_health_status(self) -> Optional[Dict[str, Any]]:
        """可选：健康检查（默认返回 None）"""
        return None
```

### 优势

1. **消除适配器** - 不再需要 `LifecycleComponentAdapter`
2. **简化代码** - 算法层组件只需实现 `reset()`
3. **灵活扩展** - ROS 层组件可覆盖其他方法获得完整监控能力
4. **向后兼容** - 现有代码无需修改

## 使用示例

### 简单算法组件

```python
from universal_controller.core.interfaces import ILifecycleComponent

class SimpleEstimator(ILifecycleComponent):
    def __init__(self):
        self._state = np.zeros(6)
    
    def reset(self) -> None:
        self._state = np.zeros(6)
```

### 完整监控组件

```python
from controller_ros.lifecycle import LifecycleMixin

class MonitoredController(LifecycleMixin):
    def __init__(self):
        super().__init__()
        self._resource = None
    
    def _do_initialize(self) -> bool:
        self._resource = create_resource()
        return self._resource is not None
    
    def _do_shutdown(self) -> None:
        if self._resource:
            self._resource.close()
    
    def _do_reset(self) -> None:
        if self._resource:
            self._resource.reset()
    
    def _get_health_details(self) -> Dict[str, Any]:
        return {'resource_active': self._resource is not None}
```

### 健康检查

```python
from controller_ros.lifecycle import HealthChecker

checker = HealthChecker()

# 直接注册任何实现 ILifecycleComponent 的组件
checker.register('ekf', ekf_estimator)
checker.register('mpc', mpc_controller)
checker.register('bridge', controller_bridge)

# 获取健康状态
summary = checker.get_summary()
print(f"All healthy: {summary['all_healthy']}")
```

## 核心组件

### LifecycleState 枚举

```python
class LifecycleState(Enum):
    UNINITIALIZED = auto()  # 组件已创建但未初始化
    RUNNING = auto()        # 组件正在运行
    SHUTDOWN = auto()       # 组件已关闭
    ERROR = auto()          # 组件处于错误状态
```

### LifecycleMixin

提供 `ILifecycleComponent` 接口的完整实现：
- 状态跟踪
- 错误处理
- 运行时间统计
- 线程安全的状态转换

### HealthChecker

统一的健康检查工具：
- 注册和管理多个组件
- 提供统一的健康检查接口
- 聚合健康状态报告

## 状态转换图

```
    UNINITIALIZED ──initialize()──> RUNNING
           │                           │
           │                      reset()│
           │                           ↓
           │                       RUNNING
           │                           │
           │                    shutdown()
           │                           │
           └───────────────────────────┘
                                       ↓
                                  SHUTDOWN
```

## 迁移指南

### 从旧版本迁移

旧代码（使用适配器）：
```python
from controller_ros.lifecycle import LifecycleComponentAdapter
checker.register('ekf', LifecycleComponentAdapter(ekf, 'ekf'))
```

新代码（直接注册）：
```python
checker.register('ekf', ekf)  # 无需适配器
```

### 向后兼容

- `ILifecycle` 作为 `ILifecycleComponent` 的别名保留
- `LifecycleComponentAdapter` 仍可使用，但会发出废弃警告
- 现有代码无需修改即可运行

## 最佳实践

1. **幂等性** - `initialize()` 和 `shutdown()` 应该是幂等的
2. **错误处理** - 生命周期方法不应抛出异常
3. **快速返回** - `get_health_status()` 应该快速返回
4. **资源清理** - `shutdown()` 应该优雅地处理部分初始化的情况
