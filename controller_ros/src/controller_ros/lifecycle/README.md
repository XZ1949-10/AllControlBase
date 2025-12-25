# 生命周期管理模块

## 概述

本模块提供统一的生命周期管理接口，用于协调 `controller_ros` 中各组件的初始化、运行、重置和关闭。

## 设计原则

1. **统一接口**：所有需要生命周期管理的组件实现 `ILifecycle` 接口
2. **分离关注**：`reset()` 重置状态，`shutdown()` 释放资源
3. **健康检查**：统一的健康状态查询接口
4. **非侵入式**：现有组件可以逐步迁移，不强制要求

## 核心组件

### ILifecycle 接口

定义了生命周期管理的标准方法：

```python
class ILifecycle(ABC):
    @abstractmethod
    def initialize(self) -> bool:
        """初始化组件，分配资源"""
        pass
    
    @abstractmethod
    def shutdown(self) -> None:
        """关闭组件，释放所有资源"""
        pass
    
    @abstractmethod
    def reset(self) -> None:
        """重置组件状态，保留资源"""
        pass
    
    @abstractmethod
    def get_health_status(self) -> Dict[str, Any]:
        """获取组件健康状态"""
        pass
    
    @property
    @abstractmethod
    def lifecycle_state(self) -> LifecycleState:
        """获取当前生命周期状态"""
        pass
```

### LifecycleState 枚举

```python
class LifecycleState(Enum):
    UNINITIALIZED = auto()  # 组件已创建但未初始化
    RUNNING = auto()        # 组件正在运行
    SHUTDOWN = auto()       # 组件已关闭
    ERROR = auto()          # 组件处于错误状态
```

### LifecycleMixin

提供 `ILifecycle` 接口的默认实现，简化组件开发：

```python
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
```

### HealthChecker

统一的健康检查工具：

```python
checker = HealthChecker()
checker.register('controller', controller_bridge)
checker.register('data_manager', data_manager)

# 获取所有组件的健康状态
status = checker.check_all()

# 获取摘要
summary = checker.get_summary()
# {'total': 2, 'healthy': 2, 'unhealthy': 0, 'all_healthy': True, ...}

# 检查是否所有组件都健康
if checker.is_all_healthy():
    print("All components healthy")
```

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

## 已实现生命周期管理的组件

| 组件 | 类 | 说明 |
|------|-----|------|
| 控制器桥接 | `ControllerBridge` | 封装 ControllerManager，提供统一的控制接口 |
| 数据管理器 | `DataManager` | 管理传感器数据缓存，支持时钟跳变检测 |

## 使用 HealthChecker 进行统一监控

```python
from controller_ros.lifecycle import HealthChecker
from controller_ros.bridge import ControllerBridge
from controller_ros.io import DataManager

# 创建组件
bridge = ControllerBridge(config)
data_manager = DataManager(get_time_func=get_time_sec)

# 注册到健康检查器
checker = HealthChecker()
checker.register('controller', bridge)
checker.register('data_manager', data_manager)

# 定期检查健康状态
if not checker.is_all_healthy():
    unhealthy = checker.get_unhealthy_components()
    print(f"Unhealthy components: {unhealthy}")
```

## reset() vs shutdown() 的区别

| 方法 | 目的 | 资源 | 后续操作 |
|------|------|------|----------|
| `reset()` | 重置状态 | 保留 | 可以继续使用 |
| `shutdown()` | 释放资源 | 释放 | 不再可用 |

### 使用场景

- **reset()**：
  - 用户请求重置控制器
  - 从错误状态恢复
  - 开始新的控制周期

- **shutdown()**：
  - 节点关闭
  - 程序退出
  - 资源需要释放

## 健康状态字典格式

```python
{
    'healthy': bool,           # 组件是否健康
    'state': str,              # 生命周期状态名称
    'component': str,          # 组件名称
    'message': str,            # 状态描述
    'last_error': str,         # 最后一次错误（可选）
    'uptime_sec': float,       # 运行时间（可选）
    'details': Dict[str, Any], # 详细信息（可选）
}
```

## 线程安全性

- `LifecycleMixin` 的生命周期方法是线程安全的
- 使用 `threading.RLock` 保护状态转换
- 建议在单线程中调用生命周期方法

## 最佳实践

1. **幂等性**：`initialize()` 和 `shutdown()` 应该是幂等的
2. **错误处理**：生命周期方法不应抛出异常
3. **快速返回**：`get_health_status()` 应该快速返回
4. **资源清理**：`shutdown()` 应该优雅地处理部分初始化的情况
