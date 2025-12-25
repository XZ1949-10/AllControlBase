"""
生命周期接口定义

定义统一的生命周期管理接口，所有需要生命周期管理的组件应实现此接口。

设计说明：
-----------
1. LifecycleState 枚举定义了组件的生命周期状态
2. ILifecycle 接口定义了生命周期管理的标准方法
3. reset() vs shutdown() 的区别：
   - reset(): 重置内部状态，保留资源，可以继续使用
   - shutdown(): 释放所有资源，组件不再可用

状态转换图：
-----------
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

线程安全性：
-----------
- 接口本身不保证线程安全
- 实现类应根据需要添加线程安全保护
- 建议在单线程中调用生命周期方法
"""

from abc import ABC, abstractmethod
from enum import Enum, auto
from typing import Dict, Any


class LifecycleState(Enum):
    """
    生命周期状态枚举
    
    状态说明：
    - UNINITIALIZED: 组件已创建但未初始化
    - RUNNING: 组件正在运行
    - SHUTDOWN: 组件已关闭，资源已释放
    - ERROR: 组件处于错误状态
    """
    UNINITIALIZED = auto()
    RUNNING = auto()
    SHUTDOWN = auto()
    ERROR = auto()


class ILifecycle(ABC):
    """
    生命周期管理接口
    
    所有需要生命周期管理的组件应实现此接口。
    
    方法说明：
    ---------
    - initialize(): 初始化组件，分配资源
    - shutdown(): 关闭组件，释放所有资源
    - reset(): 重置组件状态，保留资源
    - get_health_status(): 获取组件健康状态
    
    属性说明：
    ---------
    - lifecycle_state: 当前生命周期状态
    - is_running: 组件是否正在运行
    
    实现建议：
    ---------
    1. initialize() 应该是幂等的（多次调用结果相同）
    2. shutdown() 后不应再调用其他方法
    3. reset() 应该将组件恢复到刚初始化的状态
    4. get_health_status() 应该快速返回，不应阻塞
    """
    
    @abstractmethod
    def initialize(self) -> bool:
        """
        初始化组件
        
        分配必要的资源，建立连接，准备运行。
        
        Returns:
            bool: 初始化是否成功
        
        Raises:
            不应抛出异常，失败时返回 False
        
        Note:
            - 应该是幂等的：多次调用应该安全
            - 初始化失败后，组件应处于可以重试的状态
        """
        pass
    
    @abstractmethod
    def shutdown(self) -> None:
        """
        关闭组件，释放所有资源
        
        关闭后组件不再可用，不应再调用其他方法。
        
        Note:
            - 应该是幂等的：多次调用应该安全
            - 应该优雅地处理部分初始化的情况
            - 不应抛出异常
        """
        pass
    
    @abstractmethod
    def reset(self) -> None:
        """
        重置组件状态
        
        将组件恢复到刚初始化的状态，但保留已分配的资源。
        用于在不重启的情况下重新开始。
        
        Note:
            - 不释放资源，只重置状态
            - 重置后组件应该可以立即使用
            - 应该是幂等的
        """
        pass
    
    @abstractmethod
    def get_health_status(self) -> Dict[str, Any]:
        """
        获取组件健康状态
        
        Returns:
            Dict[str, Any]: 健康状态字典，至少包含：
                - 'healthy' (bool): 组件是否健康
                - 'state' (str): 当前生命周期状态名称
                - 'message' (str): 状态描述信息
                
                可选字段：
                - 'details' (Dict): 详细状态信息
                - 'last_error' (str): 最后一次错误信息
                - 'uptime_sec' (float): 运行时间（秒）
        
        Note:
            - 应该快速返回，不应阻塞
            - 不应抛出异常
        """
        pass
    
    @property
    @abstractmethod
    def lifecycle_state(self) -> LifecycleState:
        """
        获取当前生命周期状态
        
        Returns:
            LifecycleState: 当前状态
        """
        pass
    
    @property
    def is_running(self) -> bool:
        """
        组件是否正在运行
        
        Returns:
            bool: 如果状态为 RUNNING 返回 True
        """
        return self.lifecycle_state == LifecycleState.RUNNING
    
    @property
    def is_shutdown(self) -> bool:
        """
        组件是否已关闭
        
        Returns:
            bool: 如果状态为 SHUTDOWN 返回 True
        """
        return self.lifecycle_state == LifecycleState.SHUTDOWN
