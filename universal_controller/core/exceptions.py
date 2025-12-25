"""
自定义异常类

本模块定义了控制器系统使用的自定义异常类。

异常层次结构:
=============

ControllerError (基类)
├── ConfigurationError
│   └── ConfigValidationError
├── ComponentError
│   ├── InitializationError
│   └── ShutdownError
└── RuntimeError
    ├── SolverError
    └── TransformError

使用指南:
=========

1. 配置错误 (ConfigurationError)
   - 在系统启动时抛出
   - 应该阻止系统启动
   - 示例：无效的配置参数

2. 组件错误 (ComponentError)
   - 组件初始化或关闭失败
   - 可能需要降级处理
   - 示例：ACADOS 初始化失败

3. 运行时错误 (RuntimeError)
   - 运行时发生的可恢复错误
   - 通常应该捕获并降级处理
   - 示例：MPC 求解失败

注意:
=====

- 大多数运行时错误应该通过返回默认值或降级处理，而不是抛出异常
- 异常主要用于配置错误和不可恢复的组件错误
- 在控制循环中应避免抛出异常，以保持实时性
"""


class ControllerError(Exception):
    """控制器错误基类"""
    pass


# =============================================================================
# 配置错误
# =============================================================================

class ConfigurationError(ControllerError):
    """配置错误基类"""
    pass


class ConfigValidationError(ConfigurationError):
    """
    配置验证错误
    
    当配置参数不满足验证规则时抛出。
    
    Attributes:
        errors: 错误列表，每个元素为 (key_path, error_message)
    """
    
    def __init__(self, message: str, errors: list = None):
        super().__init__(message)
        self.errors = errors or []


# =============================================================================
# 组件错误
# =============================================================================

class ComponentError(ControllerError):
    """组件错误基类"""
    pass


class InitializationError(ComponentError):
    """
    组件初始化错误
    
    当组件初始化失败时抛出。
    """
    pass


class ShutdownError(ComponentError):
    """
    组件关闭错误
    
    当组件关闭失败时抛出。
    通常应该捕获并记录，不应阻止其他组件关闭。
    """
    pass


# =============================================================================
# 运行时错误
# =============================================================================

class ControllerRuntimeError(ControllerError):
    """
    控制器运行时错误基类
    
    注意：命名为 ControllerRuntimeError 以避免与内置 RuntimeError 冲突
    """
    pass


class SolverError(ControllerRuntimeError):
    """
    求解器错误
    
    当 MPC 或其他求解器失败时抛出。
    通常应该捕获并使用备用控制器。
    """
    pass


class TransformError(ControllerRuntimeError):
    """
    坐标变换错误
    
    当坐标变换失败时抛出。
    通常应该捕获并使用降级策略。
    """
    pass


# =============================================================================
# 导出列表
# =============================================================================

__all__ = [
    'ControllerError',
    'ConfigurationError',
    'ConfigValidationError',
    'ComponentError',
    'InitializationError',
    'ShutdownError',
    'ControllerRuntimeError',
    'SolverError',
    'TransformError',
]
