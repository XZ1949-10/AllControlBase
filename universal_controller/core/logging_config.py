"""
统一日志配置模块

提供统一的日志接口，支持 ROS 和非 ROS 环境。

使用方式:
=========

方式 1: 标准 Python 日志 (推荐)
    import logging
    logger = logging.getLogger(__name__)
    
    # 这是最简单的方式，符合 Python 最佳实践
    # 日志配置由应用层统一设置

方式 2: 使用 get_logger() (可选)
    from universal_controller.core.logging_config import get_logger
    logger = get_logger(__name__)
    
    # 提供额外的配置功能，适合需要独立配置的模块

方式 3: 使用 ThrottledLogger (频繁日志场景)
    from universal_controller.core.logging_config import ThrottledLogger
    throttled = ThrottledLogger(logger, min_interval=5.0)
    
    # 用于避免频繁触发的日志消息导致日志泛滥
    # 例如：TF2 查找失败、传感器数据异常等

日志级别规范:
=============

DEBUG:
    - 详细的调试信息，通常只在开发时启用
    - 例如：变量值、循环迭代、内部状态
    - 示例：logger.debug(f"MPC solve iteration {i}, residual={residual}")

INFO:
    - 正常运行时的重要事件
    - 状态转换、组件初始化完成、配置加载
    - 示例：logger.info("Controller state changed: NORMAL -> MPC_DEGRADED")

WARNING:
    - 潜在问题，但系统仍能正常运行
    - 配置验证警告、性能降级、数据质量问题
    - 示例：logger.warning("TF2 fallback started, using odom integration")

ERROR:
    - 错误，但系统可以继续运行（可能降级）
    - 组件初始化失败、求解器错误、通信超时
    - 示例：logger.error("ACADOS solver failed, using fallback")

CRITICAL:
    - 严重错误，系统可能无法继续运行
    - 安全关键故障、资源耗尽
    - 示例：logger.critical("Emergency stop triggered due to safety violation")

使用规范:
=========

1. 每个模块使用 logging.getLogger(__name__) 获取日志器
2. 不要在循环中记录 INFO 或更高级别的日志（使用 DEBUG）
3. 使用 f-string 格式化日志消息
4. 对于可能频繁触发的警告，使用 ThrottledLogger 避免日志泛滥
5. 错误日志应包含足够的上下文信息以便调试
"""
import logging
import sys
from typing import Optional

# 默认日志格式
DEFAULT_FORMAT = '[%(name)s] %(levelname)s: %(message)s'
DEFAULT_LEVEL = logging.INFO


def get_logger(name: str, level: Optional[int] = None) -> logging.Logger:
    """
    获取模块日志器
    
    Args:
        name: 日志器名称，通常使用 __name__
        level: 日志级别，默认为 INFO
    
    Returns:
        配置好的 Logger 实例
    
    使用示例:
        from ..core.logging_config import get_logger
        logger = get_logger(__name__)
        logger.info("Message")
        logger.warning("Warning message")
    """
    logger = logging.getLogger(name)
    
    # 只在根日志器未配置时进行配置
    if not logger.handlers and not logging.getLogger().handlers:
        handler = logging.StreamHandler(sys.stdout)
        handler.setFormatter(logging.Formatter(DEFAULT_FORMAT))
        logger.addHandler(handler)
    
    if level is not None:
        logger.setLevel(level)
    elif logger.level == logging.NOTSET:
        logger.setLevel(DEFAULT_LEVEL)
    
    return logger


def configure_logging(level: int = logging.INFO, 
                     format_str: str = DEFAULT_FORMAT) -> None:
    """
    配置全局日志设置
    
    Args:
        level: 日志级别
        format_str: 日志格式字符串
    """
    logging.basicConfig(
        level=level,
        format=format_str,
        handlers=[logging.StreamHandler(sys.stdout)]
    )


# 预定义的模块日志器名称
MODULE_NAMES = {
    'manager': 'universal_controller.manager',
    'estimator': 'universal_controller.estimator',
    'tracker': 'universal_controller.tracker',
    'safety': 'universal_controller.safety',
    'transform': 'universal_controller.transform',
    'consistency': 'universal_controller.consistency',
    'diagnostics': 'universal_controller.diagnostics',
    'health': 'universal_controller.health',
}


class ThrottledLogger:
    """
    节流日志器
    
    用于避免频繁触发的日志消息导致日志泛滥。
    
    使用示例:
        throttled = ThrottledLogger(logger, min_interval=5.0)
        # 以下消息最多每 5 秒记录一次
        throttled.warning("TF2 lookup failed", key="tf2_fail")
    """
    
    def __init__(self, logger: logging.Logger, min_interval: float = 1.0):
        """
        Args:
            logger: 底层日志器
            min_interval: 同一 key 的最小日志间隔（秒）
        """
        self._logger = logger
        self._min_interval = min_interval
        self._last_log_times: dict = {}
    
    def _should_log(self, key: str) -> bool:
        """检查是否应该记录日志"""
        import time
        current_time = time.monotonic()
        last_time = self._last_log_times.get(key, 0)
        
        if current_time - last_time >= self._min_interval:
            self._last_log_times[key] = current_time
            return True
        return False
    
    def debug(self, msg: str, key: str = None, *args, **kwargs):
        """记录 DEBUG 级别日志（带节流）"""
        if key is None or self._should_log(key):
            self._logger.debug(msg, *args, **kwargs)
    
    def info(self, msg: str, key: str = None, *args, **kwargs):
        """记录 INFO 级别日志（带节流）"""
        if key is None or self._should_log(key):
            self._logger.info(msg, *args, **kwargs)
    
    def warning(self, msg: str, key: str = None, *args, **kwargs):
        """记录 WARNING 级别日志（带节流）"""
        if key is None or self._should_log(key):
            self._logger.warning(msg, *args, **kwargs)
    
    def error(self, msg: str, key: str = None, *args, **kwargs):
        """记录 ERROR 级别日志（带节流）"""
        if key is None or self._should_log(key):
            self._logger.error(msg, *args, **kwargs)
