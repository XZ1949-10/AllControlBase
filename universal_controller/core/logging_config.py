"""
统一日志配置模块

提供统一的日志接口，支持 ROS 和非 ROS 环境。
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
