"""
参数加载工具

提供参数加载的通用功能，供 ParamLoader 和 VisualizerParamLoader 复用。

设计说明:
=========
本模块抽取了参数加载的共享逻辑：
- 类型转换
- 递归加载
- ROS1/ROS2 策略模式

这避免了 param_loader.py 和 visualizer_param_loader.py 之间的代码重复。
"""
from typing import Dict, Any, Optional, Callable
from abc import ABC, abstractmethod
import logging

logger = logging.getLogger(__name__)


# =============================================================================
# 参数加载策略接口
# =============================================================================

class IParamStrategy(ABC):
    """参数加载策略接口"""
    
    @abstractmethod
    def get_param(self, param_path: str, default: Any) -> Any:
        """
        获取单个参数
        
        Args:
            param_path: 参数路径，如 'mpc/weights/position'
            default: 默认值
        
        Returns:
            参数值，如果不存在则返回默认值
        """
        pass
    
    @abstractmethod
    def has_param(self, param_path: str) -> bool:
        """检查参数是否存在"""
        pass


class ROS1Strategy(IParamStrategy):
    """ROS1 参数加载策略"""
    
    def __init__(self, private_namespace: bool = False):
        """
        初始化 ROS1 策略
        
        Args:
            private_namespace: 是否使用私有命名空间 (~param)
        """
        import rospy
        self._rospy = rospy
        self._private = private_namespace
    
    def get_param(self, param_path: str, default: Any) -> Any:
        """获取 ROS1 参数"""
        if self._private:
            # 优先级: 私有参数 > 全局参数 > 默认值
            private_path = f"~{param_path}"
            if self._rospy.has_param(private_path):
                return self._rospy.get_param(private_path)
        return self._rospy.get_param(param_path, default)
    
    def has_param(self, param_path: str) -> bool:
        """检查参数是否存在"""
        if self._private:
            if self._rospy.has_param(f"~{param_path}"):
                return True
        return self._rospy.has_param(param_path)


class ROS2Strategy(IParamStrategy):
    """ROS2 参数加载策略"""
    
    def __init__(self, node):
        self._node = node
        self._declared_params: set = set()
    
    def get_param(self, param_path: str, default: Any) -> Any:
        """获取 ROS2 参数"""
        # ROS2 使用 . 作为分隔符
        ros2_path = param_path.replace('/', '.')
        
        try:
            if ros2_path not in self._declared_params:
                if not self._node.has_parameter(ros2_path):
                    self._node.declare_parameter(ros2_path, default)
                self._declared_params.add(ros2_path)
            return self._node.get_parameter(ros2_path).value
        except Exception:
            return default
    
    def has_param(self, param_path: str) -> bool:
        """检查参数是否存在"""
        ros2_path = param_path.replace('/', '.')
        try:
            return self._node.has_parameter(ros2_path)
        except Exception:
            return False


class DefaultStrategy(IParamStrategy):
    """默认参数策略 (非 ROS 环境)"""
    
    def get_param(self, param_path: str, default: Any) -> Any:
        return default
    
    def has_param(self, param_path: str) -> bool:
        return False


# =============================================================================
# 类型转换工具
# =============================================================================

def convert_param_type(ros_value: Any, default_value: Any, strict: bool = False) -> Any:
    """
    类型转换
    
    确保从 ROS 参数服务器读取的值与默认值类型一致。
    
    Args:
        ros_value: 从 ROS 读取的值
        default_value: 默认值
        strict: 严格模式，如果为 True 则在精度丢失时抛出异常
    
    Returns:
        类型转换后的值
    
    Raises:
        ValueError: 严格模式下类型转换会丢失精度时抛出
    """
    if default_value is None:
        return ros_value
    
    default_type = type(default_value)
    
    # 特殊处理：bool 和 int 在 Python 中有继承关系
    if default_type == bool:
        if isinstance(ros_value, bool):
            return ros_value
        return bool(ros_value)
    
    # 数值类型转换
    if default_type == int and not isinstance(ros_value, bool):
        if isinstance(ros_value, float) and ros_value != int(ros_value):
            msg = (
                f"Type conversion from float to int will lose precision: "
                f"{ros_value} -> {int(ros_value)}"
            )
            if strict:
                raise ValueError(msg)
            logger.warning(msg)
        try:
            return int(ros_value)
        except (ValueError, TypeError):
            return default_value
    
    if default_type == float:
        try:
            return float(ros_value)
        except (ValueError, TypeError):
            return default_value
    
    # 字符串
    if default_type == str:
        return str(ros_value) if ros_value is not None else default_value
    
    # 列表 - 递归转换元素类型
    if default_type == list:
        if isinstance(ros_value, list):
            if len(default_value) > 0 and len(ros_value) > 0:
                element_default = default_value[0]
                return [convert_param_type(v, element_default, strict) for v in ros_value]
            return ros_value
        return default_value
    
    # numpy 数组
    try:
        import numpy as np
        if isinstance(default_value, np.ndarray):
            if isinstance(ros_value, (list, tuple)):
                return np.array(ros_value, dtype=default_value.dtype)
            elif isinstance(ros_value, np.ndarray):
                return ros_value.astype(default_value.dtype)
            return default_value
    except ImportError:
        pass
    
    return ros_value


def load_params_recursive(
    config: Dict[str, Any], 
    prefix: str, 
    strategy: IParamStrategy,
    type_convert: bool = True,
    strict: bool = False
) -> None:
    """
    递归加载 ROS 参数
    
    遍历 config 字典的结构，对于每个叶子节点，尝试从 ROS 参数服务器读取值。
    
    Args:
        config: 配置字典（会被原地修改）
        prefix: 当前路径前缀，如 'mpc/weights'
        strategy: 参数加载策略
        type_convert: 是否进行类型转换
        strict: 严格模式，如果为 True 则在精度丢失时抛出异常
    """
    for key, value in config.items():
        param_path = f"{prefix}/{key}" if prefix else key
        
        if isinstance(value, dict):
            load_params_recursive(value, param_path, strategy, type_convert, strict)
        else:
            ros_value = strategy.get_param(param_path, value)
            
            if type_convert:
                converted_value = convert_param_type(ros_value, value, strict)
            else:
                converted_value = ros_value
            
            if converted_value != value:
                logger.debug(f"Parameter override: {param_path} = {converted_value}")
            
            config[key] = converted_value


def get_strategy(node=None, ros_version: int = None, private_namespace: bool = False) -> IParamStrategy:
    """
    获取参数加载策略
    
    Args:
        node: ROS2 节点 (ROS2) 或 None (ROS1/非 ROS)
        ros_version: ROS 版本 (1 或 2)，如果为 None 则自动检测
        private_namespace: 是否使用私有命名空间 (仅 ROS1)
    
    Returns:
        参数加载策略实例
    """
    if ros_version is None:
        from .ros_compat import ROS_VERSION
        ros_version = ROS_VERSION
    
    if ros_version == 2 and node is not None:
        return ROS2Strategy(node)
    elif ros_version == 1:
        return ROS1Strategy(private_namespace=private_namespace)
    else:
        return DefaultStrategy()
