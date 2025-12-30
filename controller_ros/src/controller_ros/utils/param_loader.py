"""
参数加载器

从 ROS 参数服务器加载配置。
支持 ROS1 和 ROS2，以及非 ROS 环境。

设计说明:
=========
本模块采用"以 DEFAULT_CONFIG 为模板"的策略加载 ROS 参数：
1. 深拷贝 DEFAULT_CONFIG 作为基础配置
2. 递归遍历 DEFAULT_CONFIG 的结构
3. 对于每个配置项，尝试从 ROS 参数服务器读取对应值
4. 如果 ROS 参数存在，则覆盖默认值；否则保留默认值

这种设计的优点：
- 无需维护单独的参数定义列表
- 自动支持所有嵌套配置
- 新增配置项无需修改本模块
- 配置结构由 universal_controller 统一定义

配置分层设计:
=============
- transform.*: 坐标变换配置（坐标系名称 + 算法参数），定义在 DEFAULT_CONFIG
- tf.*: ROS TF2 特有参数（buffer 预热、重试等），仅 ROS 层使用
- topics.*: ROS 话题配置，仅 ROS 层使用
- node.*: ROS 节点配置，仅 ROS 层使用

配置映射规则:
- YAML 中的 `group/key` 映射到 `config['group']['key']`
- 支持任意深度的嵌套，如 `mpc/weights/position` -> `config['mpc']['weights']['position']`
"""
from typing import Dict, Any, Optional, List
from abc import ABC, abstractmethod
import logging
import copy

from universal_controller.config.default_config import DEFAULT_CONFIG
from .ros_compat import ROS_VERSION

logger = logging.getLogger(__name__)


# =============================================================================
# 话题配置 (独立于算法配置，仅 ROS 层使用)
#
# 命名规范:
# - 输入话题: /controller/input/<name>
# - 输出话题: /controller/<name>
# =============================================================================
TOPICS_DEFAULTS = {
    # 输入话题
    'odom': '/controller/input/odom',
    'imu': '',  # 默认禁用，需要时在配置中启用
    'trajectory': '/controller/input/trajectory',
    'emergency_stop': '/controller/emergency_stop',
    
    # 输出话题
    'cmd_unified': '/controller/cmd',
    'diagnostics': '/controller/diagnostics',
    'state': '/controller/state',
    'attitude_cmd': '/controller/attitude_cmd',
    'debug_path': '/controller/debug_path',
}

# TF 配置 (仅 ROS TF2 特有参数，坐标系名称统一从 transform 读取)
# 
# 配置分层设计：
# - transform.*: 坐标变换配置（坐标系名称 + 算法参数），定义在 DEFAULT_CONFIG
# - tf.*: ROS TF2 特有参数（buffer 预热、重试等），仅 ROS 层使用
#
# 注意：source_frame, target_frame, timeout_ms, expected_source_frames
# 统一在 transform 配置中定义，tf 配置不再包含这些字段
TF_DEFAULTS = {
    'buffer_warmup_timeout_sec': 2.0,   # TF buffer 预热超时
    'buffer_warmup_interval_sec': 0.1,  # TF buffer 预热检查间隔
    'retry_interval_sec': 1.0,          # TF 注入重试间隔（秒）
    'max_retries': -1,                  # 最大重试次数，-1 表示无限重试
}


# =============================================================================
# 参数加载策略接口
# =============================================================================
class ParamLoaderStrategy(ABC):
    """参数加载策略基类"""
    
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


class ROS1ParamStrategy(ParamLoaderStrategy):
    """ROS1 参数加载策略
    
    ROS1 参数命名空间说明:
    - rosparam load 在 launch 文件根级别使用时，参数加载到全局命名空间 /
    - 例如 YAML 中的 system/ctrl_freq 会被加载为 /system/ctrl_freq
    - rospy.get_param('system/ctrl_freq') 会自动解析相对路径到全局路径
    """
    
    def __init__(self):
        import rospy
        self._rospy = rospy
    
    def get_param(self, param_path: str, default: Any) -> Any:
        """获取 ROS1 参数
        
        Args:
            param_path: 参数路径，如 'mpc/weights/position'
            default: 默认值
        
        Returns:
            参数值，如果不存在则返回默认值
        
        Note:
            ROS1 的 get_param 会自动处理相对路径:
            - 'system/ctrl_freq' 会解析为 '/system/ctrl_freq'
            - 不需要手动添加 '/' 前缀
        """
        return self._rospy.get_param(param_path, default)
    
    def has_param(self, param_path: str) -> bool:
        """检查参数是否存在"""
        return self._rospy.has_param(param_path)


class ROS2ParamStrategy(ParamLoaderStrategy):
    """ROS2 参数加载策略"""
    
    def __init__(self, node):
        self._node = node
        self._declared_params: set = set()
    
    def _declare_if_needed(self, param_path: str, default: Any) -> None:
        """如果参数未声明，则声明它"""
        # ROS2 使用 . 作为分隔符
        ros2_path = param_path.replace('/', '.')
        
        if ros2_path in self._declared_params:
            return
        
        try:
            if not self._node.has_parameter(ros2_path):
                self._node.declare_parameter(ros2_path, default)
            self._declared_params.add(ros2_path)
        except Exception:
            pass
    
    def get_param(self, param_path: str, default: Any) -> Any:
        """获取 ROS2 参数"""
        ros2_path = param_path.replace('/', '.')
        self._declare_if_needed(param_path, default)
        
        try:
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


class DefaultParamStrategy(ParamLoaderStrategy):
    """默认参数策略 (非 ROS 环境)"""
    
    def get_param(self, param_path: str, default: Any) -> Any:
        """返回默认值"""
        return default
    
    def has_param(self, param_path: str) -> bool:
        """非 ROS 环境，参数不存在"""
        return False


# =============================================================================
# 参数加载器
# =============================================================================
class ParamLoader:
    """
    参数加载器
    
    从 ROS 参数服务器加载配置，并与默认配置合并。
    支持 ROS1、ROS2 和非 ROS 环境。
    
    使用示例:
        # ROS1
        config = ParamLoader.load(None)
        
        # ROS2
        config = ParamLoader.load(node)
        
        # 获取话题配置
        topics = ParamLoader.get_topics(None)
    """
    
    @staticmethod
    def _get_strategy(node=None) -> ParamLoaderStrategy:
        """获取参数加载策略"""
        if ROS_VERSION == 2 and node is not None:
            return ROS2ParamStrategy(node)
        elif ROS_VERSION == 1:
            return ROS1ParamStrategy()
        else:
            return DefaultParamStrategy()
    
    @staticmethod
    def load(node=None) -> Dict[str, Any]:
        """
        加载参数
        
        Args:
            node: ROS2 节点 (ROS2) 或 None (ROS1/非 ROS)
        
        Returns:
            合并后的配置字典，包含:
            - DEFAULT_CONFIG 中的所有配置（可能被 ROS 参数覆盖）
            - 'tf' 配置（ROS TF2 特有参数）
        """
        # 1. 深拷贝默认配置
        config = copy.deepcopy(DEFAULT_CONFIG)
        
        # 2. 获取加载策略
        strategy = ParamLoader._get_strategy(node)
        
        # 3. 递归加载 ROS 参数，覆盖默认值
        ParamLoader._load_recursive(config, '', strategy)
        
        # 4. 加载 TF 配置（仅 ROS TF2 特有参数）
        tf_config = ParamLoader.get_tf_config(node)
        config['tf'] = tf_config
        
        # 5. 日志
        platform = config.get('system', {}).get('platform', 'unknown')
        ctrl_freq = config.get('system', {}).get('ctrl_freq', 50)
        v_max = config.get('constraints', {}).get('v_max', 'N/A')
        source_frame = config.get('transform', {}).get('source_frame', 'unknown')
        target_frame = config.get('transform', {}).get('target_frame', 'unknown')
        logger.info(
            f"Loaded config: platform={platform}, ctrl_freq={ctrl_freq}Hz, "
            f"v_max={v_max}, frames={source_frame}->{target_frame}"
        )
        
        return config
    
    @staticmethod
    def _load_recursive(config: Dict[str, Any], prefix: str, 
                        strategy: ParamLoaderStrategy) -> None:
        """
        递归加载 ROS 参数
        
        遍历 config 字典的结构，对于每个叶子节点，尝试从 ROS 参数服务器读取值。
        
        Args:
            config: 配置字典（会被原地修改）
            prefix: 当前路径前缀，如 'mpc/weights'
            strategy: 参数加载策略
        """
        for key, value in config.items():
            # 构建参数路径
            param_path = f"{prefix}/{key}" if prefix else key
            
            if isinstance(value, dict):
                # 递归处理嵌套字典
                ParamLoader._load_recursive(value, param_path, strategy)
            else:
                # 叶子节点：尝试从 ROS 参数服务器读取
                ros_value = strategy.get_param(param_path, value)
                
                # 类型转换：确保类型一致
                converted_value = ParamLoader._convert_type(ros_value, value)
                
                # 记录参数覆盖情况（仅当值被覆盖时）
                if converted_value != value:
                    logger.debug(f"Parameter override: {param_path} = {converted_value} (default: {value})")
                
                config[key] = converted_value
    
    @staticmethod
    def _convert_type(ros_value: Any, default_value: Any) -> Any:
        """
        类型转换
        
        确保从 ROS 参数服务器读取的值与默认值类型一致。
        这是必要的，因为 YAML 可能将 0.5 解析为 int 0 或 float 0.5。
        
        Args:
            ros_value: 从 ROS 读取的值
            default_value: 默认值
        
        Returns:
            类型转换后的值
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
            # 检查是否会丢失精度
            if isinstance(ros_value, float) and ros_value != int(ros_value):
                logger.warning(
                    f"Type conversion from float to int will lose precision: "
                    f"{ros_value} -> {int(ros_value)}. "
                    f"Consider using float type in default config."
                )
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
                # 如果默认列表非空，使用第一个元素的类型作为参考
                if len(default_value) > 0 and len(ros_value) > 0:
                    element_default = default_value[0]
                    return [ParamLoader._convert_type(v, element_default) for v in ros_value]
                return ros_value
            return default_value
        
        # numpy 数组 - 从 list 转换
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
    
    @staticmethod
    def get_topics(node=None) -> Dict[str, str]:
        """
        获取话题配置
        
        话题配置是 ROS 层特有的，不在 DEFAULT_CONFIG 中。
        """
        strategy = ParamLoader._get_strategy(node)
        topics = {}
        
        for key, default in TOPICS_DEFAULTS.items():
            param_path = f"topics/{key}"
            topics[key] = strategy.get_param(param_path, default)
        
        return topics
    
    @staticmethod
    def get_tf_config(node=None) -> Dict[str, Any]:
        """
        获取 TF 配置
        
        仅包含 ROS TF2 特有参数（buffer 预热、重试等）。
        坐标系名称统一从 transform 配置读取。
        """
        strategy = ParamLoader._get_strategy(node)
        tf_config = {}
        
        for key, default in TF_DEFAULTS.items():
            param_path = f"tf/{key}"
            value = strategy.get_param(param_path, default)
            tf_config[key] = ParamLoader._convert_type(value, default)
        
        return tf_config
