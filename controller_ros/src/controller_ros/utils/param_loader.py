"""
参数加载器

从 ROS 参数服务器加载配置。
支持 ROS1 和 ROS2，以及非 ROS 环境。
"""
from typing import Dict, Any, Optional, List
import logging
import weakref

from universal_controller.config.default_config import DEFAULT_CONFIG
from .ros_compat import ROS_VERSION  # 统一使用 ros_compat 的版本检测

logger = logging.getLogger(__name__)


# 参数定义：(参数名, 默认值)
_ROS2_PARAM_DEFINITIONS = [
    # 节点配置
    ('node.control_rate', 50.0),
    ('node.use_sim_time', False),
    # 话题配置
    ('topics.odom', '/odom'),
    ('topics.imu', '/imu'),
    ('topics.trajectory', '/nn/local_trajectory'),
    ('topics.cmd_unified', '/cmd_unified'),
    ('topics.diagnostics', '/controller/diagnostics'),
    ('topics.state', '/controller/state'),
    # 平台配置
    ('platform.type', 'differential'),
    # TF 配置
    ('tf.source_frame', 'base_link'),
    ('tf.target_frame', 'odom'),
    ('tf.timeout_ms', 10),
    # 时间同步配置
    ('time_sync.max_odom_age_ms', 100),
    ('time_sync.max_traj_age_ms', 200),
    ('time_sync.max_imu_age_ms', 50),
    # 诊断配置
    ('diagnostics.publish_rate', 5),
]


def _safe_declare_parameter(node, name: str, default_value: Any) -> None:
    """
    安全地声明 ROS2 参数，避免重复声明异常
    
    Args:
        node: ROS2 节点
        name: 参数名
        default_value: 默认值
    """
    try:
        node.declare_parameter(name, default_value)
    except Exception:
        # 参数已声明，忽略
        # 注意：rclpy.exceptions.ParameterAlreadyDeclaredException 
        # 在某些版本可能不存在，所以捕获通用异常
        pass


class ParamLoader:
    """
    参数加载器
    
    从 ROS 参数服务器加载配置，并与默认配置合并。
    支持 ROS1、ROS2 和非 ROS 环境。
    """
    
    # 使用 WeakSet 记录已初始化参数的节点，避免内存泄漏
    # 当节点被销毁时，会自动从 set 中移除
    _initialized_nodes: weakref.WeakSet = weakref.WeakSet()
    
    @staticmethod
    def load(node=None) -> Dict[str, Any]:
        """
        加载参数
        
        Args:
            node: ROS2 节点 (ROS2) 或 None (ROS1/非 ROS)
        
        Returns:
            合并后的配置字典
        """
        # 从默认配置开始
        config = ParamLoader._deep_copy(DEFAULT_CONFIG)
        
        # 根据 ROS 版本加载参数
        if ROS_VERSION == 2 and node is not None:
            ros_params = ParamLoader._load_ros2_params(node)
            ParamLoader._merge_params(config, ros_params)
        elif ROS_VERSION == 1:
            ros_params = ParamLoader._load_ros1_params()
            ParamLoader._merge_params(config, ros_params)
        # 非 ROS 环境使用默认配置
        
        logger.info(f"Loaded config with platform: {config.get('system', {}).get('platform', 'unknown')}")
        return config
    
    @staticmethod
    def _declare_all_params(node) -> None:
        """声明所有 ROS2 参数（安全方式，避免重复声明）"""
        # 使用 WeakSet，节点销毁后会自动移除
        if node in ParamLoader._initialized_nodes:
            return
        
        for param_name, default_value in _ROS2_PARAM_DEFINITIONS:
            _safe_declare_parameter(node, param_name, default_value)
        
        ParamLoader._initialized_nodes.add(node)
    
    @staticmethod
    def _load_ros2_params(node) -> Dict[str, Any]:
        """从 ROS2 参数服务器加载参数"""
        # 先声明所有参数
        ParamLoader._declare_all_params(node)
        
        params = {}
        
        # 获取参数值
        params['node'] = {
            'control_rate': node.get_parameter('node.control_rate').value,
            'use_sim_time': node.get_parameter('node.use_sim_time').value,
        }
        
        params['topics'] = {
            'odom': node.get_parameter('topics.odom').value,
            'imu': node.get_parameter('topics.imu').value,
            'trajectory': node.get_parameter('topics.trajectory').value,
            'cmd_unified': node.get_parameter('topics.cmd_unified').value,
            'diagnostics': node.get_parameter('topics.diagnostics').value,
            'state': node.get_parameter('topics.state').value,
        }
        
        params['platform'] = {
            'type': node.get_parameter('platform.type').value,
        }
        
        params['tf'] = {
            'source_frame': node.get_parameter('tf.source_frame').value,
            'target_frame': node.get_parameter('tf.target_frame').value,
            'timeout_ms': node.get_parameter('tf.timeout_ms').value,
        }
        
        params['time_sync'] = {
            'max_odom_age_ms': node.get_parameter('time_sync.max_odom_age_ms').value,
            'max_traj_age_ms': node.get_parameter('time_sync.max_traj_age_ms').value,
            'max_imu_age_ms': node.get_parameter('time_sync.max_imu_age_ms').value,
        }
        
        params['diagnostics'] = {
            'publish_rate': node.get_parameter('diagnostics.publish_rate').value,
        }
        
        return params
    
    @staticmethod
    def _load_ros1_params() -> Dict[str, Any]:
        """从 ROS1 参数服务器加载参数"""
        import rospy
        
        params = {}
        
        params['node'] = {
            'control_rate': rospy.get_param('node/control_rate', 50.0),
            'use_sim_time': rospy.get_param('/use_sim_time', False),
        }
        
        params['topics'] = {
            'odom': rospy.get_param('topics/odom', '/odom'),
            'imu': rospy.get_param('topics/imu', '/imu'),
            'trajectory': rospy.get_param('topics/trajectory', '/nn/local_trajectory'),
            'cmd_unified': rospy.get_param('topics/cmd_unified', '/cmd_unified'),
            'diagnostics': rospy.get_param('topics/diagnostics', '/controller/diagnostics'),
            'state': rospy.get_param('topics/state', '/controller/state'),
        }
        
        params['platform'] = {
            'type': rospy.get_param('platform/type', 'differential'),
        }
        
        params['tf'] = {
            'source_frame': rospy.get_param('tf/source_frame', 'base_link'),
            'target_frame': rospy.get_param('tf/target_frame', 'odom'),
            'timeout_ms': rospy.get_param('tf/timeout_ms', 10),
        }
        
        params['time_sync'] = {
            'max_odom_age_ms': rospy.get_param('time_sync/max_odom_age_ms', 100),
            'max_traj_age_ms': rospy.get_param('time_sync/max_traj_age_ms', 200),
            'max_imu_age_ms': rospy.get_param('time_sync/max_imu_age_ms', 50),
        }
        
        params['diagnostics'] = {
            'publish_rate': rospy.get_param('diagnostics/publish_rate', 5),
        }
        
        return params
    
    @staticmethod
    def _merge_params(config: Dict[str, Any], ros_params: Dict[str, Any]):
        """合并 ROS 参数到配置"""
        # 系统配置
        if 'platform' in ros_params:
            config.setdefault('system', {})
            config['system']['platform'] = ros_params['platform']['type']
        
        if 'node' in ros_params:
            config.setdefault('system', {})
            config['system']['ctrl_freq'] = int(ros_params['node']['control_rate'])
        
        # TF 配置
        if 'tf' in ros_params:
            config.setdefault('transform', {})
            config['transform']['source_frame'] = ros_params['tf']['source_frame']
            config['transform']['target_frame'] = ros_params['tf']['target_frame']
        
        # 超时配置
        if 'time_sync' in ros_params:
            config.setdefault('watchdog', {})
            config['watchdog']['odom_timeout_ms'] = ros_params['time_sync']['max_odom_age_ms']
            config['watchdog']['traj_timeout_ms'] = ros_params['time_sync']['max_traj_age_ms']
            config['watchdog']['imu_timeout_ms'] = ros_params['time_sync']['max_imu_age_ms']
        
        # 诊断配置
        if 'diagnostics' in ros_params:
            config.setdefault('diagnostics', {})
            config['diagnostics']['publish_rate'] = ros_params['diagnostics']['publish_rate']
    
    @staticmethod
    def _deep_copy(d: Dict[str, Any]) -> Dict[str, Any]:
        """深拷贝字典"""
        import copy
        return copy.deepcopy(d)
    
    @staticmethod
    def get_topics(node=None) -> Dict[str, str]:
        """
        获取话题配置
        
        注意: 在 ROS2 中，此方法会确保参数已声明。
        """
        if ROS_VERSION == 2 and node is not None:
            # 确保参数已声明
            ParamLoader._declare_all_params(node)
            return {
                'odom': node.get_parameter('topics.odom').value,
                'imu': node.get_parameter('topics.imu').value,
                'trajectory': node.get_parameter('topics.trajectory').value,
                'cmd_unified': node.get_parameter('topics.cmd_unified').value,
                'diagnostics': node.get_parameter('topics.diagnostics').value,
                'state': node.get_parameter('topics.state').value,
            }
        elif ROS_VERSION == 1:
            import rospy
            return {
                'odom': rospy.get_param('topics/odom', '/odom'),
                'imu': rospy.get_param('topics/imu', '/imu'),
                'trajectory': rospy.get_param('topics/trajectory', '/nn/local_trajectory'),
                'cmd_unified': rospy.get_param('topics/cmd_unified', '/cmd_unified'),
                'diagnostics': rospy.get_param('topics/diagnostics', '/controller/diagnostics'),
                'state': rospy.get_param('topics/state', '/controller/state'),
            }
        else:
            # 非 ROS 环境返回默认值
            return {
                'odom': '/odom',
                'imu': '/imu',
                'trajectory': '/nn/local_trajectory',
                'cmd_unified': '/cmd_unified',
                'diagnostics': '/controller/diagnostics',
                'state': '/controller/state',
            }
