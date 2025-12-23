"""
参数加载器

从 ROS 参数服务器加载配置。
"""
from typing import Dict, Any
import logging

from rclpy.node import Node

from universal_controller.config.default_config import DEFAULT_CONFIG

logger = logging.getLogger(__name__)


class ParamLoader:
    """
    参数加载器
    
    从 ROS 参数服务器加载配置，并与默认配置合并。
    """
    
    @staticmethod
    def load(node: Node) -> Dict[str, Any]:
        """
        加载参数
        
        Args:
            node: ROS2 节点
        
        Returns:
            合并后的配置字典
        """
        # 从默认配置开始
        config = ParamLoader._deep_copy(DEFAULT_CONFIG)
        
        # 声明并获取 ROS 参数
        ros_params = ParamLoader._load_ros_params(node)
        
        # 合并 ROS 参数到配置
        ParamLoader._merge_params(config, ros_params)
        
        logger.info(f"Loaded config with platform: {config.get('system', {}).get('platform', 'unknown')}")
        return config
    
    @staticmethod
    def _load_ros_params(node: Node) -> Dict[str, Any]:
        """从 ROS 参数服务器加载参数"""
        params = {}
        
        # 节点配置
        node.declare_parameter('node.control_rate', 50.0)
        node.declare_parameter('node.use_sim_time', False)
        
        # 话题配置
        node.declare_parameter('topics.odom', '/odom')
        node.declare_parameter('topics.imu', '/imu')
        node.declare_parameter('topics.trajectory', '/nn/local_trajectory')
        node.declare_parameter('topics.cmd_unified', '/cmd_unified')
        node.declare_parameter('topics.diagnostics', '/controller/diagnostics')
        
        # 平台配置
        node.declare_parameter('platform.type', 'differential')
        
        # TF 配置
        node.declare_parameter('tf.source_frame', 'base_link')
        node.declare_parameter('tf.target_frame', 'odom')
        node.declare_parameter('tf.timeout_ms', 10)
        
        # 时间同步配置
        node.declare_parameter('time_sync.max_odom_age_ms', 100)
        node.declare_parameter('time_sync.max_traj_age_ms', 200)
        node.declare_parameter('time_sync.max_imu_age_ms', 50)
        
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
    
    @staticmethod
    def _deep_copy(d: Dict[str, Any]) -> Dict[str, Any]:
        """深拷贝字典"""
        import copy
        return copy.deepcopy(d)
    
    @staticmethod
    def get_topics(node: Node) -> Dict[str, str]:
        """获取话题配置"""
        return {
            'odom': node.get_parameter('topics.odom').value,
            'imu': node.get_parameter('topics.imu').value,
            'trajectory': node.get_parameter('topics.trajectory').value,
            'cmd_unified': node.get_parameter('topics.cmd_unified').value,
            'diagnostics': node.get_parameter('topics.diagnostics').value,
        }
