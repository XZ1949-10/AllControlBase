"""
参数加载器

从 ROS 参数服务器加载配置。
支持 ROS1 和 ROS2，以及非 ROS 环境。

配置映射说明:
- ROS 参数 (YAML) -> universal_controller 配置
- system.ctrl_freq -> system.ctrl_freq
- system.platform -> system.platform
- watchdog.* -> watchdog.*
- mpc.* -> mpc.*
- attitude.* -> attitude.*
"""
from typing import Dict, Any, Optional, List, Set
import logging

from universal_controller.config.default_config import DEFAULT_CONFIG
from .ros_compat import ROS_VERSION

logger = logging.getLogger(__name__)


# =============================================================================
# ROS2 参数定义
# =============================================================================
_ROS2_PARAM_DEFINITIONS = [
    # 系统配置
    ('system.ctrl_freq', 50),
    ('system.platform', 'differential'),
    # 节点配置
    ('node.use_sim_time', False),
    # 话题配置
    ('topics.odom', '/odom'),
    ('topics.imu', '/imu'),
    ('topics.trajectory', '/nn/local_trajectory'),
    ('topics.cmd_unified', '/cmd_unified'),
    ('topics.diagnostics', '/controller/diagnostics'),
    ('topics.state', '/controller/state'),
    ('topics.emergency_stop', '/controller/emergency_stop'),
    ('topics.attitude_cmd', '/controller/attitude_cmd'),
    # TF 配置
    ('tf.source_frame', 'base_link'),
    ('tf.target_frame', 'odom'),
    ('tf.timeout_ms', 10),
    ('tf.buffer_warmup_timeout_sec', 2.0),
    ('tf.buffer_warmup_interval_sec', 0.1),
    # 超时配置
    ('watchdog.odom_timeout_ms', 200),
    ('watchdog.traj_timeout_ms', 500),
    ('watchdog.imu_timeout_ms', 100),
    ('watchdog.startup_grace_ms', 1000),
    # 诊断配置
    ('diagnostics.publish_rate', 5),
    # MPC 配置
    ('mpc.horizon', 20),
    ('mpc.horizon_degraded', 10),
    ('mpc.dt', 0.1),
    # 姿态控制配置 (四旋翼)
    ('attitude.mass', 1.5),
    ('attitude.roll_max', 0.5),
    ('attitude.pitch_max', 0.5),
    ('attitude.hover_yaw_compensation', True),
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
        pass


class ParamLoader:
    """
    参数加载器
    
    从 ROS 参数服务器加载配置，并与默认配置合并。
    支持 ROS1、ROS2 和非 ROS 环境。
    
    配置合并策略:
    1. 从 DEFAULT_CONFIG 深拷贝作为基础
    2. 从 ROS 参数服务器加载参数
    3. 将 ROS 参数合并到配置中 (覆盖默认值)
    """
    
    # 使用节点名称集合来跟踪已初始化的节点，避免 WeakSet 的潜在问题
    _initialized_node_names: Set[str] = set()
    
    @staticmethod
    def load(node=None) -> Dict[str, Any]:
        """
        加载参数
        
        Args:
            node: ROS2 节点 (ROS2) 或 None (ROS1/非 ROS)
        
        Returns:
            合并后的配置字典
        """
        config = ParamLoader._deep_copy(DEFAULT_CONFIG)
        
        if ROS_VERSION == 2 and node is not None:
            ros_params = ParamLoader._load_ros2_params(node)
            ParamLoader._merge_params(config, ros_params)
        elif ROS_VERSION == 1:
            ros_params = ParamLoader._load_ros1_params()
            ParamLoader._merge_params(config, ros_params)
        
        logger.info(
            f"Loaded config: platform={config.get('system', {}).get('platform', 'unknown')}, "
            f"ctrl_freq={config.get('system', {}).get('ctrl_freq', 50)}Hz"
        )
        return config
    
    @staticmethod
    def _declare_all_params(node) -> None:
        """声明所有 ROS2 参数"""
        # 使用节点的完全限定名称作为唯一标识
        try:
            node_name = node.get_fully_qualified_name()
        except AttributeError:
            # 如果节点没有 get_fully_qualified_name 方法，使用 get_name
            try:
                node_name = node.get_name()
            except AttributeError:
                # 最后回退到 id
                node_name = str(id(node))
        
        if node_name in ParamLoader._initialized_node_names:
            return
        
        for param_name, default_value in _ROS2_PARAM_DEFINITIONS:
            _safe_declare_parameter(node, param_name, default_value)
        
        ParamLoader._initialized_node_names.add(node_name)
    
    @staticmethod
    def _load_ros2_params(node) -> Dict[str, Any]:
        """从 ROS2 参数服务器加载参数"""
        ParamLoader._declare_all_params(node)
        
        params = {}
        
        # 系统配置
        params['system'] = {
            'ctrl_freq': node.get_parameter('system.ctrl_freq').value,
            'platform': node.get_parameter('system.platform').value,
        }
        
        # 节点配置
        params['node'] = {
            'use_sim_time': node.get_parameter('node.use_sim_time').value,
        }
        
        # 话题配置
        params['topics'] = {
            'odom': node.get_parameter('topics.odom').value,
            'imu': node.get_parameter('topics.imu').value,
            'trajectory': node.get_parameter('topics.trajectory').value,
            'cmd_unified': node.get_parameter('topics.cmd_unified').value,
            'diagnostics': node.get_parameter('topics.diagnostics').value,
            'state': node.get_parameter('topics.state').value,
            'emergency_stop': node.get_parameter('topics.emergency_stop').value,
            'attitude_cmd': node.get_parameter('topics.attitude_cmd').value,
        }
        
        # TF 配置
        params['tf'] = {
            'source_frame': node.get_parameter('tf.source_frame').value,
            'target_frame': node.get_parameter('tf.target_frame').value,
            'timeout_ms': node.get_parameter('tf.timeout_ms').value,
            'buffer_warmup_timeout_sec': node.get_parameter('tf.buffer_warmup_timeout_sec').value,
            'buffer_warmup_interval_sec': node.get_parameter('tf.buffer_warmup_interval_sec').value,
        }
        
        # 超时配置
        params['watchdog'] = {
            'odom_timeout_ms': node.get_parameter('watchdog.odom_timeout_ms').value,
            'traj_timeout_ms': node.get_parameter('watchdog.traj_timeout_ms').value,
            'imu_timeout_ms': node.get_parameter('watchdog.imu_timeout_ms').value,
            'startup_grace_ms': node.get_parameter('watchdog.startup_grace_ms').value,
        }
        
        # 诊断配置
        params['diagnostics'] = {
            'publish_rate': node.get_parameter('diagnostics.publish_rate').value,
        }
        
        # MPC 配置
        params['mpc'] = {
            'horizon': node.get_parameter('mpc.horizon').value,
            'horizon_degraded': node.get_parameter('mpc.horizon_degraded').value,
            'dt': node.get_parameter('mpc.dt').value,
        }
        
        # 姿态控制配置
        params['attitude'] = {
            'mass': node.get_parameter('attitude.mass').value,
            'roll_max': node.get_parameter('attitude.roll_max').value,
            'pitch_max': node.get_parameter('attitude.pitch_max').value,
            'hover_yaw_compensation': node.get_parameter('attitude.hover_yaw_compensation').value,
        }
        
        return params
    
    @staticmethod
    def _load_ros1_params() -> Dict[str, Any]:
        """从 ROS1 参数服务器加载参数"""
        import rospy
        
        params = {}
        
        # 系统配置
        params['system'] = {
            'ctrl_freq': rospy.get_param('system/ctrl_freq', 50),
            'platform': rospy.get_param('system/platform', 'differential'),
        }
        
        # 节点配置
        params['node'] = {
            'use_sim_time': rospy.get_param('/use_sim_time', False),
        }
        
        # 话题配置
        params['topics'] = {
            'odom': rospy.get_param('topics/odom', '/odom'),
            'imu': rospy.get_param('topics/imu', '/imu'),
            'trajectory': rospy.get_param('topics/trajectory', '/nn/local_trajectory'),
            'cmd_unified': rospy.get_param('topics/cmd_unified', '/cmd_unified'),
            'diagnostics': rospy.get_param('topics/diagnostics', '/controller/diagnostics'),
            'state': rospy.get_param('topics/state', '/controller/state'),
            'emergency_stop': rospy.get_param('topics/emergency_stop', '/controller/emergency_stop'),
            'attitude_cmd': rospy.get_param('topics/attitude_cmd', '/controller/attitude_cmd'),
        }
        
        # TF 配置
        params['tf'] = {
            'source_frame': rospy.get_param('tf/source_frame', 'base_link'),
            'target_frame': rospy.get_param('tf/target_frame', 'odom'),
            'timeout_ms': rospy.get_param('tf/timeout_ms', 10),
            'buffer_warmup_timeout_sec': rospy.get_param('tf/buffer_warmup_timeout_sec', 2.0),
            'buffer_warmup_interval_sec': rospy.get_param('tf/buffer_warmup_interval_sec', 0.1),
        }
        
        # 超时配置
        params['watchdog'] = {
            'odom_timeout_ms': rospy.get_param('watchdog/odom_timeout_ms', 200),
            'traj_timeout_ms': rospy.get_param('watchdog/traj_timeout_ms', 500),
            'imu_timeout_ms': rospy.get_param('watchdog/imu_timeout_ms', 100),
            'startup_grace_ms': rospy.get_param('watchdog/startup_grace_ms', 1000),
        }
        
        # 诊断配置
        params['diagnostics'] = {
            'publish_rate': rospy.get_param('diagnostics/publish_rate', 5),
        }
        
        # MPC 配置
        params['mpc'] = {
            'horizon': rospy.get_param('mpc/horizon', 20),
            'horizon_degraded': rospy.get_param('mpc/horizon_degraded', 10),
            'dt': rospy.get_param('mpc/dt', 0.1),
        }
        
        # 姿态控制配置
        params['attitude'] = {
            'mass': rospy.get_param('attitude/mass', 1.5),
            'roll_max': rospy.get_param('attitude/roll_max', 0.5),
            'pitch_max': rospy.get_param('attitude/pitch_max', 0.5),
            'hover_yaw_compensation': rospy.get_param('attitude/hover_yaw_compensation', True),
        }
        
        return params
    
    @staticmethod
    def _merge_params(config: Dict[str, Any], ros_params: Dict[str, Any]):
        """
        合并 ROS 参数到配置
        
        直接映射策略:
        - system.* -> system.*
        - watchdog.* -> watchdog.*
        - mpc.* -> mpc.*
        - attitude.* -> attitude.*
        - tf.* -> transform.*
        """
        # 系统配置 (直接映射)
        if 'system' in ros_params:
            config.setdefault('system', {})
            for key, value in ros_params['system'].items():
                config['system'][key] = value
        
        # 超时配置 (直接映射)
        if 'watchdog' in ros_params:
            config.setdefault('watchdog', {})
            for key, value in ros_params['watchdog'].items():
                config['watchdog'][key] = value
        
        # MPC 配置 (直接映射)
        if 'mpc' in ros_params:
            config.setdefault('mpc', {})
            for key, value in ros_params['mpc'].items():
                config['mpc'][key] = value
        
        # 姿态控制配置 (直接映射)
        if 'attitude' in ros_params:
            config.setdefault('attitude', {})
            for key, value in ros_params['attitude'].items():
                config['attitude'][key] = value
        
        # TF 配置 -> transform 配置
        if 'tf' in ros_params:
            config.setdefault('transform', {})
            config['transform']['source_frame'] = ros_params['tf'].get('source_frame', 'base_link')
            config['transform']['target_frame'] = ros_params['tf'].get('target_frame', 'odom')
        
        # 诊断配置
        if 'diagnostics' in ros_params:
            config.setdefault('diagnostics', {})
            for key, value in ros_params['diagnostics'].items():
                config['diagnostics'][key] = value
    
    @staticmethod
    def _deep_copy(d: Dict[str, Any]) -> Dict[str, Any]:
        """深拷贝字典"""
        import copy
        return copy.deepcopy(d)
    
    @staticmethod
    def get_topics(node=None) -> Dict[str, str]:
        """
        获取话题配置
        
        Returns:
            话题名称字典
        """
        if ROS_VERSION == 2 and node is not None:
            ParamLoader._declare_all_params(node)
            return {
                'odom': node.get_parameter('topics.odom').value,
                'imu': node.get_parameter('topics.imu').value,
                'trajectory': node.get_parameter('topics.trajectory').value,
                'cmd_unified': node.get_parameter('topics.cmd_unified').value,
                'diagnostics': node.get_parameter('topics.diagnostics').value,
                'state': node.get_parameter('topics.state').value,
                'emergency_stop': node.get_parameter('topics.emergency_stop').value,
                'attitude_cmd': node.get_parameter('topics.attitude_cmd').value,
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
                'emergency_stop': rospy.get_param('topics/emergency_stop', '/controller/emergency_stop'),
                'attitude_cmd': rospy.get_param('topics/attitude_cmd', '/controller/attitude_cmd'),
            }
        else:
            return {
                'odom': '/odom',
                'imu': '/imu',
                'trajectory': '/nn/local_trajectory',
                'cmd_unified': '/cmd_unified',
                'diagnostics': '/controller/diagnostics',
                'state': '/controller/state',
                'emergency_stop': '/controller/emergency_stop',
                'attitude_cmd': '/controller/attitude_cmd',
            }
    
    @staticmethod
    def get_tf_config(node=None) -> Dict[str, Any]:
        """
        获取 TF 配置
        
        Returns:
            TF 配置字典
        """
        if ROS_VERSION == 2 and node is not None:
            ParamLoader._declare_all_params(node)
            return {
                'source_frame': node.get_parameter('tf.source_frame').value,
                'target_frame': node.get_parameter('tf.target_frame').value,
                'timeout_ms': node.get_parameter('tf.timeout_ms').value,
                'buffer_warmup_timeout_sec': node.get_parameter('tf.buffer_warmup_timeout_sec').value,
                'buffer_warmup_interval_sec': node.get_parameter('tf.buffer_warmup_interval_sec').value,
            }
        elif ROS_VERSION == 1:
            import rospy
            return {
                'source_frame': rospy.get_param('tf/source_frame', 'base_link'),
                'target_frame': rospy.get_param('tf/target_frame', 'odom'),
                'timeout_ms': rospy.get_param('tf/timeout_ms', 10),
                'buffer_warmup_timeout_sec': rospy.get_param('tf/buffer_warmup_timeout_sec', 2.0),
                'buffer_warmup_interval_sec': rospy.get_param('tf/buffer_warmup_interval_sec', 0.1),
            }
        else:
            return {
                'source_frame': 'base_link',
                'target_frame': 'odom',
                'timeout_ms': 10,
                'buffer_warmup_timeout_sec': 2.0,
                'buffer_warmup_interval_sec': 0.1,
            }
