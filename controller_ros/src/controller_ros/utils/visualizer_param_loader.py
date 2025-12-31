"""
Visualizer 参数加载器

从 ROS 参数服务器加载 visualizer 配置。
复用 param_utils 中的共享逻辑。

配置加载优先级:
1. 私有参数 (~param) - 最高优先级
2. 全局参数 (param) - 次优先级  
3. 默认值 - 最低优先级

话题配置统一性:
- 共享话题 (odom, trajectory, diagnostics, emergency_stop) 优先从全局 topics 配置读取
- 确保 visualizer 与 controller 使用相同的话题
- 可视化器特有话题 (joy, cmd_vel_output, control_mode) 从 visualizer 配置读取

速度限制统一性:
- 速度限制 (v_max, omega_max) 统一从 constraints 配置读取
- 确保 visualizer、cmd_vel_adapter、controller 使用相同的限制值
"""
from typing import Dict, Any
import logging
import copy

from .param_utils import get_strategy, load_params_recursive, IParamStrategy

logger = logging.getLogger(__name__)


# =============================================================================
# Visualizer 默认配置
#
# 设计原则:
# - 共享话题从全局 topics 配置读取，不在此定义默认值
# - 速度限制从全局 constraints 配置读取，不在此定义默认值
# - 本配置只定义可视化器特有的参数
# =============================================================================
VISUALIZER_DEFAULTS = {
    'topics': {
        # 可视化器特有话题 (共享话题从全局配置读取)
        'camera_image': '',
        'joy': '/joy',
        'cmd_vel_output': '/joy_cmd_vel',
        'control_mode': '/visualizer/control_mode',
    },
    'display': {
        'update_rate': 30,
        'velocity_history_sec': 10,
        'trajectory_color': [0, 255, 0],
        'robot_color': [255, 0, 0],
        'target_color': [255, 255, 0],
    },
    'joystick': {
        'enable_button': 4,
        'linear_axis': 1,
        'angular_axis': 3,
        'left_x_axis': 0,
        'left_y_axis': 1,
        'right_x_axis': 3,
        'right_y_axis': 4,
        'deadzone': 0.1,
        'publish_rate': 20.0,
    },
    'camera': {
        'use_camera': False,
        'calibration_file': '',
    },
}

# 共享话题列表 - 从全局 topics 配置读取
SHARED_TOPICS = ['odom', 'trajectory', 'cmd_unified', 'diagnostics', 'emergency_stop']

# 共享话题的回退默认值 (仅当全局配置不存在时使用)
SHARED_TOPICS_FALLBACK = {
    'odom': '/odom',
    'trajectory': '/nn/local_trajectory',
    'cmd_unified': '/cmd_unified',
    'diagnostics': '/controller/diagnostics',
    'emergency_stop': '/controller/emergency_stop',
}


class VisualizerParamLoader:
    """
    Visualizer 参数加载器
    
    统一的配置加载接口，支持 ROS1 和 ROS2。
    
    配置来源:
    - 共享话题: 从全局 topics.* 配置读取
    - 速度限制: 从全局 constraints.* 配置读取
    - 可视化器特有配置: 从 visualizer_params.yaml 读取
    
    使用示例:
        # ROS1 (在节点初始化后调用)
        config = VisualizerParamLoader.load()
        
        # ROS2
        config = VisualizerParamLoader.load(node)
    """
    
    @staticmethod
    def load(node=None) -> Dict[str, Any]:
        """
        加载 visualizer 配置
        
        Args:
            node: ROS2 节点 (ROS2) 或 None (ROS1)
        
        Returns:
            配置字典
        """
        try:
            import rospy
            return VisualizerParamLoader._load_ros1()
        except ImportError:
            pass
        
        if node is not None:
            return VisualizerParamLoader._load_ros2(node)
        
        # 非 ROS 环境，返回默认配置 (包含共享话题回退值)
        config = copy.deepcopy(VISUALIZER_DEFAULTS)
        for topic_key, fallback in SHARED_TOPICS_FALLBACK.items():
            config['topics'][topic_key] = fallback
        # 使用 universal_controller 的默认约束值
        from universal_controller.config import DEFAULT_CONFIG
        config['constraints'] = {
            'v_max': DEFAULT_CONFIG['constraints']['v_max'],
            'omega_max': DEFAULT_CONFIG['constraints']['omega_max'],
        }
        return config
    
    @staticmethod
    def _load_ros1() -> Dict[str, Any]:
        """ROS1 配置加载"""
        import rospy
        
        config = copy.deepcopy(VISUALIZER_DEFAULTS)
        
        # 使用共享的递归加载逻辑，启用私有命名空间
        strategy = get_strategy(node=None, ros_version=1, private_namespace=True)
        load_params_recursive(config, '', strategy)
        
        # 共享话题从全局 topics 配置读取（使用统一策略）
        global_strategy = get_strategy(node=None, ros_version=1, private_namespace=False)
        for topic_key in SHARED_TOPICS:
            fallback = SHARED_TOPICS_FALLBACK.get(topic_key, '')
            config['topics'][topic_key] = global_strategy.get_param(f'topics/{topic_key}', fallback)
        
        # 速度限制从全局 constraints 配置读取（使用统一策略）
        from universal_controller.config import DEFAULT_CONFIG
        constraints_defaults = DEFAULT_CONFIG.get('constraints', {})
        config['constraints'] = {
            'v_max': global_strategy.get_param('constraints/v_max', constraints_defaults.get('v_max', 2.0)),
            'omega_max': global_strategy.get_param('constraints/omega_max', constraints_defaults.get('omega_max', 2.0)),
        }
        
        # 自动启用相机模式
        if config['topics'].get('camera_image'):
            config['camera']['use_camera'] = True
        
        return config
    
    @staticmethod
    def _load_ros2(node) -> Dict[str, Any]:
        """ROS2 配置加载"""
        config = copy.deepcopy(VISUALIZER_DEFAULTS)
        
        # 使用共享的递归加载逻辑
        strategy = get_strategy(node=node, ros_version=2)
        load_params_recursive(config, '', strategy)
        
        # 共享话题从全局 topics 配置读取（使用统一策略）
        for topic_key in SHARED_TOPICS:
            fallback = SHARED_TOPICS_FALLBACK.get(topic_key, '')
            config['topics'][topic_key] = strategy.get_param(f'topics/{topic_key}', fallback)
        
        # 速度限制从全局 constraints 配置读取（使用统一策略）
        from universal_controller.config import DEFAULT_CONFIG
        constraints_defaults = DEFAULT_CONFIG.get('constraints', {})
        config['constraints'] = {
            'v_max': strategy.get_param('constraints/v_max', constraints_defaults.get('v_max', 2.0)),
            'omega_max': strategy.get_param('constraints/omega_max', constraints_defaults.get('omega_max', 2.0)),
        }
        
        # 自动启用相机模式
        if config['topics'].get('camera_image'):
            config['camera']['use_camera'] = True
        
        return config
