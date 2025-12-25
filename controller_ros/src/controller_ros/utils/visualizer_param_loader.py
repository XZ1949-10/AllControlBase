"""
Visualizer 参数加载器

从 ROS 参数服务器加载 visualizer 配置。
设计与 ParamLoader 保持一致，但针对 visualizer 的配置结构。

配置加载优先级:
1. 私有参数 (~param) - 最高优先级
2. 全局参数 (param) - 次优先级  
3. 默认值 - 最低优先级
"""
from typing import Dict, Any, Optional
import logging

logger = logging.getLogger(__name__)


# =============================================================================
# Visualizer 默认配置
# =============================================================================
VISUALIZER_DEFAULTS = {
    'topics': {
        'odom': '/odom',
        'trajectory': '/nn/local_trajectory',
        'cmd_unified': '/cmd_unified',
        'diagnostics': '/controller/diagnostics',
        'camera_image': '',
        'joy': '/joy',
        'cmd_vel_output': '/joy_cmd_vel',
        'control_mode': '/visualizer/control_mode',
        'emergency_stop': '/controller/emergency_stop',
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
        'max_linear': 0.5,
        'max_angular': 1.0,
        'deadzone': 0.1,
    },
    'constraints': {
        'v_max': 0.5,
        'omega_max': 1.0,
    },
    'camera': {
        'use_camera': False,
        'calibration_file': '',
        'fx': 500.0,
        'fy': 500.0,
        'cx': 320.0,
        'cy': 240.0,
    },
}


class VisualizerParamLoader:
    """
    Visualizer 参数加载器
    
    统一的配置加载接口，支持 ROS1 和 ROS2。
    
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
        
        # 非 ROS 环境，返回默认配置
        import copy
        return copy.deepcopy(VISUALIZER_DEFAULTS)
    
    @staticmethod
    def _load_ros1() -> Dict[str, Any]:
        """ROS1 配置加载"""
        import rospy
        import copy
        
        config = copy.deepcopy(VISUALIZER_DEFAULTS)
        
        # 递归加载参数
        VisualizerParamLoader._load_recursive_ros1(config, '')
        
        # 自动启用相机模式
        if config['topics']['camera_image']:
            config['camera']['use_camera'] = True
        
        return config
    
    @staticmethod
    def _load_recursive_ros1(config: Dict[str, Any], prefix: str) -> None:
        """递归加载 ROS1 参数"""
        import rospy
        
        for key, value in config.items():
            param_path = f"{prefix}/{key}" if prefix else key
            
            if isinstance(value, dict):
                VisualizerParamLoader._load_recursive_ros1(value, param_path)
            else:
                # 优先级: 私有参数 > 全局参数 > 默认值
                private_path = f"~{param_path}"
                
                if rospy.has_param(private_path):
                    config[key] = rospy.get_param(private_path)
                elif rospy.has_param(param_path):
                    config[key] = rospy.get_param(param_path)
                # 否则保留默认值
    
    @staticmethod
    def _load_ros2(node) -> Dict[str, Any]:
        """ROS2 配置加载"""
        import copy
        
        config = copy.deepcopy(VISUALIZER_DEFAULTS)
        
        # 递归加载参数
        VisualizerParamLoader._load_recursive_ros2(config, '', node)
        
        # 自动启用相机模式
        if config['topics']['camera_image']:
            config['camera']['use_camera'] = True
        
        return config
    
    @staticmethod
    def _load_recursive_ros2(config: Dict[str, Any], prefix: str, node) -> None:
        """递归加载 ROS2 参数"""
        for key, value in config.items():
            param_path = f"{prefix}.{key}" if prefix else key
            
            if isinstance(value, dict):
                VisualizerParamLoader._load_recursive_ros2(value, param_path, node)
            else:
                try:
                    if not node.has_parameter(param_path):
                        node.declare_parameter(param_path, value)
                    config[key] = node.get_parameter(param_path).value
                except Exception:
                    pass  # 保留默认值
