"""
可视化器启动文件

启动 TurtleBot1 运行可视化界面。

使用方法:
    ros2 launch controller_ros tools/visualizer.launch.py
    
    # 使用自定义配置
    ros2 launch controller_ros tools/visualizer.launch.py config_file:=/path/to/config.yaml
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 获取包路径
    pkg_dir = get_package_share_directory('controller_ros')
    
    # 声明参数
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_dir, 'config', 'tools', 'visualizer_params.yaml'),
        description='Path to visualizer configuration file'
    )
    
    # 可视化节点
    visualizer_node = Node(
        package='controller_ros',
        executable='visualizer_node.py',
        name='turtlebot_visualizer',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
    )
    
    return LaunchDescription([
        config_file_arg,
        visualizer_node,
    ])
