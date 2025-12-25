#!/usr/bin/env python3
"""
TurtleBot1 运行可视化节点启动脚本

功能:
1. 轨迹可视化 - 在俯视图上显示网络输出的轨迹
2. 速度监控 - 实时显示底盘线速度和角速度
3. 手柄控制 - Xbox 手柄控制机器人，LB 键切换控制模式

使用方法 (ROS1):
    rosrun controller_ros visualizer_node.py

使用方法 (ROS2):
    ros2 run controller_ros visualizer_node.py
"""
import sys
import os

from controller_ros.visualizer import create_visualizer_node


def load_config_from_ros():
    """从 ROS 参数服务器加载配置
    
    使用统一的 VisualizerParamLoader 加载配置，
    支持私有参数和全局参数的优先级回退。
    """
    try:
        import rospy
        
        # 必须先初始化节点才能读取参数
        if not rospy.core.is_initialized():
            rospy.init_node('turtlebot_visualizer', anonymous=False)
        
        # 使用统一的参数加载器
        from controller_ros.utils import VisualizerParamLoader
        config = VisualizerParamLoader.load()
        
        # 日志输出
        if config['topics']['camera_image']:
            rospy.loginfo(f"[Visualizer] Camera topic: {config['topics']['camera_image']}")
            rospy.loginfo(f"[Visualizer] Calibration file: {config['camera']['calibration_file']}")
            
            # 检查标定文件是否存在
            calib_file = config['camera']['calibration_file']
            if calib_file:
                if os.path.exists(calib_file):
                    rospy.loginfo(f"[Visualizer] Calibration file found: {calib_file}")
                else:
                    rospy.logwarn(f"[Visualizer] Calibration file NOT found: {calib_file}")
                    rospy.logwarn("[Visualizer] Trajectory will be shown in bird's eye view mode")
            else:
                rospy.logwarn("[Visualizer] No calibration file specified, using bird's eye view mode")
        
        return config
        
    except ImportError:
        # ROS2 或非 ROS 环境
        return None


def main(args=None):
    """主入口"""
    try:
        # 从 ROS 参数加载配置
        config = load_config_from_ros()
        
        # 创建节点
        node_class = create_visualizer_node()
        node = node_class(config=config)
        
        # 启动 GUI (阻塞，直到窗口关闭)
        exit_code = node.start_gui()
        
    except KeyboardInterrupt:
        exit_code = 0
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        exit_code = 1
    
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
