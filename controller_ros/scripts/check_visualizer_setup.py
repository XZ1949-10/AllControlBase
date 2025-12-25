#!/usr/bin/env python3
"""
检查可视化器设置

用于诊断轨迹可视化问题:
1. 检查相机话题是否可用
2. 检查标定文件是否存在
3. 检查 ROS 参数是否正确设置

使用方法:
    rosrun controller_ros check_visualizer_setup.py
"""
import os
import sys

def main():
    print("=" * 60)
    print("可视化器设置检查")
    print("=" * 60)
    
    # 检查 ROS
    try:
        import rospy
        rospy.init_node('visualizer_setup_check', anonymous=True)
        print("\n[1] ROS 环境: ✓ 正常")
    except Exception as e:
        print(f"\n[1] ROS 环境: ✗ 错误 - {e}")
        return 1
    
    # 检查相机话题
    print("\n[2] 检查相机话题...")
    try:
        import rostopic
        topics = rostopic.get_topic_list()[0]
        image_topics = [t for t in topics if 'image' in t.lower()]
        
        if image_topics:
            print(f"    找到 {len(image_topics)} 个图像话题:")
            for t in image_topics[:5]:
                print(f"      - {t}")
            if len(image_topics) > 5:
                print(f"      ... 还有 {len(image_topics) - 5} 个")
        else:
            print("    ✗ 未找到图像话题")
            print("    请确保相机节点已启动")
    except Exception as e:
        print(f"    ✗ 无法获取话题列表: {e}")
    
    # 检查 ROS 参数
    print("\n[3] 检查 ROS 参数...")
    params_to_check = [
        '/turtlebot_visualizer/topics/camera_image',
        '/turtlebot_visualizer/camera/calibration_file',
        'topics/camera_image',
        'camera/calibration_file',
    ]
    
    for param in params_to_check:
        try:
            value = rospy.get_param(param)
            print(f"    {param}: {value}")
        except:
            print(f"    {param}: (未设置)")
    
    # 检查标定文件
    print("\n[4] 检查标定文件...")
    
    # 常见的标定文件位置
    possible_paths = [
        os.path.expanduser('~/homography_calib.yaml'),
        os.path.expanduser('~/.ros/homography_calib.yaml'),
        '/tmp/homography_calib.yaml',
    ]
    
    # 尝试从 rospack 获取包路径
    try:
        import rospkg
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path('controller_ros')
        possible_paths.insert(0, os.path.join(pkg_path, 'config', 'homography_calib.yaml'))
    except:
        pass
    
    found_files = []
    for path in possible_paths:
        if os.path.exists(path):
            found_files.append(path)
            print(f"    ✓ 找到: {path}")
    
    if not found_files:
        print("    ✗ 未找到标定文件")
        print("\n    请运行标定程序:")
        print("    roslaunch controller_ros trajectory_visualizer.launch calibration_mode:=true")
        print("\n    或者手动指定标定文件路径:")
        print("    roslaunch controller_ros turtlebot1.launch calibration_file:=/path/to/your/calib.yaml")
    
    # 检查 cv_bridge
    print("\n[5] 检查依赖...")
    try:
        from cv_bridge import CvBridge
        print("    ✓ cv_bridge 可用")
    except ImportError:
        print("    ✗ cv_bridge 不可用")
        print("    安装: sudo apt install ros-noetic-cv-bridge")
    
    try:
        import cv2
        print(f"    ✓ OpenCV 可用 (版本: {cv2.__version__})")
    except ImportError:
        print("    ✗ OpenCV 不可用")
        print("    安装: pip install opencv-python")
    
    try:
        from PyQt5.QtWidgets import QApplication
        print("    ✓ PyQt5 可用")
    except ImportError:
        print("    ✗ PyQt5 不可用")
        print("    安装: pip install PyQt5")
    
    print("\n" + "=" * 60)
    print("检查完成")
    print("=" * 60)
    
    return 0


if __name__ == '__main__':
    sys.exit(main())
