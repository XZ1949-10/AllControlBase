#!/usr/bin/env python3
"""
Dashboard 诊断脚本

用于排查 Dashboard 数据显示问题。

用法:
    rosrun controller_ros diagnose_dashboard.py
"""

import sys
import time

def main():
    print("=" * 60)
    print("  Dashboard 诊断工具")
    print("=" * 60)
    print()
    
    # 检查 ROS
    print("[1/5] 检查 ROS 环境...")
    try:
        import rospy
        rospy.init_node('dashboard_diagnose', anonymous=True)
        print("  ✓ ROS 环境正常")
    except ImportError:
        print("  ✗ 无法导入 rospy")
        print("    请确保已 source ROS 环境: source /opt/ros/noetic/setup.bash")
        return 1
    except Exception as e:
        print(f"  ✗ ROS 初始化失败: {e}")
        return 1
    
    # 检查消息类型
    print()
    print("[2/5] 检查消息类型...")
    try:
        from controller_ros.msg import DiagnosticsV2
        print("  ✓ DiagnosticsV2 消息类型可用")
    except ImportError as e:
        print(f"  ✗ 无法导入 DiagnosticsV2: {e}")
        print("    请确保已编译 controller_ros 包:")
        print("      cd ~/turtlebot_ws && catkin_make")
        print("      source devel/setup.bash")
        return 1
    
    # 检查话题
    print()
    print("[3/5] 检查话题...")
    topics = rospy.get_published_topics()
    topic_dict = {t[0]: t[1] for t in topics}
    
    diag_topic = '/controller/diagnostics'
    state_topic = '/controller/state'
    
    if diag_topic in topic_dict:
        print(f"  ✓ {diag_topic} 存在 (类型: {topic_dict[diag_topic]})")
    else:
        print(f"  ✗ {diag_topic} 不存在")
        print("    控制器节点可能未运行")
        print("    请启动: roslaunch controller_ros turtlebot1.launch")
    
    if state_topic in topic_dict:
        print(f"  ✓ {state_topic} 存在 (类型: {topic_dict[state_topic]})")
    else:
        print(f"  ✗ {state_topic} 不存在")
    
    # 检查诊断数据
    print()
    print("[4/5] 等待诊断数据 (5秒)...")
    
    received_data = {'diag': None, 'state': None}
    
    def diag_callback(msg):
        received_data['diag'] = msg
    
    def state_callback(msg):
        received_data['state'] = msg
    
    try:
        from controller_ros.msg import DiagnosticsV2
        from std_msgs.msg import Int32
        
        diag_sub = rospy.Subscriber(diag_topic, DiagnosticsV2, diag_callback)
        state_sub = rospy.Subscriber(state_topic, Int32, state_callback)
        
        start_time = time.time()
        while time.time() - start_time < 5.0 and not rospy.is_shutdown():
            if received_data['diag'] is not None:
                break
            rospy.sleep(0.1)
        
        if received_data['diag'] is not None:
            msg = received_data['diag']
            print(f"  ✓ 收到诊断数据!")
            print(f"    状态: {msg.state}")
            print(f"    MPC 成功: {msg.mpc_success}")
            print(f"    MPC 求解时间: {msg.mpc_solve_time_ms:.2f} ms")
            print(f"    备用控制器: {msg.backup_active}")
            print(f"    里程计超时: {msg.timeout_odom}")
            print(f"    轨迹超时: {msg.timeout_traj}")
            print(f"    紧急停止: {msg.emergency_stop}")
        else:
            print("  ✗ 未收到诊断数据")
            print("    可能原因:")
            print("    1. 控制器节点未运行")
            print("    2. 诊断发布频率过低")
            print("    3. 网络问题")
        
        if received_data['state'] is not None:
            print(f"  ✓ 收到状态数据: {received_data['state'].data}")
        else:
            print("  ✗ 未收到状态数据")
        
    except Exception as e:
        print(f"  ✗ 订阅失败: {e}")
    
    # 检查 universal_controller
    print()
    print("[5/5] 检查 universal_controller...")
    try:
        from universal_controller.dashboard.ros_data_source import ROSDashboardDataSource
        print("  ✓ ROSDashboardDataSource 可用")
    except ImportError as e:
        print(f"  ✗ 无法导入 ROSDashboardDataSource: {e}")
        print("    请确保已安装 universal_controller:")
        print("      pip install -e /path/to/universal_controller")
    
    print()
    print("=" * 60)
    print("  诊断完成")
    print("=" * 60)
    
    if received_data['diag'] is None:
        print()
        print("建议操作:")
        print("  1. 确保控制器节点正在运行:")
        print("     roslaunch controller_ros turtlebot1.launch")
        print()
        print("  2. 检查话题列表:")
        print("     rostopic list | grep controller")
        print()
        print("  3. 手动查看诊断数据:")
        print("     rostopic echo /controller/diagnostics")
    
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n诊断中断")
