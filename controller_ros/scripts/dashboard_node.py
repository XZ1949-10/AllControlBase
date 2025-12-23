#!/usr/bin/env python3
"""
Dashboard ROS 节点 - 可视化监控界面

订阅控制器诊断话题，显示实时状态监控界面。

用法:
    # 独立启动
    rosrun controller_ros dashboard_node.py
    
    # 通过 launch 文件启动
    roslaunch controller_ros controller.launch dashboard:=true
    
    # 单独启动 dashboard
    roslaunch controller_ros dashboard.launch
"""

import sys
import os

# 添加 catkin devel 路径 - 用于导入 controller_ros.msg
_devel_path = '/home/oamr/turtlebot_ws/devel/lib/python3/dist-packages'
if os.path.exists(_devel_path):
    sys.path.insert(0, _devel_path)

# 确保能找到 universal_controller 模块
# 方法1: 如果 universal_controller 在 PYTHONPATH 中
# 方法2: 如果在同一工作空间，添加相对路径
_script_dir = os.path.dirname(os.path.abspath(__file__))
_workspace_root = os.path.dirname(os.path.dirname(os.path.dirname(_script_dir)))
if os.path.exists(os.path.join(_workspace_root, 'universal_controller')):
    sys.path.insert(0, _workspace_root)

import rospy
from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import QTimer


def main():
    """主函数"""
    # 初始化 ROS 节点
    rospy.init_node('controller_dashboard', anonymous=False)
    
    rospy.loginfo("=" * 60)
    rospy.loginfo("  Universal Controller Dashboard (ROS Mode)")
    rospy.loginfo("  版本: v3.17.12")
    rospy.loginfo("=" * 60)
    
    # 检查 PyQt5
    try:
        from PyQt5.QtWidgets import QApplication
    except ImportError:
        rospy.logerr("PyQt5 not installed. Please run: pip install PyQt5")
        return 1

    # 导入 Dashboard 组件
    try:
        from universal_controller.dashboard.main_window import DashboardWindow
        from universal_controller.dashboard.ros_data_source import ROSDashboardDataSource
    except ImportError as e:
        rospy.logerr(f"Failed to import dashboard modules: {e}")
        rospy.logerr("Please ensure universal_controller is in PYTHONPATH")
        return 1
    
    # 从 ROS 参数加载配置
    config = _load_config_from_ros()
    
    rospy.loginfo(f"Platform: {config.get('system', {}).get('platform', 'unknown')}")
    rospy.loginfo(f"Control frequency: {config.get('system', {}).get('ctrl_freq', 50)} Hz")
    rospy.loginfo("")
    rospy.loginfo("Subscribing to:")
    rospy.loginfo("  - /controller/diagnostics (DiagnosticsV2)")
    rospy.loginfo("  - /controller/state (Int32)")
    rospy.loginfo("")
    
    # 创建 Qt 应用
    app = QApplication(sys.argv)
    
    # 创建 ROS 数据源
    data_source = ROSDashboardDataSource(config=config)
    
    # 创建主窗口
    window = DashboardWindow(data_source)
    window.setWindowTitle('Universal Controller Dashboard [ROS Mode]')
    window.show()
    
    rospy.loginfo("Dashboard started!")
    rospy.loginfo("Waiting for controller diagnostics...")
    rospy.loginfo("")
    
    # 设置 ROS 关闭回调
    def shutdown_callback():
        rospy.loginfo("Shutting down dashboard...")
        app.quit()
    
    rospy.on_shutdown(shutdown_callback)
    
    # 创建定时器处理 ROS 回调
    ros_timer = QTimer()
    ros_timer.timeout.connect(_process_ros_callbacks)
    ros_timer.start(10)  # 100 Hz
    
    # 运行 Qt 事件循环
    return app.exec_()


def _process_ros_callbacks():
    """处理 ROS 回调（在 Qt 事件循环中调用）"""
    if rospy.is_shutdown():
        QApplication.instance().quit()
        return
    # rospy 的回调在单独的线程中处理，这里只需检查关闭状态


def _load_config_from_ros() -> dict:
    """从 ROS 参数服务器加载配置"""
    config = {
        'system': {
            'platform': rospy.get_param('system/platform', 'differential'),
            'ctrl_freq': rospy.get_param('system/ctrl_freq', 50),
        },
        'mpc': {
            'horizon': rospy.get_param('mpc/horizon', 20),
            'horizon_degraded': rospy.get_param('mpc/horizon_degraded', 10),
            'dt': rospy.get_param('mpc/dt', 0.1),
        },
        'constraints': {
            'v_max': rospy.get_param('constraints/v_max', 2.0),
            'omega_max': rospy.get_param('constraints/omega_max', 2.0),
            'a_max': rospy.get_param('constraints/a_max', 1.5),
        },
        'watchdog': {
            'odom_timeout_ms': rospy.get_param('watchdog/odom_timeout_ms', 200),
            'traj_timeout_ms': rospy.get_param('watchdog/traj_timeout_ms', 500),
            'imu_timeout_ms': rospy.get_param('watchdog/imu_timeout_ms', 100),
        },
        'attitude': {
            'mass': rospy.get_param('attitude/mass', 1.5),
            'roll_max': rospy.get_param('attitude/roll_max', 0.5),
            'pitch_max': rospy.get_param('attitude/pitch_max', 0.5),
        },
    }
    return config


if __name__ == '__main__':
    try:
        sys.exit(main())
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.loginfo("Dashboard interrupted by user")
