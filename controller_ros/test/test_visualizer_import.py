#!/usr/bin/env python3
"""
可视化模块导入测试

验证可视化模块的所有组件可以正常导入。
"""
import sys
import os

# 添加路径
script_dir = os.path.dirname(os.path.abspath(__file__))
src_dir = os.path.join(os.path.dirname(script_dir), 'src')
if src_dir not in sys.path:
    sys.path.insert(0, src_dir)


def test_models():
    """测试数据模型导入"""
    from controller_ros.visualizer.models import (
        ControlMode, TrajectoryMode, Point2D, Point3D,
        VelocityData, TrajectoryData, RobotPose,
        JoystickState, ControllerStatus, VisualizerData
    )
    
    # 创建测试实例
    vel = VelocityData(linear_x=0.5, angular_z=0.1)
    assert vel.linear_x == 0.5
    
    traj = TrajectoryData()
    assert traj.num_points == 0
    assert traj.mode_name == 'TRACK'
    
    joy = JoystickState(left_y=0.8)
    assert joy.linear_cmd == 0.8
    
    print("✓ models OK")


def test_adapters():
    """测试适配器导入"""
    from controller_ros.visualizer.adapters import (
        JoyAdapter, ImageAdapter, VelocityAdapter
    )
    
    # 创建测试实例
    joy_adapter = JoyAdapter({'enable_button': 4})
    assert joy_adapter.enable_button_index == 4
    
    img_adapter = ImageAdapter()
    vel_adapter = VelocityAdapter()
    
    print("✓ adapters OK")


def test_handlers():
    """测试处理器导入"""
    from controller_ros.visualizer.handlers import (
        JoystickHandler, DataAggregator, JoystickConfig
    )
    
    # 测试直接构造
    config = JoystickConfig(max_linear=0.5)
    handler = JoystickHandler(config)
    assert handler.config.max_linear == 0.5
    
    # 测试 from_dict 方法 (速度限制从 constraints 读取)
    joy_config = {'enable_button': 4, 'deadzone': 0.15}
    constraints_config = {'v_max': 0.8, 'omega_max': 1.5}
    config2 = JoystickConfig.from_dict(joy_config, constraints_config)
    assert config2.enable_button == 4
    assert config2.deadzone == 0.15
    assert config2.max_linear == 0.8  # 从 constraints.v_max 读取
    assert config2.max_angular == 1.5  # 从 constraints.omega_max 读取
    
    aggregator = DataAggregator()
    data = aggregator.get_data()
    assert data.control_mode.value == 0  # NETWORK
    
    print("✓ handlers OK")


def test_ros_bridge():
    """测试 ROS 桥接层导入"""
    try:
        from controller_ros.visualizer.node.ros_bridge import (
            ROS_VERSION, ROSBridgeBase
        )
        print(f"  ROS_VERSION: {ROS_VERSION}")
        print("✓ ros_bridge OK")
    except ImportError as e:
        print(f"⚠ ros_bridge skipped (ROS not available): {e}")


def test_widgets_import():
    """测试 UI 组件导入 (不创建实例，避免需要 Qt)"""
    # 只测试导入，不创建实例
    try:
        from controller_ros.visualizer.widgets import (
            TrajectoryView, VelocityPanel, VelocityPlot,
            JoystickPanel, VisualizerStatusBar
        )
        print("✓ widgets import OK")
    except ImportError as e:
        print(f"⚠ widgets import skipped (PyQt5 not available): {e}")


def main():
    """运行所有测试"""
    print("=" * 50)
    print("可视化模块导入测试")
    print("=" * 50)
    
    test_models()
    test_adapters()
    test_handlers()
    test_ros_bridge()
    test_widgets_import()
    
    print("=" * 50)
    print("所有测试通过!")
    print("=" * 50)


if __name__ == '__main__':
    main()
