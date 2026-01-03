"""
通用控制器主入口

此模块提供命令行入口点。

用法:
    python -m universal_controller.main --help
    python -m universal_controller.main demo      # 运行演示
    python -m universal_controller.main dashboard # 运行 Dashboard (测试模式)

生产环境使用:
    from universal_controller import ControllerManager, DEFAULT_CONFIG
    
    config = DEFAULT_CONFIG.copy()
    manager = ControllerManager(config)
    manager.initialize_default_components()
    
    # 在控制循环中使用真实的 odom 和 trajectory 数据
    # data_ages 应包含数据的年龄 (秒)
    data_ages = {'odom': 0.01, 'trajectory': 0.1, 'imu': 0.005}
    cmd = manager.update(real_odom, real_trajectory, data_ages)
"""
import sys
import argparse
import math
import time


def _create_demo_trajectory(num_points: int = 20, dt: float = 0.1):
    """
    创建演示用轨迹数据 (内联实现，不依赖测试模块)
    
    生成一条简单的正弦曲线轨迹用于演示。
    """
    import numpy as np
    from .core.data_types import Trajectory, Header, Point3D
    from .core.enums import TrajectoryMode
    
    points = []
    for i in range(num_points):
        t = i * dt
        x = t * 0.5  # 向前移动
        y = 0.3 * np.sin(t * 2)  # 正弦波动
        points.append(Point3D(x=x, y=y, z=0.0))
    
    return Trajectory(
        header=Header(stamp=time.time(), frame_id='base_link'),
        points=points,
        velocities=None,
        dt_sec=dt,
        confidence=1.0,
        mode=TrajectoryMode.MODE_TRACK,
        soft_enabled=False
    )


def _create_demo_odom(x: float, y: float, z: float, theta: float, 
                      vx: float, vy: float = 0.0):
    """
    创建演示用里程计数据 (内联实现，不依赖测试模块)
    """
    from .core.data_types import Odometry, Header, Point3D
    
    # 从欧拉角计算四元数 (仅 yaw)
    qz = math.sin(theta / 2)
    qw = math.cos(theta / 2)
    
    return Odometry(
        header=Header(stamp=time.time(), frame_id='odom'),
        pose_position=Point3D(x=x, y=y, z=z),
        pose_orientation=(0.0, 0.0, qz, qw),
        twist_linear=(vx, vy, 0.0),
        twist_angular=(0.0, 0.0, 0.0)
    )


def run_demo():
    """运行演示脚本"""
    import numpy as np
    
    from .config.default_config import DEFAULT_CONFIG
    from .manager.controller_manager import ControllerManager
    
    from . import __version__
    print("=" * 60)
    print(f"通用控制器 (Universal Controller) v{__version__}")
    print("=" * 60)
    print()
    print("[!] 注意: 这是演示模式，使用模拟数据")
    print("    生产环境请使用真实的传感器数据")
    print()
    
    # 创建控制器管理器
    config = DEFAULT_CONFIG.copy()
    config['system']['platform'] = 'differential'
    
    manager = ControllerManager(config)
    manager.initialize_default_components()
    
    print(f"平台类型: {config['system']['platform']}")
    print(f"控制频率: {config['system']['ctrl_freq']} Hz")
    print(f"MPC Horizon: {config['mpc']['horizon']}")
    
    # 创建演示轨迹 (使用内联函数，不依赖测试模块)
    trajectory = _create_demo_trajectory(num_points=20, dt=0.1)
    print(f"\n轨迹点数: {len(trajectory.points)}")
    
    # 模拟控制循环
    print("\n开始控制循环...")
    print("-" * 60)
    
    x, y, theta = 0.0, 0.0, 0.0
    vx, vy = 0.0, 0.0
    
    for i in range(50):
        odom = _create_demo_odom(x, y, 0.0, theta, vx, vy)
        # 演示模式假设数据新鲜
        data_ages = {'odom': 0.0, 'trajectory': 0.0, 'imu': 0.0}
        cmd = manager.update(odom, trajectory, data_ages)
        
        dt = 0.02
        if manager.platform_config.get('velocity_heading_coupled', True):
            x += cmd.vx * np.cos(theta) * dt
            y += cmd.vx * np.sin(theta) * dt
            theta += cmd.omega * dt
            vx = cmd.vx
        else:
            x += cmd.vx * dt
            y += cmd.vy * dt
            theta += cmd.omega * dt
            vx = cmd.vx
            vy = cmd.vy
        
        if i % 10 == 0:
            state = manager.get_state()
            print(f"Step {i:3d}: pos=({x:.2f}, {y:.2f}), theta={np.degrees(theta):.1f}°, "
                  f"v={cmd.vx:.2f}, omega={cmd.omega:.2f}, state={state.name}")
    
    print("-" * 60)
    print("\n控制循环完成")
    
    diag = manager.get_diagnostics()
    print(f"\n最终状态: {manager.get_state().name}")
    print(f"MPC 成功: {diag.get('mpc_success', False)}")
    
    manager.shutdown()
    print("\n控制器已关闭")


def run_dashboard():
    """运行 Dashboard (测试模式)"""
    print("启动 Dashboard (测试模式)...")
    print("请使用: python -m universal_controller.tests.run_dashboard_mock")
    print()
    print("或者在 ROS 环境中使用:")
    print("  roslaunch controller_ros core/controller.launch dashboard:=true")
    sys.exit(1)


def main():
    """主入口"""
    parser = argparse.ArgumentParser(
        description='Universal Controller - 通用轨迹跟踪控制器',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  python -m universal_controller.main demo       # 运行演示
  python -m universal_controller.main dashboard  # 运行 Dashboard 说明

生产环境使用:
  from universal_controller import ControllerManager, DEFAULT_CONFIG
  manager = ControllerManager(DEFAULT_CONFIG.copy())
  manager.initialize_default_components()
  manager.initialize_default_components()
  data_ages = {'odom': 0.0, 'trajectory': 0.0, 'imu': 0.0}
  cmd = manager.update(odom, trajectory, data_ages)
        """
    )
    
    parser.add_argument(
        'command',
        nargs='?',
        default='demo',
        choices=['demo', 'dashboard'],
        help='要执行的命令 (默认: demo)'
    )
    
    args = parser.parse_args()
    
    if args.command == 'demo':
        run_demo()
    elif args.command == 'dashboard':
        run_dashboard()


if __name__ == "__main__":
    main()
