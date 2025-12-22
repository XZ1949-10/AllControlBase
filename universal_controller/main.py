"""
通用控制器主入口 - 演示脚本

这是一个演示脚本，展示如何使用 Universal Controller。
使用模拟数据进行测试，不适用于生产环境。

生产环境使用示例:
    from universal_controller import ControllerManager, DEFAULT_CONFIG
    
    config = DEFAULT_CONFIG.copy()
    manager = ControllerManager(config)
    manager.initialize_default_components()
    
    # 在控制循环中使用真实的 odom 和 trajectory 数据
    cmd = manager.update(real_odom, real_trajectory)

演示用法:
    python -m universal_controller.main
"""
import time
import numpy as np

from .config.default_config import DEFAULT_CONFIG
from .manager.controller_manager import ControllerManager
from .mock.test_data_generator import create_test_trajectory, create_test_odom


def main():
    """主函数"""
    print("=" * 60)
    print("通用控制器 (Universal Controller) v3.17.8")
    print("=" * 60)
    
    # 创建控制器管理器
    config = DEFAULT_CONFIG.copy()
    config['system']['platform'] = 'differential'  # 差速车
    
    manager = ControllerManager(config)
    manager.initialize_default_components()
    
    print(f"\n平台类型: {config['system']['platform']}")
    print(f"控制频率: {config['system']['ctrl_freq']} Hz")
    print(f"MPC Horizon: {config['mpc']['horizon']}")
    
    # 创建测试数据
    trajectory = create_test_trajectory(trajectory_type='sine')
    print(f"\n轨迹点数: {len(trajectory.points)}")
    
    # 模拟控制循环
    print("\n开始控制循环...")
    print("-" * 60)
    
    x, y, theta = 0.0, 0.0, 0.0
    vx, vy = 0.0, 0.0
    
    for i in range(50):
        # 创建当前状态
        odom = create_test_odom(x, y, 0.0, theta, vx, vy)
        
        # 更新控制器
        cmd = manager.update(odom, trajectory)
        
        # 简单的运动学积分
        dt = 0.02
        if manager.platform_config.get('velocity_heading_coupled', True):
            # 差速车
            x += cmd.vx * np.cos(theta) * dt
            y += cmd.vx * np.sin(theta) * dt
            theta += cmd.omega * dt
            vx = cmd.vx
        else:
            # 全向车
            x += cmd.vx * dt
            y += cmd.vy * dt
            theta += cmd.omega * dt
            vx = cmd.vx
            vy = cmd.vy
        
        # 打印状态
        if i % 10 == 0:
            state = manager.get_state()
            print(f"Step {i:3d}: pos=({x:.2f}, {y:.2f}), theta={np.degrees(theta):.1f}°, "
                  f"v={cmd.vx:.2f}, omega={cmd.omega:.2f}, state={state.name}")
    
    print("-" * 60)
    print("\n控制循环完成")
    
    # 获取最终诊断
    diag = manager.get_diagnostics()
    print(f"\n最终状态: {manager.get_state().name}")
    print(f"MPC 成功: {diag.get('mpc_success', False)}")
    
    # 清理
    manager.shutdown()
    print("\n控制器已关闭")


if __name__ == "__main__":
    main()
