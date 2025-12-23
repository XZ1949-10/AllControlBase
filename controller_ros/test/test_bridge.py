"""
桥接层单元测试
"""
import pytest
import sys
import os
import time

# 添加 src 目录到路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from universal_controller.config.default_config import DEFAULT_CONFIG
from universal_controller.core.data_types import Odometry, Trajectory, Header, Point3D
from universal_controller.core.enums import ControllerState
from universal_controller.mock import create_test_odom, create_test_trajectory


def test_controller_bridge_initialization():
    """测试控制器桥接初始化"""
    from controller_ros.bridge.controller_bridge import ControllerBridge
    
    config = DEFAULT_CONFIG.copy()
    bridge = ControllerBridge(config)
    
    assert bridge.is_initialized
    assert bridge.get_state() == ControllerState.INIT
    
    bridge.shutdown()
    assert not bridge.is_initialized


def test_controller_bridge_update():
    """测试控制器桥接更新"""
    from controller_ros.bridge.controller_bridge import ControllerBridge
    
    config = DEFAULT_CONFIG.copy()
    bridge = ControllerBridge(config)
    
    odom = create_test_odom(vx=1.0)
    trajectory = create_test_trajectory()
    
    cmd = bridge.update(odom, trajectory)
    
    assert cmd is not None
    assert hasattr(cmd, 'vx')
    assert hasattr(cmd, 'omega')
    
    bridge.shutdown()


def test_controller_bridge_reset():
    """测试控制器桥接重置"""
    from controller_ros.bridge.controller_bridge import ControllerBridge
    
    config = DEFAULT_CONFIG.copy()
    bridge = ControllerBridge(config)
    
    # 运行几次更新
    odom = create_test_odom(vx=1.0)
    trajectory = create_test_trajectory()
    
    for _ in range(5):
        bridge.update(odom, trajectory)
    
    # 重置
    bridge.reset()
    assert bridge.get_state() == ControllerState.INIT
    
    bridge.shutdown()


def test_controller_bridge_diagnostics():
    """测试控制器桥接诊断"""
    from controller_ros.bridge.controller_bridge import ControllerBridge
    
    config = DEFAULT_CONFIG.copy()
    bridge = ControllerBridge(config)
    
    odom = create_test_odom(vx=1.0)
    trajectory = create_test_trajectory()
    
    bridge.update(odom, trajectory)
    
    diag = bridge.get_diagnostics()
    assert diag is not None
    
    bridge.shutdown()


def test_controller_bridge_diagnostics_callback():
    """测试控制器桥接诊断回调"""
    from controller_ros.bridge.controller_bridge import ControllerBridge
    
    config = DEFAULT_CONFIG.copy()
    bridge = ControllerBridge(config)
    
    callback_data = []
    
    def on_diagnostics(diag):
        callback_data.append(diag)
    
    bridge.set_diagnostics_callback(on_diagnostics)
    
    odom = create_test_odom(vx=1.0)
    trajectory = create_test_trajectory()
    
    for _ in range(5):
        bridge.update(odom, trajectory)
    
    assert len(callback_data) >= 5
    
    bridge.shutdown()


def test_controller_bridge_platform_types():
    """测试不同平台类型"""
    from controller_ros.bridge.controller_bridge import ControllerBridge
    
    platforms = ['differential', 'omni', 'ackermann', 'quadrotor']
    
    for platform in platforms:
        config = DEFAULT_CONFIG.copy()
        config['system'] = DEFAULT_CONFIG['system'].copy()
        config['system']['platform'] = platform
        
        bridge = ControllerBridge(config)
        
        odom = create_test_odom(vx=1.0)
        trajectory = create_test_trajectory()
        
        cmd = bridge.update(odom, trajectory)
        assert cmd is not None
        
        bridge.shutdown()
        print(f"✓ Platform {platform} works")


if __name__ == '__main__':
    pytest.main([__file__, '-v'])

def test_controller_bridge_request_stop():
    """测试控制器桥接请求停止功能"""
    from controller_ros.bridge.controller_bridge import ControllerBridge
    
    config = DEFAULT_CONFIG.copy()
    bridge = ControllerBridge(config)
    
    # 先运行几次更新，让状态机进入 NORMAL 状态
    odom = create_test_odom(vx=1.0)
    trajectory = create_test_trajectory()
    
    for _ in range(5):
        bridge.update(odom, trajectory)
    
    # 请求停止
    success = bridge.request_stop()
    assert success, "request_stop should return True"
    
    # 新方案：状态转换发生在下一次 update() 时
    # 再执行一次 update，状态机会处理停止请求
    bridge.update(odom, trajectory)
    
    # 验证状态已变为 STOPPING
    assert bridge.get_state() == ControllerState.STOPPING, \
        f"State should be STOPPING, got {bridge.get_state()}"
    
    bridge.shutdown()
    print("✓ test_controller_bridge_request_stop passed")


def test_controller_bridge_request_stop_not_initialized():
    """测试未初始化时请求停止"""
    from controller_ros.bridge.controller_bridge import ControllerBridge
    
    config = DEFAULT_CONFIG.copy()
    bridge = ControllerBridge(config)
    bridge.shutdown()  # 关闭后再测试
    
    success = bridge.request_stop()
    assert not success, "request_stop should return False when not initialized"
    
    print("✓ test_controller_bridge_request_stop_not_initialized passed")
