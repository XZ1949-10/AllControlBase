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
