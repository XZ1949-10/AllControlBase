#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
姿态适配器测试

测试 AttitudeAdapter 的功能。
"""
import sys
import os
import pytest

# 添加 src 目录到路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from universal_controller.core.data_types import AttitudeCommand


class MockAttitudeCmd:
    """模拟 AttitudeCmd 消息"""
    def __init__(self):
        self.header = MockHeader()
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.thrust = 1.0
        self.yaw_mode = 0
        self.is_hovering = False


class MockHeader:
    """模拟 Header"""
    def __init__(self):
        self.stamp = MockTime()
        self.frame_id = ''


class MockTime:
    """模拟 ROS Time"""
    def __init__(self, sec=0, nsec=0):
        self.secs = sec
        self.nsecs = nsec
        self.sec = sec
        self.nanosec = nsec
    
    def to_sec(self):
        return self.secs + self.nsecs * 1e-9


class TestAttitudeAdapter:
    """测试 AttitudeAdapter 类"""
    
    def test_to_uc(self):
        """测试 ROS → UC 转换"""
        from controller_ros.adapters import AttitudeAdapter
        
        adapter = AttitudeAdapter()
        
        ros_msg = MockAttitudeCmd()
        ros_msg.roll = 0.1
        ros_msg.pitch = 0.2
        ros_msg.yaw = 0.3
        ros_msg.thrust = 1.5
        
        uc_cmd = adapter.to_uc(ros_msg)
        
        assert uc_cmd.roll == pytest.approx(0.1)
        assert uc_cmd.pitch == pytest.approx(0.2)
        assert uc_cmd.yaw == pytest.approx(0.3)
        assert uc_cmd.thrust == pytest.approx(1.5)
    
    def test_uc_attitude_command(self):
        """测试 UC AttitudeCommand 数据类型"""
        cmd = AttitudeCommand(
            roll=0.1,
            pitch=0.2,
            yaw=0.3,
            thrust=1.0
        )
        
        assert cmd.roll == pytest.approx(0.1)
        assert cmd.pitch == pytest.approx(0.2)
        assert cmd.yaw == pytest.approx(0.3)
        assert cmd.thrust == pytest.approx(1.0)


# 注意: TestParamLoader 测试类已移除
# 这些测试针对旧版 ParamLoader API (_deep_copy, _merge_params)
# 新版 ParamLoader 使用 copy.deepcopy 和 _load_recursive
# 相关测试已移至 test_param_loader.py

# 注意: TestTimeSync 测试类已移除
# TimeSync 已被删除，超时监控统一使用 universal_controller 的 TimeoutMonitor


class TestAttitudeAdapterToRos:
    """测试 AttitudeAdapter to_ros 转换"""
    
    def test_to_ros_basic(self):
        """测试基本 UC → ROS 转换 (需要 ROS 环境)"""
        from controller_ros.adapters import AttitudeAdapter
        
        adapter = AttitudeAdapter()
        
        uc_cmd = AttitudeCommand(
            roll=0.15,
            pitch=0.25,
            yaw=1.57,
            thrust=1.2
        )
        
        try:
            ros_msg = adapter.to_ros(uc_cmd, yaw_mode=0, is_hovering=False)
            
            assert ros_msg.roll == pytest.approx(0.15)
            assert ros_msg.pitch == pytest.approx(0.25)
            assert ros_msg.yaw == pytest.approx(1.57)
            assert ros_msg.thrust == pytest.approx(1.2)
            assert ros_msg.yaw_mode == 0
            assert ros_msg.is_hovering == False
        except ImportError:
            pytest.skip("ROS messages not available")
    
    def test_to_ros_with_yaw_mode(self):
        """测试带航向模式的转换 (需要 ROS 环境)"""
        from controller_ros.adapters import AttitudeAdapter
        
        adapter = AttitudeAdapter()
        
        uc_cmd = AttitudeCommand(roll=0.0, pitch=0.0, yaw=0.0, thrust=1.0)
        
        try:
            for mode in [0, 1, 2, 3]:
                ros_msg = adapter.to_ros(uc_cmd, yaw_mode=mode, is_hovering=False)
                assert ros_msg.yaw_mode == mode
        except ImportError:
            pytest.skip("ROS messages not available")
    
    def test_to_ros_hovering(self):
        """测试悬停状态转换 (需要 ROS 环境)"""
        from controller_ros.adapters import AttitudeAdapter
        
        adapter = AttitudeAdapter()
        
        uc_cmd = AttitudeCommand(roll=0.0, pitch=0.0, yaw=0.0, thrust=1.0)
        
        try:
            ros_msg = adapter.to_ros(uc_cmd, yaw_mode=0, is_hovering=True)
            assert ros_msg.is_hovering == True
            
            ros_msg = adapter.to_ros(uc_cmd, yaw_mode=0, is_hovering=False)
            assert ros_msg.is_hovering == False
        except ImportError:
            pytest.skip("ROS messages not available")


# 注意: TestParamLoader 和 TestParamLoaderMerge 测试类已移除
# 这些测试针对旧版 ParamLoader API (_deep_copy, _merge_params)
# 新版 ParamLoader 使用 copy.deepcopy 和 _load_recursive
# 相关测试已移至 test_param_loader.py


class TestAttitudeCommandDataclass:
    """测试 AttitudeCommand 数据类"""
    
    def test_required_values(self):
        """测试必需参数"""
        cmd = AttitudeCommand(
            roll=0.0,
            pitch=0.0,
            yaw=0.0,
            thrust=0.0
        )
        
        assert cmd.roll == 0.0
        assert cmd.pitch == 0.0
        assert cmd.yaw == 0.0
        assert cmd.thrust == 0.0
    
    def test_custom_values(self):
        """测试自定义值"""
        cmd = AttitudeCommand(
            roll=0.5,
            pitch=-0.3,
            yaw=3.14,
            thrust=0.8
        )
        
        assert cmd.roll == pytest.approx(0.5)
        assert cmd.pitch == pytest.approx(-0.3)
        assert cmd.yaw == pytest.approx(3.14)
        assert cmd.thrust == pytest.approx(0.8)
    
    def test_negative_values(self):
        """测试负值"""
        cmd = AttitudeCommand(
            roll=-0.5,
            pitch=-0.5,
            yaw=-3.14,
            thrust=0.5
        )
        
        assert cmd.roll == pytest.approx(-0.5)
        assert cmd.pitch == pytest.approx(-0.5)
        assert cmd.yaw == pytest.approx(-3.14)


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
