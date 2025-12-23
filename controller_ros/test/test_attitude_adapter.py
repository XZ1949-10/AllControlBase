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


class TestParamLoader:
    """测试 ParamLoader 类"""
    
    def test_deep_copy(self):
        """测试深拷贝"""
        from controller_ros.utils.param_loader import ParamLoader
        
        original = {'a': {'b': 1}}
        copied = ParamLoader._deep_copy(original)
        
        copied['a']['b'] = 2
        assert original['a']['b'] == 1
    
    def test_merge_params(self):
        """测试参数合并"""
        from controller_ros.utils.param_loader import ParamLoader
        
        config = {
            'system': {'ctrl_freq': 50, 'platform': 'differential'},
            'watchdog': {'odom_timeout_ms': 100},
        }
        
        ros_params = {
            'system': {'ctrl_freq': 100, 'platform': 'quadrotor'},
            'watchdog': {'odom_timeout_ms': 200, 'traj_timeout_ms': 500},
            'mpc': {'horizon': 30},
        }
        
        ParamLoader._merge_params(config, ros_params)
        
        assert config['system']['ctrl_freq'] == 100
        assert config['system']['platform'] == 'quadrotor'
        assert config['watchdog']['odom_timeout_ms'] == 200
        assert config['watchdog']['traj_timeout_ms'] == 500
        assert config['mpc']['horizon'] == 30


class TestTimeSync:
    """测试 TimeSync 类"""
    
    def test_check_freshness(self):
        """测试数据新鲜度检查"""
        from controller_ros.utils import TimeSync
        
        sync = TimeSync(
            max_odom_age_ms=100,
            max_traj_age_ms=200,
            max_imu_age_ms=50
        )
        
        # 所有数据新鲜
        ages = {'odom': 0.05, 'trajectory': 0.1, 'imu': 0.02}
        timeouts = sync.check_freshness(ages)
        
        assert timeouts['odom_timeout'] == False
        assert timeouts['traj_timeout'] == False
        assert timeouts['imu_timeout'] == False
        
        # odom 超时
        ages = {'odom': 0.15, 'trajectory': 0.1, 'imu': 0.02}
        timeouts = sync.check_freshness(ages)
        
        assert timeouts['odom_timeout'] == True
        assert timeouts['traj_timeout'] == False
        assert timeouts['imu_timeout'] == False
    
    def test_is_all_fresh(self):
        """测试所有数据新鲜度检查"""
        from controller_ros.utils import TimeSync
        
        sync = TimeSync(max_odom_age_ms=100, max_traj_age_ms=200)
        
        # 所有必需数据新鲜
        ages = {'odom': 0.05, 'trajectory': 0.1}
        assert sync.is_all_fresh(ages) == True
        
        # odom 超时
        ages = {'odom': 0.15, 'trajectory': 0.1}
        assert sync.is_all_fresh(ages) == False


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


class TestParamLoaderMerge:
    """测试 ParamLoader 合并逻辑"""
    
    def test_merge_existing_keys(self):
        """测试合并已存在的键"""
        from controller_ros.utils.param_loader import ParamLoader
        
        config = {
            'system': {'ctrl_freq': 50, 'platform': 'differential'},
        }
        
        ros_params = {
            'system': {'ctrl_freq': 100},  # 覆盖已存在的键
        }
        
        ParamLoader._merge_params(config, ros_params)
        
        assert config['system']['ctrl_freq'] == 100
        assert config['system']['platform'] == 'differential'  # 保留
    
    def test_merge_empty_ros_params(self):
        """测试空 ROS 参数合并"""
        from controller_ros.utils.param_loader import ParamLoader
        
        config = {'system': {'ctrl_freq': 50}}
        ros_params = {}
        
        ParamLoader._merge_params(config, ros_params)
        
        assert config['system']['ctrl_freq'] == 50
    
    def test_merge_adds_to_existing_section(self):
        """测试向已存在的节添加新键"""
        from controller_ros.utils.param_loader import ParamLoader
        
        config = {
            'watchdog': {'odom_timeout_ms': 100},
        }
        
        ros_params = {
            'watchdog': {'odom_timeout_ms': 200, 'traj_timeout_ms': 500},
        }
        
        ParamLoader._merge_params(config, ros_params)
        
        assert config['watchdog']['odom_timeout_ms'] == 200
        assert config['watchdog']['traj_timeout_ms'] == 500


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
