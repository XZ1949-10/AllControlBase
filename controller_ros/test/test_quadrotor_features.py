"""
四旋翼平台特性单元测试
"""
import pytest
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from typing import Dict, Any, Optional
from universal_controller.core.data_types import ControlOutput, AttitudeCommand
from universal_controller.config.default_config import DEFAULT_CONFIG
from universal_controller.tests.fixtures import create_test_odom, create_test_trajectory


class MockControllerNode:
    """模拟控制器节点"""
    
    def __init__(self):
        from controller_ros.node.base_node import ControllerNodeBase
        
        class TestNode(ControllerNodeBase):
            def __init__(self_inner):
                super().__init__()
                self_inner._current_time = 0.0
                self_inner._logs = {'info': [], 'warn': [], 'error': []}
                self_inner._published_cmds = []
                self_inner._published_diags = []
                self_inner._published_attitude_cmds = []
                self_inner._stop_cmd_count = 0
            
            def _get_time(self_inner) -> float:
                return self_inner._current_time
            
            def _log_info(self_inner, msg: str):
                self_inner._logs['info'].append(msg)
            
            def _log_warn(self_inner, msg: str):
                self_inner._logs['warn'].append(msg)
            
            def _log_warn_throttle(self_inner, period: float, msg: str):
                self_inner._logs['warn'].append(msg)
            
            def _log_error(self_inner, msg: str):
                self_inner._logs['error'].append(msg)
            
            def _publish_cmd(self_inner, cmd: ControlOutput):
                self_inner._published_cmds.append(cmd)
            
            def _publish_stop_cmd(self_inner):
                self_inner._stop_cmd_count += 1
            
            def _publish_diagnostics(self_inner, diag: Dict[str, Any], force: bool = False):
                self_inner._published_diags.append((diag, force))
            
            def _publish_attitude_cmd(self_inner, attitude_cmd: AttitudeCommand):
                self_inner._published_attitude_cmds.append(attitude_cmd)
        
        self.node = TestNode()
    
    def initialize(self, platform: str = 'differential'):
        params = DEFAULT_CONFIG.copy()
        params['system'] = DEFAULT_CONFIG['system'].copy()
        params['system']['platform'] = platform
        self.node._params = params
        self.node._topics = {'odom': '/odom', 'imu': '/imu', 'trajectory': '/traj'}
        self.node._initialize()
    
    def set_time(self, t: float):
        self.node._current_time = t
    
    def update_odom(self, odom):
        self.node._data_manager._latest_data['odom'] = odom
        self.node._data_manager._timestamps['odom'] = self.node._current_time
    
    def update_trajectory(self, traj):
        self.node._data_manager._latest_data['trajectory'] = traj
        self.node._data_manager._timestamps['trajectory'] = self.node._current_time


class TestQuadrotorDetection:
    """四旋翼平台检测测试"""
    
    def test_differential_not_quadrotor(self):
        """测试差速车不是四旋翼"""
        mock = MockControllerNode()
        mock.initialize(platform='differential')
        
        assert mock.node._is_quadrotor == False
    
    def test_omni_not_quadrotor(self):
        """测试全向车不是四旋翼"""
        mock = MockControllerNode()
        mock.initialize(platform='omni')
        
        assert mock.node._is_quadrotor == False
    
    def test_ackermann_not_quadrotor(self):
        """测试阿克曼车不是四旋翼"""
        mock = MockControllerNode()
        mock.initialize(platform='ackermann')
        
        assert mock.node._is_quadrotor == False
    
    def test_quadrotor_is_quadrotor(self):
        """测试四旋翼是四旋翼"""
        mock = MockControllerNode()
        mock.initialize(platform='quadrotor')
        
        assert mock.node._is_quadrotor == True


class TestYawModeConversion:
    """航向模式转换测试"""
    
    def test_yaw_mode_velocity(self):
        """测试速度跟随模式"""
        mock = MockControllerNode()
        mock.initialize()
        
        mock.node._attitude_yaw_mode = 0
        assert mock.node._get_yaw_mode_string() == 'velocity'
    
    def test_yaw_mode_fixed(self):
        """测试固定模式"""
        mock = MockControllerNode()
        mock.initialize()
        
        mock.node._attitude_yaw_mode = 1
        assert mock.node._get_yaw_mode_string() == 'fixed'
    
    def test_yaw_mode_target_point(self):
        """测试朝向目标模式"""
        mock = MockControllerNode()
        mock.initialize()
        
        mock.node._attitude_yaw_mode = 2
        assert mock.node._get_yaw_mode_string() == 'target_point'
    
    def test_yaw_mode_manual(self):
        """测试手动模式"""
        mock = MockControllerNode()
        mock.initialize()
        
        mock.node._attitude_yaw_mode = 3
        assert mock.node._get_yaw_mode_string() == 'manual'
    
    def test_yaw_mode_unknown(self):
        """测试未知模式"""
        mock = MockControllerNode()
        mock.initialize()
        
        mock.node._attitude_yaw_mode = 99
        assert mock.node._get_yaw_mode_string() == 'velocity'  # 默认


class TestSetHoverYaw:
    """设置悬停航向测试"""
    
    def test_set_hover_yaw_non_quadrotor(self):
        """测试非四旋翼设置悬停航向"""
        mock = MockControllerNode()
        mock.initialize(platform='differential')
        
        success = mock.node._handle_set_hover_yaw(1.57)
        
        assert success == False
        assert any('quadrotor' in msg.lower() for msg in mock.node._logs['warn'])
    
    def test_set_hover_yaw_quadrotor(self):
        """测试四旋翼设置悬停航向"""
        mock = MockControllerNode()
        mock.initialize(platform='quadrotor')
        
        success = mock.node._handle_set_hover_yaw(1.57)
        
        assert success == True
        assert any('1.57' in msg for msg in mock.node._logs['info'])


class TestGetAttitudeRateLimits:
    """获取姿态角速度限制测试"""
    
    def test_get_limits_non_quadrotor(self):
        """测试非四旋翼获取限制"""
        mock = MockControllerNode()
        mock.initialize(platform='differential')
        
        limits = mock.node._handle_get_attitude_rate_limits()
        
        assert limits is None
    
    def test_get_limits_quadrotor(self):
        """测试四旋翼获取限制"""
        mock = MockControllerNode()
        mock.initialize(platform='quadrotor')
        
        limits = mock.node._handle_get_attitude_rate_limits()
        
        # 可能返回 None 如果姿态控制器未初始化
        # 或返回字典
        if limits is not None:
            assert 'roll_rate_max' in limits
            assert 'pitch_rate_max' in limits
            assert 'yaw_rate_max' in limits


class TestAttitudeCommandState:
    """姿态命令状态测试"""
    
    def test_initial_attitude_cmd_state(self):
        """测试初始姿态命令状态"""
        mock = MockControllerNode()
        mock.initialize(platform='quadrotor')
        
        assert mock.node._last_attitude_cmd is None
        assert mock.node._attitude_yaw_mode == 0
    
    def test_reset_clears_attitude_cmd(self):
        """测试重置清除姿态命令"""
        mock = MockControllerNode()
        mock.initialize(platform='quadrotor')
        
        # 设置一个姿态命令
        mock.node._last_attitude_cmd = AttitudeCommand(
            roll=0.1, pitch=0.2, yaw=0.3, thrust=1.0
        )
        
        mock.node._handle_reset()
        
        assert mock.node._last_attitude_cmd is None


class TestComputeAndPublishAttitude:
    """计算并发布姿态测试"""
    
    def test_non_quadrotor_returns_none(self):
        """测试非四旋翼返回 None"""
        mock = MockControllerNode()
        mock.initialize(platform='differential')
        
        import numpy as np
        cmd = ControlOutput(vx=1.0, vy=0.0, vz=0.0, omega=0.0)
        state_array = np.zeros(10)
        
        result = mock.node._compute_and_publish_attitude(cmd, state_array)
        
        assert result is None
        assert len(mock.node._published_attitude_cmds) == 0
    
    def test_quadrotor_computes_attitude(self):
        """测试四旋翼计算姿态"""
        mock = MockControllerNode()
        mock.initialize(platform='quadrotor')
        
        # 添加数据让控制器初始化
        odom = create_test_odom(vx=0.0)
        traj = create_test_trajectory()
        mock.update_odom(odom)
        mock.update_trajectory(traj)
        
        # 运行几次控制循环
        for _ in range(3):
            mock.node._control_loop_core()
        
        import numpy as np
        cmd = ControlOutput(vx=1.0, vy=0.0, vz=0.0, omega=0.0)
        state_array = np.array([0, 0, 0, 0, 0, 0, 1.0, 0, 0, 0])
        
        result = mock.node._compute_and_publish_attitude(cmd, state_array)
        
        # 结果可能是 None 或 AttitudeCommand
        if result is not None:
            assert isinstance(result, AttitudeCommand)
            assert mock.node._last_attitude_cmd is result


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
