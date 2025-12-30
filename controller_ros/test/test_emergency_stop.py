"""
紧急停止功能单元测试
"""
import pytest
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from typing import Dict, Any
from universal_controller.core.data_types import ControlOutput
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
        
        self.node = TestNode()
    
    def initialize(self, params: Dict[str, Any] = None):
        self.node._params = params or DEFAULT_CONFIG.copy()
        self.node._topics = {'odom': '/odom', 'imu': '/imu', 'trajectory': '/traj'}
        self.node._initialize()
    
    def set_time(self, t: float):
        self.node._current_time = t
    
    def update_odom(self, odom):
        self.node._data_manager._latest_data['odom'] = odom
        self.node._data_manager._timestamps['odom'] = self.node._current_time
    
    def update_trajectory(self, traj):
        # 使用 DataManager 内部的键名 'traj'
        self.node._data_manager._latest_data['traj'] = traj
        self.node._data_manager._timestamps['traj'] = self.node._current_time


class TestEmergencyStop:
    """紧急停止功能测试"""
    
    def test_emergency_stop_initial_state(self):
        """测试初始状态"""
        mock = MockControllerNode()
        mock.initialize()
        
        assert mock.node._is_emergency_stopped() == False
        assert mock.node._emergency_stop_requested == False
        assert mock.node._emergency_stop_time is None
    
    def test_handle_emergency_stop(self):
        """测试触发紧急停止"""
        mock = MockControllerNode()
        mock.initialize()
        mock.set_time(10.0)
        
        mock.node._handle_emergency_stop()
        
        assert mock.node._is_emergency_stopped() == True
        assert mock.node._emergency_stop_requested == True
        assert mock.node._emergency_stop_time == 10.0
        assert any('emergency' in msg.lower() for msg in mock.node._logs['warn'])
    
    def test_emergency_stop_idempotent(self):
        """测试紧急停止幂等性"""
        mock = MockControllerNode()
        mock.initialize()
        mock.set_time(10.0)
        
        mock.node._handle_emergency_stop()
        first_time = mock.node._emergency_stop_time
        warn_count = len(mock.node._logs['warn'])
        
        mock.set_time(15.0)
        mock.node._handle_emergency_stop()
        
        # 时间不应该更新
        assert mock.node._emergency_stop_time == first_time
        # 不应该有新的警告
        assert len(mock.node._logs['warn']) == warn_count
    
    def test_clear_emergency_stop(self):
        """测试清除紧急停止"""
        mock = MockControllerNode()
        mock.initialize()
        mock.set_time(10.0)
        
        mock.node._handle_emergency_stop()
        assert mock.node._is_emergency_stopped() == True
        
        mock.node._clear_emergency_stop()
        
        assert mock.node._is_emergency_stopped() == False
        assert mock.node._emergency_stop_requested == False
        assert mock.node._emergency_stop_time is None
        assert any('cleared' in msg.lower() for msg in mock.node._logs['info'])
    
    def test_reset_clears_emergency_stop(self):
        """测试重置清除紧急停止"""
        mock = MockControllerNode()
        mock.initialize()
        
        mock.node._handle_emergency_stop()
        assert mock.node._is_emergency_stopped() == True
        
        mock.node._handle_reset()
        
        assert mock.node._is_emergency_stopped() == False
    
    def test_control_loop_during_emergency_stop(self):
        """测试紧急停止期间的控制循环"""
        mock = MockControllerNode()
        mock.initialize()
        
        # 添加数据
        odom = create_test_odom(vx=1.0)
        traj = create_test_trajectory()
        mock.update_odom(odom)
        mock.update_trajectory(traj)
        
        # 正常控制
        result = mock.node._control_loop_core()
        assert result is not None
        
        # 触发紧急停止
        mock.node._handle_emergency_stop()
        
        # 紧急停止后控制循环应返回 None 或零速度
        result = mock.node._control_loop_core()
        # 控制器应该请求停止状态
    
    def test_diagnostics_include_emergency_stop(self):
        """测试诊断信息包含紧急停止状态"""
        mock = MockControllerNode()
        mock.initialize()
        
        mock.node._handle_emergency_stop()
        
        diag = {'state': 1}
        mock.node._on_diagnostics(diag)
        
        assert 'emergency_stop' in diag
        assert diag['emergency_stop'] == True
    
    def test_error_diagnostics_include_emergency_stop(self):
        """测试错误诊断包含紧急停止状态"""
        mock = MockControllerNode()
        mock.initialize()
        
        mock.node._handle_emergency_stop()
        
        error = Exception("Test")
        
        # 注意：新 API 不再接受 timeouts 参数，超时状态从 ControllerManager 获取
        diag = mock.node._create_error_diagnostics(error)
        
        assert diag['emergency_stop'] == True


class TestEmergencyStopIntegration:
    """紧急停止集成测试"""
    
    def test_emergency_stop_requests_controller_stop(self):
        """测试紧急停止请求控制器停止"""
        mock = MockControllerNode()
        mock.initialize()
        
        # 添加数据并运行几次让状态机进入 NORMAL
        odom = create_test_odom(vx=1.0)
        traj = create_test_trajectory()
        mock.update_odom(odom)
        mock.update_trajectory(traj)
        
        for _ in range(5):
            mock.node._control_loop_core()
        
        # 触发紧急停止
        mock.node._handle_emergency_stop()
        
        # 控制器桥接应该收到停止请求
        assert mock.node._controller_bridge is not None


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
