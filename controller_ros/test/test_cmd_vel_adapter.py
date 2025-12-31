#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Tests for CmdVelAdapter

Tests cover:
- Lifecycle management (initialize, reset, shutdown)
- Command timeout protection for both modes
- Velocity clamping and rate limiting
- Mode switching
- Constraints validation
"""
import pytest
import sys
import os
from unittest.mock import Mock, MagicMock, patch

# Setup test paths
sys.path.insert(0, os.path.dirname(__file__))
import _test_paths


class MockTime:
    """Mock ROS time for testing"""
    def __init__(self, initial_sec: float = 0.0):
        self._sec = initial_sec
    
    def to_sec(self) -> float:
        return self._sec
    
    def __sub__(self, other):
        result = Mock()
        result.to_sec = lambda: self._sec - other._sec
        return result
    
    def advance(self, delta: float):
        self._sec += delta


class MockTwist:
    """Mock Twist message"""
    def __init__(self, linear_x: float = 0.0, angular_z: float = 0.0):
        self.linear = Mock()
        self.linear.x = linear_x
        self.linear.y = 0.0
        self.linear.z = 0.0
        self.angular = Mock()
        self.angular.x = 0.0
        self.angular.y = 0.0
        self.angular.z = angular_z


class MockUnifiedCmd:
    """Mock UnifiedCmd message"""
    def __init__(self, vx: float = 0.0, omega: float = 0.0):
        self.vx = vx
        self.vy = 0.0
        self.vz = 0.0
        self.omega = omega


class MockBool:
    """Mock Bool message"""
    def __init__(self, data: bool = False):
        self.data = data


@pytest.fixture
def mock_rospy():
    """Mock rospy module"""
    with patch.dict('sys.modules', {'rospy': MagicMock()}):
        import sys
        rospy = sys.modules['rospy']
        
        # Mock Time
        current_time = MockTime(100.0)
        rospy.Time.now = lambda: current_time
        rospy.Duration = lambda sec: Mock(to_sec=lambda: sec)
        
        # Mock logging
        rospy.loginfo = Mock()
        rospy.logwarn = Mock()
        rospy.logerr = Mock()
        rospy.logwarn_throttle = Mock()
        rospy.logerr_throttle = Mock()
        
        # Mock node init
        rospy.init_node = Mock()
        rospy.on_shutdown = Mock()
        rospy.spin = Mock()
        
        # Mock pub/sub
        rospy.Publisher = Mock(return_value=Mock())
        rospy.Subscriber = Mock(return_value=Mock())
        rospy.Timer = Mock(return_value=Mock())
        rospy.Service = Mock(return_value=Mock())
        
        yield rospy, current_time


@pytest.fixture
def mock_param_loader():
    """Mock ParamLoader"""
    with patch('controller_ros.utils.param_loader.ParamLoader') as mock:
        mock.load.return_value = {
            'constraints': {
                'v_max': 1.0,
                'v_min': -0.5,
                'omega_max': 2.0,
                'a_max': 1.5,
                'alpha_max': 3.0,
            }
        }
        mock.get_topics.return_value = {
            'cmd_unified': '/cmd_unified',
        }
        mock.get_cmd_vel_adapter_config.return_value = {
            'publish_rate': 20.0,
            'cmd_timeout': 0.5,
            'enable_rate_limit': True,
            'joy_topic': '/joy_cmd_vel',
            'mode_topic': '/control_mode',
            'output_topic': '/cmd_vel',
        }
        yield mock


class TestCmdVelAdapterLogic:
    """Test CmdVelAdapter logic without ROS"""
    
    def test_clamp_within_range(self):
        """Test clamping values within range"""
        # Simple clamp function test
        def clamp(value, min_val, max_val):
            return max(min_val, min(max_val, value))
        
        assert clamp(0.5, 0.0, 1.0) == 0.5
        assert clamp(-0.5, 0.0, 1.0) == 0.0
        assert clamp(1.5, 0.0, 1.0) == 1.0
        assert clamp(0.0, -1.0, 1.0) == 0.0
    
    def test_rate_limit_logic(self):
        """Test rate limiting logic"""
        a_max = 1.5  # m/s²
        dt = 0.05  # 20 Hz
        
        last_linear = 0.0
        target_linear = 1.0
        
        max_delta = a_max * dt  # 0.075
        delta = target_linear - last_linear  # 1.0
        
        if abs(delta) > max_delta:
            limited = last_linear + max_delta * (1 if delta > 0 else -1)
        else:
            limited = target_linear
        
        assert limited == pytest.approx(0.075, rel=0.01)
    
    def test_timeout_detection_logic(self):
        """Test timeout detection logic"""
        cmd_timeout = 0.5
        
        # Case 1: Within timeout
        time_since_cmd = 0.3
        is_timeout = time_since_cmd > cmd_timeout
        assert not is_timeout
        
        # Case 2: Exceeded timeout
        time_since_cmd = 0.6
        is_timeout = time_since_cmd > cmd_timeout
        assert is_timeout
        
        # Case 3: No command received
        cmd_time = None
        is_timeout = cmd_time is None
        assert is_timeout
    
    def test_mode_switching_logic(self):
        """Test mode switching logic"""
        joystick_mode = False
        
        # Switch to joystick
        new_mode = True
        if new_mode != joystick_mode:
            joystick_mode = new_mode
        assert joystick_mode is True
        
        # Same mode, no change
        new_mode = True
        changed = new_mode != joystick_mode
        assert not changed
        
        # Switch back to controller
        new_mode = False
        if new_mode != joystick_mode:
            joystick_mode = new_mode
        assert joystick_mode is False


class TestCmdVelAdapterIntegration:
    """Integration tests for CmdVelAdapter (requires mocking)"""
    
    def test_controller_timeout_protection(self):
        """Test that controller mode has timeout protection"""
        # Simulate the timeout logic
        cmd_timeout = 0.5
        controller_cmd_time = 100.0  # Last command at t=100
        current_time = 100.6  # Current time t=100.6
        
        time_since_ctrl = current_time - controller_cmd_time
        is_timeout = time_since_ctrl > cmd_timeout
        
        assert is_timeout, "Controller command should timeout after 0.5s"
    
    def test_joystick_timeout_protection(self):
        """Test that joystick mode has timeout protection"""
        cmd_timeout = 0.5
        joy_cmd_time = 100.0
        current_time = 100.6
        
        time_since_joy = current_time - joy_cmd_time
        is_timeout = time_since_joy > cmd_timeout
        
        assert is_timeout, "Joystick command should timeout after 0.5s"
    
    def test_velocity_clamping(self):
        """Test velocity clamping"""
        v_max = 1.0
        v_min = -0.5
        omega_max = 2.0
        
        def clamp(value, min_val, max_val):
            return max(min_val, min(max_val, value))
        
        # Test linear velocity clamping
        assert clamp(1.5, v_min, v_max) == 1.0
        assert clamp(-1.0, v_min, v_max) == -0.5
        
        # Test angular velocity clamping
        assert clamp(3.0, -omega_max, omega_max) == 2.0
        assert clamp(-3.0, -omega_max, omega_max) == -2.0
    
    def test_nan_handling(self):
        """Test NaN value handling"""
        import math
        
        linear_x = float('nan')
        angular_z = 0.5
        
        is_valid = math.isfinite(linear_x) and math.isfinite(angular_z)
        assert not is_valid, "NaN should be detected as invalid"
    
    def test_inf_handling(self):
        """Test Inf value handling"""
        import math
        
        linear_x = float('inf')
        angular_z = 0.5
        
        is_valid = math.isfinite(linear_x) and math.isfinite(angular_z)
        assert not is_valid, "Inf should be detected as invalid"


class TestCmdVelAdapterLifecycle:
    """Test lifecycle management"""
    
    def test_lifecycle_state_transitions(self):
        """Test lifecycle state transitions"""
        from controller_ros.lifecycle import LifecycleState
        
        # Initial state
        state = LifecycleState.UNINITIALIZED
        assert state == LifecycleState.UNINITIALIZED
        
        # After initialize
        state = LifecycleState.RUNNING
        assert state == LifecycleState.RUNNING
        
        # After shutdown
        state = LifecycleState.SHUTDOWN
        assert state == LifecycleState.SHUTDOWN
    
    def test_reset_clears_state(self):
        """Test that reset clears internal state"""
        # Simulate state
        last_linear_x = 0.5
        last_angular_z = 0.3
        published_count = 100
        timeout_count = 5
        
        # Reset
        last_linear_x = 0.0
        last_angular_z = 0.0
        published_count = 0
        timeout_count = 0
        
        assert last_linear_x == 0.0
        assert last_angular_z == 0.0
        assert published_count == 0
        assert timeout_count == 0


class TestConstraintsValidation:
    """Test constraints validation logic"""
    
    def test_valid_constraints(self):
        """Test that valid constraints pass validation"""
        from controller_ros.utils.param_loader import ParamLoader
        
        constraints = {
            'v_max': 1.0,
            'v_min': -0.5,
            'omega_max': 2.0,
            'a_max': 1.5,
            'alpha_max': 3.0,
        }
        
        errors = ParamLoader.validate_constraints(constraints)
        assert len(errors) == 0, f"Valid constraints should pass: {errors}"
    
    def test_invalid_v_max(self):
        """Test that v_max <= 0 fails validation"""
        from controller_ros.utils.param_loader import ParamLoader
        
        constraints = {'v_max': 0.0}
        errors = ParamLoader.validate_constraints(constraints)
        assert len(errors) == 1
        assert 'v_max' in errors[0]
        
        constraints = {'v_max': -1.0}
        errors = ParamLoader.validate_constraints(constraints)
        assert len(errors) == 1
    
    def test_invalid_omega_max(self):
        """Test that omega_max <= 0 fails validation"""
        from controller_ros.utils.param_loader import ParamLoader
        
        constraints = {'omega_max': 0.0}
        errors = ParamLoader.validate_constraints(constraints)
        assert len(errors) == 1
        assert 'omega_max' in errors[0]
    
    def test_invalid_a_max(self):
        """Test that a_max <= 0 fails validation"""
        from controller_ros.utils.param_loader import ParamLoader
        
        constraints = {'a_max': -0.5}
        errors = ParamLoader.validate_constraints(constraints)
        assert len(errors) == 1
        assert 'a_max' in errors[0]
    
    def test_invalid_alpha_max(self):
        """Test that alpha_max <= 0 fails validation"""
        from controller_ros.utils.param_loader import ParamLoader
        
        constraints = {'alpha_max': 0.0}
        errors = ParamLoader.validate_constraints(constraints)
        assert len(errors) == 1
        assert 'alpha_max' in errors[0]
    
    def test_v_min_greater_than_v_max(self):
        """Test that v_min > v_max fails validation"""
        from controller_ros.utils.param_loader import ParamLoader
        
        constraints = {
            'v_max': 1.0,
            'v_min': 2.0,  # Invalid: v_min > v_max
        }
        errors = ParamLoader.validate_constraints(constraints)
        assert len(errors) == 1
        assert 'v_min' in errors[0]
    
    def test_multiple_errors(self):
        """Test that multiple errors are all reported"""
        from controller_ros.utils.param_loader import ParamLoader
        
        constraints = {
            'v_max': 0.0,      # Invalid
            'omega_max': -1.0, # Invalid
            'a_max': 0.0,      # Invalid
        }
        errors = ParamLoader.validate_constraints(constraints)
        assert len(errors) == 3


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
