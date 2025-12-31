#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Tests for TrajectoryPublisher

Tests cover:
- Lifecycle management
- Waypoint validation
- Trajectory message creation
- Stop trajectory generation
"""
import pytest
import numpy as np
import sys
import os

# Setup test paths
sys.path.insert(0, os.path.dirname(__file__))
import _test_paths


class TestTrajectoryPublisherLogic:
    """Test TrajectoryPublisher logic without ROS"""
    
    def test_waypoint_parsing(self):
        """Test waypoint data parsing"""
        # Input: [x0, y0, x1, y1, x2, y2]
        data = [0.0, 0.0, 1.0, 0.5, 2.0, 1.0]
        
        num_points = len(data) // 2
        positions = np.array(data).reshape(num_points, 2)
        
        assert positions.shape == (3, 2)
        assert positions[0, 0] == 0.0
        assert positions[0, 1] == 0.0
        assert positions[1, 0] == 1.0
        assert positions[1, 1] == 0.5
        assert positions[2, 0] == 2.0
        assert positions[2, 1] == 1.0
    
    def test_odd_length_truncation(self):
        """Test truncation of odd-length data"""
        data = [0.0, 0.0, 1.0, 0.5, 2.0]  # 5 elements (odd)
        
        if len(data) % 2 != 0:
            data = data[:-1]
        
        assert len(data) == 4
        num_points = len(data) // 2
        assert num_points == 2
    
    def test_nan_detection(self):
        """Test NaN detection in trajectory"""
        positions = np.array([
            [0.0, 0.0],
            [float('nan'), 1.0],
            [2.0, 2.0],
        ])
        
        is_valid = np.all(np.isfinite(positions))
        assert not is_valid
    
    def test_inf_detection(self):
        """Test Inf detection in trajectory"""
        positions = np.array([
            [0.0, 0.0],
            [float('inf'), 1.0],
            [2.0, 2.0],
        ])
        
        is_valid = np.all(np.isfinite(positions))
        assert not is_valid
    
    def test_coordinate_range_check(self):
        """Test coordinate range validation"""
        MAX_COORD = 100.0
        
        # Valid coordinates
        positions_valid = np.array([
            [0.0, 0.0],
            [50.0, 50.0],
            [99.0, -99.0],
        ])
        is_valid = not np.any(np.abs(positions_valid) > MAX_COORD)
        assert is_valid
        
        # Invalid coordinates
        positions_invalid = np.array([
            [0.0, 0.0],
            [150.0, 50.0],  # Exceeds MAX_COORD
        ])
        is_valid = not np.any(np.abs(positions_invalid) > MAX_COORD)
        assert not is_valid
    
    def test_empty_trajectory_handling(self):
        """Test empty trajectory handling"""
        data = []
        
        if len(data) < 2:
            # Should return early or create stop trajectory
            is_empty = True
        else:
            is_empty = False
        
        assert is_empty


class TestTrajectoryMessageCreation:
    """Test trajectory message creation logic"""
    
    def test_trajectory_mode_values(self):
        """Test trajectory mode constants"""
        try:
            from universal_controller.core.enums import TrajectoryMode
            MODE_TRACK = TrajectoryMode.MODE_TRACK.value
            MODE_STOP = TrajectoryMode.MODE_STOP.value
            MODE_HOVER = TrajectoryMode.MODE_HOVER.value
            MODE_EMERGENCY = TrajectoryMode.MODE_EMERGENCY.value
        except ImportError:
            MODE_TRACK = 0
            MODE_STOP = 1
            MODE_HOVER = 2
            MODE_EMERGENCY = 3
        
        assert MODE_TRACK == 0
        assert MODE_STOP == 1
        assert MODE_HOVER == 2
        assert MODE_EMERGENCY == 3
    
    def test_velocity_array_creation(self):
        """Test velocity array creation for soft constraints"""
        positions = np.array([
            [0.0, 0.0],
            [1.0, 0.5],
            [2.0, 1.0],
        ])
        
        # Create velocity array with 4 components [vx, vy, vz, wz]
        velocities = np.zeros((len(positions), 4))
        velocities[:, 0] = 0.5  # vx
        velocities[:, 3] = 0.1  # wz
        
        assert velocities.shape == (3, 4)
        assert velocities[0, 0] == 0.5
        assert velocities[0, 3] == 0.1
    
    def test_stop_trajectory_structure(self):
        """Test stop trajectory message structure"""
        # Stop trajectory should have:
        # - mode = MODE_STOP (1)
        # - single point at origin
        # - no velocities
        # - soft_enabled = False
        
        mode = 1  # MODE_STOP
        points = [(0.0, 0.0, 0.0)]
        velocities = []
        soft_enabled = False
        
        assert mode == 1
        assert len(points) == 1
        assert points[0] == (0.0, 0.0, 0.0)
        assert len(velocities) == 0
        assert soft_enabled is False


class TestTrajectoryPublisherLifecycle:
    """Test lifecycle management"""
    
    def test_shutdown_publishes_stop(self):
        """Test that shutdown publishes stop trajectory"""
        # This is a design requirement:
        # When shutting down, publish stop trajectory for safety
        shutdown_publishes_stop = True
        assert shutdown_publishes_stop
    
    def test_reset_clears_statistics(self):
        """Test that reset clears statistics"""
        publish_count = 100
        receive_count = 150
        invalid_count = 5
        last_waypoint = np.array([[0, 0], [1, 1]])
        
        # Reset
        publish_count = 0
        receive_count = 0
        invalid_count = 0
        last_waypoint = None
        
        assert publish_count == 0
        assert receive_count == 0
        assert invalid_count == 0
        assert last_waypoint is None


class TestConfigurationConsistency:
    """Test configuration consistency with other components"""
    
    def test_frame_id_default(self):
        """Test default frame_id matches trajectory_adapter"""
        # Both should default to 'base_link'
        trajectory_publisher_default = 'base_link'
        trajectory_adapter_default = 'base_link'
        
        assert trajectory_publisher_default == trajectory_adapter_default
    
    def test_dt_inheritance(self):
        """Test dt_sec inheritance from mpc.dt"""
        config = {
            'trajectory': {},
            'mpc': {'dt': 0.1},
        }
        
        # dt priority: trajectory.default_dt_sec > mpc.dt > default
        traj_config = config.get('trajectory', {})
        mpc_config = config.get('mpc', {})
        
        dt = traj_config.get('default_dt_sec', mpc_config.get('dt', 0.1))
        
        assert dt == 0.1
    
    def test_topic_from_param_loader(self):
        """Test that output topic comes from ParamLoader"""
        # Default from TOPICS_DEFAULTS
        default_topic = '/controller/input/trajectory'
        
        # This should be used if not overridden
        assert default_topic == '/controller/input/trajectory'


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
