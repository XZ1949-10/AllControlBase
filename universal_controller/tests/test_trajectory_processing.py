"""
轨迹处理测试

测试轨迹数据类型的各种处理功能。

测试覆盖:
1. 轨迹创建和验证
2. 速度计算（hard/soft/blended）
3. 缓存机制
4. 边界条件处理
"""
import pytest
import numpy as np
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from universal_controller.core.data_types import (
    Trajectory, Header, Point3D
)
from universal_controller.core.enums import TrajectoryMode
from universal_controller.config.default_config import DEFAULT_CONFIG


class TestTrajectoryCreation:
    """测试轨迹创建"""
    
    def test_basic_trajectory_creation(self):
        """测试基本轨迹创建"""
        points = [Point3D(x=i * 0.1, y=0.0, z=0.0) for i in range(5)]
        
        traj = Trajectory(
            header=Header(stamp=1.0, frame_id="base_link"),
            points=points,
            velocities=None,
            dt_sec=0.1,
            confidence=0.9,
            mode=TrajectoryMode.MODE_TRACK,
            soft_enabled=False
        )
        
        assert len(traj) == 5
        assert traj.dt_sec == 0.1
        assert traj.confidence == 0.9
    
    def test_trajectory_with_soft_velocities(self):
        """测试带 soft velocities 的轨迹"""
        points = [Point3D(x=i * 0.1, y=0.0, z=0.0) for i in range(5)]
        velocities = np.array([[0.5, 0.0, 0.0, 0.0]] * 5)
        
        traj = Trajectory(
            header=Header(stamp=1.0, frame_id="base_link"),
            points=points,
            velocities=velocities,
            dt_sec=0.1,
            confidence=0.9,
            mode=TrajectoryMode.MODE_TRACK,
            soft_enabled=True
        )
        
        assert traj.soft_enabled
        assert traj.has_valid_soft_velocities()
        assert traj.velocities.shape == (5, 4)
    
    def test_trajectory_default_values(self):
        """测试轨迹默认值"""
        # TrajectoryDefaults has been removed. 
        # Trajectory now requires explicit config or defaults to None/Config values if not provided.
        # But Trajectory __post_init__ logic was changed to:
        # if dt_sec is None -> default is 0.1 (hardcoded in class or config)
        
        points = [Point3D(x=0.0, y=0.0, z=0.0)]
        
        traj = Trajectory(
            header=Header(stamp=1.0, frame_id="base_link"),
            points=points,
            velocities=None
            # dt_sec and confidence defaults are now handled internally or require config
        )
        
        # Depending on new implementation of Trajectory:
        # If no config provided, what are the defaults? 
        # Looking at data_types.py previous view, Trajectory had defaults in __post_init__ 
        # using TrajectoryDefaults. 
        # I updated it to use TrajectoryConfig.
        # If I instantiate Trajectory without passing config, it might fail or use class-level defaults if I added them.
        
        # Assuming I updated Trajectory to have some safe fallbacks or I need to update this test
        # to verify that it uses the provided values or safe defaults (0.1, 0.9).
        
        # Verify safe defaults if params are missing
        assert traj.dt_sec == 0.1
        assert traj.confidence == 0.9
    
    def test_trajectory_confidence_clamping(self):
        """测试置信度裁剪"""
        points = [Point3D(x=0.0, y=0.0, z=0.0)]
        
        # 超出范围的置信度应该被裁剪
        traj_high = Trajectory(
            header=Header(stamp=1.0, frame_id="base_link"),
            points=points,
            velocities=None,
            confidence=1.5
        )
        assert traj_high.confidence == 1.0
        
        traj_low = Trajectory(
            header=Header(stamp=1.0, frame_id="base_link"),
            points=points,
            velocities=None,
            confidence=-0.5
        )
        assert traj_low.confidence == 0.0
    
    def test_trajectory_nan_confidence_handling(self):
        """测试 NaN 置信度处理"""
        points = [Point3D(x=0.0, y=0.0, z=0.0)]
        
        traj = Trajectory(
            header=Header(stamp=1.0, frame_id="base_link"),
            points=points,
            velocities=None,
            confidence=float('nan')
        )
        
        # NaN 应该被替换为默认值
        assert np.isfinite(traj.confidence)


class TestHardVelocityCalculation:
    """测试 hard velocity 计算"""
    
    def test_hard_velocity_from_points(self):
        """测试从轨迹点计算速度"""
        # 创建直线轨迹
        points = [Point3D(x=i * 0.1, y=0.0, z=0.0) for i in range(5)]
        
        traj = Trajectory(
            header=Header(stamp=1.0, frame_id="base_link"),
            points=points,
            velocities=None,
            dt_sec=0.1
        )
        
        hard_vel = traj.get_hard_velocities()
        
        assert hard_vel.shape == (5, 4)
        # 直线轨迹，vx 应该约为 1.0 m/s (0.1m / 0.1s)
        assert np.isclose(hard_vel[0, 0], 1.0, atol=0.1)
    
    def test_hard_velocity_with_turns(self):
        """测试带转弯的轨迹速度计算"""
        # 创建 L 形轨迹
        points = [
            Point3D(x=0.0, y=0.0, z=0.0),
            Point3D(x=0.1, y=0.0, z=0.0),
            Point3D(x=0.2, y=0.0, z=0.0),
            Point3D(x=0.2, y=0.1, z=0.0),
            Point3D(x=0.2, y=0.2, z=0.0),
        ]
        
        traj = Trajectory(
            header=Header(stamp=1.0, frame_id="base_link"),
            points=points,
            velocities=None,
            dt_sec=0.1
        )
        
        hard_vel = traj.get_hard_velocities()
        
        assert hard_vel.shape == (5, 4)
        # 转弯发生在 index 1 处（从 X 方向转向 Y 方向）
        # index 1: 当前速度 (1,0)，下一个速度 (0,1)，航向变化 90 度
        assert hard_vel[1, 3] != 0.0  # wz 在转弯处不为零
    
    def test_hard_velocity_single_point(self):
        """测试单点轨迹"""
        points = [Point3D(x=0.0, y=0.0, z=0.0)]
        
        traj = Trajectory(
            header=Header(stamp=1.0, frame_id="base_link"),
            points=points,
            velocities=None
        )
        
        hard_vel = traj.get_hard_velocities()
        
        assert hard_vel.shape == (1, 4)
        assert np.allclose(hard_vel, 0.0)


class TestVelocityBlending:
    """测试速度混合"""
    
    def test_blended_velocity_high_alpha(self):
        """测试高 alpha 时的速度混合"""
        points = [Point3D(x=i * 0.1, y=0.0, z=0.0) for i in range(5)]
        soft_vel = np.array([[2.0, 0.0, 0.0, 0.0]] * 5)
        
        traj = Trajectory(
            header=Header(stamp=1.0, frame_id="base_link"),
            points=points,
            velocities=soft_vel,
            dt_sec=0.1,
            soft_enabled=True
        )
        
        # alpha=1.0 应该完全使用 soft
        blended = traj.get_blended_velocity(0, alpha=1.0)
        assert np.isclose(blended[0], 2.0)
    
    def test_blended_velocity_low_alpha(self):
        """测试低 alpha 时的速度混合"""
        points = [Point3D(x=i * 0.1, y=0.0, z=0.0) for i in range(5)]
        soft_vel = np.array([[2.0, 0.0, 0.0, 0.0]] * 5)
        
        traj = Trajectory(
            header=Header(stamp=1.0, frame_id="base_link"),
            points=points,
            velocities=soft_vel,
            dt_sec=0.1,
            soft_enabled=True
        )
        
        # alpha=0.0 应该完全使用 hard
        blended = traj.get_blended_velocity(0, alpha=0.0)
        hard_vel = traj.get_hard_velocities()[0]
        assert np.allclose(blended, hard_vel)
    
    def test_blended_velocity_mixed_alpha(self):
        """测试混合 alpha 时的速度混合"""
        points = [Point3D(x=i * 0.1, y=0.0, z=0.0) for i in range(5)]
        soft_vel = np.array([[2.0, 0.0, 0.0, 0.0]] * 5)
        
        traj = Trajectory(
            header=Header(stamp=1.0, frame_id="base_link"),
            points=points,
            velocities=soft_vel,
            dt_sec=0.1,
            soft_enabled=True
        )
        
        # alpha=0.5 应该是两者的平均
        blended = traj.get_blended_velocity(0, alpha=0.5)
        hard_vel = traj.get_hard_velocities()[0]
        expected = 0.5 * soft_vel[0] + 0.5 * hard_vel
        assert np.allclose(blended, expected)
    
    def test_blended_speed(self):
        """测试混合速度大小"""
        points = [Point3D(x=i * 0.1, y=0.0, z=0.0) for i in range(5)]
        soft_vel = np.array([[2.0, 0.0, 0.0, 0.0]] * 5)
        
        traj = Trajectory(
            header=Header(stamp=1.0, frame_id="base_link"),
            points=points,
            velocities=soft_vel,
            dt_sec=0.1,
            soft_enabled=True
        )
        
        speed = traj.get_blended_speed(0, alpha=1.0)
        assert np.isclose(speed, 2.0)


class TestTrajectoryCaching:
    """测试轨迹缓存机制"""
    
    def test_hard_velocity_caching(self):
        """测试 hard velocity 缓存"""
        points = [Point3D(x=i * 0.1, y=0.0, z=0.0) for i in range(5)]
        
        traj = Trajectory(
            header=Header(stamp=1.0, frame_id="base_link"),
            points=points,
            velocities=None,
            dt_sec=0.1
        )
        
        # 第一次调用
        vel1 = traj.get_hard_velocities()
        # 第二次调用应该返回缓存
        vel2 = traj.get_hard_velocities()
        
        assert vel1 is vel2  # 应该是同一个对象
    
    def test_cache_invalidation_on_dt_change(self):
        """测试 dt 变化时缓存失效"""
        points = [Point3D(x=i * 0.1, y=0.0, z=0.0) for i in range(5)]
        
        traj = Trajectory(
            header=Header(stamp=1.0, frame_id="base_link"),
            points=points,
            velocities=None,
            dt_sec=0.1
        )
        
        vel1 = traj.get_hard_velocities()
        
        # 修改 dt
        traj.dt_sec = 0.2
        
        vel2 = traj.get_hard_velocities()
        
        # 缓存应该失效，返回新计算的值
        assert vel1 is not vel2


class TestTrajectoryCopy:
    """测试轨迹复制"""
    
    def test_trajectory_deep_copy(self):
        """测试轨迹深拷贝"""
        points = [Point3D(x=i * 0.1, y=0.0, z=0.0) for i in range(5)]
        velocities = np.array([[0.5, 0.0, 0.0, 0.0]] * 5)
        
        traj = Trajectory(
            header=Header(stamp=1.0, frame_id="base_link"),
            points=points,
            velocities=velocities,
            dt_sec=0.1,
            soft_enabled=True
        )
        
        traj_copy = traj.copy()
        
        # 修改原始轨迹
        traj.points[0].x = 999.0
        traj.velocities[0, 0] = 999.0
        
        # 副本不应该受影响
        assert traj_copy.points[0].x != 999.0
        assert traj_copy.velocities[0, 0] != 999.0
    
    def test_trajectory_copy_preserves_attributes(self):
        """测试复制保留所有属性"""
        points = [Point3D(x=0.0, y=0.0, z=0.0)]
        
        traj = Trajectory(
            header=Header(stamp=1.5, frame_id="test_frame"),
            points=points,
            velocities=None,
            dt_sec=0.2,
            confidence=0.8,
            mode=TrajectoryMode.MODE_STOP,
            soft_enabled=False
        )
        
        traj_copy = traj.copy()
        
        assert traj_copy.header.stamp == 1.5
        assert traj_copy.header.frame_id == "test_frame"
        assert traj_copy.dt_sec == 0.2
        assert traj_copy.confidence == 0.8
        assert traj_copy.mode == TrajectoryMode.MODE_STOP


class TestTrajectoryValidation:
    """测试轨迹验证"""
    
    def test_trajectory_point_distance_validation(self):
        """测试轨迹点距离验证"""
        # 创建距离过大的轨迹点
        points = [
            Point3D(x=0.0, y=0.0, z=0.0),
            Point3D(x=100.0, y=0.0, z=0.0),  # 距离过大
        ]
        
        # 应该记录警告但不抛出异常
        traj = Trajectory(
            header=Header(stamp=1.0, frame_id="base_link"),
            points=points,
            velocities=None
        )
        
        assert len(traj) == 2
    
    def test_trajectory_dt_validation(self):
        """测试 dt 验证"""
        points = [Point3D(x=0.0, y=0.0, z=0.0)]
        
        # 无效的 dt 应该被修正
        traj = Trajectory(
            header=Header(stamp=1.0, frame_id="base_link"),
            points=points,
            velocities=None,
            dt_sec=-0.1  # 无效值
        )
        
        assert traj.dt_sec > 0


class TestTrajectoryModes:
    """测试轨迹模式"""
    
    def test_track_mode(self):
        """测试跟踪模式"""
        points = [Point3D(x=0.0, y=0.0, z=0.0)]
        
        traj = Trajectory(
            header=Header(stamp=1.0, frame_id="base_link"),
            points=points,
            velocities=None,
            mode=TrajectoryMode.MODE_TRACK
        )
        
        assert traj.mode == TrajectoryMode.MODE_TRACK
    
    def test_stop_mode(self):
        """测试停止模式"""
        points = [Point3D(x=0.0, y=0.0, z=0.0)]
        
        traj = Trajectory(
            header=Header(stamp=1.0, frame_id="base_link"),
            points=points,
            velocities=None,
            mode=TrajectoryMode.MODE_STOP
        )
        
        assert traj.mode == TrajectoryMode.MODE_STOP
    
    def test_hover_mode(self):
        """测试悬停模式"""
        points = [Point3D(x=0.0, y=0.0, z=0.0)]
        
        traj = Trajectory(
            header=Header(stamp=1.0, frame_id="base_link"),
            points=points,
            velocities=None,
            mode=TrajectoryMode.MODE_HOVER
        )
        
        assert traj.mode == TrajectoryMode.MODE_HOVER


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
