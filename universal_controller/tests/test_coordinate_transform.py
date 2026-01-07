"""
坐标变换测试

测试 RobustCoordinateTransformer 的坐标变换功能，
确保网络输出的局部轨迹能正确变换到里程计坐标系。

坐标系约定 (不需要建图):
- 网络输出: base_link (局部坐标系，当前位置为原点)
- 控制器工作: odom (里程计坐标系，从启动位置累积)
"""
import sys
import os
import time
import math
import numpy as np

# 添加项目根目录到路径
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from universal_controller.config.default_config import DEFAULT_CONFIG
from universal_controller.transform.robust_transformer import RobustCoordinateTransformer
from universal_controller.tests.fixtures import (
    create_test_trajectory, 
    create_test_odom,
    create_local_trajectory_with_transform
)
from universal_controller.core.data_types import Trajectory, Point3D, Header


def _get_point(traj, idx):
    """获取轨迹点坐标，兼容 numpy 数组和 Point3D 列表"""
    points = traj.points
    if isinstance(points, np.ndarray):
        return points[idx, 0], points[idx, 1], points[idx, 2] if points.shape[1] > 2 else 0.0
    else:
        p = points[idx]
        return p.x, p.y, p.z


def test_basic_transform():
    """测试基本坐标变换"""
    config = DEFAULT_CONFIG.copy()
    transformer = RobustCoordinateTransformer(config)
    
    # 设置机器人位姿: 在 odom 坐标系中 (1, 2) 位置，航向 45°
    robot_x, robot_y = 1.0, 2.0
    robot_theta = math.pi / 4  # 45°
    
    # 设置 TF 变换 (base_link -> odom)
    transformer.set_transform('odom', 'base_link', robot_x, robot_y, 0.0, robot_theta)
    
    # 创建局部轨迹 (直线向前，在 base_link 坐标系下)
    local_traj = create_test_trajectory(
        num_points=5,
        trajectory_type='straight',
        speed=1.0,
        frame_id='base_link'
    )
    
    # 变换到 odom 坐标系
    odom_traj, status = transformer.transform_trajectory(local_traj, 'odom', time.time())
    
    # 验证第一个点 (应该在机器人位置)
    p0_x, p0_y, _ = _get_point(odom_traj, 0)
    assert abs(p0_x - robot_x) < 0.01, \
        f"First point x should be {robot_x}, got {p0_x}"
    assert abs(p0_y - robot_y) < 0.01, \
        f"First point y should be {robot_y}, got {p0_y}"
    
    # 验证轨迹方向 (应该沿着 45° 方向)
    if len(odom_traj.points) > 1:
        p1_x, p1_y, _ = _get_point(odom_traj, 1)
        dx = p1_x - p0_x
        dy = p1_y - p0_y
        actual_angle = math.atan2(dy, dx)
        expected_angle = robot_theta
        angle_diff = abs(actual_angle - expected_angle)
        assert angle_diff < 0.1, \
            f"Trajectory direction should be {expected_angle:.2f}, got {actual_angle:.2f}"
    
    # 验证坐标系已更新
    assert odom_traj.header.frame_id == 'odom'
    
    print("[PASS] test_basic_transform passed")


def test_velocity_transform():
    """测试速度向量变换"""
    config = DEFAULT_CONFIG.copy()
    transformer = RobustCoordinateTransformer(config)
    
    # 设置机器人位姿: 航向 90° (朝向 Y 轴正方向)
    robot_theta = math.pi / 2
    transformer.set_transform('odom', 'base_link', 0.0, 0.0, 0.0, robot_theta)
    
    # 创建带速度的局部轨迹
    local_traj = create_test_trajectory(
        num_points=5,
        trajectory_type='straight',
        speed=1.0,
        frame_id='base_link',
        soft_enabled=True
    )
    
    # 变换到 odom 坐标系
    odom_traj, status = transformer.transform_trajectory(local_traj, 'odom', time.time())
    
    # 验证速度变换
    # 局部坐标系中 vx=1.0, vy=0 (向前)
    # odom 坐标系中应该是 vx≈0, vy≈1.0 (因为机器人朝向 Y 轴)
    if odom_traj.velocities is not None and len(odom_traj.velocities) > 0:
        vx_odom = odom_traj.velocities[0, 0]
        vy_odom = odom_traj.velocities[0, 1]
        
        assert abs(vx_odom) < 0.1, f"Odom vx should be ~0, got {vx_odom}"
        assert abs(vy_odom - 1.0) < 0.1, f"Odom vy should be ~1.0, got {vy_odom}"
    
    print("[PASS] test_velocity_transform passed")


def test_frame_validation():
    """测试坐标系验证"""
    config = DEFAULT_CONFIG.copy()
    config['transform']['warn_unexpected_frame'] = True
    transformer = RobustCoordinateTransformer(config)
    
    # 创建使用非预期坐标系的轨迹
    unexpected_traj = create_test_trajectory(
        num_points=5,
        trajectory_type='straight',
        frame_id='unexpected_frame'
    )
    
    # 设置 TF 变换
    transformer.set_transform('odom', 'unexpected_frame', 0.0, 0.0, 0.0, 0.0)
    
    # 变换应该成功，但会发出警告 (检查日志)
    world_traj, status = transformer.transform_trajectory(unexpected_traj, 'odom', time.time())
    
    # 验证警告已记录
    assert 'unexpected_frame' in transformer._warned_frames
    
    print("[PASS] test_frame_validation passed")


def test_empty_frame_fallback():
    """测试空坐标系回退"""
    config = DEFAULT_CONFIG.copy()
    config['transform']['source_frame'] = 'base_link'
    transformer = RobustCoordinateTransformer(config)
    
    # 创建空坐标系的轨迹
    empty_frame_traj = Trajectory(
        header=Header(stamp=time.time(), frame_id=''),  # 空坐标系
        points=[Point3D(1.0, 0.0, 0.0), Point3D(2.0, 0.0, 0.0)],
        velocities=None,
        dt_sec=0.1,
        confidence=0.9
    )
    
    # 设置 TF 变换 (使用默认 source_frame)
    transformer.set_transform('odom', 'base_link', 1.0, 2.0, 0.0, 0.0)
    
    # 变换应该使用默认 source_frame
    world_traj, status = transformer.transform_trajectory(empty_frame_traj, 'odom', time.time())
    
    # 验证变换正确应用
    p0_x, p0_y, _ = _get_point(world_traj, 0)
    assert abs(p0_x - 2.0) < 0.01  # 1.0 + 1.0
    assert abs(p0_y - 2.0) < 0.01  # 0.0 + 2.0
    
    print("[PASS] test_empty_frame_fallback passed")


def test_transform_with_rotation():
    """测试带旋转的变换"""
    config = DEFAULT_CONFIG.copy()
    transformer = RobustCoordinateTransformer(config)
    
    # 测试不同的旋转角度
    test_angles = [0, math.pi/4, math.pi/2, math.pi, -math.pi/2]
    
    for theta in test_angles:
        transformer.set_transform('odom', 'base_link', 0.0, 0.0, 0.0, theta)
        
        # 创建局部轨迹: 一个点在 base_link 的 (1, 0) 位置
        local_traj = Trajectory(
            header=Header(stamp=time.time(), frame_id='base_link'),
            points=[Point3D(1.0, 0.0, 0.0)],
            velocities=None,
            dt_sec=0.1,
            confidence=0.9
        )
        
        odom_traj, _ = transformer.transform_trajectory(local_traj, 'odom', time.time())
        
        # 验证旋转正确
        # 局部 (1, 0) 旋转 theta 后应该是 (cos(theta), sin(theta))
        expected_x = math.cos(theta)
        expected_y = math.sin(theta)
        
        p0_x, p0_y, _ = _get_point(odom_traj, 0)
        assert abs(p0_x - expected_x) < 0.01, \
            f"theta={theta:.2f}: x should be {expected_x:.2f}, got {p0_x:.2f}"
        assert abs(p0_y - expected_y) < 0.01, \
            f"theta={theta:.2f}: y should be {expected_y:.2f}, got {p0_y:.2f}"
    
    print("[PASS] test_transform_with_rotation passed")


def test_transform_consistency():
    """测试变换一致性 (使用辅助函数验证)"""
    config = DEFAULT_CONFIG.copy()
    transformer = RobustCoordinateTransformer(config)
    
    # 使用辅助函数创建局部和 odom 轨迹
    robot_x, robot_y, robot_theta = 5.0, 3.0, math.pi / 6
    
    local_traj, expected_odom_traj = create_local_trajectory_with_transform(
        robot_x, robot_y, robot_theta,
        num_points=10,
        trajectory_type='straight',
        speed=1.0,
        soft_enabled=True
    )
    
    # 设置 TF 变换
    transformer.set_transform('odom', 'base_link', robot_x, robot_y, 0.0, robot_theta)
    
    # 变换局部轨迹
    actual_odom_traj, _ = transformer.transform_trajectory(local_traj, 'odom', time.time())
    
    # 比较结果 - 兼容 numpy 数组和 Point3D 列表
    num_points = len(actual_odom_traj.points)
    for i in range(num_points):
        actual_x, actual_y, _ = _get_point(actual_odom_traj, i)
        expected_x, expected_y, _ = _get_point(expected_odom_traj, i)
        assert abs(actual_x - expected_x) < 0.01, \
            f"Point {i}: x mismatch {actual_x} vs {expected_x}"
        assert abs(actual_y - expected_y) < 0.01, \
            f"Point {i}: y mismatch {actual_y} vs {expected_y}"
    
    # 比较速度
    if actual_odom_traj.velocities is not None and expected_odom_traj.velocities is not None:
        for i in range(len(actual_odom_traj.velocities)):
            for j in range(2):  # vx, vy
                actual_v = actual_odom_traj.velocities[i, j]
                expected_v = expected_odom_traj.velocities[i, j]
                assert abs(actual_v - expected_v) < 0.01, \
                    f"Velocity [{i},{j}]: mismatch {actual_v} vs {expected_v}"
    
    print("[PASS] test_transform_consistency passed")


def test_fallback_mode():
    """测试 TF2 不可用时的降级模式"""
    config = DEFAULT_CONFIG.copy()
    transformer = RobustCoordinateTransformer(config)
    
    # 强制禁用 TF2
    transformer.set_tf2_available(False)
    
    # 设置状态估计器 (模拟)
    from universal_controller.estimator.adaptive_ekf import AdaptiveEKFEstimator
    estimator = AdaptiveEKFEstimator(config)
    
    # 设置估计器状态
    odom = create_test_odom(x=1.0, y=2.0, theta=0.5, vx=1.0)
    estimator.update_odom(odom, 0.0)
    
    # transformer.set_state_estimator(estimator) -> Removed
    fallback_state = estimator.get_state()
    
    # 先设置一个 TF2 位置作为基准
    transformer._last_tf2_position = np.array([1.0, 2.0, 0.0])
    transformer._last_tf2_yaw = 0.5
    
    # 创建轨迹
    local_traj = create_test_trajectory(
        num_points=5,
        trajectory_type='straight',
        frame_id='base_link'
    )
    
    # 变换 (应该使用 fallback)
    world_traj, status = transformer.transform_trajectory(local_traj, 'odom', time.time(), fallback_state=fallback_state)
    
    # 验证状态
    assert status.name.startswith('FALLBACK'), f"Should be in fallback mode, got {status.name}"
    
    print("[PASS] test_fallback_mode passed")


def test_delay_compensation():
    """测试延迟补偿"""
    config = DEFAULT_CONFIG.copy()
    transformer = RobustCoordinateTransformer(config)
    
    current_time = time.time()
    trajectory_time = current_time - 0.1  # 100ms 前的轨迹
    
    # 创建带时间戳的轨迹
    local_traj = Trajectory(
        header=Header(stamp=trajectory_time, frame_id='base_link'),
        points=[Point3D(1.0, 0.0, 0.0)],
        velocities=None,
        dt_sec=0.1,
        confidence=0.9
    )
    
    # 设置 TF 变换
    transformer.set_transform('odom', 'base_link', 0.0, 0.0, 0.0, 0.0, stamp=trajectory_time)
    
    # 变换应该使用轨迹时间戳
    world_traj, status = transformer.transform_trajectory(local_traj, 'odom', current_time)
    
    # 验证变换成功
    assert world_traj is not None
    
    print("[PASS] test_delay_compensation passed")


def run_all_tests():
    """运行所有坐标变换测试"""
    print("\n" + "="*60)
    print("Running Coordinate Transform Tests")
    print("="*60 + "\n")
    
    tests = [
        test_basic_transform,
        test_velocity_transform,
        test_frame_validation,
        test_empty_frame_fallback,
        test_transform_with_rotation,
        test_transform_consistency,
        test_fallback_mode,
        test_delay_compensation,
    ]
    
    passed = 0
    failed = 0
    
    for test in tests:
        try:
            test()
            passed += 1
        except Exception as e:
            print(f"✗ {test.__name__} FAILED: {e}")
            import traceback
            traceback.print_exc()
            failed += 1
    
    print("\n" + "="*60)
    print(f"Results: {passed} passed, {failed} failed")
    print("="*60)
    
    return failed == 0


if __name__ == '__main__':
    success = run_all_tests()
    sys.exit(0 if success else 1)
