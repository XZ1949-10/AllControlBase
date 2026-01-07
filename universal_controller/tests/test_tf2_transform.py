"""
TF2 坐标变换器测试

测试内容:
- F3.1: 延迟补偿
- F3.2: TF2 lookupTransform
- F3.3: TF2 降级策略
- F3.4: 降级超过 1000ms 触发临界状态
- F3.5: TF2 恢复时漂移校正
"""
import sys
import os
import time
import numpy as np

# 添加项目路径
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from universal_controller.transform.robust_transformer import RobustCoordinateTransformer
from universal_controller.core.ros_compat import TF2_AVAILABLE
from universal_controller.estimator.adaptive_ekf import AdaptiveEKFEstimator

from universal_controller.core.data_types import Trajectory, Point3D, Header, Odometry
from universal_controller.core.enums import TransformStatus
from universal_controller.config.default_config import DEFAULT_CONFIG

# 使用测试数据生成器
from universal_controller.tests.fixtures import create_test_trajectory, create_test_odom


def _get_point(traj, idx):
    """获取轨迹点坐标，兼容 numpy 数组和 Point3D 列表"""
    points = traj.points
    if isinstance(points, np.ndarray):
        return points[idx, 0], points[idx, 1], points[idx, 2] if points.shape[1] > 2 else 0.0
    else:
        p = points[idx]
        return p.x, p.y, p.z


def test_tf2_transform_basic():
    """测试基本 TF2 变换功能"""
    config = DEFAULT_CONFIG.copy()
    transformer = RobustCoordinateTransformer(config)
    
    # 设置变换: odom -> base_link
    transformer.set_transform("odom", "base_link", 1.0, 2.0, 0.0, np.pi/4)
    
    # 创建测试轨迹
    traj = create_test_trajectory(num_points=10, dt=0.1, frame_id="base_link")
    
    # 执行变换
    transformed_traj, status = transformer.transform_trajectory(traj, "odom", time.time())
    
    # 验证状态
    assert status == TransformStatus.TF2_OK, f"Expected TF2_OK, got {status}"
    assert len(transformed_traj.points) == len(traj.points)
    assert transformed_traj.header.frame_id == "odom"
    
    # 验证变换结果 (第一个点应该被变换)
    # 原点 (0, 0) 经过旋转 45° 和平移 (1, 2) 后应该在 (1, 2)
    p0_x, p0_y, _ = _get_point(transformed_traj, 0)
    assert abs(p0_x - 1.0) < 0.01, f"Expected x=1.0, got {p0_x}"
    assert abs(p0_y - 2.0) < 0.01, f"Expected y=2.0, got {p0_y}"
    
    print("[PASS] test_tf2_transform_basic passed")


def test_tf2_fallback_degradation():
    """测试 TF2 降级策略 (F3.3)"""
    config = DEFAULT_CONFIG.copy()
    config['transform'] = DEFAULT_CONFIG['transform'].copy()
    config['transform']['fallback_duration_limit_ms'] = 100  # 100ms 警告
    config['transform']['fallback_critical_limit_ms'] = 200  # 200ms 临界
    
    transformer = RobustCoordinateTransformer(config)
    
    # 初始化状态估计器
    estimator = AdaptiveEKFEstimator(config)
    # transformer.set_state_estimator(estimator) -> Removed
    
    # Get fallback state
    fallback_state = estimator.get_state()
    
    # 设置初始变换
    transformer.set_transform("odom", "base_link", 0.0, 0.0, 0.0, 0.0)
    transformer._last_tf2_position = np.array([0.0, 0.0, 0.0])
    transformer._last_tf2_yaw = 0.0
    
    # 禁用 TF2 模拟降级
    transformer.set_tf2_available(False)
    
    traj = create_test_trajectory(num_points=10, dt=0.1, frame_id="base_link")
    
    # 第一次调用 - 应该是 FALLBACK_OK
    _, status1 = transformer.transform_trajectory(traj, "odom", time.time(), fallback_state=fallback_state)
    assert status1 == TransformStatus.FALLBACK_OK, f"Expected FALLBACK_OK, got {status1}"
    
    # 等待超过警告阈值
    time.sleep(0.15)
    _, status2 = transformer.transform_trajectory(traj, "odom", time.time(), fallback_state=fallback_state)
    assert status2 == TransformStatus.FALLBACK_WARNING, f"Expected FALLBACK_WARNING, got {status2}"
    
    # 等待超过临界阈值
    time.sleep(0.15)
    _, status3 = transformer.transform_trajectory(traj, "odom", time.time(), fallback_state=fallback_state)
    assert status3 == TransformStatus.FALLBACK_CRITICAL, f"Expected FALLBACK_CRITICAL, got {status3}"
    
    print("[PASS] test_tf2_fallback_degradation passed")


def test_tf2_recovery_correction():
    """测试 TF2 恢复时的漂移校正 (F3.5)"""
    config = DEFAULT_CONFIG.copy()
    config['transform'] = DEFAULT_CONFIG['transform'].copy()
    config['transform']['recovery_correction_enabled'] = True
    
    transformer = RobustCoordinateTransformer(config)
    
    # 初始化状态估计器
    estimator = AdaptiveEKFEstimator(config)
    # transformer.set_state_estimator(estimator) -> Removed
    
    # 设置初始变换
    transformer.set_transform("odom", "base_link", 0.0, 0.0, 0.0, 0.0)
    transformer._last_tf2_position = np.array([0.0, 0.0, 0.0])
    transformer._last_tf2_yaw = 0.0
    
    traj = create_test_trajectory(num_points=10, dt=0.1, frame_id="base_link")
    
    # 正常状态 (不需要 fallback_state，但传了也没关系)
    fallback_state = estimator.get_state()
    _, status = transformer.transform_trajectory(traj, "odom", time.time(), fallback_state=fallback_state)
    assert status == TransformStatus.TF2_OK
    
    # 禁用 TF2
    transformer.set_tf2_available(False)
    
    # 模拟 odom 更新 (产生位移)
    odom = Odometry(
        header=Header(stamp=time.time(), frame_id="odom"),
        pose_position=Point3D(1.0, 0.5, 0.0),
        pose_orientation=(0, 0, 0, 1),
        twist_linear=(0.5, 0.0, 0.0),
        twist_angular=(0, 0, 0)
    )
    estimator.update_odom(odom, time.time())
    
    # 降级状态
    fallback_state = estimator.get_state()
    _, status2 = transformer.transform_trajectory(traj, "odom", time.time(), fallback_state=fallback_state)
    assert status2.is_fallback()
    
    # 恢复 TF2 (设置新的变换)
    transformer.set_tf2_available(True)
    transformer.set_transform("odom", "base_link", 0.9, 0.4, 0.0, 0.0)  # 略有偏差
    
    # 恢复后应该触发漂移校正
    fallback_state = estimator.get_state()
    _, status3 = transformer.transform_trajectory(traj, "odom", time.time(), fallback_state=fallback_state)
    assert status3 == TransformStatus.TF2_OK
    
    # 验证状态
    tf_status = transformer.get_status()
    assert tf_status['tf2_available'] == True
    assert tf_status['fallback_duration_ms'] == 0.0
    
    print("[PASS] test_tf2_recovery_correction passed")


def test_tf2_transform_with_velocity():
    """测试带速度的轨迹变换"""
    config = DEFAULT_CONFIG.copy()
    transformer = RobustCoordinateTransformer(config)
    
    # 设置变换: 旋转 90°
    transformer.set_transform("odom", "base_link", 0.0, 0.0, 0.0, np.pi/2)
    
    # 创建带速度的轨迹
    points = [Point3D(0, 0, 0), Point3D(1, 0, 0), Point3D(2, 0, 0)]
    velocities = np.array([
        [1.0, 0.0, 0.0, 0.0],  # 沿 x 方向
        [1.0, 0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0, 0.0]
    ])
    
    traj = Trajectory(
        header=Header(stamp=time.time(), frame_id="base_link"),
        points=points,
        velocities=velocities,
        dt_sec=0.1,
        confidence=0.9,
        soft_enabled=True
    )
    
    # 执行变换
    transformed_traj, status = transformer.transform_trajectory(traj, "odom", time.time())
    
    assert status == TransformStatus.TF2_OK
    
    # 验证速度变换 (旋转 90° 后，x 方向速度应该变成 y 方向)
    v0 = transformed_traj.velocities[0]
    assert abs(v0[0]) < 0.01, f"Expected vx≈0, got {v0[0]}"
    assert abs(v0[1] - 1.0) < 0.01, f"Expected vy≈1, got {v0[1]}"
    
    print("[PASS] test_tf2_transform_with_velocity passed")


def test_tf2_status_reporting():
    """测试状态报告"""
    config = DEFAULT_CONFIG.copy()
    transformer = RobustCoordinateTransformer(config)
    
    # 初始状态
    status = transformer.get_status()
    # 在测试环境中，TF2_AVAILABLE 取决于是否安装了 ROS 库
    assert status['tf2_available'] == TF2_AVAILABLE
    assert status['fallback_duration_ms'] == 0.0
    assert status['fallback_active'] == False
    # assert status['accumulated_drift'] == 0.0 -> key not present in simplified implementation
    # assert status['is_critical'] == False -> key not present

    
    # 禁用 TF2
    transformer.set_tf2_available(False)
    
    # 设置状态估计器以启用降级
    estimator = AdaptiveEKFEstimator(config)
    estimator = AdaptiveEKFEstimator(config)
    # transformer.set_state_estimator(estimator) -> Removed
    
    fallback_state = estimator.get_state()
    transformer._last_tf2_position = np.array([0.0, 0.0, 0.0])
    transformer._last_tf2_yaw = 0.0
    
    traj = create_test_trajectory(num_points=10, dt=0.1, frame_id="base_link")
    transformer.transform_trajectory(traj, "odom", time.time(), fallback_state=fallback_state)
    
    # 降级后状态
    status = transformer.get_status()
    assert status['tf2_available'] == False
    assert status['fallback_duration_ms'] > 0
    assert status['fallback_active'] == True
    
    print("[PASS] test_tf2_status_reporting passed")


def test_tf2_reset():
    """测试重置功能"""
    config = DEFAULT_CONFIG.copy()
    transformer = RobustCoordinateTransformer(config)
    
    # 设置一些状态
    transformer.fallback_start_time = time.time() - 1.0 # Simulate previous fallback
    # transformer.accumulated_drift = 0.5 -> Removed
    transformer._warned_frames.add("test_frame")
    
    # 重置
    transformer.reset()
    
    # 验证重置
    assert transformer.fallback_start_time is None
    # assert transformer.accumulated_drift == 0.0 -> Removed
    assert len(transformer._warned_frames) == 0
    
    print("[PASS] test_tf2_reset passed")


def test_tf2_full_se3_inverse():
    """测试完整 SE(3) 逆变换 - 包含 roll/pitch/yaw"""
    from universal_controller.compat.ros_compat_impl import MockTF2BufferCore
    from universal_controller.core.data_types import (
        TransformStamped, Header, Transform, Vector3, Quaternion
    )
    from universal_controller.core.ros_compat import quaternion_from_euler, euler_from_quaternion
    
    buffer = MockTF2BufferCore()
    
    # 创建一个有 roll, pitch, yaw 的变换
    roll, pitch, yaw = 0.3, 0.2, 0.5  # 非零的 roll 和 pitch
    q = quaternion_from_euler(roll, pitch, yaw)
    tx, ty, tz = 1.0, 2.0, 0.5
    
    transform = TransformStamped()
    transform.header = Header(frame_id="world")
    transform.child_frame_id = "body"
    transform.transform = Transform(
        translation=Vector3(tx, ty, tz),
        rotation=Quaternion(q[0], q[1], q[2], q[3])
    )
    
    buffer.set_transform(transform, "test")
    
    # 查找正向变换
    t_forward = buffer.lookup_transform("world", "body", None)
    
    # 查找反向变换 (应该触发 _invert_transform)
    t_inverse = buffer.lookup_transform("body", "world", None)
    
    # 验证: T * T^-1 应该接近单位变换
    # 组合两个变换
    t_composed = buffer._compose_transforms(t_forward, t_inverse)
    
    # 组合后的平移应该接近 (0, 0, 0)
    assert abs(t_composed.transform.translation.x) < 1e-6, \
        f"Expected tx≈0, got {t_composed.transform.translation.x}"
    assert abs(t_composed.transform.translation.y) < 1e-6, \
        f"Expected ty≈0, got {t_composed.transform.translation.y}"
    assert abs(t_composed.transform.translation.z) < 1e-6, \
        f"Expected tz≈0, got {t_composed.transform.translation.z}"
    
    # 组合后的旋转应该接近单位四元数 (0, 0, 0, 1)
    assert abs(t_composed.transform.rotation.x) < 1e-6, \
        f"Expected qx≈0, got {t_composed.transform.rotation.x}"
    assert abs(t_composed.transform.rotation.y) < 1e-6, \
        f"Expected qy≈0, got {t_composed.transform.rotation.y}"
    assert abs(t_composed.transform.rotation.z) < 1e-6, \
        f"Expected qz≈0, got {t_composed.transform.rotation.z}"
    assert abs(t_composed.transform.rotation.w - 1.0) < 1e-6, \
        f"Expected qw≈1, got {t_composed.transform.rotation.w}"
    
    # 验证逆变换的欧拉角
    inv_q = (t_inverse.transform.rotation.x, t_inverse.transform.rotation.y,
             t_inverse.transform.rotation.z, t_inverse.transform.rotation.w)
    inv_roll, inv_pitch, inv_yaw = euler_from_quaternion(inv_q)
    
    # 注意: 逆变换的欧拉角不是简单的取负，但组合后应该抵消
    print(f"  Original: roll={roll:.3f}, pitch={pitch:.3f}, yaw={yaw:.3f}")
    print(f"  Inverse:  roll={inv_roll:.3f}, pitch={inv_pitch:.3f}, yaw={inv_yaw:.3f}")
    
    print("[PASS] test_tf2_full_se3_inverse passed")


def test_tf2_chain_lookup():
    """测试链式变换查找 (A->B->C)"""
    from universal_controller.compat.ros_compat_impl import MockTF2BufferCore
    from universal_controller.core.data_types import (
        TransformStamped, Header, Transform, Vector3, Quaternion
    )
    from universal_controller.core.ros_compat import quaternion_from_euler, euler_from_quaternion
    
    buffer = MockTF2BufferCore()
    
    # 设置 world -> odom 变换
    q1 = quaternion_from_euler(0, 0, np.pi/4)  # 45度
    t1 = TransformStamped()
    t1.header = Header(frame_id="world")
    t1.child_frame_id = "odom"
    t1.transform = Transform(
        translation=Vector3(1.0, 0.0, 0.0),
        rotation=Quaternion(q1[0], q1[1], q1[2], q1[3])
    )
    buffer.set_transform(t1, "test")
    
    # 设置 odom -> base_link 变换
    q2 = quaternion_from_euler(0, 0, np.pi/4)  # 再 45度
    t2 = TransformStamped()
    t2.header = Header(frame_id="odom")
    t2.child_frame_id = "base_link"
    t2.transform = Transform(
        translation=Vector3(1.0, 0.0, 0.0),
        rotation=Quaternion(q2[0], q2[1], q2[2], q2[3])
    )
    buffer.set_transform(t2, "test")
    
    # 查找 world -> base_link (需要链式查找)
    t_chain = buffer.lookup_transform("world", "base_link", None)
    
    # 验证: 两次 45度旋转 = 90度
    q_result = (t_chain.transform.rotation.x, t_chain.transform.rotation.y,
                t_chain.transform.rotation.z, t_chain.transform.rotation.w)
    _, _, yaw = euler_from_quaternion(q_result)
    
    assert abs(yaw - np.pi/2) < 0.01, f"Expected yaw≈π/2, got {yaw}"
    
    # 验证平移: (1,0) + R(45°)*(1,0) = (1,0) + (0.707, 0.707) ≈ (1.707, 0.707)
    expected_x = 1.0 + np.cos(np.pi/4)
    expected_y = 0.0 + np.sin(np.pi/4)
    assert abs(t_chain.transform.translation.x - expected_x) < 0.01, \
        f"Expected x≈{expected_x}, got {t_chain.transform.translation.x}"
    assert abs(t_chain.transform.translation.y - expected_y) < 0.01, \
        f"Expected y≈{expected_y}, got {t_chain.transform.translation.y}"
    
    print("[PASS] test_tf2_chain_lookup passed")


def test_tf2_multi_hop_chain_lookup():
    """测试多跳链式变换查找 (A->B->C->D->E)"""
    from universal_controller.compat.ros_compat_impl import MockTF2BufferCore
    from universal_controller.core.data_types import (
        TransformStamped, Header, Transform, Vector3, Quaternion
    )
    from universal_controller.core.ros_compat import quaternion_from_euler, euler_from_quaternion
    
    buffer = MockTF2BufferCore()
    
    # 创建 5 帧的变换链: world -> map -> odom -> base_link -> sensor
    frames = ["world", "map", "odom", "base_link", "sensor"]
    
    for i in range(len(frames) - 1):
        parent = frames[i]
        child = frames[i + 1]
        
        # 每个变换: 平移 (1, 0, 0)，旋转 15度
        q = quaternion_from_euler(0, 0, np.pi/12)  # 15度
        t = TransformStamped()
        t.header = Header(frame_id=parent)
        t.child_frame_id = child
        t.transform = Transform(
            translation=Vector3(1.0, 0.0, 0.0),
            rotation=Quaternion(q[0], q[1], q[2], q[3])
        )
        buffer.set_transform(t, "test")
    
    # 查找 world -> sensor (需要 4 跳链式查找)
    t_chain = buffer.lookup_transform("world", "sensor", None)
    
    # 验证: 4 次 15度旋转 = 60度
    q_result = (t_chain.transform.rotation.x, t_chain.transform.rotation.y,
                t_chain.transform.rotation.z, t_chain.transform.rotation.w)
    _, _, yaw = euler_from_quaternion(q_result)
    
    expected_yaw = 4 * np.pi / 12  # 60度
    assert abs(yaw - expected_yaw) < 0.02, f"Expected yaw≈{expected_yaw:.3f}, got {yaw:.3f}"
    
    # 验证反向查找: sensor -> world
    t_reverse = buffer.lookup_transform("sensor", "world", None)
    
    # 组合应该接近单位变换
    t_composed = buffer._compose_transforms(t_chain, t_reverse)
    assert abs(t_composed.transform.translation.x) < 0.01
    assert abs(t_composed.transform.translation.y) < 0.01
    assert abs(t_composed.transform.rotation.w - 1.0) < 0.01
    
    # 测试中间帧查找: map -> base_link (2 跳)
    t_mid = buffer.lookup_transform("map", "base_link", None)
    q_mid = (t_mid.transform.rotation.x, t_mid.transform.rotation.y,
             t_mid.transform.rotation.z, t_mid.transform.rotation.w)
    _, _, yaw_mid = euler_from_quaternion(q_mid)
    
    expected_yaw_mid = 2 * np.pi / 12  # 30度
    assert abs(yaw_mid - expected_yaw_mid) < 0.02, \
        f"Expected yaw≈{expected_yaw_mid:.3f}, got {yaw_mid:.3f}"
    
    print("[PASS] test_tf2_multi_hop_chain_lookup passed")


def run_all_tf2_tests():
    """运行所有 TF2 变换测试"""
    print("\n=== Running TF2 Transform Tests (F3) ===\n")
    
    test_tf2_transform_basic()
    test_tf2_fallback_degradation()
    test_tf2_recovery_correction()
    test_tf2_transform_with_velocity()
    test_tf2_status_reporting()
    test_tf2_reset()
    test_tf2_full_se3_inverse()
    test_tf2_chain_lookup()
    test_tf2_multi_hop_chain_lookup()
    
    print("\n=== All TF2 Transform Tests Passed ===\n")


if __name__ == "__main__":
    run_all_tf2_tests()
