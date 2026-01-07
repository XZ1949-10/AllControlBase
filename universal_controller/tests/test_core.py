"""核心模块测试 - 符合 v3.17.6 需求"""
import numpy as np
import time
import sys
import os

# 添加项目路径
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from universal_controller.core.data_types import (
    Trajectory, ControlOutput, ConsistencyResult, Point3D, Header,
    DiagnosticsV2, Odometry, Imu, TimeoutStatus
)
from universal_controller.core.enums import ControllerState, TrajectoryMode, TransformStatus
from universal_controller.config.default_config import DEFAULT_CONFIG, get_config_value, PLATFORM_CONFIG


def test_control_output_copy():
    """测试 ControlOutput 深拷贝 (v3.17)"""
    original = ControlOutput(
        vx=1.0, vy=0.5, vz=0.0, omega=0.1,
        frame_id="world", success=True,
        solve_time_ms=5.0,
        health_metrics={'kkt_residual': 0.001, 'condition_number': 100}
    )
    
    copied = original.copy()
    
    # 修改原始对象
    original.vx = 2.0
    original.health_metrics['kkt_residual'] = 0.1
    
    # 验证拷贝不受影响
    assert copied.vx == 1.0, f"Expected 1.0, got {copied.vx}"
    assert copied.health_metrics['kkt_residual'] == 0.001
    print("✓ test_control_output_copy passed")


def test_trajectory_get_velocities_unified():
    """测试 Trajectory.get_velocities() 统一接口 (v3.17.5)"""
    # 测试 1: soft_enabled=True 且有 velocities
    traj1 = Trajectory(
        header=Header(stamp=time.time(), frame_id="world"),
        points=[Point3D(0, 0, 0), Point3D(1, 0, 0)],
        velocities=np.array([[1.0, 0.0, 0.0, 0.0]]),
        dt_sec=0.1,
        confidence=0.9,
        soft_enabled=True
    )
    vel1 = traj1.get_velocities()
    assert np.allclose(vel1[0], [1.0, 0.0, 0.0, 0.0])
    assert traj1.has_valid_soft_velocities() == True
    
    # 测试 2: soft_enabled=False，应返回 hard velocities
    traj2 = Trajectory(
        header=Header(stamp=time.time(), frame_id="world"),
        points=[Point3D(0, 0, 0), Point3D(1, 0, 0)],
        velocities=np.array([[1.0, 0.0, 0.0, 0.0]]),
        dt_sec=0.1,
        confidence=0.9,
        soft_enabled=False
    )
    vel2 = traj2.get_velocities()
    assert vel2.shape[1] == 4
    assert traj2.has_valid_soft_velocities() == False
    
    # 测试 3: velocities=None，应返回 hard velocities
    traj3 = Trajectory(
        header=Header(stamp=time.time(), frame_id="world"),
        points=[Point3D(0, 0, 0), Point3D(1, 0, 0)],
        velocities=None,
        dt_sec=0.1,
        confidence=0.9,
        soft_enabled=True
    )
    vel3 = traj3.get_velocities()
    assert vel3 is not None
    assert vel3.shape[1] == 4
    
    print("✓ test_trajectory_get_velocities_unified passed")


def test_diagnostics_v2_to_ros_msg():
    """测试 DiagnosticsV2.to_ros_msg() (v3.17.6)"""
    diag = DiagnosticsV2(
        header=Header(stamp=time.time(), frame_id=''),
        state=int(ControllerState.NORMAL),
        mpc_success=True,
        mpc_solve_time_ms=5.0,
        backup_active=False,
        mpc_health_kkt_residual=0.001,
        mpc_health_condition_number=100.0,
        mpc_health_consecutive_near_timeout=0,
        mpc_health_degradation_warning=False,
        mpc_health_can_recover=True,
        consistency_curvature=0.9,
        consistency_velocity_dir=0.95,
        consistency_temporal=0.85,
        consistency_alpha_soft=0.8,
        consistency_data_valid=True,
        estimator_covariance_norm=0.1,
        estimator_innovation_norm=0.05,
        estimator_slip_probability=0.0,
        estimator_imu_drift_detected=False,
        estimator_imu_bias=np.zeros(3),
        estimator_imu_available=True,
        tracking_lateral_error=0.1,
        tracking_longitudinal_error=0.2,
        tracking_heading_error=0.05,
        tracking_prediction_error=0.0,
        transform_tf2_available=True,
        transform_tf2_injected=False,
        transform_fallback_duration_ms=0.0,
        transform_accumulated_drift=0.0,
        transform_source_frame='base_link',
        transform_target_frame='odom',
        transform_error_message='',
        timeout_odom=False,
        timeout_traj=False,
        timeout_traj_grace_exceeded=False,
        timeout_imu=False,
        timeout_last_odom_age_ms=10.0,
        timeout_last_traj_age_ms=15.0,
        timeout_last_imu_age_ms=5.0,
        timeout_in_startup_grace=False,
        cmd_vx=1.0,
        cmd_vy=0.0,
        cmd_vz=0.0,
        cmd_omega=0.1,
        cmd_frame_id="base_link",
        transition_progress=0.0
    )
    
    ros_msg = diag.to_ros_msg()
    
    assert isinstance(ros_msg, dict)
    assert ros_msg['state'] == int(ControllerState.NORMAL)
    assert ros_msg['mpc_success'] == True
    assert 'mpc_health' in ros_msg
    assert 'consistency' in ros_msg
    assert 'timeout' in ros_msg
    assert ros_msg['cmd']['vx'] == 1.0
    
    print("✓ test_diagnostics_v2_to_ros_msg passed")


def test_default_config_completeness():
    """测试默认配置完整性 (v3.17.5)"""
    required_paths = [
        'system.ctrl_freq',
        'mpc.horizon',
        'watchdog.odom_timeout_ms',
        'consistency.alpha_min',
        'safety.v_stop_thresh',
        'transform.target_frame',
        'backup.lookahead_dist',
        'constraints.v_max',
        'ekf.adaptive.base_slip_thresh',
    ]
    
    for path in required_paths:
        # 直接从 DEFAULT_CONFIG 获取值
        value = get_config_value(DEFAULT_CONFIG, path)
        assert value is not None, f"Missing default for {path}"
    
    print("✓ test_default_config_completeness passed")


def test_get_config_value():
    """测试配置获取方法 (v3.17.5)"""
    config = {
        'mpc': {
            'horizon': 30,  # 覆盖默认值
        }
    }
    
    # 测试覆盖值
    assert get_config_value(config, 'mpc.horizon') == 30
    
    # 测试使用 fallback_config 获取默认值
    # mpc.dt 默认值为 0.1，与 trajectory.default_dt_sec 一致
    assert get_config_value(config, 'mpc.dt', fallback_config=DEFAULT_CONFIG) == 0.1
    
    # 测试不存在的路径
    assert get_config_value(config, 'nonexistent.path', 'default') == 'default'
    
    print("✓ test_get_config_value passed")


def test_transform_status():
    """测试 TransformStatus 枚举方法"""
    assert TransformStatus.TF2_OK.is_critical() == False
    assert TransformStatus.TF2_OK.is_fallback() == False
    assert TransformStatus.FALLBACK_CRITICAL.is_critical() == True
    assert TransformStatus.FALLBACK_WARNING.is_fallback() == True
    
    print("✓ test_transform_status passed")


def test_trajectory_copy():
    """测试 Trajectory 深拷贝"""
    original = Trajectory(
        header=Header(stamp=time.time(), frame_id="world"),
        points=[Point3D(0, 0, 0), Point3D(1, 1, 0)],
        velocities=np.array([[1.0, 0.5, 0.0, 0.1]]),
        dt_sec=0.1,
        confidence=0.9,
        soft_enabled=True
    )
    
    copied = original.copy()
    
    # 修改原始对象
    # 修改原始对象
    # 注意：points 默认是只读的，需要先解锁才能修改以验证深拷贝
    original.points.flags.writeable = True
    original.points[0, 0] = 100.0
    original.velocities[0, 0] = 100
    
    # 验证拷贝不受影响
    assert copied.points[0, 0] == 0.0
    assert copied.velocities[0, 0] == 1.0
    
    print("✓ test_trajectory_copy passed")


if __name__ == '__main__':
    test_control_output_copy()
    test_trajectory_get_velocities_unified()
    test_diagnostics_v2_to_ros_msg()
    test_default_config_completeness()
    test_get_config_value()
    test_transform_status()
    test_trajectory_copy()
    print("\n✅ All core tests passed!")
