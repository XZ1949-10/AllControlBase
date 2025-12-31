"""DiagnosticsPublisher 测试"""
import pytest
from universal_controller.diagnostics.publisher import DiagnosticsPublisher
from universal_controller.core.data_types import (
    ControlOutput, EstimatorOutput, ConsistencyResult, 
    MPCHealthStatus, TimeoutStatus
)
from universal_controller.core.enums import ControllerState
import numpy as np


def test_diagnostics_publisher_init():
    """测试 DiagnosticsPublisher 初始化"""
    publisher = DiagnosticsPublisher()
    
    assert publisher.get_last_published() is None
    assert len(publisher._callbacks) == 0
    
    print("✓ test_diagnostics_publisher_init passed")


def test_diagnostics_publisher_callback():
    """测试诊断回调功能"""
    publisher = DiagnosticsPublisher()
    
    received_data = []
    
    def callback(data):
        received_data.append(data)
    
    publisher.add_callback(callback)
    
    # 创建测试数据
    cmd = ControlOutput(vx=1.0, vy=0.0, vz=0.0, omega=0.5, success=True)
    timeout_status = TimeoutStatus(
        odom_timeout=False, traj_timeout=False, traj_grace_exceeded=False,
        imu_timeout=False, last_odom_age_ms=10.0, last_traj_age_ms=20.0,
        last_imu_age_ms=15.0, in_startup_grace=False
    )
    
    publisher.publish(
        current_time=1000.0,
        state=ControllerState.NORMAL,
        cmd=cmd,
        state_output=None,
        consistency=None,
        mpc_health=None,
        timeout_status=timeout_status,
        transform_status={'fallback_duration_ms': 0.0, 'accumulated_drift': 0.0},
        tracking_error=None,
        transition_progress=0.0,
        tf2_critical=False
    )
    
    assert len(received_data) == 1
    assert received_data[0]['state'] == int(ControllerState.NORMAL)
    assert received_data[0]['cmd']['vx'] == 1.0
    
    print("✓ test_diagnostics_publisher_callback passed")


def test_diagnostics_publisher_multiple_callbacks():
    """测试多个回调函数"""
    publisher = DiagnosticsPublisher()
    
    results1 = []
    results2 = []
    
    def callback1(data):
        results1.append(data)
    
    def callback2(data):
        results2.append(data)
    
    publisher.add_callback(callback1)
    publisher.add_callback(callback2)
    
    cmd = ControlOutput(vx=0.5, vy=0.0, vz=0.0, omega=0.0, success=True)
    timeout_status = TimeoutStatus(
        odom_timeout=False, traj_timeout=False, traj_grace_exceeded=False,
        imu_timeout=False, last_odom_age_ms=10.0, last_traj_age_ms=20.0,
        last_imu_age_ms=15.0, in_startup_grace=False
    )
    
    publisher.publish(
        current_time=1000.0,
        state=ControllerState.NORMAL,
        cmd=cmd,
        state_output=None,
        consistency=None,
        mpc_health=None,
        timeout_status=timeout_status,
        transform_status={'fallback_duration_ms': 0.0, 'accumulated_drift': 0.0},
        tracking_error=None,
        transition_progress=0.0,
        tf2_critical=False
    )
    
    assert len(results1) == 1
    assert len(results2) == 1
    
    # 移除一个回调
    publisher.remove_callback(callback1)
    
    publisher.publish(
        current_time=1001.0,
        state=ControllerState.NORMAL,
        cmd=cmd,
        state_output=None,
        consistency=None,
        mpc_health=None,
        timeout_status=timeout_status,
        transform_status={'fallback_duration_ms': 0.0, 'accumulated_drift': 0.0},
        tracking_error=None,
        transition_progress=0.0,
        tf2_critical=False
    )
    
    assert len(results1) == 1  # 不再增加
    assert len(results2) == 2  # 继续增加
    
    print("✓ test_diagnostics_publisher_multiple_callbacks passed")


def test_diagnostics_publisher_clear_callbacks():
    """测试清除所有回调"""
    publisher = DiagnosticsPublisher()
    
    results = []
    
    def callback(data):
        results.append(data)
    
    publisher.add_callback(callback)
    publisher.clear_callbacks()
    
    cmd = ControlOutput(vx=0.5, vy=0.0, vz=0.0, omega=0.0, success=True)
    timeout_status = TimeoutStatus(
        odom_timeout=False, traj_timeout=False, traj_grace_exceeded=False,
        imu_timeout=False, last_odom_age_ms=10.0, last_traj_age_ms=20.0,
        last_imu_age_ms=15.0, in_startup_grace=False
    )
    
    publisher.publish(
        current_time=1000.0,
        state=ControllerState.NORMAL,
        cmd=cmd,
        state_output=None,
        consistency=None,
        mpc_health=None,
        timeout_status=timeout_status,
        transform_status={'fallback_duration_ms': 0.0, 'accumulated_drift': 0.0},
        tracking_error=None,
        transition_progress=0.0,
        tf2_critical=False
    )
    
    assert len(results) == 0  # 回调已清除
    
    print("✓ test_diagnostics_publisher_clear_callbacks passed")


def test_diagnostics_publisher_with_full_data():
    """测试完整数据发布"""
    publisher = DiagnosticsPublisher()
    
    received_data = []
    publisher.add_callback(lambda d: received_data.append(d))
    
    # 创建完整的测试数据
    cmd = ControlOutput(vx=1.5, vy=0.2, vz=0.1, omega=0.3, success=True, solve_time_ms=5.0)
    
    state_output = EstimatorOutput(
        state=np.array([1.0, 2.0, 0.0, 1.5, 0.2, 0.1, 0.5, 0.3]),
        covariance=np.eye(8) * 0.1,
        covariance_norm=0.8,
        innovation_norm=0.1,
        imu_bias=np.array([0.01, 0.02, 0.03]),
        slip_probability=0.1,
        anomalies=[],
        imu_available=True,
        imu_drift_detected=False
    )
    
    consistency = ConsistencyResult(
        alpha=0.9,
        kappa_consistency=0.95,
        v_dir_consistency=0.98,
        temporal_smooth=0.92,
        should_disable_soft=False,
        data_valid=True
    )
    
    mpc_health = MPCHealthStatus(
        healthy=True,
        degradation_warning=False,
        can_recover=True,
        consecutive_near_timeout=0,
        kkt_residual=0.001,
        condition_number=100.0
    )
    
    timeout_status = TimeoutStatus(
        odom_timeout=False, traj_timeout=False, traj_grace_exceeded=False,
        imu_timeout=False, last_odom_age_ms=10.0, last_traj_age_ms=20.0,
        last_imu_age_ms=15.0, in_startup_grace=False
    )
    
    tracking_error = {
        'lateral_error': 0.05,
        'longitudinal_error': 0.1,
        'heading_error': 0.02,
        'prediction_error': 0.03
    }
    
    publisher.publish(
        current_time=1000.0,
        state=ControllerState.NORMAL,
        cmd=cmd,
        state_output=state_output,
        consistency=consistency,
        mpc_health=mpc_health,
        timeout_status=timeout_status,
        transform_status={'fallback_duration_ms': 0.0, 'accumulated_drift': 0.0},
        tracking_error=tracking_error,
        transition_progress=0.5,
        tf2_critical=False
    )
    
    assert len(received_data) == 1
    data = received_data[0]
    
    # 验证各字段
    assert data['state'] == int(ControllerState.NORMAL)
    assert data['mpc_success'] == True
    assert data['mpc_solve_time_ms'] == 5.0
    
    assert data['consistency']['alpha_soft'] == 0.9
    assert data['consistency']['curvature'] == 0.95
    
    assert data['mpc_health']['kkt_residual'] == 0.001
    # 注意: DiagnosticsV2 中使用 degradation_warning 而非 healthy
    assert data['mpc_health']['degradation_warning'] == False
    
    assert data['estimator_health']['covariance_norm'] == 0.8
    assert data['estimator_health']['imu_available'] == True
    
    assert data['tracking']['lateral_error'] == 0.05
    
    assert data['cmd']['vx'] == 1.5
    assert data['cmd']['omega'] == 0.3
    
    assert data['transition_progress'] == 0.5
    
    print("✓ test_diagnostics_publisher_with_full_data passed")


def test_diagnostics_publisher_get_last_published():
    """测试获取最后发布的数据"""
    publisher = DiagnosticsPublisher()
    
    cmd = ControlOutput(vx=1.0, vy=0.0, vz=0.0, omega=0.0, success=True)
    timeout_status = TimeoutStatus(
        odom_timeout=False, traj_timeout=False, traj_grace_exceeded=False,
        imu_timeout=False, last_odom_age_ms=10.0, last_traj_age_ms=20.0,
        last_imu_age_ms=15.0, in_startup_grace=False
    )
    
    publisher.publish(
        current_time=1000.0,
        state=ControllerState.NORMAL,
        cmd=cmd,
        state_output=None,
        consistency=None,
        mpc_health=None,
        timeout_status=timeout_status,
        transform_status={'fallback_duration_ms': 0.0, 'accumulated_drift': 0.0},
        tracking_error=None,
        transition_progress=0.0,
        tf2_critical=False
    )
    
    last = publisher.get_last_published()
    assert last is not None
    assert last['cmd']['vx'] == 1.0
    
    # 发布新数据
    cmd2 = ControlOutput(vx=2.0, vy=0.0, vz=0.0, omega=0.0, success=True)
    publisher.publish(
        current_time=1001.0,
        state=ControllerState.NORMAL,
        cmd=cmd2,
        state_output=None,
        consistency=None,
        mpc_health=None,
        timeout_status=timeout_status,
        transform_status={'fallback_duration_ms': 0.0, 'accumulated_drift': 0.0},
        tracking_error=None,
        transition_progress=0.0,
        tf2_critical=False
    )
    
    last = publisher.get_last_published()
    assert last['cmd']['vx'] == 2.0  # 应该是最新的
    
    print("✓ test_diagnostics_publisher_get_last_published passed")


def test_diagnostics_publisher_prediction_error_nan():
    """测试 prediction_error 的 NaN 语义保留"""
    import math
    publisher = DiagnosticsPublisher()
    
    received_data = []
    publisher.add_callback(lambda d: received_data.append(d))
    
    cmd = ControlOutput(vx=1.0, vy=0.0, vz=0.0, omega=0.0, success=True)
    timeout_status = TimeoutStatus(
        odom_timeout=False, traj_timeout=False, traj_grace_exceeded=False,
        imu_timeout=False, last_odom_age_ms=10.0, last_traj_age_ms=20.0,
        last_imu_age_ms=15.0, in_startup_grace=False
    )
    
    # 测试 1: tracking_error=None 时，prediction_error 应为 NaN
    publisher.publish(
        current_time=1000.0,
        state=ControllerState.NORMAL,
        cmd=cmd,
        state_output=None,
        consistency=None,
        mpc_health=None,
        timeout_status=timeout_status,
        transform_status={'fallback_duration_ms': 0.0, 'accumulated_drift': 0.0},
        tracking_error=None,  # 无跟踪误差数据
        transition_progress=0.0,
        tf2_critical=False
    )
    
    assert len(received_data) == 1
    assert math.isnan(received_data[0]['tracking']['prediction_error'])
    
    # 测试 2: tracking_error 存在但没有 prediction_error 时，应为 NaN
    received_data.clear()
    publisher.publish(
        current_time=1001.0,
        state=ControllerState.NORMAL,
        cmd=cmd,
        state_output=None,
        consistency=None,
        mpc_health=None,
        timeout_status=timeout_status,
        transform_status={'fallback_duration_ms': 0.0, 'accumulated_drift': 0.0},
        tracking_error={'lateral_error': 0.1},  # 没有 prediction_error
        transition_progress=0.0,
        tf2_critical=False
    )
    
    assert len(received_data) == 1
    assert math.isnan(received_data[0]['tracking']['prediction_error'])
    
    # 测试 3: prediction_error 显式为 NaN 时，应保留 NaN
    received_data.clear()
    publisher.publish(
        current_time=1002.0,
        state=ControllerState.NORMAL,
        cmd=cmd,
        state_output=None,
        consistency=None,
        mpc_health=None,
        timeout_status=timeout_status,
        transform_status={'fallback_duration_ms': 0.0, 'accumulated_drift': 0.0},
        tracking_error={'prediction_error': float('nan')},
        transition_progress=0.0,
        tf2_critical=False
    )
    
    assert len(received_data) == 1
    assert math.isnan(received_data[0]['tracking']['prediction_error'])
    
    # 测试 4: prediction_error 有有效值时，应正常传递
    received_data.clear()
    publisher.publish(
        current_time=1003.0,
        state=ControllerState.NORMAL,
        cmd=cmd,
        state_output=None,
        consistency=None,
        mpc_health=None,
        timeout_status=timeout_status,
        transform_status={'fallback_duration_ms': 0.0, 'accumulated_drift': 0.0},
        tracking_error={'prediction_error': 0.05},
        transition_progress=0.0,
        tf2_critical=False
    )
    
    assert len(received_data) == 1
    assert received_data[0]['tracking']['prediction_error'] == 0.05
    
    print("✓ test_diagnostics_publisher_prediction_error_nan passed")


if __name__ == '__main__':
    test_diagnostics_publisher_init()
    test_diagnostics_publisher_callback()
    test_diagnostics_publisher_multiple_callbacks()
    test_diagnostics_publisher_clear_callbacks()
    test_diagnostics_publisher_with_full_data()
    test_diagnostics_publisher_get_last_published()
    test_diagnostics_publisher_prediction_error_nan()
    print("\nAll DiagnosticsPublisher tests passed!")
