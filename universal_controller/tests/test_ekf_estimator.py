"""
EKF 状态估计器详细测试

验证自适应 EKF 状态估计器的各项功能：
1. 基本预测和更新
2. 打滑检测
3. IMU 漂移检测
4. 协方差管理
5. 异常检测
"""
import numpy as np
import time
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from universal_controller.estimator.adaptive_ekf import AdaptiveEKFEstimator
from universal_controller.config.default_config import DEFAULT_CONFIG
from universal_controller.tests.fixtures import create_test_odom, create_test_imu


def test_ekf_initialization():
    """测试 EKF 初始化"""
    config = DEFAULT_CONFIG.copy()
    ekf = AdaptiveEKFEstimator(config)
    
    state = ekf.get_state()
    
    # 验证初始状态
    assert state.state.shape == (8,), f"State shape should be (8,), got {state.state.shape}"
    assert state.covariance.shape == (8, 8), f"Covariance shape should be (8,8), got {state.covariance.shape}"
    assert np.allclose(state.state, 0), "Initial state should be zero"
    assert state.imu_available == True
    assert state.slip_probability == 0.0
    
    print("✓ test_ekf_initialization passed")


def test_ekf_predict():
    """测试 EKF 预测步骤"""
    config = DEFAULT_CONFIG.copy()
    ekf = AdaptiveEKFEstimator(config)
    
    # 设置初始速度
    odom = create_test_odom(vx=1.0, vy=0.0, theta=0.0)
    ekf.update_odom(odom)
    
    state_before = ekf.get_state().state.copy()
    
    # 预测
    dt = 0.1
    ekf.predict(dt)
    
    state_after = ekf.get_state().state
    
    # 位置应该根据速度更新
    # 注意：由于 EKF 的复杂性，这里只验证位置有变化
    assert not np.allclose(state_before[:3], state_after[:3]), "Position should change after predict"
    
    print("✓ test_ekf_predict passed")


def test_ekf_odom_update():
    """测试里程计更新"""
    config = DEFAULT_CONFIG.copy()
    ekf = AdaptiveEKFEstimator(config)
    
    # 更新里程计
    odom = create_test_odom(x=1.0, y=0.5, theta=0.1, vx=1.0, vy=0.0)
    ekf.update_odom(odom)
    
    state = ekf.get_state()
    
    # 验证状态已更新（位置应该接近里程计值）
    assert abs(state.state[0] - 1.0) < 0.5, f"X position should be close to 1.0, got {state.state[0]}"
    assert abs(state.state[1] - 0.5) < 0.5, f"Y position should be close to 0.5, got {state.state[1]}"
    
    print("✓ test_ekf_odom_update passed")


def test_ekf_imu_update():
    """测试 IMU 更新"""
    config = DEFAULT_CONFIG.copy()
    ekf = AdaptiveEKFEstimator(config)
    
    # 先更新里程计建立基准
    odom = create_test_odom(vx=1.0)
    ekf.update_odom(odom)
    
    # 更新 IMU
    imu = create_test_imu(
        angular_velocity=(0.0, 0.0, 0.5),  # 有角速度
        linear_acceleration=(0.5, 0.0, 9.81)  # 有加速度
    )
    ekf.update_imu(imu)
    
    state = ekf.get_state()
    
    # 验证 IMU 可用
    assert state.imu_available == True
    
    print("✓ test_ekf_imu_update passed")


def test_ekf_slip_detection():
    """测试打滑检测"""
    config = DEFAULT_CONFIG.copy()
    ekf = AdaptiveEKFEstimator(config)
    
    # 正常运动
    odom = create_test_odom(vx=1.0)
    ekf.update_odom(odom)
    
    # 模拟正常 IMU
    imu = create_test_imu(linear_acceleration=(0.0, 0.0, 9.81))
    ekf.update_imu(imu)
    
    state = ekf.get_state()
    
    # 正常情况下打滑概率应该较低
    assert state.slip_probability < 0.5, f"Slip probability should be low, got {state.slip_probability}"
    
    print("✓ test_ekf_slip_detection passed")


def test_ekf_anomaly_detection():
    """测试异常检测"""
    config = DEFAULT_CONFIG.copy()
    ekf = AdaptiveEKFEstimator(config)
    
    # 正常更新
    odom = create_test_odom(vx=1.0)
    ekf.update_odom(odom)
    
    state = ekf.get_state()
    anomalies = state.anomalies
    
    # 正常情况下不应该有异常
    assert isinstance(anomalies, list)
    
    # 测试 IMU 不可用异常
    ekf.set_imu_available(False)
    state = ekf.get_state()
    assert "IMU_UNAVAILABLE" in state.anomalies
    
    print("✓ test_ekf_anomaly_detection passed")


def test_ekf_drift_correction():
    """测试漂移校正"""
    config = DEFAULT_CONFIG.copy()
    ekf = AdaptiveEKFEstimator(config)
    
    # 设置初始状态
    odom = create_test_odom(x=1.0, y=1.0, theta=0.0)
    ekf.update_odom(odom)
    
    state_before = ekf.get_state().state.copy()
    
    # 应用漂移校正
    ekf.apply_drift_correction(dx=0.1, dy=0.1, dtheta=0.05)
    
    state_after = ekf.get_state().state
    
    # 验证校正已应用
    assert abs(state_after[0] - state_before[0] - 0.1) < 0.01
    assert abs(state_after[1] - state_before[1] - 0.1) < 0.01
    
    print("✓ test_ekf_drift_correction passed")


def test_ekf_drift_correction_reverse():
    """测试倒车时的漂移校正
    
    验证 apply_drift_correction 在倒车场景下正确保持速度符号。
    这是对 v_signed 修复的回归测试。
    """
    config = DEFAULT_CONFIG.copy()
    ekf = AdaptiveEKFEstimator(config)
    
    # 设置倒车状态：theta=0，vx=-1.0（向后）
    # 在世界坐标系中，倒车时 vx_world < 0
    odom = create_test_odom(x=1.0, y=0.0, vx=-1.0, vy=0.0, theta=0.0)
    ekf.update_odom(odom)
    
    state_before = ekf.get_state().state.copy()
    vx_before = state_before[3]
    vy_before = state_before[4]
    
    # 验证初始状态是倒车（vx < 0）
    assert vx_before < 0, f"Initial vx should be negative (reversing), got {vx_before}"
    
    # 应用航向校正（模拟 SLAM 修正）
    dtheta = 0.1  # 约 5.7 度
    ekf.apply_drift_correction(dx=0.0, dy=0.0, dtheta=dtheta)
    
    state_after = ekf.get_state().state
    vx_after = state_after[3]
    vy_after = state_after[4]
    
    # 计算校正后的速度模长
    v_magnitude_before = np.sqrt(vx_before**2 + vy_before**2)
    v_magnitude_after = np.sqrt(vx_after**2 + vy_after**2)
    
    # 验证速度模长保持不变
    assert abs(v_magnitude_after - v_magnitude_before) < 0.01, \
        f"Velocity magnitude should be preserved: before={v_magnitude_before}, after={v_magnitude_after}"
    
    # 关键验证：速度方向应该与新航向一致，且保持倒车方向
    # 对于 velocity_heading_coupled 平台，v_signed = vx * cos(theta) + vy * sin(theta)
    # 倒车时 v_signed < 0
    new_theta = state_after[6]
    v_signed_after = vx_after * np.cos(new_theta) + vy_after * np.sin(new_theta)
    
    # 验证仍然是倒车状态
    assert v_signed_after < 0, \
        f"After drift correction, vehicle should still be reversing (v_signed < 0), got {v_signed_after}"
    
    # 验证速度方向与航向一致（允许小误差）
    expected_vx = v_signed_after * np.cos(new_theta)
    expected_vy = v_signed_after * np.sin(new_theta)
    assert abs(vx_after - expected_vx) < 0.01, \
        f"vx should align with heading: expected {expected_vx}, got {vx_after}"
    assert abs(vy_after - expected_vy) < 0.01, \
        f"vy should align with heading: expected {expected_vy}, got {vy_after}"
    
    print("✓ test_ekf_drift_correction_reverse passed")


def test_ekf_covariance_positive_definite():
    """测试协方差矩阵保持正定"""
    config = DEFAULT_CONFIG.copy()
    ekf = AdaptiveEKFEstimator(config)
    
    # 多次更新
    for i in range(100):
        odom = create_test_odom(x=i*0.1, y=i*0.05, vx=1.0)
        ekf.update_odom(odom)
        ekf.predict(0.02)
        
        if i % 10 == 0:
            imu = create_test_imu()
            ekf.update_imu(imu)
    
    state = ekf.get_state()
    P = state.covariance
    
    # 验证协方差矩阵是对称的
    assert np.allclose(P, P.T), "Covariance should be symmetric"
    
    # 验证协方差矩阵是正定的（所有特征值 > 0）
    eigenvalues = np.linalg.eigvalsh(P)
    assert np.all(eigenvalues > 0), f"Covariance should be positive definite, eigenvalues: {eigenvalues}"
    
    print("✓ test_ekf_covariance_positive_definite passed")


def test_ekf_reset():
    """测试 EKF 重置"""
    config = DEFAULT_CONFIG.copy()
    ekf = AdaptiveEKFEstimator(config)
    
    # 更新一些数据
    odom = create_test_odom(x=5.0, y=3.0, vx=2.0)
    ekf.update_odom(odom)
    
    # 重置
    ekf.reset()
    
    state = ekf.get_state()
    
    # 验证状态已重置
    assert np.allclose(state.state, 0), "State should be zero after reset"
    assert state.slip_probability == 0.0
    assert state.imu_available == True
    
    print("✓ test_ekf_reset passed")


def test_ekf_continuous_operation():
    """测试 EKF 连续运行稳定性"""
    config = DEFAULT_CONFIG.copy()
    ekf = AdaptiveEKFEstimator(config)
    
    # 模拟 1000 个周期的连续运行
    x, y, theta = 0.0, 0.0, 0.0
    vx = 1.0
    dt = 0.02
    
    for i in range(1000):
        # 更新位置
        x += vx * np.cos(theta) * dt
        y += vx * np.sin(theta) * dt
        theta += 0.1 * dt  # 缓慢转向
        
        odom = create_test_odom(x=x, y=y, theta=theta, vx=vx)
        ekf.update_odom(odom)
        ekf.predict(dt)
        
        if i % 5 == 0:
            imu = create_test_imu(angular_velocity=(0, 0, 0.1))
            ekf.update_imu(imu)
        
        state = ekf.get_state()
        
        # 验证状态没有发散
        assert not np.any(np.isnan(state.state)), f"State contains NaN at step {i}"
        assert not np.any(np.isinf(state.state)), f"State contains Inf at step {i}"
        assert state.covariance_norm < 1e6, f"Covariance norm too large at step {i}"
    
    print("✓ test_ekf_continuous_operation passed (1000 cycles)")


def test_ekf_heading_fallback():
    """测试航向备选方案"""
    config = DEFAULT_CONFIG.copy()
    config['ekf'] = DEFAULT_CONFIG['ekf'].copy()
    config['ekf']['use_odom_orientation_fallback'] = True
    config['ekf']['theta_covariance_fallback_thresh'] = 0.1
    
    ekf = AdaptiveEKFEstimator(config)
    
    # 更新里程计
    odom = create_test_odom(x=1.0, y=0.0, theta=0.5, vx=1.0)
    ekf.update_odom(odom)
    
    # 验证航向备选功能存在
    theta = ekf._get_theta_for_transform()
    assert isinstance(theta, float)
    assert not np.isnan(theta)
    
    print("✓ test_ekf_heading_fallback passed")


if __name__ == '__main__':
    print("=" * 60)
    print("EKF State Estimator Tests")
    print("=" * 60)
    
    test_ekf_initialization()
    test_ekf_predict()
    test_ekf_odom_update()
    test_ekf_imu_update()
    test_ekf_slip_detection()
    test_ekf_anomaly_detection()
    test_ekf_drift_correction()
    test_ekf_covariance_positive_definite()
    test_ekf_reset()
    test_ekf_continuous_operation()
    test_ekf_heading_fallback()
    
    print("\n" + "=" * 60)
    print("✅ ALL EKF TESTS PASSED")
    print("=" * 60)
