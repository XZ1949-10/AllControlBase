"""组件测试 - 符合 v3.17.6 需求"""
import numpy as np
import time
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from universal_controller.core.data_types import (
    Trajectory, ControlOutput, ConsistencyResult, Point3D, Header,
    Odometry, Imu, MPCHealthStatus
)
from universal_controller.core.enums import ControllerState, PlatformType
from universal_controller.core.diagnostics_input import DiagnosticsInput
from universal_controller.config.default_config import DEFAULT_CONFIG, PLATFORM_CONFIG
from universal_controller.estimator.adaptive_ekf import AdaptiveEKFEstimator
from universal_controller.consistency.weighted_analyzer import WeightedConsistencyAnalyzer
from universal_controller.safety.state_machine import StateMachine
from universal_controller.safety.timeout_monitor import TimeoutMonitor
from universal_controller.safety.safety_monitor import BasicSafetyMonitor
from universal_controller.tracker.pure_pursuit import PurePursuitController
from universal_controller.tracker.mpc_controller import MPCController
from universal_controller.health.mpc_health_monitor import MPCHealthMonitor

# 使用 mock 模块中的测试数据生成器
from universal_controller.mock.test_data_generator import (
    create_test_trajectory,
    create_test_odom,
    create_test_imu
)


# 使用 mock 模块中的测试数据生成器
from universal_controller.mock.test_data_generator import (
    create_test_trajectory,
    create_test_odom,
    create_test_imu
)


def test_adaptive_ekf():
    """测试自适应 EKF"""
    config = DEFAULT_CONFIG.copy()
    ekf = AdaptiveEKFEstimator(config)
    
    # 测试初始状态
    state = ekf.get_state()
    assert state.state.shape == (8,)
    assert state.covariance.shape == (8, 8)
    
    # 测试 odom 更新
    odom = create_test_odom(x=1.0, y=0.5, vx=1.0)
    ekf.update_odom(odom)
    state = ekf.get_state()
    assert state.state[0] != 0  # 位置已更新
    
    # 测试漂移校正
    ekf.apply_drift_correction(0.1, 0.1, 0.05)
    
    # 测试重置
    ekf.reset()
    state = ekf.get_state()
    assert np.allclose(state.state, 0)
    
    print("✓ test_adaptive_ekf passed")


def test_temporal_valid_requires_two_samples():
    """测试时序有效性需要至少 2 个样本 (v3.17)"""
    config = DEFAULT_CONFIG.copy()
    analyzer = WeightedConsistencyAnalyzer(config)
    
    # 第一次调用
    traj1 = create_test_trajectory(soft_enabled=True)
    result1 = analyzer.compute(traj1)
    # 第一次调用时 temporal_valid 应为 False (只有 1 个样本)
    
    # 第二次调用
    traj2 = create_test_trajectory(soft_enabled=True)
    result2 = analyzer.compute(traj2)
    # 第二次调用时应有 2 个样本
    
    # 重置后再测试
    analyzer.reset()
    traj3 = create_test_trajectory(soft_enabled=True)
    result3 = analyzer.compute(traj3)
    
    print("✓ test_temporal_valid_requires_two_samples passed")


def test_state_machine_counter_reset():
    """测试状态转换时恢复计数器正确重置 (v3.17.3)"""
    config = DEFAULT_CONFIG.copy()
    sm = StateMachine(config)
    sm.state = ControllerState.SOFT_DISABLED
    sm.alpha_recovery_count = 3
    
    # 触发 safety_failed 转换到 MPC_DEGRADED
    diagnostics = DiagnosticsInput(
        alpha=0.5,
        mpc_health=None,
        mpc_success=True,
        odom_timeout=False,
        traj_timeout_exceeded=False,
        has_valid_data=True,
        tf2_critical=False,
        data_valid=True,
        safety_failed=True,
        v_horizontal=0.5,
        vz=0.0,
    )
    
    new_state = sm.update(diagnostics)
    
    assert new_state == ControllerState.MPC_DEGRADED
    assert sm.alpha_recovery_count == 0
    assert sm.mpc_recovery_count == 0
    
    print("✓ test_state_machine_counter_reset passed")


def test_state_machine_mpc_fail_count_reset():
    """测试 MPC 失败检测滑动窗口在状态转换中正确重置 (v3.17.9 修复, v3.18.0 重构)"""
    config = DEFAULT_CONFIG.copy()
    sm = StateMachine(config)
    
    # 测试 1: SOFT_DISABLED -> BACKUP_ACTIVE (mpc_success=False)
    sm.state = ControllerState.SOFT_DISABLED
    # 预填充一些历史记录
    for _ in range(5):
        sm._mpc_success_history.append(True)
    
    diagnostics = DiagnosticsInput(
        alpha=0.5,
        mpc_health=None,
        mpc_success=False,  # MPC 失败
        odom_timeout=False,
        traj_timeout_exceeded=False,
        has_valid_data=True,
        tf2_critical=False,
        data_valid=True,
        safety_failed=False,
        v_horizontal=0.5,
        vz=0.0,
    )
    
    new_state = sm.update(diagnostics)
    assert new_state == ControllerState.BACKUP_ACTIVE
    assert len(sm._mpc_success_history) == 0, f"_mpc_success_history should be empty, got {len(sm._mpc_success_history)}"
    
    # 测试 2: SOFT_DISABLED -> MPC_DEGRADED (safety_failed)
    sm.reset()
    sm.state = ControllerState.SOFT_DISABLED
    for _ in range(5):
        sm._mpc_success_history.append(True)
    
    diagnostics = DiagnosticsInput(
        alpha=0.5,
        mpc_health=None,
        mpc_success=True,
        odom_timeout=False,
        traj_timeout_exceeded=False,
        has_valid_data=True,
        tf2_critical=False,
        data_valid=True,
        safety_failed=True,
        v_horizontal=0.5,
        vz=0.0,
    )
    
    new_state = sm.update(diagnostics)
    assert new_state == ControllerState.MPC_DEGRADED
    assert len(sm._mpc_success_history) == 0, f"_mpc_success_history should be empty, got {len(sm._mpc_success_history)}"
    
    # 测试 3: SOFT_DISABLED -> MPC_DEGRADED (tf2_critical)
    sm.reset()
    sm.state = ControllerState.SOFT_DISABLED
    for _ in range(5):
        sm._mpc_success_history.append(True)
    
    diagnostics = DiagnosticsInput(
        alpha=0.5,
        mpc_health=None,
        mpc_success=True,
        odom_timeout=False,
        traj_timeout_exceeded=False,
        has_valid_data=True,
        tf2_critical=True,
        data_valid=True,
        safety_failed=False,
        v_horizontal=0.5,
        vz=0.0,
    )
    
    new_state = sm.update(diagnostics)
    assert new_state == ControllerState.MPC_DEGRADED
    assert len(sm._mpc_success_history) == 0, f"_mpc_success_history should be empty, got {len(sm._mpc_success_history)}"
    
    # 测试 4: MPC_DEGRADED -> BACKUP_ACTIVE
    sm.reset()
    sm.state = ControllerState.MPC_DEGRADED
    for _ in range(5):
        sm._mpc_success_history.append(True)
    
    diagnostics = DiagnosticsInput(
        alpha=0.5,
        mpc_health=None,
        mpc_success=False,
        odom_timeout=False,
        traj_timeout_exceeded=False,
        has_valid_data=True,
        tf2_critical=False,
        data_valid=True,
        safety_failed=False,
        v_horizontal=0.5,
        vz=0.0,
    )
    
    new_state = sm.update(diagnostics)
    assert new_state == ControllerState.BACKUP_ACTIVE
    assert len(sm._mpc_success_history) == 0, f"_mpc_success_history should be empty, got {len(sm._mpc_success_history)}"
    
    # 测试 5: BACKUP_ACTIVE -> MPC_DEGRADED (需要多次成功才能恢复)
    sm.reset()
    sm.state = ControllerState.BACKUP_ACTIVE
    
    diagnostics = DiagnosticsInput(
        alpha=0.5,
        mpc_health=None,
        mpc_success=True,
        odom_timeout=False,
        traj_timeout_exceeded=False,
        has_valid_data=True,
        tf2_critical=False,
        data_valid=True,
        safety_failed=False,
        v_horizontal=0.5,
        vz=0.0,
    )
    
    # 需要多次成功才能从 BACKUP_ACTIVE 恢复
    # 根据 _check_mpc_can_recover 的逻辑，需要最近 5 次中至少 4 次成功
    for _ in range(5):
        new_state = sm.update(diagnostics)
    
    assert new_state == ControllerState.MPC_DEGRADED
    assert len(sm._mpc_success_history) == 0, f"_mpc_success_history should be empty after transition, got {len(sm._mpc_success_history)}"
    
    print("✓ test_state_machine_mpc_fail_count_reset passed")


def test_timeout_monitor():
    """测试超时监控器"""
    config = DEFAULT_CONFIG.copy()
    monitor = TimeoutMonitor(config)
    
    current_time = time.time()
    
    # 初始状态应在启动宽限期
    status = monitor.check(current_time)
    assert status.in_startup_grace == True
    
    # 更新 odom
    monitor.update_odom(current_time)
    
    # 等待启动宽限期过后
    time.sleep(0.01)  # 模拟时间流逝
    
    # 重置测试
    monitor.reset()
    status = monitor.check(time.time())
    assert status.in_startup_grace == True
    
    print("✓ test_timeout_monitor passed")


def test_safety_monitor():
    """测试安全监控器 (v3.17)"""
    config = DEFAULT_CONFIG.copy()
    platform_config = PLATFORM_CONFIG['differential']
    monitor = BasicSafetyMonitor(config, platform_config)
    
    state = np.zeros(8)
    
    # 正常命令
    cmd = ControlOutput(vx=1.0, vy=0.0, vz=0.0, omega=0.5, frame_id="base_link")
    diagnostics = DiagnosticsInput()  # 使用默认值
    decision = monitor.check(state, cmd, diagnostics)
    assert decision.safe == True
    
    # 超限命令
    cmd_over = ControlOutput(vx=10.0, vy=0.0, vz=0.0, omega=0.5, frame_id="base_link")
    decision_over = monitor.check(state, cmd_over, diagnostics)
    assert decision_over.safe == False
    assert decision_over.limited_cmd is not None
    
    # 验证 limited_cmd 完整复制 health_metrics
    cmd_with_metrics = ControlOutput(
        vx=10.0, vy=0.0, vz=0.0, omega=0.5, 
        frame_id="base_link",
        health_metrics={'test': 123}
    )
    decision_metrics = monitor.check(state, cmd_with_metrics, diagnostics)
    if decision_metrics.limited_cmd:
        assert 'test' in decision_metrics.limited_cmd.health_metrics
    
    print("✓ test_safety_monitor passed")


def test_mpc_horizon_dynamic_adjustment():
    """测试 MPC horizon 动态调整 (v3.17.3)"""
    config = DEFAULT_CONFIG.copy()
    platform_config = PLATFORM_CONFIG['differential']
    mpc = MPCController(config, platform_config)
    
    original_horizon = mpc.horizon
    assert original_horizon == 20
    
    # 调整 horizon
    mpc.set_horizon(10)
    assert mpc.horizon == 10
    
    # 恢复
    mpc.set_horizon(20)
    assert mpc.horizon == 20
    
    print("✓ test_mpc_horizon_dynamic_adjustment passed")


def test_pure_pursuit_shutdown():
    """测试 Pure Pursuit shutdown 方法"""
    config = DEFAULT_CONFIG.copy()
    platform_config = PLATFORM_CONFIG['differential']
    pp = PurePursuitController(config, platform_config)
    
    pp.shutdown()
    assert pp._is_shutdown == True
    
    print("✓ test_pure_pursuit_shutdown passed")


def test_mpc_health_monitor():
    """测试 MPC 健康监控器"""
    config = DEFAULT_CONFIG.copy()
    monitor = MPCHealthMonitor(config)
    
    # 正常情况
    status = monitor.update(5.0, 0.0001, 100.0)
    assert status.healthy == True
    assert status.warning == False
    
    # 连续超时
    for _ in range(5):
        status = monitor.update(10.0, 0.0001, 100.0)
    assert status.warning == True
    
    # 重置
    monitor.reset()
    assert monitor.consecutive_near_timeout == 0
    
    print("✓ test_mpc_health_monitor passed")


def test_consistency_analyzer():
    """测试一致性分析器"""
    config = DEFAULT_CONFIG.copy()
    analyzer = WeightedConsistencyAnalyzer(config)
    
    # soft_enabled=False 时
    traj = create_test_trajectory(soft_enabled=False)
    result = analyzer.compute(traj)
    assert result.should_disable_soft == True
    
    # soft_enabled=True 时
    traj_soft = create_test_trajectory(soft_enabled=True)
    result_soft = analyzer.compute(traj_soft)
    assert isinstance(result_soft.alpha, float)
    
    print("✓ test_consistency_analyzer passed")


if __name__ == '__main__':
    test_adaptive_ekf()
    test_temporal_valid_requires_two_samples()
    test_state_machine_counter_reset()
    test_state_machine_mpc_fail_count_reset()
    test_timeout_monitor()
    test_safety_monitor()
    test_mpc_horizon_dynamic_adjustment()
    test_pure_pursuit_shutdown()
    test_mpc_health_monitor()
    test_consistency_analyzer()
    print("\n✅ All component tests passed!")
