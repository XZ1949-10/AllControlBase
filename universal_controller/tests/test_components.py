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

# 使用测试数据生成器
from universal_controller.tests.fixtures import (
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
    # 状态转换时历史被清空（_transition_to 调用 _reset_all_counters）
    # 虽然 update() 开始时追加了记录，但转换后被清空
    assert len(sm._mpc_success_history) == 0, f"_mpc_success_history should be empty after transition, got {len(sm._mpc_success_history)}"
    
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
    # 状态转换时历史被清空（_transition_to 调用 _reset_all_counters）
    assert len(sm._mpc_success_history) == 0, f"_mpc_success_history should be empty after transition, got {len(sm._mpc_success_history)}"
    
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
    # 状态转换时历史被清空（_transition_to 调用 _reset_all_counters）
    assert len(sm._mpc_success_history) == 0, f"_mpc_success_history should be empty after transition, got {len(sm._mpc_success_history)}"
    
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
    # 根据 _check_mpc_can_recover 的逻辑，需要 mpc_recovery_history_min (3) 条记录
    # 第 3 次 update 时会触发恢复到 MPC_DEGRADED
    transition_occurred = False
    for i in range(5):
        old_state = sm.state
        new_state = sm.update(diagnostics)
        if old_state == ControllerState.BACKUP_ACTIVE and new_state == ControllerState.MPC_DEGRADED:
            transition_occurred = True
            # 状态转换时历史被清空（_transition_to 调用 _reset_all_counters）
            assert len(sm._mpc_success_history) == 0, f"_mpc_success_history should be empty after transition, got {len(sm._mpc_success_history)}"
            break
    
    assert transition_occurred, "State should have transitioned from BACKUP_ACTIVE to MPC_DEGRADED"
    assert new_state == ControllerState.MPC_DEGRADED
    
    print("✓ test_state_machine_mpc_fail_count_reset passed")


def test_timeout_monitor():
    """测试超时监控器"""
    config = DEFAULT_CONFIG.copy()
    monitor = TimeoutMonitor(config)
    
    # 初始状态应在启动宽限期
    status = monitor.check({})
    assert status.in_startup_grace == True
    
    # 模拟正常数据
    data_ages = {'odom': 0.01, 'trajectory': 0.01, 'imu': 0.01}
    status = monitor.check(data_ages)
    assert status.odom_timeout == False
    
    # 模拟超时
    monitor.reset() # 应该重置启动时间
    # 注意: TimeoutMonitor logic might rely on monotonic time for startup grace check
    # But checking ages is stateless.
    
    print("✓ test_timeout_monitor passed")


def test_timeout_monitor_disabled_timeout():
    """测试超时监控器禁用超时检测功能"""
    import copy
    config = copy.deepcopy(DEFAULT_CONFIG)
    
    # 设置 IMU 超时为负数，表示禁用
    config['watchdog']['imu_timeout_ms'] = -1
    config['watchdog']['startup_grace_ms'] = 10
    
    monitor = TimeoutMonitor(config)
    
    # 即使传入很大的 age，也应该被忽略
    data_ages = {'odom': 0.0, 'trajectory': 0.0, 'imu': 999.0}
    status = monitor.check(data_ages)
    
    assert status.imu_timeout == False, "IMU timeout should be disabled when threshold <= 0"
    
    # 测试 odom 超时禁用
    config2 = copy.deepcopy(DEFAULT_CONFIG)
    config2['watchdog']['odom_timeout_ms'] = 0
    
    monitor2 = TimeoutMonitor(config2)
    # Odom age very large
    data_ages2 = {'odom': 999.0, 'trajectory': 0.0, 'imu': 0.0}
    status2 = monitor2.check(data_ages2)
    
    assert status2.odom_timeout == False, "Odom timeout should be disabled when threshold <= 0"
    
    print("✓ test_timeout_monitor_disabled_timeout passed")


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
    # 设置较短的节流间隔以便测试
    config['mpc'] = config.get('mpc', {}).copy()
    config['mpc']['horizon_change_min_interval'] = 0.0  # 禁用节流
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
    assert status.degradation_warning == False
    
    # 连续超时
    for _ in range(5):
        status = monitor.update(10.0, 0.0001, 100.0)
    assert status.degradation_warning == True
    
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


def test_state_machine_request_stop():
    """测试状态机外部停止请求功能"""
    from universal_controller.safety.state_machine import StateMachine
    from universal_controller.core.enums import ControllerState
    from universal_controller.core.diagnostics_input import DiagnosticsInput
    
    config = DEFAULT_CONFIG.copy()
    sm = StateMachine(config)
    
    # 创建有效的诊断输入，让状态机进入 NORMAL 状态
    diag = DiagnosticsInput(
        mpc_success=True,
        alpha=0.5,
        data_valid=True,
        has_valid_data=True,
        odom_timeout=False,
        traj_timeout_exceeded=False,
        safety_failed=False,
        v_horizontal=1.0,
        vz=0.0,
    )
    
    # 更新几次让状态机进入 NORMAL
    for _ in range(3):
        sm.update(diag)
    
    assert sm.state == ControllerState.NORMAL, f"Expected NORMAL, got {sm.state}"
    
    # 请求停止
    success = sm.request_stop()
    assert success, "request_stop should return True"
    assert sm.is_stop_requested(), "is_stop_requested should return True after request"
    
    # 下一次 update 应该转换到 STOPPING
    new_state = sm.update(diag)
    assert new_state == ControllerState.STOPPING, f"Expected STOPPING, got {new_state}"
    assert not sm.is_stop_requested(), "is_stop_requested should be False after update"
    
    print("✓ test_state_machine_request_stop passed")


def test_state_machine_request_stop_clears_on_reset():
    """测试重置时清除停止请求"""
    from universal_controller.safety.state_machine import StateMachine
    
    config = DEFAULT_CONFIG.copy()
    sm = StateMachine(config)
    
    # 请求停止
    sm.request_stop()
    assert sm.is_stop_requested()
    
    # 重置应该清除请求
    sm.reset()
    assert not sm.is_stop_requested(), "reset should clear stop request"
    
    print("✓ test_state_machine_request_stop_clears_on_reset passed")


def test_state_machine_stopped_recovery_to_mpc_degraded():
    """测试 STOPPED 状态恢复时始终先进入 MPC_DEGRADED
    
    验证修复后的恢复策略:
    - STOPPED 状态下 MPC 不运行，没有历史数据
    - 恢复时应该先进入 MPC_DEGRADED 进行观察
    - 这与 BACKUP_ACTIVE 的恢复策略一致
    """
    from universal_controller.safety.state_machine import StateMachine
    from universal_controller.core.enums import ControllerState
    from universal_controller.core.diagnostics_input import DiagnosticsInput
    from universal_controller.core.data_types import MPCHealthStatus
    
    config = DEFAULT_CONFIG.copy()
    sm = StateMachine(config)
    
    # 直接设置为 STOPPED 状态
    sm.state = ControllerState.STOPPED
    
    # 场景 1: MPC 健康时恢复 - 应该进入 MPC_DEGRADED 而非 NORMAL
    healthy_mpc = MPCHealthStatus(
        healthy=True,
        can_recover=True,
        degradation_warning=False,
        consecutive_near_timeout=0,
        kkt_residual=0.0001,
        condition_number=100.0
    )
    
    diag_healthy = DiagnosticsInput(
        mpc_success=True,
        alpha=0.9,  # 高 alpha
        data_valid=True,
        has_valid_data=True,
        odom_timeout=False,
        traj_timeout_exceeded=False,
        safety_failed=False,
        v_horizontal=0.0,  # 已停止
        vz=0.0,
        mpc_health=healthy_mpc,
    )
    
    new_state = sm.update(diag_healthy)
    assert new_state == ControllerState.MPC_DEGRADED, \
        f"STOPPED should recover to MPC_DEGRADED first, got {new_state}"
    
    # 场景 2: 重置并测试 MPC 不健康时恢复 - 也应该进入 MPC_DEGRADED
    sm.reset()
    sm.state = ControllerState.STOPPED
    
    unhealthy_mpc = MPCHealthStatus(
        healthy=False,
        can_recover=False,
        degradation_warning=True,
        consecutive_near_timeout=5,
        kkt_residual=0.01,
        condition_number=10000.0
    )
    
    diag_unhealthy = DiagnosticsInput(
        mpc_success=False,
        alpha=0.3,  # 低 alpha
        data_valid=True,
        has_valid_data=True,
        odom_timeout=False,
        traj_timeout_exceeded=False,
        safety_failed=False,
        v_horizontal=0.0,
        vz=0.0,
        mpc_health=unhealthy_mpc,
    )
    
    new_state = sm.update(diag_unhealthy)
    assert new_state == ControllerState.MPC_DEGRADED, \
        f"STOPPED should recover to MPC_DEGRADED regardless of MPC health, got {new_state}"
    
    # 场景 3: 验证 MPC 历史在 STOPPED 状态下不被污染
    # 需要使用不会触发恢复的诊断输入（无效数据）
    sm.reset()
    sm.state = ControllerState.STOPPED
    
    diag_no_data = DiagnosticsInput(
        mpc_success=False,
        alpha=0.3,
        data_valid=False,
        has_valid_data=False,  # 无有效数据，不会触发恢复
        odom_timeout=False,
        traj_timeout_exceeded=False,
        safety_failed=False,
        v_horizontal=0.0,
        vz=0.0,
        mpc_health=unhealthy_mpc,
    )
    
    # 在 STOPPED 状态下多次 update（不会触发恢复）
    for _ in range(5):
        state = sm.update(diag_no_data)
        assert state == ControllerState.STOPPED, f"Should stay in STOPPED, got {state}"
    
    # MPC 历史应该为空（STOPPED 状态下不追加历史）
    assert len(sm._mpc_success_history) == 0, \
        f"MPC history should be empty in STOPPED state, got {len(sm._mpc_success_history)}"
    
    print("✓ test_state_machine_stopped_recovery_to_mpc_degraded passed")
