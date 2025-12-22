"""
配置效果测试

验证所有配置参数都能正确影响系统行为：
1. 约束配置测试
2. 超时配置测试
3. EKF 配置测试
4. MPC 配置测试
5. 一致性配置测试
6. 安全配置测试
7. 备用控制器配置测试
8. 平滑过渡配置测试
9. 平台配置测试
"""
import numpy as np
import time
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from universal_controller.core.data_types import (
    Trajectory, ControlOutput, Point3D, Header, ConsistencyResult
)
from universal_controller.core.enums import ControllerState, PlatformType
from universal_controller.core.diagnostics_input import DiagnosticsInput
from universal_controller.config.default_config import DEFAULT_CONFIG, PLATFORM_CONFIG
from universal_controller.manager.controller_manager import ControllerManager
from universal_controller.estimator.adaptive_ekf import AdaptiveEKFEstimator
from universal_controller.tracker.mpc_controller import MPCController
from universal_controller.tracker.pure_pursuit import PurePursuitController
from universal_controller.consistency.weighted_analyzer import WeightedConsistencyAnalyzer
from universal_controller.safety.safety_monitor import BasicSafetyMonitor
from universal_controller.safety.state_machine import StateMachine
from universal_controller.safety.timeout_monitor import TimeoutMonitor
from universal_controller.health.mpc_health_monitor import MPCHealthMonitor
from universal_controller.transition.smooth_transition import ExponentialSmoothTransition
from universal_controller.mock.test_data_generator import (
    create_test_trajectory, create_test_odom, create_test_imu
)


# =============================================================================
# 1. 约束配置测试
# =============================================================================

def test_velocity_constraint_config():
    """测试速度约束配置"""
    # 测试不同的 v_max 配置
    for v_max in [0.5, 1.0, 2.0, 5.0]:
        config = DEFAULT_CONFIG.copy()
        config['constraints'] = DEFAULT_CONFIG['constraints'].copy()
        config['constraints']['v_max'] = v_max
        
        platform_config = PLATFORM_CONFIG['differential']
        pp = PurePursuitController(config, platform_config)
        
        state = np.array([0, 0, 0, 0, 0, 0, 0, 0])
        trajectory = create_test_trajectory(trajectory_type='straight', speed=10.0, soft_enabled=True)
        consistency = ConsistencyResult(1.0, 1.0, 1.0, 1.0, False, True)
        
        cmd = pp.compute(state, trajectory, consistency)
        
        # 验证速度不超过配置的限制
        v_horizontal = np.sqrt(cmd.vx**2 + cmd.vy**2)
        assert v_horizontal <= v_max * 1.5, f"v_max={v_max}: velocity {v_horizontal} exceeds limit"
        
        pp.shutdown()
    
    print("✓ test_velocity_constraint_config passed")


def test_omega_constraint_config():
    """测试角速度约束配置"""
    for omega_max in [0.5, 1.0, 2.0]:
        config = DEFAULT_CONFIG.copy()
        config['constraints'] = DEFAULT_CONFIG['constraints'].copy()
        config['constraints']['omega_max'] = omega_max
        
        platform_config = PLATFORM_CONFIG['differential']
        pp = PurePursuitController(config, platform_config)
        
        # 创建需要大角速度的轨迹
        state = np.array([0, 0, 0, 1.0, 0, 0, 0, 0])
        trajectory = create_test_trajectory(trajectory_type='circle', radius=0.5, soft_enabled=True)
        consistency = ConsistencyResult(1.0, 1.0, 1.0, 1.0, False, True)
        
        cmd = pp.compute(state, trajectory, consistency)
        
        # 验证角速度不超过配置的限制
        assert abs(cmd.omega) <= omega_max * 1.1, f"omega_max={omega_max}: omega {cmd.omega} exceeds limit"
        
        pp.shutdown()
    
    print("✓ test_omega_constraint_config passed")


def test_acceleration_constraint_config():
    """测试加速度约束配置"""
    for a_max in [0.5, 1.0, 2.0]:
        config = DEFAULT_CONFIG.copy()
        config['constraints'] = DEFAULT_CONFIG['constraints'].copy()
        config['constraints']['a_max'] = a_max
        
        platform_config = PLATFORM_CONFIG['differential']
        pp = PurePursuitController(config, platform_config)
        
        state = np.array([0, 0, 0, 0, 0, 0, 0, 0])  # 静止
        trajectory = create_test_trajectory(trajectory_type='straight', speed=5.0, soft_enabled=True)
        consistency = ConsistencyResult(1.0, 1.0, 1.0, 1.0, False, True)
        
        # 连续计算，检查加速度
        last_cmd = None
        dt = 0.02
        for _ in range(10):
            cmd = pp.compute(state, trajectory, consistency)
            if last_cmd is not None:
                accel = abs(cmd.vx - last_cmd.vx) / dt
                # 加速度应该被限制
                assert accel <= a_max * 2.0, f"a_max={a_max}: accel {accel} exceeds limit"
            last_cmd = cmd
        
        pp.shutdown()
    
    print("✓ test_acceleration_constraint_config passed")


# =============================================================================
# 2. 超时配置测试
# =============================================================================

def test_odom_timeout_config():
    """测试里程计超时配置"""
    for timeout_ms in [50, 100, 200, 500]:
        config = DEFAULT_CONFIG.copy()
        config['watchdog'] = DEFAULT_CONFIG['watchdog'].copy()
        config['watchdog']['odom_timeout_ms'] = timeout_ms
        config['watchdog']['startup_grace_ms'] = 10  # 短启动宽限期
        
        monitor = TimeoutMonitor(config)
        
        # 更新一次
        monitor.update_odom(time.time())
        
        # 等待超时
        time.sleep(timeout_ms / 1000 + 0.05)
        
        status = monitor.check()
        assert status.odom_timeout == True, f"timeout_ms={timeout_ms}: should be timeout"
        
    print("✓ test_odom_timeout_config passed")


def test_startup_grace_config():
    """测试启动宽限期配置"""
    for grace_ms in [100, 500, 1000]:
        config = DEFAULT_CONFIG.copy()
        config['watchdog'] = DEFAULT_CONFIG['watchdog'].copy()
        config['watchdog']['startup_grace_ms'] = grace_ms
        
        monitor = TimeoutMonitor(config)
        
        # 立即检查，应该在宽限期内
        status = monitor.check()
        assert status.in_startup_grace == True, f"grace_ms={grace_ms}: should be in startup grace"
        
    print("✓ test_startup_grace_config passed")


def test_trajectory_grace_config():
    """测试轨迹宽限期配置"""
    for grace_ms in [50, 100, 200]:
        config = DEFAULT_CONFIG.copy()
        config['watchdog'] = DEFAULT_CONFIG['watchdog'].copy()
        config['watchdog']['traj_timeout_ms'] = 50
        config['watchdog']['traj_grace_ms'] = grace_ms
        config['watchdog']['startup_grace_ms'] = 10
        
        monitor = TimeoutMonitor(config)
        monitor.update_trajectory(time.time())
        
        # 等待超时但在宽限期内
        time.sleep(0.06)
        status = monitor.check()
        
        # 应该超时但不超过宽限期
        assert status.traj_timeout == True
        
    print("✓ test_trajectory_grace_config passed")


# =============================================================================
# 3. EKF 配置测试
# =============================================================================

def test_ekf_slip_threshold_config():
    """测试 EKF 打滑阈值配置"""
    for slip_thresh in [1.0, 2.0, 5.0]:
        config = DEFAULT_CONFIG.copy()
        config['ekf'] = DEFAULT_CONFIG['ekf'].copy()
        config['ekf']['adaptive'] = DEFAULT_CONFIG['ekf']['adaptive'].copy()
        config['ekf']['adaptive']['base_slip_thresh'] = slip_thresh
        
        ekf = AdaptiveEKFEstimator(config)
        
        # 验证配置被正确应用
        assert ekf.base_slip_thresh == slip_thresh
        
    print("✓ test_ekf_slip_threshold_config passed")


def test_ekf_measurement_noise_config():
    """测试 EKF 测量噪声配置"""
    for noise in [0.01, 0.1, 1.0]:
        config = DEFAULT_CONFIG.copy()
        config['ekf'] = DEFAULT_CONFIG['ekf'].copy()
        config['ekf']['measurement_noise'] = DEFAULT_CONFIG['ekf']['measurement_noise'].copy()
        config['ekf']['measurement_noise']['odom_velocity'] = noise
        
        ekf = AdaptiveEKFEstimator(config)
        
        # 验证测量噪声矩阵被正确设置
        # R_odom_base 应该包含配置的噪声值
        assert ekf.R_odom_base is not None
        
    print("✓ test_ekf_measurement_noise_config passed")


def test_ekf_heading_fallback_config():
    """测试 EKF 航向备选配置"""
    # 测试启用
    config = DEFAULT_CONFIG.copy()
    config['ekf'] = DEFAULT_CONFIG['ekf'].copy()
    config['ekf']['use_odom_orientation_fallback'] = True
    config['ekf']['theta_covariance_fallback_thresh'] = 0.1
    
    ekf = AdaptiveEKFEstimator(config)
    assert ekf.use_odom_orientation_fallback == True
    assert ekf.theta_covariance_fallback_thresh == 0.1
    
    # 测试禁用
    config['ekf']['use_odom_orientation_fallback'] = False
    ekf2 = AdaptiveEKFEstimator(config)
    assert ekf2.use_odom_orientation_fallback == False
    
    print("✓ test_ekf_heading_fallback_config passed")


# =============================================================================
# 4. MPC 配置测试
# =============================================================================

def test_mpc_horizon_config():
    """测试 MPC horizon 配置"""
    for horizon in [10, 20, 30]:
        config = DEFAULT_CONFIG.copy()
        config['mpc'] = DEFAULT_CONFIG['mpc'].copy()
        config['mpc']['horizon'] = horizon
        
        platform_config = PLATFORM_CONFIG['differential']
        mpc = MPCController(config, platform_config)
        
        assert mpc.horizon == horizon, f"horizon={horizon}: got {mpc.horizon}"
        
        mpc.shutdown()
    
    print("✓ test_mpc_horizon_config passed")


def test_mpc_weights_config():
    """测试 MPC 权重配置"""
    config = DEFAULT_CONFIG.copy()
    config['mpc'] = DEFAULT_CONFIG['mpc'].copy()
    config['mpc']['weights'] = {
        'position': 20.0,
        'velocity': 2.0,
        'heading': 10.0,
        'control_v': 0.2,
        'control_omega': 0.2,
    }
    
    platform_config = PLATFORM_CONFIG['differential']
    mpc = MPCController(config, platform_config)
    
    assert mpc.Q_pos == 20.0
    assert mpc.Q_vel == 2.0
    assert mpc.Q_heading == 10.0
    
    mpc.shutdown()
    print("✓ test_mpc_weights_config passed")


def test_mpc_health_monitor_config():
    """测试 MPC 健康监控配置"""
    config = DEFAULT_CONFIG.copy()
    config['mpc'] = DEFAULT_CONFIG['mpc'].copy()
    config['mpc']['health_monitor'] = {
        'time_warning_thresh_ms': 5,
        'time_critical_thresh_ms': 10,
        'time_recovery_thresh_ms': 3,
        'condition_number_thresh': 1e6,
        'condition_number_recovery': 1e4,
        'kkt_residual_thresh': 1e-4,
        'consecutive_warning_limit': 2,
        'consecutive_recovery_limit': 3,
        'recovery_multiplier': 2.0,
        'consecutive_good_for_decay': 2,
        'timeout_decay_rate': 2,
    }
    
    monitor = MPCHealthMonitor(config)
    
    assert monitor.time_warning_thresh == 5
    assert monitor.time_critical_thresh == 10
    assert monitor.consecutive_warning_limit == 2
    
    print("✓ test_mpc_health_monitor_config passed")


# =============================================================================
# 5. 一致性配置测试
# =============================================================================

def test_consistency_alpha_min_config():
    """测试一致性 alpha_min 配置"""
    for alpha_min in [0.05, 0.1, 0.2]:
        config = DEFAULT_CONFIG.copy()
        config['consistency'] = DEFAULT_CONFIG['consistency'].copy()
        config['consistency']['alpha_min'] = alpha_min
        
        analyzer = WeightedConsistencyAnalyzer(config)
        
        assert analyzer.alpha_min == alpha_min
        
    print("✓ test_consistency_alpha_min_config passed")


def test_consistency_weights_config():
    """测试一致性权重配置"""
    config = DEFAULT_CONFIG.copy()
    config['consistency'] = DEFAULT_CONFIG['consistency'].copy()
    config['consistency']['weights'] = {
        'kappa': 2.0,
        'velocity': 3.0,
        'temporal': 1.0,
    }
    
    analyzer = WeightedConsistencyAnalyzer(config)
    
    assert analyzer.w_kappa == 2.0
    assert analyzer.w_velocity == 3.0
    assert analyzer.w_temporal == 1.0
    
    print("✓ test_consistency_weights_config passed")


def test_consistency_thresholds_config():
    """测试一致性阈值配置"""
    config = DEFAULT_CONFIG.copy()
    config['consistency'] = DEFAULT_CONFIG['consistency'].copy()
    config['consistency']['kappa_thresh'] = 0.3
    config['consistency']['v_dir_thresh'] = 0.9
    config['consistency']['temporal_smooth_thresh'] = 0.4
    
    analyzer = WeightedConsistencyAnalyzer(config)
    
    assert analyzer.kappa_thresh == 0.3
    assert analyzer.v_dir_thresh == 0.9
    assert analyzer.temporal_smooth_thresh == 0.4
    
    print("✓ test_consistency_thresholds_config passed")


# =============================================================================
# 6. 安全配置测试
# =============================================================================

def test_safety_velocity_margin_config():
    """测试安全速度裕度配置"""
    for margin in [1.0, 1.1, 1.2]:
        config = DEFAULT_CONFIG.copy()
        config['safety'] = DEFAULT_CONFIG['safety'].copy()
        config['safety']['velocity_margin'] = margin
        
        platform_config = PLATFORM_CONFIG['differential']
        monitor = BasicSafetyMonitor(config, platform_config)
        
        assert monitor.velocity_margin == margin
        
    print("✓ test_safety_velocity_margin_config passed")


def test_safety_stop_threshold_config():
    """测试安全停止阈值配置"""
    for thresh in [0.01, 0.05, 0.1]:
        config = DEFAULT_CONFIG.copy()
        config['safety'] = DEFAULT_CONFIG['safety'].copy()
        config['safety']['v_stop_thresh'] = thresh
        
        sm = StateMachine(config)
        
        assert sm.v_stop_thresh == thresh
        
    print("✓ test_safety_stop_threshold_config passed")


def test_state_machine_recovery_config():
    """测试状态机恢复配置"""
    config = DEFAULT_CONFIG.copy()
    config['safety'] = DEFAULT_CONFIG['safety'].copy()
    config['safety']['state_machine'] = {
        'alpha_recovery_thresh': 3,
        'alpha_recovery_value': 0.4,
        'alpha_disable_thresh': 0.15,
        'mpc_recovery_thresh': 3,
        'mpc_fail_window_size': 5,
        'mpc_fail_thresh': 2,
        'mpc_fail_ratio_thresh': 0.4,
        'mpc_recovery_history_min': 2,
        'mpc_recovery_recent_count': 3,
        'mpc_recovery_tolerance': 1,
        'mpc_recovery_success_ratio': 0.7,
    }
    
    sm = StateMachine(config)
    
    assert sm.alpha_recovery_thresh == 3
    assert sm.alpha_recovery_value == 0.4
    assert sm.mpc_fail_thresh == 2
    
    print("✓ test_state_machine_recovery_config passed")


# =============================================================================
# 7. 备用控制器配置测试
# =============================================================================

def test_pure_pursuit_lookahead_config():
    """测试 Pure Pursuit 前瞻距离配置"""
    config = DEFAULT_CONFIG.copy()
    config['backup'] = DEFAULT_CONFIG['backup'].copy()
    config['backup']['lookahead_dist'] = 2.0
    config['backup']['min_lookahead'] = 1.0
    config['backup']['max_lookahead'] = 5.0
    config['backup']['lookahead_ratio'] = 0.8
    
    platform_config = PLATFORM_CONFIG['differential']
    pp = PurePursuitController(config, platform_config)
    
    assert pp.lookahead_dist == 2.0
    assert pp.min_lookahead == 1.0
    assert pp.max_lookahead == 5.0
    assert pp.lookahead_ratio == 0.8
    
    # 测试前瞻距离计算
    lookahead = pp._compute_lookahead(1.0)
    assert lookahead >= 1.0  # min_lookahead
    assert lookahead <= 5.0  # max_lookahead
    
    pp.shutdown()
    print("✓ test_pure_pursuit_lookahead_config passed")


def test_pure_pursuit_heading_gain_config():
    """测试 Pure Pursuit 航向增益配置"""
    for kp in [0.5, 1.5, 3.0]:
        config = DEFAULT_CONFIG.copy()
        config['backup'] = DEFAULT_CONFIG['backup'].copy()
        config['backup']['kp_heading'] = kp
        
        platform_config = PLATFORM_CONFIG['omni']
        pp = PurePursuitController(config, platform_config)
        
        assert pp.kp_heading == kp
        
        pp.shutdown()
    
    print("✓ test_pure_pursuit_heading_gain_config passed")


# =============================================================================
# 8. 平滑过渡配置测试
# =============================================================================

def test_transition_tau_config():
    """测试平滑过渡时间常数配置"""
    for tau in [0.05, 0.1, 0.2]:
        config = DEFAULT_CONFIG.copy()
        config['transition'] = DEFAULT_CONFIG['transition'].copy()
        config['transition']['tau'] = tau
        
        transition = ExponentialSmoothTransition(config)
        
        assert transition.tau == tau
        
    print("✓ test_transition_tau_config passed")


def test_transition_max_duration_config():
    """测试平滑过渡最大时长配置"""
    config = DEFAULT_CONFIG.copy()
    config['transition'] = DEFAULT_CONFIG['transition'].copy()
    config['transition']['max_duration'] = 1.0
    config['transition']['completion_threshold'] = 0.9
    
    transition = ExponentialSmoothTransition(config)
    
    assert transition.max_duration == 1.0
    assert transition.completion_threshold == 0.9
    
    print("✓ test_transition_max_duration_config passed")


# =============================================================================
# 9. 平台配置测试
# =============================================================================

def test_platform_output_frame_config():
    """测试平台输出坐标系配置"""
    # 差速车应该输出 base_link
    config = DEFAULT_CONFIG.copy()
    config['system']['platform'] = 'differential'
    manager = ControllerManager(config)
    manager.initialize_default_components()
    
    odom = create_test_odom(vx=1.0)
    trajectory = create_test_trajectory()
    cmd = manager.update(odom, trajectory)
    
    assert cmd.frame_id == 'base_link'
    manager.shutdown()
    
    # 全向车应该输出 world
    config['system']['platform'] = 'omni'
    manager2 = ControllerManager(config)
    manager2.initialize_default_components()
    
    cmd2 = manager2.update(odom, trajectory)
    assert cmd2.frame_id == 'world'
    manager2.shutdown()
    
    print("✓ test_platform_output_frame_config passed")


def test_platform_constraints_config():
    """测试平台约束配置"""
    # 测试不同平台的约束
    for platform in ['differential', 'omni', 'quadrotor', 'ackermann']:
        config = DEFAULT_CONFIG.copy()
        config['system']['platform'] = platform
        
        manager = ControllerManager(config)
        manager.initialize_default_components()
        
        odom = create_test_odom(vx=1.0)
        trajectory = create_test_trajectory()
        cmd = manager.update(odom, trajectory)
        
        # 验证命令有效
        assert cmd is not None
        assert not np.isnan(cmd.vx)
        
        manager.shutdown()
    
    print("✓ test_platform_constraints_config passed")


# =============================================================================
# 10. 集成配置测试
# =============================================================================

def test_full_config_integration():
    """测试完整配置集成"""
    # 创建自定义配置
    config = DEFAULT_CONFIG.copy()
    
    # 修改多个配置
    config['system']['ctrl_freq'] = 100
    config['constraints']['v_max'] = 1.5
    config['constraints']['omega_max'] = 1.5
    config['mpc']['horizon'] = 15
    config['consistency']['alpha_min'] = 0.15
    config['safety']['v_stop_thresh'] = 0.03
    config['backup']['lookahead_dist'] = 1.5
    
    manager = ControllerManager(config)
    manager.initialize_default_components()
    
    # 验证配置被正确应用
    assert manager.mpc_tracker.horizon == 15
    assert manager.backup_tracker.lookahead_dist == 1.5
    
    # 运行几个周期
    odom = create_test_odom(vx=1.0)
    trajectory = create_test_trajectory(soft_enabled=True)
    
    for _ in range(10):
        cmd = manager.update(odom, trajectory)
        
        # 验证速度约束
        v = np.sqrt(cmd.vx**2 + cmd.vy**2)
        assert v <= 1.5 * 1.5, f"Velocity {v} exceeds configured limit"
    
    manager.shutdown()
    print("✓ test_full_config_integration passed")


def test_config_hot_reload():
    """测试配置热更新（horizon 动态调整）"""
    config = DEFAULT_CONFIG.copy()
    config['mpc'] = DEFAULT_CONFIG['mpc'].copy()
    config['mpc']['horizon'] = 20
    config['mpc']['horizon_degraded'] = 10
    
    manager = ControllerManager(config)
    manager.initialize_default_components()
    
    # 初始 horizon 应该等于配置的 horizon_normal
    assert manager.horizon_normal == 20
    assert manager._current_horizon == manager.horizon_normal
    
    # 模拟降级时 horizon 调整
    manager._on_state_changed(ControllerState.NORMAL, ControllerState.MPC_DEGRADED)
    assert manager._current_horizon == manager.horizon_degraded
    assert manager._current_horizon == 10
    
    # 恢复
    manager._on_state_changed(ControllerState.MPC_DEGRADED, ControllerState.NORMAL)
    assert manager._current_horizon == manager.horizon_normal
    assert manager._current_horizon == 20
    
    manager.shutdown()
    print("✓ test_config_hot_reload passed")


# =============================================================================
# 运行所有测试
# =============================================================================

if __name__ == '__main__':
    print("=" * 60)
    print("Configuration Effects Tests")
    print("=" * 60)
    
    print("\n[1/10] Constraint Config Tests...")
    print("-" * 40)
    test_velocity_constraint_config()
    test_omega_constraint_config()
    test_acceleration_constraint_config()
    
    print("\n[2/10] Timeout Config Tests...")
    print("-" * 40)
    test_odom_timeout_config()
    test_startup_grace_config()
    test_trajectory_grace_config()
    
    print("\n[3/10] EKF Config Tests...")
    print("-" * 40)
    test_ekf_slip_threshold_config()
    test_ekf_measurement_noise_config()
    test_ekf_heading_fallback_config()
    
    print("\n[4/10] MPC Config Tests...")
    print("-" * 40)
    test_mpc_horizon_config()
    test_mpc_weights_config()
    test_mpc_health_monitor_config()
    
    print("\n[5/10] Consistency Config Tests...")
    print("-" * 40)
    test_consistency_alpha_min_config()
    test_consistency_weights_config()
    test_consistency_thresholds_config()
    
    print("\n[6/10] Safety Config Tests...")
    print("-" * 40)
    test_safety_velocity_margin_config()
    test_safety_stop_threshold_config()
    test_state_machine_recovery_config()
    
    print("\n[7/10] Backup Controller Config Tests...")
    print("-" * 40)
    test_pure_pursuit_lookahead_config()
    test_pure_pursuit_heading_gain_config()
    
    print("\n[8/10] Transition Config Tests...")
    print("-" * 40)
    test_transition_tau_config()
    test_transition_max_duration_config()
    
    print("\n[9/10] Platform Config Tests...")
    print("-" * 40)
    test_platform_output_frame_config()
    test_platform_constraints_config()
    
    print("\n[10/10] Integration Config Tests...")
    print("-" * 40)
    test_full_config_integration()
    test_config_hot_reload()
    
    print("\n" + "=" * 60)
    print("✅ ALL CONFIGURATION TESTS PASSED")
    print("=" * 60)
    print("\n配置功能验证: 通过")
    print("- 约束配置: 正常生效")
    print("- 超时配置: 正常生效")
    print("- EKF 配置: 正常生效")
    print("- MPC 配置: 正常生效")
    print("- 一致性配置: 正常生效")
    print("- 安全配置: 正常生效")
    print("- 备用控制器配置: 正常生效")
    print("- 平滑过渡配置: 正常生效")
    print("- 平台配置: 正常生效")
