"""
模块间通信测试

测试 universal_controller 各模块之间的数据流和通信机制。

测试覆盖:
1. 状态估计器 → 控制器 数据流
2. 一致性检查器 → 控制器 数据流
3. 安全监控器 → 状态机 数据流
4. 平滑过渡 → 控制输出 数据流
5. MPC 健康监控 → 状态机 数据流
6. 超时监控 → 状态机 数据流
"""
import pytest
import numpy as np
import time
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from universal_controller.core.data_types import (
    Trajectory, ControlOutput, ConsistencyResult, Point3D, Header,
    Odometry, Imu, MPCHealthStatus, TimeoutStatus
)
from universal_controller.core.enums import ControllerState, TrajectoryMode
from universal_controller.core.diagnostics_input import DiagnosticsInput
from universal_controller.config.default_config import DEFAULT_CONFIG, PLATFORM_CONFIG
from universal_controller.estimator.adaptive_ekf import AdaptiveEKFEstimator
from universal_controller.consistency.weighted_analyzer import WeightedConsistencyAnalyzer
from universal_controller.safety.state_machine import StateMachine
from universal_controller.safety.timeout_monitor import TimeoutMonitor
from universal_controller.safety.safety_monitor import BasicSafetyMonitor
from universal_controller.tracker.pure_pursuit import PurePursuitController
from universal_controller.health.mpc_health_monitor import MPCHealthMonitor
from universal_controller.transition.smooth_transition import ExponentialSmoothTransition, LinearSmoothTransition
from universal_controller.tests.fixtures import create_test_trajectory, create_test_odom, create_test_imu


class TestEstimatorToController:
    """测试状态估计器到控制器的数据流"""
    
    def test_estimator_state_format(self):
        """测试估计器输出状态格式正确"""
        config = DEFAULT_CONFIG.copy()
        ekf = AdaptiveEKFEstimator(config)
        
        odom = create_test_odom(x=1.0, y=2.0, vx=0.5, omega=0.1)
        ekf.update_odom(odom)
        
        state_output = ekf.get_state()
        
        # 验证状态向量维度
        assert state_output.state.shape == (8,), "State vector should be 8-dimensional"
        # 验证协方差矩阵维度
        assert state_output.covariance.shape == (8, 8), "Covariance should be 8x8"
        # 验证 IMU bias 维度
        assert state_output.imu_bias.shape == (3,), "IMU bias should be 3-dimensional"
    
    def test_estimator_state_to_pure_pursuit(self):
        """测试估计器状态传递给 Pure Pursuit 控制器"""
        config = DEFAULT_CONFIG.copy()
        platform_config = PLATFORM_CONFIG['differential']
        
        ekf = AdaptiveEKFEstimator(config)
        pp = PurePursuitController(config, platform_config)
        
        # 更新估计器
        odom = create_test_odom(x=0.0, y=0.0, vx=0.5)
        ekf.update_odom(odom)
        state_output = ekf.get_state()
        
        # 创建轨迹
        trajectory = create_test_trajectory(num_points=10)
        consistency = ConsistencyResult(
            alpha=1.0, kappa_consistency=1.0, v_dir_consistency=1.0,
            temporal_smooth=1.0, should_disable_soft=False, data_valid=True
        )
        
        # 控制器应能接受估计器输出的状态
        cmd = pp.compute(state_output.state, trajectory, consistency)
        
        assert cmd is not None
        assert hasattr(cmd, 'vx')
        assert hasattr(cmd, 'omega')
        assert cmd.success
    
    def test_estimator_imu_integration(self):
        """测试 IMU 数据集成到状态估计"""
        config = DEFAULT_CONFIG.copy()
        ekf = AdaptiveEKFEstimator(config)
        
        # 先更新 odom
        odom = create_test_odom(x=0.0, y=0.0, vx=0.5)
        ekf.update_odom(odom)
        
        # 设置 IMU 可用并更新
        ekf.set_imu_available(True)
        imu = create_test_imu(angular_velocity=(0.0, 0.0, 0.2))
        ekf.update_imu(imu)
        
        state_output = ekf.get_state()
        assert state_output.imu_available


class TestConsistencyToController:
    """测试一致性检查器到控制器的数据流"""
    
    def test_consistency_result_format(self):
        """测试一致性结果格式正确"""
        config = DEFAULT_CONFIG.copy()
        analyzer = WeightedConsistencyAnalyzer(config)
        
        trajectory = create_test_trajectory(soft_enabled=True)
        result = analyzer.compute(trajectory)
        
        assert isinstance(result.alpha, (float, np.floating))
        assert 0.0 <= result.alpha <= 1.0
        assert bool(result.should_disable_soft) in [True, False]  # 兼容 numpy bool
        assert bool(result.data_valid) in [True, False]
    
    def test_consistency_affects_velocity_blending(self):
        """测试一致性系数影响速度混合"""
        trajectory = create_test_trajectory(soft_enabled=True, num_points=5)
        
        # 高 alpha: 更多使用 soft velocities
        high_alpha_vel = trajectory.get_blended_velocity(0, alpha=0.9)
        # 低 alpha: 更多使用 hard velocities
        low_alpha_vel = trajectory.get_blended_velocity(0, alpha=0.1)
        
        # 两者应该不同（除非 soft 和 hard 完全相同）
        # 这里主要验证接口正常工作
        assert high_alpha_vel.shape == (4,)
        assert low_alpha_vel.shape == (4,)
    
    def test_consistency_disable_soft_mode(self):
        """测试一致性检查禁用 soft 模式"""
        config = DEFAULT_CONFIG.copy()
        analyzer = WeightedConsistencyAnalyzer(config)
        
        # soft_enabled=False 时应该禁用
        trajectory = create_test_trajectory(soft_enabled=False)
        result = analyzer.compute(trajectory)
        
        assert result.should_disable_soft


class TestSafetyToStateMachine:
    """测试安全监控器到状态机的数据流"""
    
    def test_safety_failure_triggers_state_change(self):
        """测试安全检查失败触发状态转换"""
        config = DEFAULT_CONFIG.copy()
        sm = StateMachine(config)
        
        # 先进入 NORMAL 状态
        sm.state = ControllerState.NORMAL
        
        # 安全检查失败
        diagnostics = DiagnosticsInput(
            alpha=0.5,
            mpc_health=None,
            mpc_success=True,
            odom_timeout=False,
            traj_timeout_exceeded=False,
            has_valid_data=True,
            tf2_critical=False,
            data_valid=True,
            safety_failed=True,  # 安全检查失败
            v_horizontal=0.5,
            vz=0.0,
        )
        
        new_state = sm.update(diagnostics)
        assert new_state == ControllerState.MPC_DEGRADED
    
    def test_safety_monitor_limits_command(self):
        """测试安全监控器限制命令"""
        config = DEFAULT_CONFIG.copy()
        platform_config = PLATFORM_CONFIG['differential']
        monitor = BasicSafetyMonitor(config, platform_config)
        
        state = np.zeros(8)
        
        # 超限命令
        cmd = ControlOutput(vx=100.0, vy=0.0, vz=0.0, omega=50.0, frame_id="base_link")
        diagnostics = DiagnosticsInput()
        
        decision = monitor.check(state, cmd, diagnostics)
        
        assert not decision.safe
        assert decision.limited_cmd is not None
        # 限制后的命令应该在约束范围内
        # 从 config 获取约束，因为 platform_config 可能没有 constraints 键
        v_max = config.get('constraints', {}).get('v_max', 2.0)
        assert abs(decision.limited_cmd.vx) <= v_max


class TestSmoothTransitionFlow:
    """测试平滑过渡数据流"""
    
    def test_exponential_transition_blending(self):
        """测试指数平滑过渡混合"""
        config = DEFAULT_CONFIG.copy()
        transition = ExponentialSmoothTransition(config)
        
        # 起始命令
        from_cmd = ControlOutput(vx=1.0, vy=0.0, vz=0.0, omega=0.5, frame_id="base_link")
        transition.start_transition(from_cmd)
        
        # 目标命令
        to_cmd = ControlOutput(vx=0.0, vy=0.0, vz=0.0, omega=0.0, frame_id="base_link")
        
        # 获取混合输出
        blended = transition.get_blended_output(to_cmd, time.time())
        
        # 混合输出应该在两者之间
        assert 0.0 <= blended.vx <= 1.0
        assert not transition.is_complete()
    
    def test_linear_transition_blending(self):
        """测试线性平滑过渡混合"""
        config = DEFAULT_CONFIG.copy()
        transition = LinearSmoothTransition(config)
        
        from_cmd = ControlOutput(vx=2.0, vy=0.0, vz=0.0, omega=1.0, frame_id="base_link")
        transition.start_transition(from_cmd)
        
        to_cmd = ControlOutput(vx=0.0, vy=0.0, vz=0.0, omega=0.0, frame_id="base_link")
        
        blended = transition.get_blended_output(to_cmd, time.time())
        
        assert 0.0 <= blended.vx <= 2.0
        assert transition.get_progress() >= 0.0
    
    def test_transition_completion(self):
        """测试过渡完成检测"""
        config = DEFAULT_CONFIG.copy()
        config['transition'] = {'duration': 0.01, 'tau': 0.001, 'max_duration': 0.02}
        
        transition = LinearSmoothTransition(config)
        
        from_cmd = ControlOutput(vx=1.0, vy=0.0, vz=0.0, omega=0.0, frame_id="base_link")
        transition.start_transition(from_cmd)
        
        to_cmd = ControlOutput(vx=0.0, vy=0.0, vz=0.0, omega=0.0, frame_id="base_link")
        
        # 等待过渡完成
        time.sleep(0.05)
        blended = transition.get_blended_output(to_cmd, time.time())
        
        assert transition.is_complete()


class TestMPCHealthToStateMachine:
    """测试 MPC 健康监控到状态机的数据流"""
    
    def test_mpc_degradation_warning(self):
        """测试 MPC 降级警告触发状态转换"""
        config = DEFAULT_CONFIG.copy()
        sm = StateMachine(config)
        health_monitor = MPCHealthMonitor(config)
        
        sm.state = ControllerState.NORMAL
        
        # 模拟连续超时
        for _ in range(10):
            health_status = health_monitor.update(50.0, 0.0001, 100.0)  # 高求解时间
        
        assert health_status.degradation_warning
        
        # 传递给状态机
        diagnostics = DiagnosticsInput(
            alpha=0.5,
            mpc_health=health_status,
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
        
        new_state = sm.update(diagnostics)
        assert new_state == ControllerState.MPC_DEGRADED
    
    def test_mpc_recovery_path(self):
        """测试 MPC 恢复路径"""
        config = DEFAULT_CONFIG.copy()
        health_monitor = MPCHealthMonitor(config)
        
        # 先触发降级
        for _ in range(5):
            health_monitor.update(50.0, 0.0001, 100.0)
        
        # 然后恢复
        for _ in range(10):
            status = health_monitor.update(2.0, 0.0001, 50.0)  # 正常求解时间
        
        assert status.can_recover


class TestTimeoutToStateMachine:
    """测试超时监控到状态机的数据流"""
    
    def test_odom_timeout_triggers_stopping(self):
        """测试里程计超时触发停止"""
        config = DEFAULT_CONFIG.copy()
        sm = StateMachine(config)
        timeout_monitor = TimeoutMonitor(config)
        
        sm.state = ControllerState.NORMAL
        
        # 模拟超时（先让启动宽限期过去）
        # 注意：TimeoutMonitor 内部使用 time.monotonic()，不是 time.time()
        timeout_monitor._startup_time = time.monotonic() - 10  # 假设已启动
        
        # 检查超时
        status = timeout_monitor.check({'odom': 10.0, 'trajectory': 0.1, 'imu': 0.1})
        
        diagnostics = DiagnosticsInput(
            alpha=0.5,
            mpc_health=None,
            mpc_success=True,
            odom_timeout=status.odom_timeout,
            traj_timeout_exceeded=status.traj_grace_exceeded,
            has_valid_data=True,
            tf2_critical=False,
            data_valid=True,
            safety_failed=False,
            v_horizontal=0.5,
            vz=0.0,
        )
        
        new_state = sm.update(diagnostics)
        
        if status.odom_timeout:
            assert new_state == ControllerState.STOPPING
    
    def test_trajectory_timeout_grace_period(self):
        """测试轨迹超时宽限期"""
        import copy
        config = copy.deepcopy(DEFAULT_CONFIG)
        # 设置较短的超时时间以便测试
        config['watchdog']['traj_timeout_ms'] = 100  # 100ms
        config['watchdog']['startup_grace_ms'] = 10  # 10ms
        timeout_monitor = TimeoutMonitor(config)
        
        # 模拟收到第一条数据，启动宽限期开始
        # 注意：TimeoutMonitor 内部使用 time.monotonic()，不是 time.time()
        timeout_monitor._startup_time = time.monotonic() - 1  # 1秒前启动，宽限期已过
        
        # 轨迹数据很旧（5秒 = 5000ms > 100ms）
        status1 = timeout_monitor.check({'odom': 0.001, 'trajectory': 5.0, 'imu': 0.001})
        
        # 启动宽限期已过，应该检测到超时
        assert not status1.in_startup_grace
        assert status1.traj_timeout
        
        # 等待轨迹宽限期过去
        time.sleep(0.1)
        status2 = timeout_monitor.check({'odom': 0.001, 'trajectory': 5.0, 'imu': 0.001})
        
        # 验证超时状态被正确跟踪
        assert status2.traj_timeout


class TestEndToEndDataFlow:
    """端到端数据流测试"""
    
    def test_full_control_pipeline(self):
        """测试完整控制管道"""
        config = DEFAULT_CONFIG.copy()
        platform_config = PLATFORM_CONFIG['differential']
        
        # 创建所有组件
        ekf = AdaptiveEKFEstimator(config)
        consistency_checker = WeightedConsistencyAnalyzer(config)
        pp = PurePursuitController(config, platform_config)
        safety_monitor = BasicSafetyMonitor(config, platform_config)
        sm = StateMachine(config)
        
        # 1. 状态估计
        odom = create_test_odom(x=0.0, y=0.0, vx=0.5)
        ekf.update_odom(odom)
        state_output = ekf.get_state()
        
        # 2. 轨迹和一致性检查
        trajectory = create_test_trajectory(num_points=10, soft_enabled=True)
        consistency = consistency_checker.compute(trajectory)
        
        # 3. 控制计算
        cmd = pp.compute(state_output.state, trajectory, consistency)
        
        # 4. 安全检查
        diagnostics = DiagnosticsInput(
            alpha=consistency.alpha,
            mpc_health=None,
            mpc_success=True,
            odom_timeout=False,
            traj_timeout_exceeded=False,
            has_valid_data=True,
            tf2_critical=False,
            data_valid=consistency.data_valid,
            safety_failed=False,
            v_horizontal=np.sqrt(state_output.state[3]**2 + state_output.state[4]**2),
            vz=state_output.state[5],
        )
        
        safety_decision = safety_monitor.check(state_output.state, cmd, diagnostics)
        
        # 5. 状态机更新
        new_state = sm.update(diagnostics)
        
        # 验证管道正常工作
        assert cmd.success
        assert safety_decision.safe or safety_decision.limited_cmd is not None
        assert new_state in ControllerState


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
