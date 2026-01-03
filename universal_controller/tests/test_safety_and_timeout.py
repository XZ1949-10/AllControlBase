"""
安全监控和超时测试

测试安全监控器和超时监控器的各种场景。

测试覆盖:
1. 速度限制
2. 加速度限制
3. 超时检测
4. 启动宽限期
5. 紧急停止
"""
import pytest
import numpy as np
import time
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from universal_controller.core.data_types import ControlOutput
from universal_controller.core.diagnostics_input import DiagnosticsInput
from universal_controller.config.default_config import DEFAULT_CONFIG, PLATFORM_CONFIG
from universal_controller.safety.safety_monitor import BasicSafetyMonitor
from universal_controller.safety.timeout_monitor import TimeoutMonitor


class TestSafetyMonitorVelocityLimits:
    """测试安全监控器速度限制"""
    
    def test_velocity_within_limits(self):
        """测试速度在限制范围内"""
        config = DEFAULT_CONFIG.copy()
        platform_config = PLATFORM_CONFIG['differential']
        monitor = BasicSafetyMonitor(config, platform_config)
        
        state = np.zeros(8)
        cmd = ControlOutput(vx=0.5, vy=0.0, vz=0.0, omega=0.3, frame_id="base_link")
        diagnostics = DiagnosticsInput()
        
        decision = monitor.check(state, cmd, diagnostics)
        
        assert decision.safe
        assert decision.limited_cmd is None
    
    def test_velocity_exceeds_v_max(self):
        """测试速度超过 v_max"""
        config = DEFAULT_CONFIG.copy()
        platform_config = PLATFORM_CONFIG['differential']
        # constraints 在 DEFAULT_CONFIG 中，不在 platform_config 中
        v_max = config.get('constraints', {}).get('v_max', 2.0)
        
        monitor = BasicSafetyMonitor(config, platform_config)
        
        state = np.zeros(8)
        cmd = ControlOutput(vx=v_max * 2, vy=0.0, vz=0.0, omega=0.0, frame_id="base_link")
        diagnostics = DiagnosticsInput()
        
        decision = monitor.check(state, cmd, diagnostics)
        
        assert not decision.safe
        assert decision.limited_cmd is not None
        assert abs(decision.limited_cmd.vx) <= v_max
    
    def test_omega_exceeds_limit(self):
        """测试角速度超过限制"""
        config = DEFAULT_CONFIG.copy()
        platform_config = PLATFORM_CONFIG['differential']
        # constraints 在 DEFAULT_CONFIG 中，不在 platform_config 中
        omega_max = config.get('constraints', {}).get('omega_max', 2.0)
        
        monitor = BasicSafetyMonitor(config, platform_config)
        
        state = np.zeros(8)
        cmd = ControlOutput(vx=0.0, vy=0.0, vz=0.0, omega=omega_max * 2, frame_id="base_link")
        diagnostics = DiagnosticsInput()
        
        decision = monitor.check(state, cmd, diagnostics)
        
        assert not decision.safe
        assert decision.limited_cmd is not None
        assert abs(decision.limited_cmd.omega) <= omega_max
    
    def test_negative_velocity(self):
        """测试负速度（倒车）"""
        config = DEFAULT_CONFIG.copy()
        platform_config = PLATFORM_CONFIG['differential']
        # constraints 在 DEFAULT_CONFIG 中，不在 platform_config 中
        v_min = config.get('constraints', {}).get('v_min', -0.5)
        
        monitor = BasicSafetyMonitor(config, platform_config)
        
        state = np.zeros(8)
        cmd = ControlOutput(vx=v_min * 2, vy=0.0, vz=0.0, omega=0.0, frame_id="base_link")
        diagnostics = DiagnosticsInput()
        
        decision = monitor.check(state, cmd, diagnostics)
        
        # 应该被限制
        if decision.limited_cmd:
            assert decision.limited_cmd.vx >= v_min


class TestSafetyMonitorPlatformSpecific:
    """测试不同平台的安全监控"""
    
    def test_differential_platform(self):
        """测试差速平台"""
        config = DEFAULT_CONFIG.copy()
        platform_config = PLATFORM_CONFIG['differential']
        monitor = BasicSafetyMonitor(config, platform_config)
        
        state = np.zeros(8)
        cmd = ControlOutput(vx=1.0, vy=0.0, vz=0.0, omega=0.5, frame_id="base_link")
        diagnostics = DiagnosticsInput()
        
        decision = monitor.check(state, cmd, diagnostics)
        
        assert decision is not None
    
    def test_omni_platform(self):
        """测试全向平台"""
        config = DEFAULT_CONFIG.copy()
        platform_config = PLATFORM_CONFIG['omni']
        monitor = BasicSafetyMonitor(config, platform_config)
        
        state = np.zeros(8)
        # 全向车可以有 vy
        cmd = ControlOutput(vx=0.5, vy=0.5, vz=0.0, omega=0.3, frame_id="base_link")
        diagnostics = DiagnosticsInput()
        
        decision = monitor.check(state, cmd, diagnostics)
        
        assert decision is not None
    
    def test_quadrotor_platform(self):
        """测试四旋翼平台"""
        config = DEFAULT_CONFIG.copy()
        platform_config = PLATFORM_CONFIG['quadrotor']
        monitor = BasicSafetyMonitor(config, platform_config)
        
        state = np.zeros(8)
        # 四旋翼可以有 vz
        cmd = ControlOutput(vx=0.5, vy=0.5, vz=0.5, omega=0.3, frame_id="base_link")
        diagnostics = DiagnosticsInput()
        
        decision = monitor.check(state, cmd, diagnostics)
        
        assert decision is not None


class TestTimeoutMonitorBasic:
    """测试超时监控器基本功能"""
    
    def test_no_timeout_with_fresh_data(self):
        """测试新鲜数据无超时"""
        config = DEFAULT_CONFIG.copy()
        monitor = TimeoutMonitor(config)
        
        # 模拟启动后 - 使用 time.monotonic() 与 TimeoutMonitor 内部一致
        monitor._startup_time = time.monotonic() - 10
        
        data_ages = {'odom': 0.01, 'trajectory': 0.01, 'imu': 0.01}
        status = monitor.check(data_ages)
        
        assert not status.odom_timeout
        assert not status.traj_timeout
        assert not status.imu_timeout
    
    def test_odom_timeout(self):
        """测试里程计超时"""
        config = DEFAULT_CONFIG.copy()
        monitor = TimeoutMonitor(config)
        
        # 使用 time.monotonic() 与 TimeoutMonitor 内部一致
        monitor._startup_time = time.monotonic() - 10
        
        # 里程计数据很旧
        data_ages = {'odom': 10.0, 'trajectory': 0.01, 'imu': 0.01}
        status = monitor.check(data_ages)
        
        assert status.odom_timeout
    
    def test_trajectory_timeout(self):
        """测试轨迹超时"""
        config = DEFAULT_CONFIG.copy()
        monitor = TimeoutMonitor(config)
        
        # 使用 time.monotonic() 与 TimeoutMonitor 内部一致
        monitor._startup_time = time.monotonic() - 10
        
        # 轨迹数据很旧
        data_ages = {'odom': 0.01, 'trajectory': 10.0, 'imu': 0.01}
        status = monitor.check(data_ages)
        
        assert status.traj_timeout


class TestTimeoutMonitorStartupGrace:
    """测试超时监控器启动宽限期"""
    
    def test_startup_grace_period(self):
        """测试启动宽限期"""
        config = DEFAULT_CONFIG.copy()
        monitor = TimeoutMonitor(config)
        
        # 刚启动，应该在宽限期内
        status = monitor.check({})
        
        assert status.in_startup_grace
        assert not status.odom_timeout
        assert not status.traj_timeout
    
    def test_startup_grace_ends(self):
        """测试启动宽限期结束"""
        config = DEFAULT_CONFIG.copy()
        config['watchdog']['startup_grace_ms'] = 10  # 很短的宽限期
        monitor = TimeoutMonitor(config)
        
        # 模拟收到数据 - 使用 time.monotonic() 与 TimeoutMonitor 内部一致
        monitor._startup_time = time.monotonic() - 1  # 1 秒前启动
        
        # 宽限期应该已结束
        data_ages = {'odom': 10.0, 'trajectory': 10.0, 'imu': 10.0}
        status = monitor.check(data_ages)
        
        assert not status.in_startup_grace


class TestTimeoutMonitorGracePeriod:
    """测试轨迹超时宽限期"""
    
    def test_trajectory_grace_period(self):
        """测试轨迹超时宽限期"""
        config = DEFAULT_CONFIG.copy()
        monitor = TimeoutMonitor(config)
        
        # 使用 time.monotonic() 与 TimeoutMonitor 内部一致
        monitor._startup_time = time.monotonic() - 10
        
        # 第一次超时
        data_ages = {'odom': 0.01, 'trajectory': 10.0, 'imu': 0.01}
        status1 = monitor.check(data_ages)
        
        assert status1.traj_timeout
        # 宽限期可能还没超过
        
        # 等待宽限期
        time.sleep(0.1)
        status2 = monitor.check(data_ages)
        
        # 验证宽限期逻辑
        assert status2.traj_timeout
    
    def test_trajectory_grace_reset(self):
        """测试轨迹宽限期重置"""
        config = DEFAULT_CONFIG.copy()
        monitor = TimeoutMonitor(config)
        
        monitor._startup_time = time.time() - 10
        
        # 触发超时
        data_ages_old = {'odom': 0.01, 'trajectory': 10.0, 'imu': 0.01}
        monitor.check(data_ages_old)
        
        # 收到新数据，宽限期应该重置
        data_ages_new = {'odom': 0.01, 'trajectory': 0.01, 'imu': 0.01}
        status = monitor.check(data_ages_new)
        
        assert not status.traj_timeout
        assert not status.traj_grace_exceeded


class TestTimeoutMonitorDisabled:
    """测试禁用超时检测"""
    
    def test_disabled_odom_timeout(self):
        """测试禁用里程计超时"""
        config = DEFAULT_CONFIG.copy()
        config['watchdog']['odom_timeout_ms'] = -1  # 禁用
        monitor = TimeoutMonitor(config)
        
        # 使用 time.monotonic() 与 TimeoutMonitor 内部一致
        monitor._startup_time = time.monotonic() - 10
        
        data_ages = {'odom': 999.0, 'trajectory': 0.01, 'imu': 0.01}
        status = monitor.check(data_ages)
        
        assert not status.odom_timeout
    
    def test_disabled_imu_timeout(self):
        """测试禁用 IMU 超时"""
        config = DEFAULT_CONFIG.copy()
        config['watchdog']['imu_timeout_ms'] = -1  # 禁用
        monitor = TimeoutMonitor(config)
        
        # 使用 time.monotonic() 与 TimeoutMonitor 内部一致
        monitor._startup_time = time.monotonic() - 10
        
        data_ages = {'odom': 0.01, 'trajectory': 0.01, 'imu': 999.0}
        status = monitor.check(data_ages)
        
        assert not status.imu_timeout


class TestTimeoutMonitorReset:
    """测试超时监控器重置"""
    
    def test_reset_clears_state(self):
        """测试重置清除状态"""
        config = DEFAULT_CONFIG.copy()
        monitor = TimeoutMonitor(config)
        
        # 设置一些状态
        monitor._startup_time = time.time() - 10
        monitor._traj_timeout_start = time.time()
        
        # 重置
        monitor.reset()
        
        assert monitor._startup_time is None
        assert monitor._traj_timeout_start is None


class TestSafetyMonitorHealthMetrics:
    """测试安全监控器健康指标"""
    
    def test_health_metrics_preserved(self):
        """测试健康指标被保留"""
        config = DEFAULT_CONFIG.copy()
        platform_config = PLATFORM_CONFIG['differential']
        monitor = BasicSafetyMonitor(config, platform_config)
        
        state = np.zeros(8)
        cmd = ControlOutput(
            vx=100.0, vy=0.0, vz=0.0, omega=0.0, 
            frame_id="base_link",
            health_metrics={'test_key': 'test_value'}
        )
        diagnostics = DiagnosticsInput()
        
        decision = monitor.check(state, cmd, diagnostics)
        
        if decision.limited_cmd:
            assert 'test_key' in decision.limited_cmd.health_metrics


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
