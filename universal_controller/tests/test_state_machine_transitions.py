"""
状态机转换测试

全面测试状态机的所有状态转换路径和边界条件。

测试覆盖:
1. 所有状态转换路径
2. 计数器重置逻辑
3. 超时处理
4. 外部停止请求
5. 恢复逻辑
"""
import pytest
import numpy as np
import time
from universal_controller.core.ros_compat import get_monotonic_time
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from universal_controller.core.enums import ControllerState
from universal_controller.core.diagnostics_input import DiagnosticsInput
from universal_controller.core.data_types import MPCHealthStatus
from universal_controller.config.default_config import DEFAULT_CONFIG
from universal_controller.safety.state_machine import StateMachine


def create_healthy_mpc_status():
    """创建健康的 MPC 状态"""
    return MPCHealthStatus(
        healthy=True,
        can_recover=True,
        degradation_warning=False,
        consecutive_near_timeout=0,
        kkt_residual=0.0001,
        condition_number=100.0
    )


def create_unhealthy_mpc_status():
    """创建不健康的 MPC 状态"""
    return MPCHealthStatus(
        healthy=False,
        can_recover=False,
        degradation_warning=True,
        consecutive_near_timeout=10,
        kkt_residual=0.1,
        condition_number=10000.0
    )


def create_normal_diagnostics(**kwargs):
    """创建正常诊断输入"""
    defaults = {
        'alpha': 0.8,
        'mpc_health': create_healthy_mpc_status(),
        'mpc_success': True,
        'odom_timeout': False,
        'traj_timeout_exceeded': False,
        'has_valid_data': True,
        'tf2_critical': False,
        'data_valid': True,
        'safety_failed': False,
        'v_horizontal': 0.5,
        'vz': 0.0,
    }
    defaults.update(kwargs)
    return DiagnosticsInput(**defaults)


class TestInitStateTransitions:
    """测试 INIT 状态转换"""
    
    def test_init_to_normal_with_valid_data(self):
        """INIT -> NORMAL: 有有效数据"""
        config = DEFAULT_CONFIG.copy()
        sm = StateMachine(config)
        
        assert sm.state == ControllerState.INIT
        
        diag = create_normal_diagnostics()
        new_state = sm.update(diag)
        
        assert new_state == ControllerState.NORMAL
    
    def test_init_stays_without_valid_data(self):
        """INIT 保持: 无有效数据"""
        config = DEFAULT_CONFIG.copy()
        sm = StateMachine(config)
        
        diag = create_normal_diagnostics(has_valid_data=False)
        new_state = sm.update(diag)
        
        assert new_state == ControllerState.INIT
    
    def test_init_to_stopping_on_timeout(self):
        """INIT -> STOPPING: 超时"""
        config = DEFAULT_CONFIG.copy()
        sm = StateMachine(config)
        
        diag = create_normal_diagnostics(odom_timeout=True)
        new_state = sm.update(diag)
        
        assert new_state == ControllerState.STOPPING


class TestNormalStateTransitions:
    """测试 NORMAL 状态转换"""
    
    def test_normal_to_soft_disabled_low_alpha(self):
        """NORMAL -> SOFT_DISABLED: 低 alpha"""
        import copy
        config = copy.deepcopy(DEFAULT_CONFIG)
        # 设置 alpha_disable_thresh > 0 以启用 alpha 检查
        config['safety']['state_machine']['alpha_disable_thresh'] = 0.1
        sm = StateMachine(config)
        sm.state = ControllerState.NORMAL
        
        diag = create_normal_diagnostics(alpha=0.05, data_valid=True)
        new_state = sm.update(diag)
        
        assert new_state == ControllerState.SOFT_DISABLED
    
    def test_normal_to_mpc_degraded_on_safety_fail(self):
        """NORMAL -> MPC_DEGRADED: 安全检查失败"""
        config = DEFAULT_CONFIG.copy()
        sm = StateMachine(config)
        sm.state = ControllerState.NORMAL
        
        diag = create_normal_diagnostics(safety_failed=True)
        new_state = sm.update(diag)
        
        assert new_state == ControllerState.MPC_DEGRADED
    
    def test_normal_to_mpc_degraded_on_warning(self):
        """NORMAL -> MPC_DEGRADED: MPC 降级警告"""
        config = DEFAULT_CONFIG.copy()
        sm = StateMachine(config)
        sm.state = ControllerState.NORMAL
        
        unhealthy = create_unhealthy_mpc_status()
        diag = create_normal_diagnostics(mpc_health=unhealthy)
        new_state = sm.update(diag)
        
        assert new_state == ControllerState.MPC_DEGRADED
    
    def test_normal_to_backup_on_mpc_failures(self):
        """NORMAL -> BACKUP_ACTIVE: 连续 MPC 失败"""
        config = DEFAULT_CONFIG.copy()
        sm = StateMachine(config)
        sm.state = ControllerState.NORMAL
        
        # 连续 MPC 失败
        for _ in range(sm.mpc_fail_thresh + 1):
            diag = create_normal_diagnostics(mpc_success=False)
            new_state = sm.update(diag)
        
        assert new_state == ControllerState.BACKUP_ACTIVE
    
    def test_normal_to_stopping_on_timeout(self):
        """NORMAL -> STOPPING: 超时"""
        config = DEFAULT_CONFIG.copy()
        sm = StateMachine(config)
        sm.state = ControllerState.NORMAL
        
        diag = create_normal_diagnostics(traj_timeout_exceeded=True)
        new_state = sm.update(diag)
        
        assert new_state == ControllerState.STOPPING


class TestSoftDisabledTransitions:
    """测试 SOFT_DISABLED 状态转换"""
    
    def test_soft_disabled_to_normal_recovery(self):
        """SOFT_DISABLED -> NORMAL: alpha 恢复"""
        config = DEFAULT_CONFIG.copy()
        sm = StateMachine(config)
        sm.state = ControllerState.SOFT_DISABLED
        
        # 需要连续多次高 alpha 才能恢复
        healthy = create_healthy_mpc_status()
        for _ in range(sm.alpha_recovery_thresh + 1):
            diag = create_normal_diagnostics(alpha=0.9, mpc_health=healthy)
            new_state = sm.update(diag)
        
        assert new_state == ControllerState.NORMAL
    
    def test_soft_disabled_to_mpc_degraded_on_safety_fail(self):
        """SOFT_DISABLED -> MPC_DEGRADED: 安全检查失败"""
        config = DEFAULT_CONFIG.copy()
        sm = StateMachine(config)
        sm.state = ControllerState.SOFT_DISABLED
        
        diag = create_normal_diagnostics(safety_failed=True)
        new_state = sm.update(diag)
        
        assert new_state == ControllerState.MPC_DEGRADED
    
    def test_soft_disabled_to_backup_on_mpc_fail(self):
        """SOFT_DISABLED -> BACKUP_ACTIVE: MPC 失败"""
        config = DEFAULT_CONFIG.copy()
        sm = StateMachine(config)
        sm.state = ControllerState.SOFT_DISABLED
        
        diag = create_normal_diagnostics(mpc_success=False)
        new_state = sm.update(diag)
        
        assert new_state == ControllerState.BACKUP_ACTIVE


class TestMPCDegradedTransitions:
    """测试 MPC_DEGRADED 状态转换"""
    
    def test_mpc_degraded_to_backup_on_mpc_fail(self):
        """MPC_DEGRADED -> BACKUP_ACTIVE: MPC 失败"""
        config = DEFAULT_CONFIG.copy()
        sm = StateMachine(config)
        sm.state = ControllerState.MPC_DEGRADED
        
        diag = create_normal_diagnostics(mpc_success=False)
        new_state = sm.update(diag)
        
        assert new_state == ControllerState.BACKUP_ACTIVE
    
    def test_mpc_degraded_to_normal_recovery(self):
        """MPC_DEGRADED -> NORMAL: 恢复"""
        config = DEFAULT_CONFIG.copy()
        sm = StateMachine(config)
        sm.state = ControllerState.MPC_DEGRADED
        
        healthy = create_healthy_mpc_status()
        healthy.can_recover = True
        
        # 需要连续多次成功才能恢复
        for _ in range(sm.mpc_recovery_thresh + 1):
            diag = create_normal_diagnostics(alpha=0.9, mpc_health=healthy)
            new_state = sm.update(diag)
        
        assert new_state == ControllerState.NORMAL
    
    def test_mpc_degraded_to_soft_disabled_recovery(self):
        """MPC_DEGRADED -> SOFT_DISABLED: 低 alpha 恢复"""
        import copy
        config = copy.deepcopy(DEFAULT_CONFIG)
        # 设置 alpha_disable_thresh > 0 以启用 alpha 检查
        config['safety']['state_machine']['alpha_disable_thresh'] = 0.1
        sm = StateMachine(config)
        sm.state = ControllerState.MPC_DEGRADED
        
        healthy = create_healthy_mpc_status()
        healthy.can_recover = True
        
        # 低 alpha 但可以恢复
        for _ in range(sm.mpc_recovery_thresh + 1):
            diag = create_normal_diagnostics(alpha=0.05, mpc_health=healthy)
            new_state = sm.update(diag)
        
        assert new_state == ControllerState.SOFT_DISABLED


class TestBackupActiveTransitions:
    """测试 BACKUP_ACTIVE 状态转换"""
    
    def test_backup_to_mpc_degraded_recovery(self):
        """BACKUP_ACTIVE -> MPC_DEGRADED: MPC 恢复"""
        config = DEFAULT_CONFIG.copy()
        sm = StateMachine(config)
        sm.state = ControllerState.BACKUP_ACTIVE
        
        # 需要连续多次 MPC 成功才能恢复
        for i in range(sm.mpc_recovery_history_min + sm.mpc_recovery_recent_count):
            diag = create_normal_diagnostics(mpc_success=True)
            new_state = sm.update(diag)
            if new_state == ControllerState.MPC_DEGRADED:
                break
        
        assert new_state == ControllerState.MPC_DEGRADED
    
    def test_backup_stays_on_mpc_fail(self):
        """BACKUP_ACTIVE 保持: MPC 持续失败"""
        config = DEFAULT_CONFIG.copy()
        sm = StateMachine(config)
        sm.state = ControllerState.BACKUP_ACTIVE
        
        for _ in range(5):
            diag = create_normal_diagnostics(mpc_success=False)
            new_state = sm.update(diag)
        
        assert new_state == ControllerState.BACKUP_ACTIVE


class TestStoppingTransitions:
    """测试 STOPPING 状态转换"""
    
    def test_stopping_to_stopped_when_velocity_zero(self):
        """STOPPING -> STOPPED: 速度为零"""
        config = DEFAULT_CONFIG.copy()
        sm = StateMachine(config)
        sm.state = ControllerState.STOPPING
        sm._stopping_start_time = time.time()
        
        diag = create_normal_diagnostics(v_horizontal=0.0, vz=0.0)
        new_state = sm.update(diag)
        
        assert new_state == ControllerState.STOPPED
    
    def test_stopping_stays_with_velocity(self):
        """STOPPING 保持: 仍有速度"""
        config = DEFAULT_CONFIG.copy()
        sm = StateMachine(config)
        sm.state = ControllerState.STOPPING
        sm._stopping_start_time = time.time()
        
        diag = create_normal_diagnostics(v_horizontal=0.5, vz=0.0)
        new_state = sm.update(diag)
        
        assert new_state == ControllerState.STOPPING
    
    def test_stopping_timeout_forces_stopped(self):
        """STOPPING -> STOPPED: 超时强制停止"""
        config = DEFAULT_CONFIG.copy()
        sm = StateMachine(config)
        sm.state = ControllerState.STOPPING
        # 使用 get_monotonic_time() 与 StateMachine 内部一致
        sm._stopping_start_time = get_monotonic_time() - 10  # 模拟超时
        
        diag = create_normal_diagnostics(v_horizontal=0.5, vz=0.0)
        new_state = sm.update(diag)
        
        assert new_state == ControllerState.STOPPED


class TestStoppedTransitions:
    """测试 STOPPED 状态转换"""
    
    def test_stopped_to_mpc_degraded_recovery(self):
        """STOPPED -> MPC_DEGRADED: 恢复"""
        config = DEFAULT_CONFIG.copy()
        sm = StateMachine(config)
        sm.state = ControllerState.STOPPED
        
        diag = create_normal_diagnostics()
        new_state = sm.update(diag)
        
        # STOPPED 恢复时先进入 MPC_DEGRADED
        assert new_state == ControllerState.MPC_DEGRADED
    
    def test_stopped_stays_without_valid_data(self):
        """STOPPED 保持: 无有效数据"""
        config = DEFAULT_CONFIG.copy()
        sm = StateMachine(config)
        sm.state = ControllerState.STOPPED
        
        diag = create_normal_diagnostics(has_valid_data=False)
        new_state = sm.update(diag)
        
        assert new_state == ControllerState.STOPPED


class TestExternalStopRequest:
    """测试外部停止请求"""
    
    def test_request_stop_from_normal(self):
        """从 NORMAL 状态请求停止"""
        config = DEFAULT_CONFIG.copy()
        sm = StateMachine(config)
        sm.state = ControllerState.NORMAL
        
        success = sm.request_stop()
        assert success
        assert sm.is_stop_requested()
        
        diag = create_normal_diagnostics()
        new_state = sm.update(diag)
        
        assert new_state == ControllerState.STOPPING
        assert not sm.is_stop_requested()  # 请求已处理
    
    def test_request_stop_from_backup(self):
        """从 BACKUP_ACTIVE 状态请求停止"""
        config = DEFAULT_CONFIG.copy()
        sm = StateMachine(config)
        sm.state = ControllerState.BACKUP_ACTIVE
        
        success = sm.request_stop()
        assert success
        
        diag = create_normal_diagnostics()
        new_state = sm.update(diag)
        
        assert new_state == ControllerState.STOPPING
    
    def test_request_stop_ignored_when_stopping(self):
        """STOPPING 状态下忽略停止请求"""
        config = DEFAULT_CONFIG.copy()
        sm = StateMachine(config)
        sm.state = ControllerState.STOPPING
        
        success = sm.request_stop()
        assert not success  # 已经在停止中
    
    def test_request_stop_cleared_on_reset(self):
        """重置时清除停止请求"""
        config = DEFAULT_CONFIG.copy()
        sm = StateMachine(config)
        
        sm.request_stop()
        assert sm.is_stop_requested()
        
        sm.reset()
        assert not sm.is_stop_requested()


class TestCounterReset:
    """测试计数器重置"""
    
    def test_counters_reset_on_state_transition(self):
        """状态转换时重置计数器"""
        config = DEFAULT_CONFIG.copy()
        sm = StateMachine(config)
        sm.state = ControllerState.SOFT_DISABLED
        sm.alpha_recovery_count = 5
        sm.mpc_recovery_count = 3
        
        # 触发状态转换
        diag = create_normal_diagnostics(safety_failed=True)
        sm.update(diag)
        
        assert sm.alpha_recovery_count == 0
        assert sm.mpc_recovery_count == 0
    
    def test_mpc_history_cleared_on_transition(self):
        """状态转换时清空 MPC 历史"""
        config = DEFAULT_CONFIG.copy()
        sm = StateMachine(config)
        sm.state = ControllerState.NORMAL
        
        # 填充一些历史
        for _ in range(5):
            sm._mpc_success_history.append(True)
        
        # 触发状态转换
        diag = create_normal_diagnostics(safety_failed=True)
        sm.update(diag)
        
        assert len(sm._mpc_success_history) == 0


class TestStateDurationMonitoring:
    """测试状态持续时间监控"""
    
    def test_state_duration_tracking(self):
        """测试状态持续时间跟踪"""
        config = DEFAULT_CONFIG.copy()
        sm = StateMachine(config)
        
        # 等待一小段时间
        time.sleep(0.01)
        
        duration = sm.get_state_duration()
        assert duration >= 0.01
    
    def test_state_duration_reset_on_transition(self):
        """状态转换时重置持续时间"""
        config = DEFAULT_CONFIG.copy()
        sm = StateMachine(config)
        
        # 等待一段时间
        time.sleep(0.01)
        
        # 触发状态转换
        diag = create_normal_diagnostics()
        sm.update(diag)
        
        # 新状态的持续时间应该很短
        duration = sm.get_state_duration()
        assert duration < 0.1


class TestStateTimeoutAutoStop:
    """测试状态超时自动停止功能 (问题 6 补充测试)"""
    
    def test_mpc_degraded_timeout_triggers_stopping(self):
        """MPC_DEGRADED 状态超时触发 STOPPING"""
        import copy
        config = copy.deepcopy(DEFAULT_CONFIG)
        # 设置很短的超时时间以便测试
        config['safety']['state_machine']['degraded_state_timeout'] = 0.01  # 10ms
        config['safety']['state_machine']['enable_state_timeout_stop'] = True
        sm = StateMachine(config)
        
        # 进入 MPC_DEGRADED 状态
        sm.state = ControllerState.MPC_DEGRADED
        sm._state_entry_time = get_monotonic_time()
        
        # 等待超时
        time.sleep(0.02)
        
        # 更新状态机，应该触发超时转换
        diag = create_normal_diagnostics()
        new_state = sm.update(diag)
        
        assert new_state == ControllerState.STOPPING
    
    def test_backup_active_timeout_triggers_stopping(self):
        """BACKUP_ACTIVE 状态超时触发 STOPPING"""
        import copy
        config = copy.deepcopy(DEFAULT_CONFIG)
        config['safety']['state_machine']['backup_state_timeout'] = 0.01  # 10ms
        config['safety']['state_machine']['enable_state_timeout_stop'] = True
        sm = StateMachine(config)
        
        # 进入 BACKUP_ACTIVE 状态
        sm.state = ControllerState.BACKUP_ACTIVE
        sm._state_entry_time = get_monotonic_time()
        
        # 等待超时
        time.sleep(0.02)
        
        diag = create_normal_diagnostics()
        new_state = sm.update(diag)
        
        assert new_state == ControllerState.STOPPING
    
    def test_timeout_disabled_only_warns(self):
        """超时禁用时只告警不停止"""
        import copy
        config = copy.deepcopy(DEFAULT_CONFIG)
        config['safety']['state_machine']['degraded_state_timeout'] = 0.01  # 10ms
        config['safety']['state_machine']['enable_state_timeout_stop'] = False  # 禁用自动停止
        sm = StateMachine(config)
        
        sm.state = ControllerState.MPC_DEGRADED
        sm._state_entry_time = get_monotonic_time()
        
        time.sleep(0.02)
        
        diag = create_normal_diagnostics()
        new_state = sm.update(diag)
        
        # 不应该转换到 STOPPING，应该保持在 MPC_DEGRADED 或正常恢复
        assert new_state != ControllerState.STOPPING or new_state == ControllerState.MPC_DEGRADED
    
    def test_timeout_callback_invoked(self):
        """超时时调用回调函数"""
        import copy
        config = copy.deepcopy(DEFAULT_CONFIG)
        config['safety']['state_machine']['degraded_state_timeout'] = 0.01
        config['safety']['state_machine']['enable_state_timeout_stop'] = False
        sm = StateMachine(config)
        
        callback_data = []
        
        def on_timeout(state, duration):
            callback_data.append({'state': state, 'duration': duration})
        
        sm.set_state_timeout_callback(on_timeout)
        sm.state = ControllerState.MPC_DEGRADED
        sm._state_entry_time = get_monotonic_time()
        
        time.sleep(0.02)
        
        diag = create_normal_diagnostics()
        sm.update(diag)
        
        # 回调应该被调用
        assert len(callback_data) == 1
        assert callback_data[0]['state'] == ControllerState.MPC_DEGRADED
        assert callback_data[0]['duration'] >= 0.01
    
    def test_timeout_callback_only_once_per_state(self):
        """每个状态周期只触发一次超时回调"""
        import copy
        config = copy.deepcopy(DEFAULT_CONFIG)
        config['safety']['state_machine']['degraded_state_timeout'] = 0.01
        config['safety']['state_machine']['enable_state_timeout_stop'] = False
        sm = StateMachine(config)
        
        callback_count = [0]
        
        def on_timeout(state, duration):
            callback_count[0] += 1
        
        sm.set_state_timeout_callback(on_timeout)
        sm.state = ControllerState.MPC_DEGRADED
        sm._state_entry_time = get_monotonic_time()
        
        time.sleep(0.02)
        
        # 多次更新
        for _ in range(5):
            diag = create_normal_diagnostics()
            sm.update(diag)
        
        # 回调只应该被调用一次
        assert callback_count[0] == 1
    
    def test_normal_state_no_timeout(self):
        """NORMAL 状态不受超时影响"""
        import copy
        config = copy.deepcopy(DEFAULT_CONFIG)
        config['safety']['state_machine']['degraded_state_timeout'] = 0.01
        config['safety']['state_machine']['enable_state_timeout_stop'] = True
        sm = StateMachine(config)
        
        sm.state = ControllerState.NORMAL
        sm._state_entry_time = get_monotonic_time()
        
        time.sleep(0.02)
        
        diag = create_normal_diagnostics()
        new_state = sm.update(diag)
        
        # NORMAL 状态不应该因超时而转换
        assert new_state != ControllerState.STOPPING


class TestConcurrentStopRequests:
    """测试并发停止请求 (问题 7 补充测试)"""
    
    def test_concurrent_stop_requests_thread_safety(self):
        """测试多线程同时调用 request_stop() 的线程安全性"""
        import threading
        
        config = DEFAULT_CONFIG.copy()
        sm = StateMachine(config)
        sm.state = ControllerState.NORMAL
        
        results = []
        errors = []
        
        def request_stop():
            try:
                result = sm.request_stop()
                results.append(result)
            except Exception as e:
                errors.append(e)
        
        # 创建多个线程同时请求停止
        threads = [threading.Thread(target=request_stop) for _ in range(20)]
        
        for t in threads:
            t.start()
        for t in threads:
            t.join()
        
        # 不应该有异常
        assert len(errors) == 0
        
        # 至少有一个请求成功
        assert any(results)
        
        # 停止请求应该被设置
        assert sm.is_stop_requested()
    
    def test_concurrent_stop_and_update(self):
        """测试并发停止请求和状态机更新"""
        import threading
        
        config = DEFAULT_CONFIG.copy()
        sm = StateMachine(config)
        sm.state = ControllerState.NORMAL
        
        stop_results = []
        update_results = []
        errors = []
        
        def request_stop():
            try:
                result = sm.request_stop()
                stop_results.append(result)
            except Exception as e:
                errors.append(('stop', e))
        
        def do_update():
            try:
                diag = create_normal_diagnostics()
                result = sm.update(diag)
                update_results.append(result)
            except Exception as e:
                errors.append(('update', e))
        
        # 创建混合的线程
        threads = []
        for i in range(10):
            if i % 2 == 0:
                threads.append(threading.Thread(target=request_stop))
            else:
                threads.append(threading.Thread(target=do_update))
        
        for t in threads:
            t.start()
        for t in threads:
            t.join()
        
        # 不应该有异常
        assert len(errors) == 0
        
        # 状态机应该处于有效状态
        assert sm.state in ControllerState
    
    def test_is_stop_requested_thread_safety(self):
        """测试 is_stop_requested() 的线程安全性"""
        import threading
        
        config = DEFAULT_CONFIG.copy()
        sm = StateMachine(config)
        sm.state = ControllerState.NORMAL
        
        check_results = []
        
        def check_and_request():
            for _ in range(100):
                sm.request_stop()
                check_results.append(sm.is_stop_requested())
                sm.clear_stop_request()
        
        threads = [threading.Thread(target=check_and_request) for _ in range(5)]
        
        for t in threads:
            t.start()
        for t in threads:
            t.join()
        
        # 所有检查都应该返回布尔值，不应该崩溃
        assert all(isinstance(r, bool) for r in check_results)
    
    def test_stop_request_cleared_after_processing(self):
        """测试停止请求在处理后被清除"""
        config = DEFAULT_CONFIG.copy()
        sm = StateMachine(config)
        sm.state = ControllerState.NORMAL
        
        # 请求停止
        sm.request_stop()
        assert sm.is_stop_requested()
        
        # 更新状态机处理请求
        diag = create_normal_diagnostics()
        new_state = sm.update(diag)
        
        # 请求应该被清除
        assert not sm.is_stop_requested()
        assert new_state == ControllerState.STOPPING


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
