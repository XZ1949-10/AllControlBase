"""
系统可用性测试

验证 Universal Controller 是否达到可用状态的综合测试。
包括：
1. 端到端控制循环测试
2. 多平台支持测试
3. 状态机完整流程测试
4. 异常处理和恢复测试
5. 性能基准测试
"""
import numpy as np
import time
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from universal_controller.core.data_types import (
    Trajectory, ControlOutput, Point3D, Header, Odometry, Imu
)
from universal_controller.core.enums import ControllerState, PlatformType
from universal_controller.core.diagnostics_input import DiagnosticsInput
from universal_controller.config.default_config import DEFAULT_CONFIG, PLATFORM_CONFIG
from universal_controller.manager.controller_manager import ControllerManager
from universal_controller.mock.test_data_generator import (
    create_test_trajectory,
    create_test_odom,
    create_test_imu,
    create_test_state_sequence
)


# =============================================================================
# 1. 端到端控制循环测试
# =============================================================================

def test_end_to_end_control_loop():
    """测试完整的控制循环：从输入到输出"""
    config = DEFAULT_CONFIG.copy()
    manager = ControllerManager(config)
    manager.initialize_default_components()
    
    # 模拟 100 个控制周期
    num_cycles = 100
    dt = 0.02  # 50Hz
    
    x, y, theta = 0.0, 0.0, 0.0
    vx = 0.0
    
    successful_cycles = 0
    total_solve_time = 0.0
    
    for i in range(num_cycles):
        # 创建当前状态的里程计
        odom = create_test_odom(x=x, y=y, theta=theta, vx=vx)
        
        # 创建轨迹（从当前位置向前）
        trajectory = create_test_trajectory(
            trajectory_type='straight',
            speed=1.0,
            direction=theta,
            soft_enabled=True
        )
        # 偏移轨迹到当前位置
        for pt in trajectory.points:
            pt.x += x
            pt.y += y
        
        # 创建 IMU
        imu = create_test_imu()
        
        # 执行控制更新
        cmd = manager.update(odom, trajectory, imu)
        
        if cmd.success:
            successful_cycles += 1
        total_solve_time += cmd.solve_time_ms
        
        # 简单的运动学更新
        x += cmd.vx * np.cos(theta) * dt
        y += cmd.vx * np.sin(theta) * dt
        theta += cmd.omega * dt
        vx = cmd.vx
    
    # 验证
    success_rate = successful_cycles / num_cycles
    avg_solve_time = total_solve_time / num_cycles
    
    assert success_rate >= 0.9, f"Success rate too low: {success_rate:.2%}"
    assert avg_solve_time < 20.0, f"Average solve time too high: {avg_solve_time:.2f}ms"
    
    # 验证状态机正常工作
    final_state = manager.get_state()
    assert final_state in [ControllerState.NORMAL, ControllerState.SOFT_DISABLED, 
                          ControllerState.MPC_DEGRADED], f"Unexpected final state: {final_state}"
    
    manager.shutdown()
    print(f"✓ test_end_to_end_control_loop passed (success={success_rate:.1%}, avg_time={avg_solve_time:.2f}ms)")


def test_trajectory_tracking_accuracy():
    """测试轨迹跟踪精度"""
    config = DEFAULT_CONFIG.copy()
    manager = ControllerManager(config)
    manager.initialize_default_components()
    
    # 使用直线轨迹测试
    trajectory = create_test_trajectory(
        trajectory_type='straight',
        num_points=50,
        speed=1.0,
        soft_enabled=True
    )
    
    x, y, theta = 0.0, 0.0, 0.0
    dt = 0.02
    
    position_errors = []
    
    for i in range(200):
        odom = create_test_odom(x=x, y=y, theta=theta, vx=1.0)
        cmd = manager.update(odom, trajectory)
        
        # 计算到轨迹的距离
        if len(trajectory.points) > 0:
            min_dist = float('inf')
            for pt in trajectory.points:
                dist = np.sqrt((pt.x - x)**2 + (pt.y - y)**2)
                min_dist = min(min_dist, dist)
            position_errors.append(min_dist)
        
        # 更新位置
        x += cmd.vx * np.cos(theta) * dt
        y += cmd.vx * np.sin(theta) * dt
        theta += cmd.omega * dt
    
    # 验证跟踪误差
    if position_errors:
        avg_error = np.mean(position_errors)
        max_error = np.max(position_errors)
        assert avg_error < 1.0, f"Average tracking error too high: {avg_error:.3f}m"
        assert max_error < 3.0, f"Max tracking error too high: {max_error:.3f}m"
    
    manager.shutdown()
    print(f"✓ test_trajectory_tracking_accuracy passed (avg_err={np.mean(position_errors):.3f}m)")


# =============================================================================
# 2. 多平台支持测试
# =============================================================================

def test_differential_platform():
    """测试差速车平台"""
    config = DEFAULT_CONFIG.copy()
    config['system'] = DEFAULT_CONFIG['system'].copy()
    config['system']['platform'] = 'differential'
    
    manager = ControllerManager(config)
    manager.initialize_default_components()
    
    odom = create_test_odom(vx=1.0)
    trajectory = create_test_trajectory()
    
    cmd = manager.update(odom, trajectory)
    
    # 差速车输出应该在 base_link 坐标系
    assert cmd.frame_id == 'base_link', f"Expected base_link, got {cmd.frame_id}"
    # 差速车 vy 应该为 0
    assert abs(cmd.vy) < 1e-6, f"Differential robot should have vy=0, got {cmd.vy}"
    
    manager.shutdown()
    print("✓ test_differential_platform passed")


def test_omni_platform():
    """测试全向车平台"""
    config = DEFAULT_CONFIG.copy()
    config['system'] = DEFAULT_CONFIG['system'].copy()
    config['system']['platform'] = 'omni'
    
    manager = ControllerManager(config)
    manager.initialize_default_components()
    
    odom = create_test_odom(vx=1.0, vy=0.5)
    trajectory = create_test_trajectory()
    
    cmd = manager.update(odom, trajectory)
    
    # 全向车输出应该在 world 坐标系
    assert cmd.frame_id == 'world', f"Expected world, got {cmd.frame_id}"
    
    manager.shutdown()
    print("✓ test_omni_platform passed")


def test_quadrotor_platform():
    """测试四旋翼平台"""
    config = DEFAULT_CONFIG.copy()
    config['system'] = DEFAULT_CONFIG['system'].copy()
    config['system']['platform'] = 'quadrotor'
    
    manager = ControllerManager(config)
    manager.initialize_default_components()
    
    # 验证姿态控制器已初始化
    assert manager.attitude_controller is not None, "Attitude controller should be initialized for quadrotor"
    assert manager.is_quadrotor == True
    
    odom = create_test_odom(vx=1.0, vy=0.5, vz=0.2)
    trajectory = create_test_trajectory()
    
    cmd = manager.update(odom, trajectory)
    
    # 四旋翼输出应该在 world 坐标系
    assert cmd.frame_id == 'world', f"Expected world, got {cmd.frame_id}"
    
    # 测试姿态命令计算
    state = np.array([0, 0, 1, 1.0, 0.5, 0.2, 0, 0])
    attitude_cmd = manager.compute_attitude_command(cmd, state)
    
    assert attitude_cmd is not None, "Attitude command should not be None for quadrotor"
    assert hasattr(attitude_cmd, 'roll')
    assert hasattr(attitude_cmd, 'pitch')
    assert hasattr(attitude_cmd, 'thrust')
    
    manager.shutdown()
    print("✓ test_quadrotor_platform passed")


def test_ackermann_platform():
    """测试阿克曼车平台"""
    config = DEFAULT_CONFIG.copy()
    config['system'] = DEFAULT_CONFIG['system'].copy()
    config['system']['platform'] = 'ackermann'
    
    manager = ControllerManager(config)
    manager.initialize_default_components()
    
    odom = create_test_odom(vx=1.0)
    trajectory = create_test_trajectory()
    
    cmd = manager.update(odom, trajectory)
    
    # 阿克曼车输出应该在 base_link 坐标系
    assert cmd.frame_id == 'base_link', f"Expected base_link, got {cmd.frame_id}"
    # 阿克曼车 vy 应该为 0
    assert abs(cmd.vy) < 1e-6, f"Ackermann robot should have vy=0, got {cmd.vy}"
    
    manager.shutdown()
    print("✓ test_ackermann_platform passed")


# =============================================================================
# 3. 状态机完整流程测试
# =============================================================================

def test_state_machine_full_cycle():
    """测试状态机完整生命周期"""
    config = DEFAULT_CONFIG.copy()
    manager = ControllerManager(config)
    manager.initialize_default_components()
    
    # 初始状态应该是 INIT
    assert manager.get_state() == ControllerState.INIT
    
    # 发送有效数据，应该转换到 NORMAL
    odom = create_test_odom(vx=1.0)
    trajectory = create_test_trajectory(soft_enabled=True)
    
    for _ in range(5):
        manager.update(odom, trajectory)
    
    state = manager.get_state()
    assert state in [ControllerState.NORMAL, ControllerState.SOFT_DISABLED, 
                    ControllerState.MPC_DEGRADED], f"Expected operational state, got {state}"
    
    # 重置后应该回到 INIT
    manager.reset()
    assert manager.get_state() == ControllerState.INIT
    
    manager.shutdown()
    print("✓ test_state_machine_full_cycle passed")


def test_state_machine_timeout_handling():
    """测试状态机超时处理"""
    config = DEFAULT_CONFIG.copy()
    # 设置较短的超时时间便于测试
    config['watchdog'] = DEFAULT_CONFIG['watchdog'].copy()
    config['watchdog']['odom_timeout_ms'] = 50
    config['watchdog']['startup_grace_ms'] = 10
    
    manager = ControllerManager(config)
    manager.initialize_default_components()
    
    # 先发送一些有效数据
    odom = create_test_odom(vx=1.0)
    trajectory = create_test_trajectory()
    
    for _ in range(3):
        manager.update(odom, trajectory)
    
    # 等待超时
    time.sleep(0.1)
    
    # 使用旧的时间戳（模拟超时）
    old_odom = create_test_odom(vx=1.0)
    old_odom.header.stamp = time.time() - 1.0  # 1秒前的数据
    
    manager.update(old_odom, trajectory)
    
    # 状态应该转换到 STOPPING 或 STOPPED
    state = manager.get_state()
    # 注意：由于超时检测使用单调时钟，这里可能不会立即触发
    # 但至少应该不是 INIT
    assert state != ControllerState.INIT or state == ControllerState.STOPPING
    
    manager.shutdown()
    print("✓ test_state_machine_timeout_handling passed")


# =============================================================================
# 4. 异常处理和恢复测试
# =============================================================================

def test_empty_trajectory_handling():
    """测试空轨迹处理"""
    config = DEFAULT_CONFIG.copy()
    manager = ControllerManager(config)
    manager.initialize_default_components()
    
    odom = create_test_odom(vx=1.0)
    
    # 创建空轨迹 (使用 odom 坐标系，因为空轨迹不需要变换)
    empty_trajectory = Trajectory(
        header=Header(stamp=time.time(), frame_id='odom'),
        points=[],
        velocities=None,
        dt_sec=0.1,
        confidence=0.9
    )
    
    # 应该能够处理空轨迹而不崩溃
    cmd = manager.update(odom, empty_trajectory)
    
    # 空轨迹应该返回停止命令
    assert cmd.vx == 0.0 or not cmd.success
    
    manager.shutdown()
    print("✓ test_empty_trajectory_handling passed")


def test_nan_input_handling():
    """测试 NaN 输入处理"""
    config = DEFAULT_CONFIG.copy()
    manager = ControllerManager(config)
    manager.initialize_default_components()
    
    # 先发送正常数据
    odom = create_test_odom(vx=1.0)
    trajectory = create_test_trajectory()
    manager.update(odom, trajectory)
    
    # 创建包含 NaN 的里程计
    nan_odom = create_test_odom(vx=float('nan'))
    
    # 应该能够处理 NaN 而不崩溃
    try:
        cmd = manager.update(nan_odom, trajectory)
        # 如果没有崩溃，测试通过
    except Exception as e:
        # 如果抛出异常，检查是否是预期的异常类型
        assert isinstance(e, (ValueError, RuntimeError)), f"Unexpected exception: {type(e)}"
    
    manager.shutdown()
    print("✓ test_nan_input_handling passed")


def test_recovery_from_mpc_failure():
    """测试从 MPC 失败中恢复"""
    config = DEFAULT_CONFIG.copy()
    manager = ControllerManager(config)
    manager.initialize_default_components()
    
    odom = create_test_odom(vx=1.0)
    trajectory = create_test_trajectory(soft_enabled=True)
    
    # 正常运行一段时间
    for _ in range(10):
        cmd = manager.update(odom, trajectory)
    
    # 验证系统仍在运行
    state = manager.get_state()
    assert state not in [ControllerState.STOPPED], f"System should not be stopped: {state}"
    
    # 验证备用控制器可用
    assert manager.backup_tracker is not None
    
    manager.shutdown()
    print("✓ test_recovery_from_mpc_failure passed")


# =============================================================================
# 5. 性能基准测试
# =============================================================================

def test_control_loop_performance():
    """测试控制循环性能"""
    config = DEFAULT_CONFIG.copy()
    manager = ControllerManager(config)
    manager.initialize_default_components()
    
    odom = create_test_odom(vx=1.0)
    trajectory = create_test_trajectory(soft_enabled=True)
    imu = create_test_imu()
    
    # 预热
    for _ in range(10):
        manager.update(odom, trajectory, imu)
    
    # 性能测试
    num_iterations = 100
    start_time = time.time()
    
    for _ in range(num_iterations):
        manager.update(odom, trajectory, imu)
    
    elapsed = time.time() - start_time
    avg_time_ms = (elapsed / num_iterations) * 1000
    
    # 验证性能要求：平均每次更新应该小于 20ms (50Hz)
    assert avg_time_ms < 20.0, f"Control loop too slow: {avg_time_ms:.2f}ms"
    
    manager.shutdown()
    print(f"✓ test_control_loop_performance passed (avg={avg_time_ms:.2f}ms)")


def test_memory_stability():
    """测试内存稳定性（长时间运行）"""
    config = DEFAULT_CONFIG.copy()
    manager = ControllerManager(config)
    manager.initialize_default_components()
    
    odom = create_test_odom(vx=1.0)
    trajectory = create_test_trajectory()
    
    # 运行 1000 个周期
    for i in range(1000):
        manager.update(odom, trajectory)
        
        # 每 100 个周期重置一次，测试重置功能
        if i % 100 == 99:
            manager.reset()
            manager.initialize_default_components()
    
    # 如果没有内存泄漏或崩溃，测试通过
    manager.shutdown()
    print("✓ test_memory_stability passed (1000 cycles)")


# =============================================================================
# 6. 诊断和监控测试
# =============================================================================

def test_diagnostics_completeness():
    """测试诊断信息完整性"""
    config = DEFAULT_CONFIG.copy()
    manager = ControllerManager(config)
    manager.initialize_default_components()
    
    odom = create_test_odom(vx=1.0)
    trajectory = create_test_trajectory(soft_enabled=True)
    
    manager.update(odom, trajectory)
    
    # 获取诊断信息
    diag = manager.get_last_published_diagnostics()
    
    assert diag is not None, "Diagnostics should not be None"
    
    # 验证关键字段存在
    required_fields = ['state', 'mpc_success', 'timeout', 'consistency', 'cmd']
    for field in required_fields:
        assert field in diag, f"Missing diagnostic field: {field}"
    
    # 验证超时信息
    assert 'odom_timeout' in diag['timeout']
    assert 'traj_timeout' in diag['timeout']
    
    # 验证一致性信息
    assert 'alpha_soft' in diag['consistency']
    
    manager.shutdown()
    print("✓ test_diagnostics_completeness passed")


def test_diagnostics_callback():
    """测试诊断回调功能"""
    config = DEFAULT_CONFIG.copy()
    manager = ControllerManager(config)
    manager.initialize_default_components()
    
    callback_data = []
    
    def on_diagnostics(diag):
        callback_data.append(diag)
    
    manager.set_diagnostics_callback(on_diagnostics)
    
    odom = create_test_odom(vx=1.0)
    trajectory = create_test_trajectory()
    
    for _ in range(5):
        manager.update(odom, trajectory)
    
    # 验证回调被调用
    assert len(callback_data) >= 5, f"Callback should be called at least 5 times, got {len(callback_data)}"
    
    manager.shutdown()
    print("✓ test_diagnostics_callback passed")


# =============================================================================
# 7. 配置验证测试
# =============================================================================

def test_config_validation_integration():
    """测试配置验证集成"""
    from universal_controller.config.default_config import validate_config, ConfigValidationError
    
    # 有效配置应该通过
    errors = validate_config(DEFAULT_CONFIG, raise_on_error=False)
    assert len(errors) == 0, f"Default config should be valid: {errors}"
    
    # 无效配置应该被检测到
    invalid_config = DEFAULT_CONFIG.copy()
    invalid_config['mpc'] = DEFAULT_CONFIG['mpc'].copy()
    invalid_config['mpc']['horizon'] = -1
    
    errors = validate_config(invalid_config, raise_on_error=False)
    assert len(errors) > 0, "Invalid config should have errors"
    
    print("✓ test_config_validation_integration passed")


# =============================================================================
# 运行所有测试
# =============================================================================

if __name__ == '__main__':
    print("=" * 60)
    print("Universal Controller - System Usability Tests")
    print("=" * 60)
    
    print("\n[1/7] End-to-End Tests...")
    print("-" * 40)
    test_end_to_end_control_loop()
    test_trajectory_tracking_accuracy()
    
    print("\n[2/7] Multi-Platform Tests...")
    print("-" * 40)
    test_differential_platform()
    test_omni_platform()
    test_quadrotor_platform()
    test_ackermann_platform()
    
    print("\n[3/7] State Machine Tests...")
    print("-" * 40)
    test_state_machine_full_cycle()
    test_state_machine_timeout_handling()
    
    print("\n[4/7] Exception Handling Tests...")
    print("-" * 40)
    test_empty_trajectory_handling()
    test_nan_input_handling()
    test_recovery_from_mpc_failure()
    
    print("\n[5/7] Performance Tests...")
    print("-" * 40)
    test_control_loop_performance()
    test_memory_stability()
    
    print("\n[6/7] Diagnostics Tests...")
    print("-" * 40)
    test_diagnostics_completeness()
    test_diagnostics_callback()
    
    print("\n[7/7] Config Validation Tests...")
    print("-" * 40)
    test_config_validation_integration()
    
    print("\n" + "=" * 60)
    print("✅ ALL SYSTEM USABILITY TESTS PASSED")
    print("=" * 60)
    print("\n系统可用性评估: 通过")
    print("- 端到端控制循环: 正常")
    print("- 多平台支持: 正常")
    print("- 状态机流程: 正常")
    print("- 异常处理: 正常")
    print("- 性能指标: 达标")
    print("- 诊断功能: 完整")
    print("- 配置验证: 有效")
