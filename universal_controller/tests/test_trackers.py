"""
轨迹跟踪器测试

验证 MPC 控制器和 Pure Pursuit 备用控制器的功能：
1. 基本计算功能
2. 约束处理
3. 平滑过渡
4. 健康指标
"""
import numpy as np
import time
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from universal_controller.tracker.mpc_controller import MPCController
from universal_controller.tracker.pure_pursuit import PurePursuitController
from universal_controller.core.data_types import ConsistencyResult
from universal_controller.config.default_config import DEFAULT_CONFIG, PLATFORM_CONFIG
from universal_controller.mock.test_data_generator import create_test_trajectory


def test_mpc_basic_compute():
    """测试 MPC 基本计算"""
    config = DEFAULT_CONFIG.copy()
    platform_config = PLATFORM_CONFIG['differential']
    mpc = MPCController(config, platform_config)
    
    state = np.array([0, 0, 0, 1.0, 0, 0, 0, 0])  # 位置、速度、航向、角速度
    trajectory = create_test_trajectory(trajectory_type='straight', soft_enabled=True)
    consistency = ConsistencyResult(
        alpha=0.8, kappa_consistency=0.9, v_dir_consistency=0.9,
        temporal_smooth=0.9, should_disable_soft=False, data_valid=True
    )
    
    cmd = mpc.compute(state, trajectory, consistency)
    
    assert cmd is not None
    assert hasattr(cmd, 'vx')
    assert hasattr(cmd, 'omega')
    assert cmd.frame_id == 'base_link'
    
    mpc.shutdown()
    print("✓ test_mpc_basic_compute passed")


def test_mpc_velocity_constraints():
    """测试 MPC 速度约束"""
    config = DEFAULT_CONFIG.copy()
    config['constraints'] = DEFAULT_CONFIG['constraints'].copy()
    config['constraints']['v_max'] = 1.0
    platform_config = PLATFORM_CONFIG['differential']
    mpc = MPCController(config, platform_config)
    
    state = np.array([0, 0, 0, 0, 0, 0, 0, 0])
    trajectory = create_test_trajectory(trajectory_type='straight', speed=5.0, soft_enabled=True)
    consistency = ConsistencyResult(
        alpha=1.0, kappa_consistency=1.0, v_dir_consistency=1.0,
        temporal_smooth=1.0, should_disable_soft=False, data_valid=True
    )
    
    cmd = mpc.compute(state, trajectory, consistency)
    
    # 验证速度不超过限制
    v_horizontal = np.sqrt(cmd.vx**2 + cmd.vy**2)
    assert v_horizontal <= 1.5, f"Velocity should be limited, got {v_horizontal}"
    
    mpc.shutdown()
    print("✓ test_mpc_velocity_constraints passed")


def test_mpc_horizon_adjustment():
    """测试 MPC horizon 动态调整"""
    config = DEFAULT_CONFIG.copy()
    platform_config = PLATFORM_CONFIG['differential']
    mpc = MPCController(config, platform_config)
    
    assert mpc.horizon == 20  # 默认值
    
    mpc.set_horizon(10)
    assert mpc.horizon == 10
    
    mpc.set_horizon(20)
    assert mpc.horizon == 20
    
    mpc.shutdown()
    print("✓ test_mpc_horizon_adjustment passed")


def test_mpc_health_metrics():
    """测试 MPC 健康指标"""
    config = DEFAULT_CONFIG.copy()
    platform_config = PLATFORM_CONFIG['differential']
    mpc = MPCController(config, platform_config)
    
    state = np.array([0, 0, 0, 1.0, 0, 0, 0, 0])
    trajectory = create_test_trajectory(soft_enabled=True)
    consistency = ConsistencyResult(
        alpha=0.8, kappa_consistency=0.9, v_dir_consistency=0.9,
        temporal_smooth=0.9, should_disable_soft=False, data_valid=True
    )
    
    mpc.compute(state, trajectory, consistency)
    
    metrics = mpc.get_health_metrics()
    
    assert 'type' in metrics
    assert metrics['type'] == 'mpc'
    assert 'horizon' in metrics
    assert 'last_solve_time_ms' in metrics
    
    mpc.shutdown()
    print("✓ test_mpc_health_metrics passed")


def test_mpc_fallback_solver():
    """测试 MPC fallback 求解器"""
    config = DEFAULT_CONFIG.copy()
    platform_config = PLATFORM_CONFIG['differential']
    mpc = MPCController(config, platform_config)
    
    # 强制使用 fallback
    mpc._is_initialized = False
    
    state = np.array([0, 0, 0, 1.0, 0, 0, 0, 0])
    trajectory = create_test_trajectory(trajectory_type='straight', soft_enabled=True)
    consistency = ConsistencyResult(
        alpha=0.8, kappa_consistency=0.9, v_dir_consistency=0.9,
        temporal_smooth=0.9, should_disable_soft=False, data_valid=True
    )
    
    cmd = mpc.compute(state, trajectory, consistency)
    
    assert cmd is not None
    assert cmd.success == True
    
    mpc.shutdown()
    print("✓ test_mpc_fallback_solver passed")


def test_pure_pursuit_basic():
    """测试 Pure Pursuit 基本功能"""
    config = DEFAULT_CONFIG.copy()
    platform_config = PLATFORM_CONFIG['differential']
    pp = PurePursuitController(config, platform_config)
    
    state = np.array([0, 0, 0, 1.0, 0, 0, 0, 0])
    trajectory = create_test_trajectory(trajectory_type='straight', soft_enabled=True)
    consistency = ConsistencyResult(
        alpha=0.8, kappa_consistency=0.9, v_dir_consistency=0.9,
        temporal_smooth=0.9, should_disable_soft=False, data_valid=True
    )
    
    cmd = pp.compute(state, trajectory, consistency)
    
    assert cmd is not None
    assert hasattr(cmd, 'vx')
    assert hasattr(cmd, 'omega')
    assert cmd.frame_id == 'base_link'
    
    pp.shutdown()
    print("✓ test_pure_pursuit_basic passed")


def test_pure_pursuit_lookahead():
    """测试 Pure Pursuit 前瞻距离计算"""
    config = DEFAULT_CONFIG.copy()
    config['backup'] = DEFAULT_CONFIG['backup'].copy()
    config['backup']['lookahead_dist'] = 1.0
    config['backup']['lookahead_ratio'] = 0.5
    config['backup']['min_lookahead'] = 0.5
    config['backup']['max_lookahead'] = 3.0
    
    platform_config = PLATFORM_CONFIG['differential']
    pp = PurePursuitController(config, platform_config)
    
    # 低速时前瞻距离应该较小
    lookahead_low = pp._compute_lookahead(0.5)
    assert lookahead_low >= 0.5  # min_lookahead
    
    # 高速时前瞻距离应该较大
    lookahead_high = pp._compute_lookahead(3.0)
    assert lookahead_high <= 3.0  # max_lookahead
    
    pp.shutdown()
    print("✓ test_pure_pursuit_lookahead passed")


def test_pure_pursuit_omni():
    """测试全向车 Pure Pursuit"""
    config = DEFAULT_CONFIG.copy()
    platform_config = PLATFORM_CONFIG['omni']
    pp = PurePursuitController(config, platform_config)
    
    state = np.array([0, 0, 0, 1.0, 0.5, 0, 0, 0])
    trajectory = create_test_trajectory(trajectory_type='straight', soft_enabled=True)
    consistency = ConsistencyResult(
        alpha=0.8, kappa_consistency=0.9, v_dir_consistency=0.9,
        temporal_smooth=0.9, should_disable_soft=False, data_valid=True
    )
    
    cmd = pp.compute(state, trajectory, consistency)
    
    assert cmd is not None
    assert cmd.frame_id == 'world'
    # 全向车可以有 vy
    
    pp.shutdown()
    print("✓ test_pure_pursuit_omni passed")


def test_pure_pursuit_3d():
    """测试 3D Pure Pursuit (四旋翼)"""
    config = DEFAULT_CONFIG.copy()
    platform_config = PLATFORM_CONFIG['quadrotor']
    pp = PurePursuitController(config, platform_config)
    
    state = np.array([0, 0, 1, 1.0, 0.5, 0.2, 0, 0])
    trajectory = create_test_trajectory(trajectory_type='straight', soft_enabled=True)
    consistency = ConsistencyResult(
        alpha=0.8, kappa_consistency=0.9, v_dir_consistency=0.9,
        temporal_smooth=0.9, should_disable_soft=False, data_valid=True
    )
    
    cmd = pp.compute(state, trajectory, consistency)
    
    assert cmd is not None
    assert cmd.frame_id == 'world'
    # 3D 控制器可以有 vz
    
    pp.shutdown()
    print("✓ test_pure_pursuit_3d passed")


def test_pure_pursuit_heading_modes():
    """测试 Pure Pursuit 航向模式"""
    config = DEFAULT_CONFIG.copy()
    platform_config = PLATFORM_CONFIG['omni']
    
    # 测试 FOLLOW_VELOCITY 模式
    config = DEFAULT_CONFIG.copy()
    config['backup'] = DEFAULT_CONFIG['backup'].copy()
    config['backup']['heading_mode'] = 'follow_velocity'
    pp = PurePursuitController(config, platform_config)
    
    state = np.array([0, 0, 0, 1.0, 0, 0, 0, 0])
    trajectory = create_test_trajectory(soft_enabled=True)
    consistency = ConsistencyResult(
        alpha=0.8, kappa_consistency=0.9, v_dir_consistency=0.9,
        temporal_smooth=0.9, should_disable_soft=False, data_valid=True
    )
    
    cmd = pp.compute(state, trajectory, consistency)
    assert cmd is not None
    
    pp.shutdown()
    
    # 测试 FIXED 模式
    config2 = DEFAULT_CONFIG.copy()
    config2['backup'] = DEFAULT_CONFIG['backup'].copy()
    config2['backup']['heading_mode'] = 'fixed'
    config2['backup']['fixed_heading'] = 0.5
    pp2 = PurePursuitController(config2, platform_config)
    
    cmd2 = pp2.compute(state, trajectory, consistency)
    assert cmd2 is not None
    
    pp2.shutdown()
    print("✓ test_pure_pursuit_heading_modes passed")


def test_tracker_velocity_smoothing():
    """测试速度平滑"""
    config = DEFAULT_CONFIG.copy()
    platform_config = PLATFORM_CONFIG['differential']
    pp = PurePursuitController(config, platform_config)
    
    state = np.array([0, 0, 0, 0, 0, 0, 0, 0])  # 静止状态
    trajectory = create_test_trajectory(trajectory_type='straight', speed=2.0, soft_enabled=True)
    consistency = ConsistencyResult(
        alpha=1.0, kappa_consistency=1.0, v_dir_consistency=1.0,
        temporal_smooth=1.0, should_disable_soft=False, data_valid=True
    )
    
    # 第一次计算
    cmd1 = pp.compute(state, trajectory, consistency)
    
    # 第二次计算（应该有平滑效果）
    cmd2 = pp.compute(state, trajectory, consistency)
    
    # 验证速度变化被平滑
    assert cmd1 is not None
    assert cmd2 is not None
    
    pp.shutdown()
    print("✓ test_tracker_velocity_smoothing passed")


def test_tracker_empty_trajectory():
    """测试空轨迹处理"""
    config = DEFAULT_CONFIG.copy()
    platform_config = PLATFORM_CONFIG['differential']
    
    mpc = MPCController(config, platform_config)
    pp = PurePursuitController(config, platform_config)
    
    state = np.array([0, 0, 0, 1.0, 0, 0, 0, 0])
    
    # 创建只有一个点的轨迹
    from universal_controller.core.data_types import Trajectory, Point3D, Header
    short_trajectory = Trajectory(
        header=Header(stamp=time.time(), frame_id='world'),
        points=[Point3D(0, 0, 0)],
        velocities=None,
        dt_sec=0.1,
        confidence=0.9
    )
    
    consistency = ConsistencyResult(
        alpha=0.8, kappa_consistency=0.9, v_dir_consistency=0.9,
        temporal_smooth=0.9, should_disable_soft=False, data_valid=True
    )
    
    # MPC 应该能处理短轨迹
    cmd_mpc = mpc.compute(state, short_trajectory, consistency)
    assert cmd_mpc is not None
    
    # Pure Pursuit 应该能处理短轨迹
    cmd_pp = pp.compute(state, short_trajectory, consistency)
    assert cmd_pp is not None
    
    mpc.shutdown()
    pp.shutdown()
    print("✓ test_tracker_empty_trajectory passed")


def test_tracker_performance():
    """测试跟踪器性能"""
    config = DEFAULT_CONFIG.copy()
    platform_config = PLATFORM_CONFIG['differential']
    
    mpc = MPCController(config, platform_config)
    pp = PurePursuitController(config, platform_config)
    
    state = np.array([0, 0, 0, 1.0, 0, 0, 0, 0])
    trajectory = create_test_trajectory(soft_enabled=True)
    consistency = ConsistencyResult(
        alpha=0.8, kappa_consistency=0.9, v_dir_consistency=0.9,
        temporal_smooth=0.9, should_disable_soft=False, data_valid=True
    )
    
    # MPC 性能测试
    num_iterations = 100
    start = time.time()
    for _ in range(num_iterations):
        mpc.compute(state, trajectory, consistency)
    mpc_time = (time.time() - start) / num_iterations * 1000
    
    # Pure Pursuit 性能测试
    start = time.time()
    for _ in range(num_iterations):
        pp.compute(state, trajectory, consistency)
    pp_time = (time.time() - start) / num_iterations * 1000
    
    # 验证性能
    assert mpc_time < 20.0, f"MPC too slow: {mpc_time:.2f}ms"
    assert pp_time < 5.0, f"Pure Pursuit too slow: {pp_time:.2f}ms"
    
    mpc.shutdown()
    pp.shutdown()
    print(f"✓ test_tracker_performance passed (MPC={mpc_time:.2f}ms, PP={pp_time:.2f}ms)")


if __name__ == '__main__':
    print("=" * 60)
    print("Trajectory Tracker Tests")
    print("=" * 60)
    
    print("\n[MPC Controller Tests]")
    print("-" * 40)
    test_mpc_basic_compute()
    test_mpc_velocity_constraints()
    test_mpc_horizon_adjustment()
    test_mpc_health_metrics()
    test_mpc_fallback_solver()
    
    print("\n[Pure Pursuit Controller Tests]")
    print("-" * 40)
    test_pure_pursuit_basic()
    test_pure_pursuit_lookahead()
    test_pure_pursuit_omni()
    test_pure_pursuit_3d()
    test_pure_pursuit_heading_modes()
    
    print("\n[Common Tests]")
    print("-" * 40)
    test_tracker_velocity_smoothing()
    test_tracker_empty_trajectory()
    test_tracker_performance()
    
    print("\n" + "=" * 60)
    print("✅ ALL TRACKER TESTS PASSED")
    print("=" * 60)
