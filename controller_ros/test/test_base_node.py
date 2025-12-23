"""
ControllerNodeBase 单元测试
"""
import pytest
import sys
import os
import time

# 添加 src 目录到路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from typing import Dict, Any, Optional
from universal_controller.core.data_types import ControlOutput
from universal_controller.config.default_config import DEFAULT_CONFIG
from universal_controller.mock import create_test_odom, create_test_trajectory


class MockControllerNode:
    """
    模拟控制器节点，用于测试 ControllerNodeBase
    
    继承 ControllerNodeBase 并实现所有抽象方法。
    """
    
    def __init__(self):
        # 导入基类
        from controller_ros.node.base_node import ControllerNodeBase
        
        # 创建一个简单的子类
        class TestNode(ControllerNodeBase):
            def __init__(self_inner):
                super().__init__()
                self_inner._current_time = 0.0
                self_inner._logs = {'info': [], 'warn': [], 'error': []}
                self_inner._published_cmds = []
                self_inner._published_diags = []
                self_inner._stop_cmd_count = 0
            
            def _get_time(self_inner) -> float:
                return self_inner._current_time
            
            def _log_info(self_inner, msg: str):
                self_inner._logs['info'].append(msg)
            
            def _log_warn(self_inner, msg: str):
                self_inner._logs['warn'].append(msg)
            
            def _log_warn_throttle(self_inner, period: float, msg: str):
                self_inner._logs['warn'].append(msg)
            
            def _log_error(self_inner, msg: str):
                self_inner._logs['error'].append(msg)
            
            def _publish_cmd(self_inner, cmd: ControlOutput):
                self_inner._published_cmds.append(cmd)
            
            def _publish_stop_cmd(self_inner):
                self_inner._stop_cmd_count += 1
            
            def _publish_diagnostics(self_inner, diag: Dict[str, Any], force: bool = False):
                self_inner._published_diags.append((diag, force))
        
        self.node = TestNode()
    
    def initialize(self, params: Dict[str, Any] = None):
        """初始化节点"""
        self.node._params = params or DEFAULT_CONFIG.copy()
        self.node._topics = {
            'odom': '/odom',
            'imu': '/imu',
            'trajectory': '/nn/local_trajectory',
        }
        self.node._initialize()
    
    def set_time(self, t: float):
        """设置当前时间"""
        self.node._current_time = t
    
    def update_odom(self, odom):
        """更新里程计数据"""
        # 直接设置 UC 数据
        self.node._data_manager._latest_data['odom'] = odom
        self.node._data_manager._timestamps['odom'] = self.node._current_time
    
    def update_trajectory(self, traj):
        """更新轨迹数据"""
        self.node._data_manager._latest_data['trajectory'] = traj
        self.node._data_manager._timestamps['trajectory'] = self.node._current_time


def test_base_node_initialization():
    """测试基类初始化"""
    mock = MockControllerNode()
    mock.initialize()
    
    assert mock.node._controller_bridge is not None
    assert mock.node._data_manager is not None
    assert mock.node._time_sync is not None
    assert mock.node._platform_type == 'differential'


def test_base_node_control_loop_no_data():
    """测试控制循环 - 无数据"""
    mock = MockControllerNode()
    mock.initialize()
    
    # 无数据时应返回 None
    result = mock.node._control_loop_core()
    
    assert result is None
    assert mock.node._waiting_for_data == True


def test_base_node_control_loop_with_data():
    """测试控制循环 - 有数据"""
    mock = MockControllerNode()
    mock.initialize()
    
    # 添加测试数据
    odom = create_test_odom(vx=1.0)
    traj = create_test_trajectory()
    
    mock.update_odom(odom)
    mock.update_trajectory(traj)
    
    # 执行控制循环
    result = mock.node._control_loop_core()
    
    assert result is not None
    assert isinstance(result, ControlOutput)
    assert mock.node._waiting_for_data == False


def test_base_node_handle_reset():
    """测试重置处理"""
    mock = MockControllerNode()
    mock.initialize()
    
    # 添加数据
    odom = create_test_odom(vx=1.0)
    traj = create_test_trajectory()
    mock.update_odom(odom)
    mock.update_trajectory(traj)
    
    # 执行一次控制
    mock.node._control_loop_core()
    assert mock.node._waiting_for_data == False
    
    # 重置
    mock.node._handle_reset()
    
    assert mock.node._waiting_for_data == True
    assert mock.node._consecutive_errors == 0
    assert mock.node._data_manager.get_latest_odom() is None


def test_base_node_handle_get_diagnostics():
    """测试获取诊断"""
    mock = MockControllerNode()
    mock.initialize()
    
    # 添加数据并执行控制
    odom = create_test_odom(vx=1.0)
    traj = create_test_trajectory()
    mock.update_odom(odom)
    mock.update_trajectory(traj)
    mock.node._control_loop_core()
    
    # 获取诊断
    diag = mock.node._handle_get_diagnostics()
    
    assert diag is not None


def test_base_node_handle_set_state_stopping():
    """测试设置状态 - STOPPING"""
    from universal_controller.core.enums import ControllerState
    
    mock = MockControllerNode()
    mock.initialize()
    
    # 添加数据并执行几次控制，让状态机进入 NORMAL
    odom = create_test_odom(vx=1.0)
    traj = create_test_trajectory()
    mock.update_odom(odom)
    mock.update_trajectory(traj)
    
    for _ in range(5):
        mock.node._control_loop_core()
    
    # 请求 STOPPING 状态
    success = mock.node._handle_set_state(ControllerState.STOPPING.value)
    
    assert success == True


def test_base_node_handle_set_state_invalid():
    """测试设置状态 - 无效状态"""
    mock = MockControllerNode()
    mock.initialize()
    
    # 请求无效状态
    success = mock.node._handle_set_state(999)
    
    assert success == False
    assert len(mock.node._logs['warn']) > 0


def test_base_node_error_handling():
    """测试错误处理"""
    mock = MockControllerNode()
    mock.initialize()
    
    # 模拟错误 - 需要先增加 consecutive_errors（模拟 _control_loop_core 的行为）
    timeouts = {'odom_timeout': True, 'traj_timeout': False, 'imu_timeout': False}
    error = Exception("Test error")
    
    # 模拟 _control_loop_core 中的行为：先增加计数，再调用 _handle_control_error
    mock.node._consecutive_errors += 1
    mock.node._handle_control_error(error, timeouts)
    
    assert mock.node._consecutive_errors == 1
    assert mock.node._stop_cmd_count == 1
    assert len(mock.node._published_diags) == 1
    
    diag, force = mock.node._published_diags[0]
    assert force == True
    assert diag['mpc_success'] == False


def test_base_node_error_throttling():
    """测试错误日志节流"""
    mock = MockControllerNode()
    mock.initialize()
    
    timeouts = {'odom_timeout': False, 'traj_timeout': False, 'imu_timeout': False}
    error = Exception("Test error")
    
    # 连续触发多次错误（模拟 _control_loop_core 的行为）
    for i in range(15):
        mock.node._consecutive_errors += 1
        mock.node._handle_control_error(error, timeouts)
    
    # 检查错误计数
    assert mock.node._consecutive_errors == 15
    
    # 错误日志数量应该是 10（前 10 次）
    assert len(mock.node._logs['error']) == 10


def test_base_node_create_error_diagnostics():
    """测试创建错误诊断"""
    mock = MockControllerNode()
    mock.initialize()
    
    # 添加一些数据
    odom = create_test_odom(vx=1.0)
    mock.update_odom(odom)
    mock.set_time(0.1)  # 数据年龄 0.1 秒
    
    timeouts = {'odom_timeout': True, 'traj_timeout': True, 'imu_timeout': False}
    error = Exception("Test error")
    mock.node._consecutive_errors = 5
    
    diag = mock.node._create_error_diagnostics(error, timeouts)
    
    assert diag['state'] == 0
    assert diag['mpc_success'] == False
    assert diag['error_message'] == "Test error"
    assert diag['consecutive_errors'] == 5
    assert diag['timeout']['odom_timeout'] == True
    assert diag['timeout']['traj_timeout'] == True
    assert diag['timeout']['imu_timeout'] == False
    assert abs(diag['timeout']['last_odom_age_ms'] - 100.0) < 1.0


def test_base_node_on_diagnostics():
    """测试诊断回调"""
    mock = MockControllerNode()
    mock.initialize()
    
    # 模拟诊断数据
    diag = {'state': 1, 'mpc_success': True}
    
    mock.node._on_diagnostics(diag)
    
    # 应该添加 transform 信息
    assert 'transform' in diag
    assert 'tf2_available' in diag['transform']
    
    # 应该发布诊断
    assert len(mock.node._published_diags) == 1


def test_base_node_shutdown():
    """测试关闭"""
    mock = MockControllerNode()
    mock.initialize()
    
    mock.node.shutdown()
    
    # 应该记录关闭日志
    assert any('shutdown' in msg.lower() for msg in mock.node._logs['info'])


def test_base_node_platform_config():
    """测试不同平台配置"""
    platforms = ['differential', 'omni', 'ackermann', 'quadrotor']
    
    for platform in platforms:
        mock = MockControllerNode()
        params = DEFAULT_CONFIG.copy()
        params['system'] = DEFAULT_CONFIG['system'].copy()
        params['system']['platform'] = platform
        
        mock.initialize(params)
        
        assert mock.node._platform_type == platform
        print(f"✓ Platform {platform} initialized correctly")


if __name__ == '__main__':
    pytest.main([__file__, '-v'])


# ==================== TF2 注入优化测试 ====================

class MockTFBridge:
    """模拟 TF Bridge"""
    
    def __init__(self, initialized: bool = True, can_transform_result: bool = True):
        self.is_initialized = initialized
        self._can_transform_result = can_transform_result
        self._lookup_callback_set = False
        self._can_transform_call_count = 0
    
    def can_transform(self, target_frame: str, source_frame: str, timeout_sec: float = 0.01) -> bool:
        self._can_transform_call_count += 1
        return self._can_transform_result
    
    def lookup_transform(self, target_frame: str, source_frame: str, 
                        time=None, timeout_sec: float = 0.01):
        return {
            'translation': (0.0, 0.0, 0.0),
            'rotation': (0.0, 0.0, 0.0, 1.0)
        }


def test_tf2_injection_blocking_success():
    """测试 TF2 注入 - 阻塞模式成功"""
    mock = MockControllerNode()
    mock.initialize()
    
    # 设置 TF bridge
    tf_bridge = MockTFBridge(initialized=True, can_transform_result=True)
    mock.node._tf_bridge = tf_bridge
    
    # 注入 TF2
    mock.node._inject_tf2_to_controller(blocking=True)
    
    assert mock.node._tf2_injected == True
    assert mock.node._tf2_injection_attempted == True
    # 应该只调用一次 can_transform（因为第一次就成功了）
    assert tf_bridge._can_transform_call_count == 1


def test_tf2_injection_blocking_timeout():
    """测试 TF2 注入 - 阻塞模式超时"""
    mock = MockControllerNode()
    # 使用较短的超时时间进行测试
    params = DEFAULT_CONFIG.copy()
    params['tf'] = {
        'source_frame': 'base_link',
        'target_frame': 'odom',
        'buffer_warmup_timeout_sec': 0.2,  # 200ms 超时
        'buffer_warmup_interval_sec': 0.05,  # 50ms 间隔
    }
    mock.initialize(params)
    
    # 设置 TF bridge - can_transform 始终返回 False
    tf_bridge = MockTFBridge(initialized=True, can_transform_result=False)
    mock.node._tf_bridge = tf_bridge
    
    # 注入 TF2
    start_time = time.time()
    mock.node._inject_tf2_to_controller(blocking=True)
    elapsed = time.time() - start_time
    
    # 应该等待约 0.2 秒
    assert elapsed >= 0.15  # 允许一些误差
    assert elapsed < 0.5  # 不应该等太久
    
    # 即使超时，也应该注入回调（让 RobustCoordinateTransformer 处理 fallback）
    assert mock.node._tf2_injected == True
    assert mock.node._tf2_injection_attempted == True
    
    # 应该有警告日志
    assert any('not ready' in msg.lower() for msg in mock.node._logs['warn'])


def test_tf2_injection_non_blocking():
    """测试 TF2 注入 - 非阻塞模式"""
    mock = MockControllerNode()
    mock.initialize()
    
    # 设置 TF bridge - can_transform 返回 False
    tf_bridge = MockTFBridge(initialized=True, can_transform_result=False)
    mock.node._tf_bridge = tf_bridge
    
    # 非阻塞注入
    start_time = time.time()
    mock.node._inject_tf2_to_controller(blocking=False)
    elapsed = time.time() - start_time
    
    # 应该立即返回
    assert elapsed < 0.1
    
    # 应该只调用一次 can_transform
    assert tf_bridge._can_transform_call_count == 1
    
    # 应该注入回调
    assert mock.node._tf2_injected == True


def test_tf2_injection_not_initialized():
    """测试 TF2 注入 - TF2 未初始化"""
    mock = MockControllerNode()
    mock.initialize()
    
    # 设置 TF bridge - 未初始化
    tf_bridge = MockTFBridge(initialized=False)
    mock.node._tf_bridge = tf_bridge
    
    # 注入 TF2
    mock.node._inject_tf2_to_controller(blocking=True)
    
    # 不应该注入
    assert mock.node._tf2_injected == False
    assert mock.node._tf2_injection_attempted == False


def test_tf2_reinjection():
    """测试 TF2 重新注入"""
    mock = MockControllerNode()
    mock.initialize()
    
    # 初始状态：未注入
    assert mock.node._tf2_injected == False
    assert mock.node._tf2_injection_attempted == False
    
    # 设置 TF bridge
    tf_bridge = MockTFBridge(initialized=True, can_transform_result=True)
    mock.node._tf_bridge = tf_bridge
    
    # 标记已尝试但未成功（模拟初始化时 TF2 不可用的情况）
    mock.node._tf2_injection_attempted = True
    mock.node._tf2_injected = False
    
    # 尝试重新注入
    mock.node._try_tf2_reinjection()
    
    # 应该成功注入
    assert mock.node._tf2_injected == True


def test_tf2_reinjection_already_injected():
    """测试 TF2 重新注入 - 已经注入"""
    mock = MockControllerNode()
    mock.initialize()
    
    # 设置 TF bridge
    tf_bridge = MockTFBridge(initialized=True, can_transform_result=True)
    mock.node._tf_bridge = tf_bridge
    
    # 标记已注入
    mock.node._tf2_injected = True
    mock.node._tf2_injection_attempted = True
    
    # 尝试重新注入
    initial_call_count = tf_bridge._can_transform_call_count
    mock.node._try_tf2_reinjection()
    
    # 不应该再次调用 can_transform
    assert tf_bridge._can_transform_call_count == initial_call_count


def test_control_loop_tf2_retry():
    """测试控制循环中的 TF2 重试"""
    mock = MockControllerNode()
    mock.initialize()
    
    # 设置 TF bridge
    tf_bridge = MockTFBridge(initialized=True, can_transform_result=True)
    mock.node._tf_bridge = tf_bridge
    
    # 标记已尝试但未成功
    mock.node._tf2_injection_attempted = True
    mock.node._tf2_injected = False
    
    # 添加数据
    odom = create_test_odom(vx=1.0)
    traj = create_test_trajectory()
    mock.update_odom(odom)
    mock.update_trajectory(traj)
    
    # 执行控制循环 50 次（应该触发一次重试）
    for i in range(50):
        mock.node._control_loop_core()
    
    # 应该已经重新注入
    assert mock.node._tf2_injected == True


def test_error_diagnostics_includes_tf2_injected():
    """测试错误诊断包含 tf2_injected 状态"""
    mock = MockControllerNode()
    mock.initialize()
    
    # 设置 TF bridge
    tf_bridge = MockTFBridge(initialized=True, can_transform_result=True)
    mock.node._tf_bridge = tf_bridge
    mock.node._tf2_injected = True
    
    timeouts = {'odom_timeout': False, 'traj_timeout': False, 'imu_timeout': False}
    error = Exception("Test error")
    
    diag = mock.node._create_error_diagnostics(error, timeouts)
    
    assert 'transform' in diag
    assert diag['transform']['tf2_available'] == True
    assert diag['transform']['tf2_injected'] == True


def test_on_diagnostics_includes_tf2_injected():
    """测试诊断回调包含 tf2_injected 状态"""
    mock = MockControllerNode()
    mock.initialize()
    
    # 设置 TF bridge
    tf_bridge = MockTFBridge(initialized=True, can_transform_result=True)
    mock.node._tf_bridge = tf_bridge
    mock.node._tf2_injected = True
    
    diag = {'state': 1}
    mock.node._on_diagnostics(diag)
    
    assert 'transform' in diag
    assert diag['transform']['tf2_injected'] == True


def test_reset_clears_tf2_retry_counter():
    """测试重置清除 TF2 重试计数器"""
    mock = MockControllerNode()
    mock.initialize()
    
    # 设置重试计数器
    mock.node._tf2_retry_counter = 25
    
    # 重置
    mock.node._handle_reset()
    
    # 计数器应该被重置
    assert mock.node._tf2_retry_counter == 0
