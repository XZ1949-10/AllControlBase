"""
TF2InjectionManager 单元测试
"""
import pytest
import sys
import os
import time

# 添加 src 目录到路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from controller_ros.utils.tf2_injection_manager import TF2InjectionManager


class MockTFBridge:
    """模拟 TF Bridge"""
    
    def __init__(self, initialized: bool = True, can_transform_result: bool = True):
        self.is_initialized = initialized
        self._can_transform_result = can_transform_result
        self._lookup_transform_calls = []
        self._can_transform_calls = []
    
    def can_transform(self, target_frame: str, source_frame: str, timeout_sec: float = 0.01) -> bool:
        self._can_transform_calls.append((target_frame, source_frame, timeout_sec))
        return self._can_transform_result
    
    def lookup_transform(self, target_frame: str, source_frame: str, 
                        time=None, timeout_sec: float = 0.01) -> dict:
        self._lookup_transform_calls.append((target_frame, source_frame, time, timeout_sec))
        return {
            'translation': (0.0, 0.0, 0.0),
            'rotation': (0.0, 0.0, 0.0, 1.0)
        }


class MockCoordTransformer:
    """模拟坐标变换器"""
    
    def __init__(self):
        self._tf2_callback = None
    
    def set_tf2_lookup_callback(self, callback):
        self._tf2_callback = callback


class MockControllerManager:
    """模拟 ControllerManager"""
    
    def __init__(self, has_coord_transformer: bool = True):
        if has_coord_transformer:
            self.coord_transformer = MockCoordTransformer()
        else:
            self.coord_transformer = None


class TestTF2InjectionManager:
    """测试 TF2InjectionManager"""
    
    def test_initialization(self):
        """测试初始化"""
        tf_bridge = MockTFBridge()
        manager = MockControllerManager()
        
        injection_manager = TF2InjectionManager(
            tf_bridge=tf_bridge,
            controller_manager=manager,
        )
        
        assert injection_manager.is_injected == False
        assert injection_manager.injection_attempted == False
        assert injection_manager.retry_count == 0
    
    def test_inject_success(self):
        """测试成功注入"""
        tf_bridge = MockTFBridge(initialized=True, can_transform_result=True)
        manager = MockControllerManager()
        
        log_messages = []
        
        injection_manager = TF2InjectionManager(
            tf_bridge=tf_bridge,
            controller_manager=manager,
            log_info=lambda msg: log_messages.append(('info', msg)),
            log_warn=lambda msg: log_messages.append(('warn', msg)),
        )
        
        result = injection_manager.inject(blocking=False)
        
        assert result == True
        assert injection_manager.is_injected == True
        assert injection_manager.injection_attempted == True
        assert manager.coord_transformer._tf2_callback is not None
    
    def test_inject_tf_bridge_none(self):
        """测试 TF bridge 为 None"""
        manager = MockControllerManager()
        
        injection_manager = TF2InjectionManager(
            tf_bridge=None,
            controller_manager=manager,
        )
        
        result = injection_manager.inject(blocking=False)
        
        assert result == False
        assert injection_manager.is_injected == False
    
    def test_inject_tf_not_initialized(self):
        """测试 TF2 未初始化"""
        tf_bridge = MockTFBridge(initialized=False)
        manager = MockControllerManager()
        
        injection_manager = TF2InjectionManager(
            tf_bridge=tf_bridge,
            controller_manager=manager,
        )
        
        result = injection_manager.inject(blocking=False)
        
        assert result == False
        assert injection_manager.is_injected == False
    
    def test_inject_no_coord_transformer(self):
        """测试没有坐标变换器"""
        tf_bridge = MockTFBridge()
        manager = MockControllerManager(has_coord_transformer=False)
        
        injection_manager = TF2InjectionManager(
            tf_bridge=tf_bridge,
            controller_manager=manager,
        )
        
        result = injection_manager.inject(blocking=False)
        
        assert result == False
        assert injection_manager.is_injected == False
    
    def test_inject_controller_manager_none(self):
        """测试 ControllerManager 为 None"""
        tf_bridge = MockTFBridge()
        
        injection_manager = TF2InjectionManager(
            tf_bridge=tf_bridge,
            controller_manager=None,
        )
        
        result = injection_manager.inject(blocking=False)
        
        assert result == False
        assert injection_manager.is_injected == False
    
    def test_try_reinjection_already_injected(self):
        """测试已注入时不重试"""
        tf_bridge = MockTFBridge()
        manager = MockControllerManager()
        
        injection_manager = TF2InjectionManager(
            tf_bridge=tf_bridge,
            controller_manager=manager,
        )
        
        # 先注入
        injection_manager.inject(blocking=False)
        assert injection_manager.is_injected == True
        
        # 尝试重新注入，应该返回 False（不需要重试）
        result = injection_manager.try_reinjection_if_needed()
        assert result == False
    
    def test_try_reinjection_not_attempted(self):
        """测试未尝试过注入时不重试"""
        tf_bridge = MockTFBridge()
        manager = MockControllerManager()
        
        injection_manager = TF2InjectionManager(
            tf_bridge=tf_bridge,
            controller_manager=manager,
        )
        
        # 未尝试过注入
        assert injection_manager.injection_attempted == False
        
        # 尝试重新注入，应该返回 False
        result = injection_manager.try_reinjection_if_needed()
        assert result == False
    
    def test_try_reinjection_interval_time_based(self):
        """测试基于时间的重试间隔（含指数退避）"""
        tf_bridge = MockTFBridge(initialized=True, can_transform_result=False)
        manager = MockControllerManager(has_coord_transformer=False)  # 让注入失败
        
        # 使用可控的时间函数
        mock_time = [0.0]
        def get_mock_time():
            return mock_time[0]
        
        injection_manager = TF2InjectionManager(
            tf_bridge=tf_bridge,
            controller_manager=manager,
            config={
                'retry_interval_sec': 1.0,  # 初始 1 秒重试间隔
                'backoff_multiplier': 2.0,  # 指数退避倍数
                'max_retry_interval_sec': 10.0,  # 最大间隔
            },
            get_time_func=get_mock_time,
        )
        
        # 手动设置状态（模拟初始注入失败后的状态）
        injection_manager._injection_attempted_event.set()
        injection_manager._injected_event.clear()
        
        # 第一次调用应该触发重试（因为 _last_retry_time 为 None）
        # 重试后间隔变为 2.0s
        result = injection_manager.try_reinjection_if_needed()
        assert result == True
        assert injection_manager.retry_count == 1
        
        # 时间未过（0.5s < 2.0s 当前间隔），不应该重试
        mock_time[0] = 0.5
        result = injection_manager.try_reinjection_if_needed()
        assert result == False
        assert injection_manager.retry_count == 1
        
        # 时间已过当前间隔 2.0s，应该重试（间隔变为 4.0s）
        mock_time[0] = 2.5  # 距上次 2.5s > 2.0s
        result = injection_manager.try_reinjection_if_needed()
        assert result == True
        assert injection_manager.retry_count == 2
        
        # 验证指数退避：3.0s 后（< 4.0s 新间隔），不应该重试
        mock_time[0] = 5.0  # 距上次 2.5s < 4.0s
        result = injection_manager.try_reinjection_if_needed()
        assert result == False
        assert injection_manager.retry_count == 2
        
        # 4.0s 后应该重试（间隔变为 8.0s）
        mock_time[0] = 7.0  # 距上次 4.5s > 4.0s
        result = injection_manager.try_reinjection_if_needed()
        assert result == True
        assert injection_manager.retry_count == 3
    
    def test_try_reinjection_max_retries(self):
        """测试最大重试次数"""
        tf_bridge = MockTFBridge(initialized=True)
        manager = MockControllerManager(has_coord_transformer=False)  # 让注入失败
        
        # 使用可控的时间函数
        mock_time = [0.0]
        def get_mock_time():
            return mock_time[0]
        
        injection_manager = TF2InjectionManager(
            tf_bridge=tf_bridge,
            controller_manager=manager,
            config={
                'retry_interval_sec': 0.1,  # 短间隔
                'max_retries': 3,
                'backoff_multiplier': 1.0,  # 禁用指数退避，保持固定间隔
            },
            get_time_func=get_mock_time,
        )
        
        # 手动设置状态（使用 Event）
        injection_manager._injection_attempted_event.set()
        injection_manager._injected_event.clear()
        
        # 前 3 次应该重试
        for i in range(3):
            mock_time[0] = i * 0.2  # 每次增加 0.2 秒
            result = injection_manager.try_reinjection_if_needed()
            assert result == True, f"Should retry at iteration {i}"
            assert injection_manager.is_injected == False
        
        # 第 4 次不应该重试（超过最大次数）
        mock_time[0] = 1.0
        result = injection_manager.try_reinjection_if_needed()
        assert result == False
    
    def test_try_reinjection_unlimited(self):
        """测试无限重试"""
        tf_bridge = MockTFBridge(initialized=True)
        manager = MockControllerManager(has_coord_transformer=False)  # 让注入失败
        
        # 使用可控的时间函数
        mock_time = [0.0]
        def get_mock_time():
            return mock_time[0]
        
        injection_manager = TF2InjectionManager(
            tf_bridge=tf_bridge,
            controller_manager=manager,
            config={
                'retry_interval_sec': 0.1,
                'max_retries': -1,  # 无限重试
                'backoff_multiplier': 1.0,  # 禁用指数退避，保持固定间隔
            },
            get_time_func=get_mock_time,
        )
        
        # 手动设置状态（使用 Event）
        injection_manager._injection_attempted_event.set()
        injection_manager._injected_event.clear()
        
        # 应该一直重试（测试 10 次）
        for i in range(10):
            mock_time[0] = i * 0.2  # 每次增加 0.2 秒
            result = injection_manager.try_reinjection_if_needed()
            assert result == True, f"Should retry at iteration {i}"
            assert injection_manager.is_injected == False
    
    def test_reset(self):
        """测试重置"""
        tf_bridge = MockTFBridge()
        manager = MockControllerManager()
        
        # 使用可控的时间函数
        mock_time = [0.0]
        def get_mock_time():
            return mock_time[0]
        
        injection_manager = TF2InjectionManager(
            tf_bridge=tf_bridge,
            controller_manager=manager,
            get_time_func=get_mock_time,
        )
        
        # 模拟一些状态（使用 Event）
        injection_manager._injection_attempted_event.set()
        with injection_manager._lock:
            injection_manager._last_retry_time = 10.0
        
        # 重置
        injection_manager.reset()
        
        # 重试时间应该重置
        with injection_manager._lock:
            assert injection_manager._last_retry_time == None
        # 注入状态不应该重置
        assert injection_manager.injection_attempted == True
    
    def test_get_status(self):
        """测试获取状态"""
        tf_bridge = MockTFBridge()
        manager = MockControllerManager()
        
        injection_manager = TF2InjectionManager(
            tf_bridge=tf_bridge,
            controller_manager=manager,
            config={
                'retry_interval_sec': 2.0,
            },
            transform_config={
                'source_frame': 'base_link',
                'target_frame': 'odom',
            },
        )
        
        injection_manager.inject(blocking=False)
        
        status = injection_manager.get_status()
        
        assert status['injected'] == True
        assert status['injection_attempted'] == True
        assert status['source_frame'] == 'base_link'
        assert status['target_frame'] == 'odom'
        assert status['current_retry_interval_sec'] == 2.0  # 成功注入后重置为初始值
    
    def test_custom_config(self):
        """测试自定义配置"""
        tf_bridge = MockTFBridge()
        manager = MockControllerManager()
        
        injection_manager = TF2InjectionManager(
            tf_bridge=tf_bridge,
            controller_manager=manager,
            config={
                'buffer_warmup_timeout_sec': 5.0,
                'buffer_warmup_interval_sec': 0.2,
                'retry_interval_sec': 2.0,
                'max_retry_interval_sec': 60.0,
                'backoff_multiplier': 1.5,
                'max_retries': 10,
            },
            transform_config={
                'source_frame': 'custom_base',
                'target_frame': 'custom_odom',
            },
        )
        
        assert injection_manager._source_frame == 'custom_base'
        assert injection_manager._target_frame == 'custom_odom'
        assert injection_manager._buffer_warmup_timeout_sec == 5.0
        assert injection_manager._buffer_warmup_interval_sec == 0.2
        assert injection_manager._initial_retry_interval == 2.0
        assert injection_manager._max_retry_interval == 60.0
        assert injection_manager._backoff_multiplier == 1.5
        assert injection_manager._max_retries == 10


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
