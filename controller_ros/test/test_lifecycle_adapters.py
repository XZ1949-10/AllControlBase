"""
生命周期接口测试

测试统一的 ILifecycleComponent 接口和 HealthChecker。

v3.19 重构后：
- 不再需要适配器，组件直接注册到 HealthChecker
- LifecycleComponentAdapter 已废弃，但保留向后兼容
"""
import pytest
import time
import warnings


class TestUnifiedInterface:
    """测试统一的 ILifecycleComponent 接口"""
    
    def test_simple_component_direct_registration(self):
        """测试简单组件直接注册（无需适配器）"""
        from controller_ros.lifecycle import HealthChecker
        
        class SimpleComponent:
            """只实现 reset() 的简单组件"""
            def __init__(self):
                self.reset_count = 0
            
            def reset(self):
                self.reset_count += 1
        
        checker = HealthChecker()
        component = SimpleComponent()
        
        # 直接注册，无需适配器
        checker.register('simple', component)
        
        # 检查健康状态（使用默认状态）
        status = checker.check('simple')
        assert status is not None
        assert status['healthy'] == True
        assert 'adapter' not in status  # 无适配器标记
    
    def test_component_with_health_status(self):
        """测试实现 get_health_status 的组件"""
        from controller_ros.lifecycle import HealthChecker
        
        class HealthyComponent:
            def reset(self):
                pass
            
            def get_health_status(self):
                return {
                    'healthy': True,
                    'state': 'RUNNING',
                    'message': 'All good',
                    'details': {'metric': 42},
                }
        
        checker = HealthChecker()
        checker.register('healthy', HealthyComponent())
        
        status = checker.check('healthy')
        assert status['healthy'] == True
        assert status['state'] == 'RUNNING'
        assert status['details']['metric'] == 42
    
    def test_component_returning_none_health_status(self):
        """测试 get_health_status 返回 None 的组件"""
        from controller_ros.lifecycle import HealthChecker
        
        class MinimalComponent:
            def reset(self):
                pass
            
            def get_health_status(self):
                return None  # 不支持健康检查
        
        checker = HealthChecker()
        checker.register('minimal', MinimalComponent())
        
        status = checker.check('minimal')
        assert status is not None
        assert status['healthy'] == True  # 默认健康


class TestLifecycleMixin:
    """测试 LifecycleMixin"""
    
    def test_mixin_basic_lifecycle(self):
        """测试 Mixin 基本生命周期"""
        from controller_ros.lifecycle import LifecycleMixin, LifecycleState
        
        class MyComponent(LifecycleMixin):
            def __init__(self):
                super().__init__()
                self.initialized = False
                self.reset_count = 0
            
            def _do_initialize(self) -> bool:
                self.initialized = True
                return True
            
            def _do_shutdown(self) -> None:
                self.initialized = False
            
            def _do_reset(self) -> None:
                self.reset_count += 1
        
        comp = MyComponent()
        assert comp.lifecycle_state == LifecycleState.UNINITIALIZED
        
        # 初始化
        assert comp.initialize() == True
        assert comp.lifecycle_state == LifecycleState.RUNNING
        assert comp.initialized == True
        
        # 重置
        comp.reset()
        assert comp.reset_count == 1
        
        # 健康状态
        status = comp.get_health_status()
        assert status['healthy'] == True
        assert status['state'] == 'RUNNING'
        
        # 关闭
        comp.shutdown()
        assert comp.lifecycle_state == LifecycleState.SHUTDOWN
    
    def test_mixin_with_health_checker(self):
        """测试 Mixin 组件与 HealthChecker 集成"""
        from controller_ros.lifecycle import LifecycleMixin, HealthChecker
        
        class TrackedComponent(LifecycleMixin):
            def _do_initialize(self) -> bool:
                return True
            def _do_shutdown(self) -> None:
                pass
            def _do_reset(self) -> None:
                pass
            def _get_health_details(self):
                return {'custom': 'data'}
        
        comp = TrackedComponent()
        comp.initialize()
        
        checker = HealthChecker()
        checker.register('tracked', comp)
        
        status = checker.check('tracked')
        assert status['healthy'] == True
        assert status['details']['custom'] == 'data'


class TestDeprecatedAdapter:
    """测试废弃的适配器（向后兼容）"""
    
    def test_adapter_deprecation_warning(self):
        """测试适配器发出废弃警告"""
        from controller_ros.lifecycle import LifecycleComponentAdapter
        
        class OldComponent:
            def reset(self): pass
            def shutdown(self): pass
        
        # 使用 pytest.warns 来捕获警告
        with pytest.warns(DeprecationWarning, match="deprecated"):
            adapter = LifecycleComponentAdapter(OldComponent(), 'old')
    
    def test_adapter_still_works(self):
        """测试适配器仍然可用"""
        from controller_ros.lifecycle import LifecycleComponentAdapter, HealthChecker
        
        class LegacyComponent:
            def __init__(self):
                self.reset_called = False
            def reset(self):
                self.reset_called = True
            def shutdown(self):
                pass
        
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", DeprecationWarning)
            
            comp = LegacyComponent()
            adapter = LifecycleComponentAdapter(comp, 'legacy')
            
            checker = HealthChecker()
            checker.register('legacy', adapter)
            
            status = checker.check('legacy')
            assert status['healthy'] == True
            assert status['adapter'] == 'LifecycleComponentAdapter'


class TestWithRealComponents:
    """使用真实组件的集成测试"""
    
    def test_with_adaptive_ekf(self):
        """测试与 AdaptiveEKFEstimator 的集成"""
        from controller_ros.lifecycle import HealthChecker
        from universal_controller.estimator.adaptive_ekf import AdaptiveEKFEstimator
        
        config = {'system': {'platform': 'differential'}}
        ekf = AdaptiveEKFEstimator(config)
        
        checker = HealthChecker()
        checker.register('ekf', ekf)  # 直接注册，无需适配器
        
        status = checker.check('ekf')
        assert status['healthy'] == True
        assert 'adapter' not in status  # 无适配器
        
        # 测试 reset
        ekf.reset()
        status = checker.check('ekf')
        assert status['healthy'] == True
    
    def test_with_controller_bridge(self):
        """测试与 ControllerBridge 的集成"""
        from controller_ros.lifecycle import HealthChecker
        from controller_ros.bridge import ControllerBridge
        
        config = {'system': {'platform': 'differential'}}
        bridge = ControllerBridge(config)
        
        checker = HealthChecker()
        checker.register('bridge', bridge)
        
        status = checker.check('bridge')
        assert status['healthy'] == True
        
        bridge.shutdown()
    
    def test_with_data_manager(self):
        """测试与 DataManager 的集成"""
        from controller_ros.lifecycle import HealthChecker
        from controller_ros.io.data_manager import DataManager
        
        data_manager = DataManager(get_time_func=time.time)
        
        checker = HealthChecker()
        checker.register('data_manager', data_manager)
        
        status = checker.check('data_manager')
        assert status['healthy'] == True
        
        data_manager.shutdown()
    
    def test_mixed_components(self):
        """测试混合注册不同类型的组件"""
        from controller_ros.lifecycle import HealthChecker, LifecycleMixin
        from universal_controller.estimator.adaptive_ekf import AdaptiveEKFEstimator
        
        class CustomComponent(LifecycleMixin):
            def _do_initialize(self) -> bool:
                return True
            def _do_shutdown(self) -> None:
                pass
            def _do_reset(self) -> None:
                pass
        
        checker = HealthChecker()
        
        # 注册 LifecycleMixin 组件
        custom = CustomComponent()
        custom.initialize()
        checker.register('custom', custom)
        
        # 注册 ILifecycleComponent 组件（来自 universal_controller）
        ekf = AdaptiveEKFEstimator({'system': {'platform': 'differential'}})
        checker.register('ekf', ekf)
        
        # 检查所有组件
        summary = checker.get_summary()
        assert summary['total'] == 2
        assert summary['all_healthy'] == True


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
