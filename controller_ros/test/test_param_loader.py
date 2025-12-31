"""
ParamLoader 单元测试

测试配置加载器的核心功能：
1. 递归加载 ROS 参数
2. 类型转换
3. TF -> transform 映射
4. 默认值回退
"""
import pytest
from unittest.mock import MagicMock, patch
from typing import Dict, Any


class TestParamLoaderTypeConversion:
    """测试类型转换功能"""
    
    def test_convert_int_to_float(self):
        """测试 int -> float 转换"""
        from controller_ros.utils.param_utils import convert_param_type
        
        # YAML 可能将 0.0 解析为 int 0
        result = convert_param_type(0, 0.5)
        assert isinstance(result, float)
        assert result == 0.0
    
    def test_convert_float_to_int(self):
        """测试 float -> int 转换"""
        from controller_ros.utils.param_utils import convert_param_type
        
        result = convert_param_type(10.0, 5)
        assert isinstance(result, int)
        assert result == 10
    
    def test_convert_bool_preserved(self):
        """测试 bool 类型保持"""
        from controller_ros.utils.param_utils import convert_param_type
        
        result = convert_param_type(True, False)
        assert isinstance(result, bool)
        assert result is True
    
    def test_convert_string_preserved(self):
        """测试字符串类型保持"""
        from controller_ros.utils.param_utils import convert_param_type
        
        result = convert_param_type("test", "default")
        assert isinstance(result, str)
        assert result == "test"
    
    def test_convert_list_preserved(self):
        """测试列表类型保持"""
        from controller_ros.utils.param_utils import convert_param_type
        
        result = convert_param_type([1, 2, 3], [])
        assert isinstance(result, list)
        assert result == [1, 2, 3]
    
    def test_convert_none_default(self):
        """测试 None 默认值"""
        from controller_ros.utils.param_utils import convert_param_type
        
        result = convert_param_type("any_value", None)
        assert result == "any_value"


class TestParamLoaderRecursive:
    """测试递归加载功能"""
    
    def test_load_nested_config(self):
        """测试嵌套配置加载"""
        from controller_ros.utils.param_utils import load_params_recursive, DefaultStrategy
        
        # 模拟配置结构
        config = {
            'mpc': {
                'horizon': 20,
                'weights': {
                    'position': 10.0,
                    'velocity': 1.0,
                }
            }
        }
        
        strategy = DefaultStrategy()
        
        # 递归加载（使用默认策略，不会改变值）
        load_params_recursive(config, '', strategy)
        
        # 验证结构保持
        assert config['mpc']['horizon'] == 20
        assert config['mpc']['weights']['position'] == 10.0
        assert config['mpc']['weights']['velocity'] == 1.0
    
    def test_load_with_mock_ros_params(self):
        """测试使用模拟 ROS 参数"""
        from controller_ros.utils.param_utils import load_params_recursive, IParamStrategy
        
        # 创建模拟策略
        class MockStrategy(IParamStrategy):
            def __init__(self, params: Dict[str, Any]):
                self._params = params
            
            def get_param(self, param_path: str, default: Any) -> Any:
                return self._params.get(param_path, default)
            
            def has_param(self, param_path: str) -> bool:
                return param_path in self._params
        
        # 模拟 ROS 参数
        ros_params = {
            'mpc/horizon': 15,
            'mpc/weights/position': 5.0,
        }
        
        config = {
            'mpc': {
                'horizon': 20,
                'weights': {
                    'position': 10.0,
                    'velocity': 1.0,
                }
            }
        }
        
        strategy = MockStrategy(ros_params)
        load_params_recursive(config, '', strategy)
        
        # 验证 ROS 参数覆盖了默认值
        assert config['mpc']['horizon'] == 15
        assert config['mpc']['weights']['position'] == 5.0
        # 未设置的参数保持默认值
        assert config['mpc']['weights']['velocity'] == 1.0


class TestParamLoaderConfigSeparation:
    """测试 transform 配置统一"""
    
    def test_transform_ros_defaults_contains_tf2_params(self):
        """测试 TRANSFORM_ROS_DEFAULTS 包含 ROS TF2 特有参数"""
        from controller_ros.utils.param_loader import TRANSFORM_ROS_DEFAULTS
        
        # TRANSFORM_ROS_DEFAULTS 应该包含 ROS TF2 特有参数
        expected_keys = {
            'buffer_warmup_timeout_sec',
            'buffer_warmup_interval_sec',
            'retry_interval_sec',
            'max_retry_interval_sec',
            'backoff_multiplier',
            'max_retries',
        }
        
        assert set(TRANSFORM_ROS_DEFAULTS.keys()) == expected_keys
        
        # 坐标系名称不应该在 TRANSFORM_ROS_DEFAULTS 中（它们在 DEFAULT_CONFIG['transform'] 中）
        assert 'source_frame' not in TRANSFORM_ROS_DEFAULTS
        assert 'target_frame' not in TRANSFORM_ROS_DEFAULTS
        assert 'timeout_ms' not in TRANSFORM_ROS_DEFAULTS
    
    def test_transform_config_contains_frame_names(self):
        """测试 transform 配置包含坐标系名称"""
        from universal_controller.config.modules_config import TRANSFORM_CONFIG
        
        # transform 配置应该包含坐标系名称
        assert 'source_frame' in TRANSFORM_CONFIG
        assert 'target_frame' in TRANSFORM_CONFIG
        assert 'timeout_ms' in TRANSFORM_CONFIG


class TestParamLoaderTopics:
    """测试话题配置加载"""
    
    def test_get_topics_defaults(self):
        """测试获取默认话题配置"""
        from controller_ros.utils.param_loader import ParamLoader, TOPICS_DEFAULTS
        
        topics = ParamLoader.get_topics(None)
        
        # 验证默认值
        for key, default in TOPICS_DEFAULTS.items():
            assert key in topics
            assert topics[key] == default


class TestParamLoaderIntegration:
    """集成测试"""
    
    def test_load_returns_valid_config(self):
        """测试 load() 返回有效配置"""
        from controller_ros.utils.param_loader import ParamLoader
        from universal_controller.config.default_config import DEFAULT_CONFIG
        
        # 在非 ROS 环境下加载
        config = ParamLoader.load(None)
        
        # 验证返回的配置包含所有必要的键
        for key in DEFAULT_CONFIG.keys():
            assert key in config, f"Missing key: {key}"
    
    def test_load_preserves_nested_structure(self):
        """测试 load() 保持嵌套结构"""
        from controller_ros.utils.param_loader import ParamLoader
        
        config = ParamLoader.load(None)
        
        # 验证嵌套结构
        assert 'weights' in config['mpc']
        assert 'position' in config['mpc']['weights']
        assert 'health_monitor' in config['mpc']
        assert 'solver' in config['mpc']
        assert 'fallback' in config['mpc']
    
    def test_load_includes_transform_config_with_tf2_params(self):
        """测试 load() 包含统一的 transform 配置
        
        验证 TF2 参数被正确加载到 config['transform'] 中，
        供 TF2InjectionManager 使用。
        
        配置统一设计：
        - transform: 包含坐标系名称 + 算法参数 + ROS TF2 特有参数
        - tf: 已废弃，向后兼容时仍支持读取
        """
        from controller_ros.utils.param_loader import ParamLoader, TRANSFORM_ROS_DEFAULTS
        
        config = ParamLoader.load(None)
        
        # 验证 transform 配置存在
        assert 'transform' in config, "config should contain 'transform' key"
        
        # 验证 transform 配置包含坐标系名称
        assert 'source_frame' in config['transform']
        assert 'target_frame' in config['transform']
        assert 'timeout_ms' in config['transform']
        
        # 验证 transform 配置包含 ROS TF2 特有参数
        for key in TRANSFORM_ROS_DEFAULTS.keys():
            assert key in config['transform'], f"Missing transform key: {key}"
    
    def test_load_includes_diagnostics_publish_rate(self):
        """测试 load() 包含诊断发布率配置
        
        验证 diagnostics.publish_rate 被正确加载。
        """
        from controller_ros.utils.param_loader import ParamLoader
        
        config = ParamLoader.load(None)
        
        # 验证 diagnostics 配置存在
        assert 'diagnostics' in config
        assert 'publish_rate' in config['diagnostics']
        assert isinstance(config['diagnostics']['publish_rate'], int)
        assert config['diagnostics']['publish_rate'] > 0


class TestConfigKeyMatching:
    """测试配置键名匹配
    
    验证 turtlebot1.yaml 中的键名与 DEFAULT_CONFIG 结构匹配
    """
    
    def test_yaml_keys_match_default_config(self):
        """验证 YAML 配置键与 DEFAULT_CONFIG 匹配"""
        import yaml
        import os
        from universal_controller.config.default_config import DEFAULT_CONFIG
        
        # 读取 turtlebot1.yaml
        yaml_path = os.path.join(
            os.path.dirname(__file__), 
            '..', 'config', 'platforms', 'turtlebot1.yaml'
        )
        
        if not os.path.exists(yaml_path):
            pytest.skip("turtlebot1.yaml not found")
        
        with open(yaml_path, 'r', encoding='utf-8') as f:
            yaml_config = yaml.safe_load(f)
        
        # 检查顶级键
        # ROS 层特有的键，不需要在 DEFAULT_CONFIG 中定义
        ros_only_keys = {'node', 'topics', 'tf', 'visualizer', 'cmd_vel_adapter'}
        
        for key in yaml_config.keys():
            if key not in ros_only_keys:
                assert key in DEFAULT_CONFIG, \
                    f"YAML key '{key}' not in DEFAULT_CONFIG"
        
        # 检查嵌套键 (mpc.weights)
        if 'mpc' in yaml_config and 'weights' in yaml_config['mpc']:
            for weight_key in yaml_config['mpc']['weights'].keys():
                assert weight_key in DEFAULT_CONFIG['mpc']['weights'], \
                    f"YAML key 'mpc.weights.{weight_key}' not in DEFAULT_CONFIG"
        
        # 检查 consistency.weights
        if 'consistency' in yaml_config and 'weights' in yaml_config['consistency']:
            for weight_key in yaml_config['consistency']['weights'].keys():
                assert weight_key in DEFAULT_CONFIG['consistency']['weights'], \
                    f"YAML key 'consistency.weights.{weight_key}' not in DEFAULT_CONFIG"
        
        # 检查 ekf (不是 estimator)
        if 'ekf' in yaml_config:
            assert 'ekf' in DEFAULT_CONFIG, "ekf should be in DEFAULT_CONFIG"
            # 不应该有 estimator 键
            assert 'estimator' not in yaml_config, \
                "YAML should use 'ekf' not 'estimator'"
        
        # 检查 backup (不是 safety.backup_*)
        if 'backup' in yaml_config:
            assert 'backup' in DEFAULT_CONFIG, "backup should be in DEFAULT_CONFIG"


class TestDtConfigInheritance:
    """测试 dt 配置继承 (单一真相源原则)
    
    验证 trajectory.default_dt_sec 自动从 mpc.dt 继承的机制。
    """
    
    def test_dt_inheritance_from_mpc(self):
        """测试 trajectory.default_dt_sec 从 mpc.dt 继承"""
        from controller_ros.utils.param_loader import ParamLoader
        
        config = ParamLoader.load(None)
        
        # 验证两者一致
        mpc_dt = config['mpc']['dt']
        traj_dt = config['trajectory']['default_dt_sec']
        
        assert mpc_dt == traj_dt, \
            f"mpc.dt ({mpc_dt}) should equal trajectory.default_dt_sec ({traj_dt})"
    
    def test_sync_dt_config_inherits_when_not_set(self):
        """测试 _sync_dt_config 在 trajectory.default_dt_sec 未设置时继承"""
        from controller_ros.utils.param_loader import ParamLoader
        
        # 模拟配置：只有 mpc.dt，没有 trajectory.default_dt_sec
        config = {
            'mpc': {'dt': 0.05},
            'trajectory': {'low_speed_thresh': 0.1}  # 没有 default_dt_sec
        }
        
        ParamLoader._sync_dt_config(config)
        
        # 验证继承
        assert config['trajectory']['default_dt_sec'] == 0.05
    
    def test_sync_dt_config_preserves_explicit_value(self):
        """测试 _sync_dt_config 保留显式配置的值"""
        from controller_ros.utils.param_loader import ParamLoader
        
        # 模拟配置：两者都有值
        config = {
            'mpc': {'dt': 0.05},
            'trajectory': {'default_dt_sec': 0.2}  # 显式配置
        }
        
        ParamLoader._sync_dt_config(config)
        
        # 验证保留显式值
        assert config['trajectory']['default_dt_sec'] == 0.2
    
    def test_sync_dt_config_handles_missing_mpc(self):
        """测试 _sync_dt_config 处理缺少 mpc 配置的情况"""
        from controller_ros.utils.param_loader import ParamLoader
        
        # 模拟配置：没有 mpc
        config = {
            'trajectory': {'low_speed_thresh': 0.1}
        }
        
        # 不应该抛出异常
        ParamLoader._sync_dt_config(config)
        
        # trajectory 应该保持不变（没有 default_dt_sec）
        assert 'default_dt_sec' not in config['trajectory']
    
    def test_sync_dt_config_handles_missing_trajectory(self):
        """测试 _sync_dt_config 处理缺少 trajectory 配置的情况"""
        from controller_ros.utils.param_loader import ParamLoader
        
        # 模拟配置：没有 trajectory
        config = {
            'mpc': {'dt': 0.05}
        }
        
        ParamLoader._sync_dt_config(config)
        
        # 应该创建 trajectory 并设置 default_dt_sec
        assert 'trajectory' in config
        assert config['trajectory']['default_dt_sec'] == 0.05
