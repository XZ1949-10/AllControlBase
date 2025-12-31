"""
参数加载器

从 ROS 参数服务器加载配置。
支持 ROS1 和 ROS2，以及非 ROS 环境。

设计说明:
=========
本模块采用"以 DEFAULT_CONFIG 为模板"的策略加载 ROS 参数：
1. 深拷贝 DEFAULT_CONFIG 作为基础配置
2. 递归遍历 DEFAULT_CONFIG 的结构
3. 对于每个配置项，尝试从 ROS 参数服务器读取对应值
4. 如果 ROS 参数存在，则覆盖默认值；否则保留默认值
5. 加载完成后进行配置验证（逻辑一致性检查）

配置验证策略:
=============
- FATAL 级别错误: 阻止启动，抛出异常
- ERROR 级别错误: 默认阻止启动，可通过 strict=False 跳过
- WARNING 级别: 记录警告日志，不阻止启动

配置分层设计:
=============
- transform.*: 坐标变换配置（坐标系名称 + 算法参数 + ROS TF2 参数）
- topics.*: ROS 话题配置，仅 ROS 层使用
"""
from typing import Dict, Any, List, Tuple
import logging
import copy

from universal_controller.config.default_config import DEFAULT_CONFIG
from universal_controller.config.validation import (
    validate_logical_consistency, 
    ValidationSeverity,
    ConfigValidationError
)
from .param_utils import (
    IParamStrategy, get_strategy, load_params_recursive, convert_param_type
)

logger = logging.getLogger(__name__)


# =============================================================================
# 话题配置 (独立于算法配置，仅 ROS 层使用)
#
# 命名规范:
# - 输入话题: /controller/input/<name>
# - 输出话题: /controller/<name>
# =============================================================================
TOPICS_DEFAULTS = {
    # 输入话题
    'odom': '/controller/input/odom',
    'imu': '',  # 默认禁用，需要时在配置中启用
    'trajectory': '/controller/input/trajectory',
    'emergency_stop': '/controller/emergency_stop',
    
    # 输出话题
    'cmd_unified': '/controller/cmd',
    'diagnostics': '/controller/diagnostics',
    'state': '/controller/state',
    'attitude_cmd': '/controller/attitude_cmd',
    'debug_path': '/controller/debug_path',
}

# =============================================================================
# ROS TF2 扩展配置
#
# 这些参数是 ROS TF2 特有的，扩展 DEFAULT_CONFIG['transform'] 配置。
# 在 ParamLoader.load() 中会合并到 config['transform'] 中。
#
# 设计说明：
# - 所有坐标变换相关配置统一在 transform.* 命名空间下
# - 算法层参数定义在 DEFAULT_CONFIG['transform']
# - ROS 层扩展参数定义在此处，加载时合并
# =============================================================================
TRANSFORM_ROS_DEFAULTS = {
    'buffer_warmup_timeout_sec': 2.0,   # TF buffer 预热超时
    'buffer_warmup_interval_sec': 0.1,  # TF buffer 预热检查间隔
    'retry_interval_sec': 1.0,          # TF 注入初始重试间隔（秒）
    'max_retry_interval_sec': 30.0,     # TF 注入最大重试间隔（秒）
    'backoff_multiplier': 2.0,          # TF 注入重试退避倍数
    'max_retries': -1,                  # 最大重试次数，-1 表示无限重试
}


# =============================================================================
# 时钟配置 (ROS 层特有)
#
# 用于 DataManager 的时钟跳变检测。
# =============================================================================
CLOCK_DEFAULTS = {
    'jitter_tolerance': 0.001,      # 时钟抖动容忍度（秒）
    'jump_threshold': 1.0,          # 时钟大幅跳变阈值（秒）
    'max_events': 10,               # 最多保留的时钟跳变事件数
}


# =============================================================================
# cmd_vel 适配器配置 (ROS 层特有)
#
# 这些参数是 cmd_vel_adapter 节点特有的，不在 DEFAULT_CONFIG 中。
# 注意: 速度/加速度限制统一从 constraints.* 读取，不在此处定义
# =============================================================================
CMD_VEL_ADAPTER_DEFAULTS = {
    'publish_rate': 20.0,           # 命令发布频率 (Hz)
    'cmd_timeout': 0.5,             # 命令超时 (秒)，适用于所有命令源（控制器和手柄）
    'enable_rate_limit': True,      # 是否启用速度变化率限制 (对所有命令生效，作为安全防线)
    'joy_topic': '/joy_cmd_vel',    # 手柄输入话题
    'mode_topic': '/visualizer/control_mode',  # 模式切换话题
    'output_topic': '/cmd_vel',     # 输出话题
}


# =============================================================================
# 约束配置验证规则 (用于 cmd_vel_adapter 等只需要约束配置的场景)
# =============================================================================
CONSTRAINTS_VALIDATION_RULES = {
    'constraints.v_max': (0.01, 100.0, '最大速度 (m/s)'),
    'constraints.v_min': (None, None, '最小速度 (m/s)'),  # 允许负值（倒车）
    'constraints.omega_max': (0.01, 50.0, '最大角速度 (rad/s)'),
    'constraints.a_max': (0.01, 50.0, '最大加速度 (m/s²)'),
    'constraints.alpha_max': (0.01, 100.0, '最大角加速度 (rad/s²)'),
}


# =============================================================================
# 参数加载器
# =============================================================================
class ParamLoader:
    """
    参数加载器
    
    从 ROS 参数服务器加载配置，并与默认配置合并。
    支持 ROS1、ROS2 和非 ROS 环境。
    
    使用示例:
        # ROS1
        config = ParamLoader.load(None)
        
        # ROS2
        config = ParamLoader.load(node)
        
        # 获取话题配置
        topics = ParamLoader.get_topics(None)
    """
    
    @staticmethod
    def load(node=None, validate: bool = True, strict: bool = True) -> Dict[str, Any]:
        """
        加载参数
        
        Args:
            node: ROS2 节点 (ROS2) 或 None (ROS1/非 ROS)
            validate: 是否进行配置验证，默认 True
            strict: 是否严格模式，默认 True
                   - True: FATAL/ERROR 级别错误会抛出异常阻止启动
                   - False: 只有 FATAL 级别错误会阻止启动
        
        Returns:
            合并后的配置字典
        
        Raises:
            ConfigValidationError: 当配置存在致命错误时
        """
        # 1. 深拷贝默认配置
        config = copy.deepcopy(DEFAULT_CONFIG)
        
        # 2. 获取加载策略
        strategy = get_strategy(node)
        
        # 3. 递归加载 ROS 参数，覆盖默认值
        load_params_recursive(config, '', strategy)
        
        # 4. 加载 ROS TF2 扩展参数，合并到 transform 配置中
        ParamLoader._load_transform_ros_params(config, strategy)
        
        # 5. 统一处理 dt 配置继承 (单一真相源)
        ParamLoader._sync_dt_config(config)
        
        # 6. 配置验证
        if validate:
            errors = ParamLoader._validate_config(config, strict=strict)
            # 错误已在 _validate_config 中处理（记录日志或抛出异常）
        
        # 7. 日志
        platform = config.get('system', {}).get('platform', 'unknown')
        ctrl_freq = config.get('system', {}).get('ctrl_freq', 50)
        v_max = config.get('constraints', {}).get('v_max', 'N/A')
        source_frame = config.get('transform', {}).get('source_frame', 'unknown')
        target_frame = config.get('transform', {}).get('target_frame', 'unknown')
        logger.info(
            f"Loaded config: platform={platform}, ctrl_freq={ctrl_freq}Hz, "
            f"v_max={v_max}, frames={source_frame}->{target_frame}"
        )
        
        return config
    
    @staticmethod
    def _validate_config(config: Dict[str, Any], strict: bool = True) -> List[Tuple[str, str, ValidationSeverity]]:
        """
        验证配置的逻辑一致性
        
        Args:
            config: 配置字典
            strict: 是否严格模式
        
        Returns:
            错误列表
        
        Raises:
            ConfigValidationError: 当存在致命错误或严格模式下存在严重错误时
        """
        try:
            errors = validate_logical_consistency(config)
        except Exception as e:
            # 验证过程中的异常不应被静默处理
            # 在 strict 模式下重新抛出，非 strict 模式下记录警告
            error_msg = f"Config validation failed with exception: {e}"
            if strict:
                logger.error(error_msg)
                raise ConfigValidationError(error_msg) from e
            else:
                logger.warning(f"{error_msg}. Continuing with unvalidated config (strict=False).")
                return []
        
        if not errors:
            return []
        
        # 分类错误
        fatal_errors = [(k, m, s) for k, m, s in errors if s == ValidationSeverity.FATAL]
        error_errors = [(k, m, s) for k, m, s in errors if s == ValidationSeverity.ERROR]
        warning_errors = [(k, m, s) for k, m, s in errors if s == ValidationSeverity.WARNING]
        
        # 记录警告
        for key, msg, _ in warning_errors:
            logger.warning(f"Config warning [{key}]: {msg}")
        
        # 记录严重错误
        for key, msg, _ in error_errors:
            logger.error(f"Config error [{key}]: {msg}")
        
        # 致命错误始终阻止启动
        if fatal_errors:
            fatal_msgs = '\n'.join([f'  - {key}: {msg}' for key, msg, _ in fatal_errors])
            raise ConfigValidationError(f'配置存在致命错误，无法启动:\n{fatal_msgs}')
        
        # 严格模式下，严重错误也阻止启动
        if strict and error_errors:
            error_msgs = '\n'.join([f'  - {key}: {msg}' for key, msg, _ in error_errors])
            raise ConfigValidationError(
                f'配置验证失败 (严格模式):\n{error_msgs}\n'
                f'如需跳过这些错误，请使用 ParamLoader.load(strict=False)'
            )
        
        return errors
    
    @staticmethod
    def _load_transform_ros_params(config: Dict[str, Any], strategy: IParamStrategy) -> None:
        """
        加载 ROS TF2 扩展参数到 transform 配置中
        
        从 transform.* 命名空间读取 ROS TF2 特有参数。
        """
        transform_config = config.setdefault('transform', {})
        
        for key, default in TRANSFORM_ROS_DEFAULTS.items():
            param_path = f"transform/{key}"
            value = strategy.get_param(param_path, default)
            transform_config[key] = convert_param_type(value, default)
    
    @staticmethod
    def _sync_dt_config(config: Dict[str, Any]) -> None:
        """
        统一处理 dt 配置继承 (单一真相源原则)
        
        设计原则:
        - mpc.dt 是时间步长的主配置 (用户只需配置这一个)
        - trajectory.default_dt_sec 自动继承 mpc.dt
        - 如果用户显式配置了 trajectory.default_dt_sec，则使用用户配置
        
        继承规则:
        1. 如果 trajectory.default_dt_sec 未被显式配置，则从 mpc.dt 继承
        2. 如果两者都被显式配置且不一致，验证阶段会发出警告
        3. 继承完成后，下游模块无需再处理继承逻辑
        
        这样设计的好处:
        - 用户只需配置 mpc.dt，减少配置复杂度
        - 确保 MPC 和轨迹处理使用相同的时间步长
        - 下游模块代码更简洁，无需重复继承逻辑
        """
        mpc_config = config.get('mpc', {})
        traj_config = config.setdefault('trajectory', {})
        
        mpc_dt = mpc_config.get('dt')
        
        # 检查 trajectory.default_dt_sec 是否被显式配置
        # 注意: 这里检查的是用户是否在 YAML 中配置了此项
        # 如果 traj_config 中没有 'default_dt_sec' 键，说明使用的是默认值
        traj_dt_explicitly_set = 'default_dt_sec' in traj_config
        
        if not traj_dt_explicitly_set and mpc_dt is not None:
            # trajectory.default_dt_sec 未显式配置，从 mpc.dt 继承
            traj_config['default_dt_sec'] = mpc_dt
            logger.debug(f"trajectory.default_dt_sec inherited from mpc.dt: {mpc_dt}")
    
    @staticmethod
    def get_topics(node=None) -> Dict[str, str]:
        """
        获取话题配置
        
        话题配置是 ROS 层特有的，不在 DEFAULT_CONFIG 中。
        """
        strategy = get_strategy(node)
        topics = {}
        
        for key, default in TOPICS_DEFAULTS.items():
            param_path = f"topics/{key}"
            topics[key] = strategy.get_param(param_path, default)
        
        return topics
    
    @staticmethod
    def get_cmd_vel_adapter_config(node=None) -> Dict[str, Any]:
        """
        获取 cmd_vel 适配器配置
        
        cmd_vel_adapter 配置是 ROS 层特有的，不在 DEFAULT_CONFIG 中。
        """
        strategy = get_strategy(node)
        config = {}
        
        for key, default in CMD_VEL_ADAPTER_DEFAULTS.items():
            param_path = f"cmd_vel_adapter/{key}"
            value = strategy.get_param(param_path, default)
            config[key] = convert_param_type(value, default)
        
        return config
    
    @staticmethod
    def validate_constraints(constraints: Dict[str, Any]) -> List[str]:
        """
        验证约束配置
        
        专门用于 cmd_vel_adapter 等只需要约束配置的场景。
        
        Args:
            constraints: 约束配置字典
        
        Returns:
            错误消息列表，空列表表示验证通过
        """
        errors = []
        
        # 致命错误检查：这些值必须大于 0
        v_max = constraints.get('v_max')
        if v_max is not None and v_max <= 0:
            errors.append(f"constraints.v_max ({v_max}) 必须大于 0")
        
        omega_max = constraints.get('omega_max')
        if omega_max is not None and omega_max <= 0:
            errors.append(f"constraints.omega_max ({omega_max}) 必须大于 0")
        
        a_max = constraints.get('a_max')
        if a_max is not None and a_max <= 0:
            errors.append(f"constraints.a_max ({a_max}) 必须大于 0")
        
        alpha_max = constraints.get('alpha_max')
        if alpha_max is not None and alpha_max <= 0:
            errors.append(f"constraints.alpha_max ({alpha_max}) 必须大于 0")
        
        # 逻辑一致性检查
        v_min = constraints.get('v_min')
        if v_min is not None and v_max is not None and v_min > v_max:
            errors.append(f"constraints.v_min ({v_min}) 不应大于 v_max ({v_max})")
        
        return errors
    
    @staticmethod
    def get_clock_config(node=None) -> Dict[str, Any]:
        """
        获取时钟配置
        
        时钟配置用于 DataManager 的时钟跳变检测。
        """
        strategy = get_strategy(node)
        config = {}
        
        for key, default in CLOCK_DEFAULTS.items():
            param_path = f"clock/{key}"
            value = strategy.get_param(param_path, default)
            config[key] = convert_param_type(value, default)
        
        return config
