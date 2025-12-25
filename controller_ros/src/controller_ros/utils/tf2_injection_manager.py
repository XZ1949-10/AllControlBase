"""
TF2 注入管理器

管理 TF2 到坐标变换器的注入逻辑，包括：
- 初始注入（阻塞等待 buffer 预热）
- 运行时重试注入
- 注入状态跟踪

将 TF2 注入相关的状态和逻辑从 ControllerNodeBase 中提取出来，
提高代码可读性和可测试性。
"""
from typing import Any, Callable, Dict, Optional
import time
import logging

logger = logging.getLogger(__name__)


class TF2InjectionManager:
    """
    TF2 注入管理器
    
    职责:
    - 管理 TF2 注入状态
    - 处理初始注入和运行时重试
    - 提供注入状态查询
    
    使用方法:
        manager = TF2InjectionManager(
            tf_bridge=tf_bridge,
            controller_manager=controller_manager,
            config=tf_config,
            log_info=node.log_info,
            log_warn=node.log_warn,
        )
        
        # 初始注入（阻塞）
        manager.inject(blocking=True)
        
        # 控制循环中调用
        manager.try_reinjection_if_needed()
    """
    
    # 默认重试间隔（秒）
    DEFAULT_RETRY_INTERVAL_SEC = 1.0
    
    def __init__(
        self,
        tf_bridge: Any,
        controller_manager: Any,
        config: Optional[Dict[str, Any]] = None,
        log_info: Optional[Callable[[str], None]] = None,
        log_warn: Optional[Callable[[str], None]] = None,
        get_time_func: Optional[Callable[[], float]] = None,
    ):
        """
        初始化 TF2 注入管理器
        
        Args:
            tf_bridge: TF2 桥接对象，需要有 is_initialized, can_transform, lookup_transform 属性/方法
            controller_manager: ControllerManager 实例，需要有 coord_transformer 属性
            config: TF2 配置字典，包含以下可选键:
                - source_frame: 源坐标系 (默认 'base_link')
                - target_frame: 目标坐标系 (默认 'odom')
                - buffer_warmup_timeout_sec: 阻塞等待超时 (默认 2.0)
                - buffer_warmup_interval_sec: 等待间隔 (默认 0.1)
                - retry_interval_sec: 重试间隔（秒）(默认 1.0) [推荐使用]
                - retry_interval_cycles: [已废弃] 重试间隔周期数，仅为向后兼容保留
                - max_retries: 最大重试次数，-1 表示无限 (默认 -1)
            log_info: 信息日志函数
            log_warn: 警告日志函数
            get_time_func: 获取当前时间的函数（秒），默认使用 time.monotonic()
        """
        self._tf_bridge = tf_bridge
        self._controller_manager = controller_manager
        self._config = config or {}
        
        # 日志函数
        self._log_info = log_info or (lambda msg: logger.info(msg))
        self._log_warn = log_warn or (lambda msg: logger.warning(msg))
        
        # 时间函数（使用单调时钟，不受系统时间调整影响）
        self._get_time = get_time_func or time.monotonic
        
        # 配置参数
        self._source_frame = self._config.get('source_frame', 'base_link')
        self._target_frame = self._config.get('target_frame', 'odom')
        self._buffer_warmup_timeout_sec = self._config.get('buffer_warmup_timeout_sec', 2.0)
        self._buffer_warmup_interval_sec = self._config.get('buffer_warmup_interval_sec', 0.1)
        self._max_retries = self._config.get('max_retries', -1)  # -1 = 无限
        
        # 重试间隔配置（优先使用 retry_interval_sec，向后兼容 retry_interval_cycles）
        if 'retry_interval_sec' in self._config:
            self._retry_interval_sec = self._config['retry_interval_sec']
        elif 'retry_interval_cycles' in self._config:
            # 向后兼容：将周期数转换为秒
            # 优先从 system.ctrl_freq 获取控制频率，否则使用默认值 50Hz
            # 注意：config 可能是 tf 配置的子集，需要从外部传入 ctrl_freq
            ctrl_freq = self._config.get('ctrl_freq', 50)
            cycles = self._config['retry_interval_cycles']
            self._retry_interval_sec = cycles / ctrl_freq
            self._log_warn(
                f"'retry_interval_cycles' is deprecated, use 'retry_interval_sec' instead. "
                f"Converting {cycles} cycles to {self._retry_interval_sec:.2f}s "
                f"(using ctrl_freq={ctrl_freq}Hz). "
                f"To avoid this warning, set 'retry_interval_sec' directly in tf config."
            )
        else:
            self._retry_interval_sec = self.DEFAULT_RETRY_INTERVAL_SEC
        
        # 状态
        self._injected = False
        self._injection_attempted = False
        self._retry_count = 0  # 已重试次数
        self._last_retry_time: Optional[float] = None  # 上次重试时间
    
    @property
    def is_injected(self) -> bool:
        """TF2 是否已成功注入"""
        return self._injected
    
    @property
    def injection_attempted(self) -> bool:
        """是否已尝试过注入"""
        return self._injection_attempted
    
    @property
    def retry_count(self) -> int:
        """已重试次数"""
        return self._retry_count
    
    def inject(self, blocking: bool = True) -> bool:
        """
        执行 TF2 注入
        
        Args:
            blocking: 是否阻塞等待 TF2 buffer 预热
        
        Returns:
            是否成功注入
        """
        if self._tf_bridge is None:
            self._log_info("TF bridge is None, skipping TF2 injection")
            return False
        
        if not getattr(self._tf_bridge, 'is_initialized', False):
            self._log_info("TF2 not available, using fallback coordinate transform")
            return False
        
        if self._controller_manager is None:
            self._log_warn("Controller manager is None, cannot inject TF2")
            return False
        
        coord_transformer = getattr(self._controller_manager, 'coord_transformer', None)
        if coord_transformer is None:
            self._log_info("ControllerManager has no coord_transformer, TF2 injection skipped")
            return False
        
        # 检查 TF2 buffer 是否就绪
        tf2_ready = self._wait_for_tf2_ready(blocking)
        
        self._injection_attempted = True
        # 记录注入尝试时间，确保首次重试也遵循间隔设置
        self._last_retry_time = self._get_time()
        
        # 注入回调
        if hasattr(coord_transformer, 'set_tf2_lookup_callback'):
            lookup_func = getattr(self._tf_bridge, 'lookup_transform', None)
            if lookup_func is not None:
                coord_transformer.set_tf2_lookup_callback(lookup_func)
                self._injected = True
                
                if tf2_ready:
                    self._log_info("TF2 successfully injected to coordinate transformer")
                else:
                    self._log_info(
                        "TF2 callback injected, but buffer not ready yet. "
                        "Will use fallback until TF data arrives."
                    )
                return True
            else:
                self._log_warn("TF bridge has no lookup_transform method")
        else:
            self._log_warn("Coordinate transformer does not support TF2 injection")
        
        return False
    
    def _wait_for_tf2_ready(self, blocking: bool) -> bool:
        """
        等待 TF2 buffer 就绪
        
        Args:
            blocking: 是否阻塞等待
        
        Returns:
            TF2 是否就绪
        
        Note:
            此方法使用 time.monotonic() 而非 self._get_time()，因为：
            1. 阻塞等待需要测量真实的墙钟时间间隔
            2. 仿真时间在阻塞期间可能不会前进（如 Gazebo 暂停时）
            3. 单调时钟不受系统时间调整影响，更适合测量超时
        """
        can_transform_func = getattr(self._tf_bridge, 'can_transform', None)
        if can_transform_func is None:
            return False
        
        if blocking:
            # 使用单调时钟测量阻塞等待时间
            start_time = time.monotonic()
            while time.monotonic() - start_time < self._buffer_warmup_timeout_sec:
                try:
                    if can_transform_func(
                        self._target_frame, 
                        self._source_frame, 
                        timeout_sec=0.01
                    ):
                        elapsed = time.monotonic() - start_time
                        self._log_info(
                            f"TF2 buffer ready: {self._source_frame} -> {self._target_frame} "
                            f"(waited {elapsed:.2f}s)"
                        )
                        return True
                except Exception:
                    pass
                time.sleep(self._buffer_warmup_interval_sec)
            
            self._log_warn(
                f"TF2 buffer not ready after {self._buffer_warmup_timeout_sec}s, "
                f"will use fallback until TF becomes available"
            )
            return False
        else:
            try:
                return can_transform_func(
                    self._target_frame, 
                    self._source_frame, 
                    timeout_sec=0.01
                )
            except Exception:
                return False
    
    def try_reinjection_if_needed(self) -> bool:
        """
        在控制循环中调用，尝试重新注入 TF2（如果需要）
        
        使用时间间隔而非循环计数，确保重试行为与控制频率无关。
        
        Returns:
            是否执行了重新注入尝试
        """
        # 已经注入成功，无需重试
        if self._injected:
            return False
        
        # 尚未尝试过初始注入，不进行重试
        if not self._injection_attempted:
            return False
        
        # TF bridge 不可用
        if self._tf_bridge is None:
            return False
        
        if not getattr(self._tf_bridge, 'is_initialized', False):
            return False
        
        # 检查是否超过最大重试次数
        if self._max_retries >= 0 and self._retry_count >= self._max_retries:
            return False
        
        # 检查是否到达重试间隔（使用时间而非计数器）
        now = self._get_time()
        if self._last_retry_time is not None:
            elapsed = now - self._last_retry_time
            if elapsed < self._retry_interval_sec:
                return False
        
        # 更新重试时间
        self._last_retry_time = now
        
        # 执行重试
        self._retry_count += 1
        self.inject(blocking=False)
        return True
    
    def reset(self) -> None:
        """
        重置注入状态
        
        在控制器重置时调用。
        """
        self._last_retry_time = None
        # 注意：不重置 _injected 和 _injection_attempted，
        # 因为 TF2 回调一旦注入就保持有效
    
    def get_status(self) -> Dict[str, Any]:
        """
        获取注入状态
        
        Returns:
            状态字典
        """
        return {
            'injected': self._injected,
            'injection_attempted': self._injection_attempted,
            'retry_count': self._retry_count,
            'retry_interval_sec': self._retry_interval_sec,
            'source_frame': self._source_frame,
            'target_frame': self._target_frame,
        }
