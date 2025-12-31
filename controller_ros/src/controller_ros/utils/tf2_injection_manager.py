"""
TF2 注入管理器

管理 TF2 到坐标变换器的注入逻辑，包括：
- 初始注入（阻塞等待 buffer 预热）
- 运行时重试注入（指数退避策略）
- 注入状态跟踪

将 TF2 注入相关的状态和逻辑从 ControllerNodeBase 中提取出来，
提高代码可读性和可测试性。

线程安全性:
- 使用 threading.Event 管理注入状态，确保线程安全
- inject() 和 try_reinjection_if_needed() 可以从不同线程安全调用
"""
from typing import Any, Callable, Dict, Optional
import time
import logging
import threading

logger = logging.getLogger(__name__)


class TF2InjectionManager:
    """
    TF2 注入管理器
    
    职责:
    - 管理 TF2 注入状态
    - 处理初始注入和运行时重试（指数退避）
    - 提供注入状态查询
    
    重试策略:
    - 使用指数退避：初始间隔 -> 2x -> 4x -> ... -> 最大间隔
    - 成功注入后重置退避状态
    - 可配置最大重试次数
    
    线程安全性:
    - 使用 threading.Event 管理注入状态，确保线程安全
    - inject() 和 try_reinjection_if_needed() 可以从不同线程安全调用
    - is_injected 属性是线程安全的
    
    配置参数 (从 TRANSFORM_ROS_DEFAULTS 读取):
    - buffer_warmup_timeout_sec: TF buffer 预热超时 (默认 2.0)
    - buffer_warmup_interval_sec: TF buffer 预热检查间隔 (默认 0.1)
    - retry_interval_sec: 初始重试间隔 (默认 1.0)
    - max_retry_interval_sec: 最大重试间隔 (默认 30.0)
    - backoff_multiplier: 退避倍数 (默认 2.0)
    - max_retries: 最大重试次数，-1 表示无限 (默认 -1)
    
    使用方法:
        manager = TF2InjectionManager(
            tf_bridge=tf_bridge,
            controller_manager=controller_manager,
            config=tf_config,
            transform_config=transform_config,
            log_info=node.log_info,
            log_warn=node.log_warn,
        )
        
        # 初始注入（阻塞）
        manager.inject(blocking=True)
        
        # 控制循环中调用
        manager.try_reinjection_if_needed()
    """
    
    # 默认重试配置 - 与 TRANSFORM_ROS_DEFAULTS 保持一致
    DEFAULT_BUFFER_WARMUP_TIMEOUT_SEC = 2.0
    DEFAULT_BUFFER_WARMUP_INTERVAL_SEC = 0.1
    DEFAULT_RETRY_INTERVAL_SEC = 1.0
    DEFAULT_MAX_RETRY_INTERVAL_SEC = 30.0
    DEFAULT_BACKOFF_MULTIPLIER = 2.0
    DEFAULT_MAX_RETRIES = -1
    
    def __init__(
        self,
        tf_bridge: Any,
        controller_manager: Any,
        config: Optional[Dict[str, Any]] = None,
        transform_config: Optional[Dict[str, Any]] = None,
        log_info: Optional[Callable[[str], None]] = None,
        log_warn: Optional[Callable[[str], None]] = None,
        get_time_func: Optional[Callable[[], float]] = None,
    ):
        """
        初始化 TF2 注入管理器
        
        Args:
            tf_bridge: TF2 桥接对象
            controller_manager: ControllerManager 实例
            config: TF2 配置字典（ROS TF2 特有参数），包含以下可选键:
                - buffer_warmup_timeout_sec: 阻塞等待超时 (默认 2.0)
                - buffer_warmup_interval_sec: 等待间隔 (默认 0.1)
                - retry_interval_sec: 初始重试间隔（秒）(默认 1.0)
                - max_retry_interval_sec: 最大重试间隔（秒）(默认 30.0)
                - backoff_multiplier: 退避倍数 (默认 2.0)
                - max_retries: 最大重试次数，-1 表示无限 (默认 -1)
            transform_config: 坐标变换配置字典
            log_info: 信息日志函数
            log_warn: 警告日志函数
            get_time_func: 获取当前时间的函数（秒）
        """
        self._tf_bridge = tf_bridge
        self._controller_manager = controller_manager
        self._config = config or {}
        self._transform_config = transform_config or {}
        
        # 日志函数
        self._log_info = log_info or (lambda msg: logger.info(msg))
        self._log_warn = log_warn or (lambda msg: logger.warning(msg))
        
        # 时间函数（使用单调时钟）
        self._get_time = get_time_func or time.monotonic
        
        # 坐标系配置
        self._source_frame = self._transform_config.get('source_frame', 'base_link')
        self._target_frame = self._transform_config.get('target_frame', 'odom')
        
        # TF2 特有配置 - 使用类常量作为默认值
        self._buffer_warmup_timeout_sec = self._config.get(
            'buffer_warmup_timeout_sec', self.DEFAULT_BUFFER_WARMUP_TIMEOUT_SEC
        )
        self._buffer_warmup_interval_sec = self._config.get(
            'buffer_warmup_interval_sec', self.DEFAULT_BUFFER_WARMUP_INTERVAL_SEC
        )
        self._max_retries = self._config.get('max_retries', self.DEFAULT_MAX_RETRIES)
        
        # 指数退避配置
        self._initial_retry_interval = self._config.get(
            'retry_interval_sec', self.DEFAULT_RETRY_INTERVAL_SEC
        )
        self._max_retry_interval = self._config.get(
            'max_retry_interval_sec', self.DEFAULT_MAX_RETRY_INTERVAL_SEC
        )
        self._backoff_multiplier = self._config.get(
            'backoff_multiplier', self.DEFAULT_BACKOFF_MULTIPLIER
        )
        
        # 状态 - 使用 threading.Event 确保线程安全
        self._injected_event = threading.Event()
        self._injection_attempted_event = threading.Event()
        
        # 重试状态（使用锁保护）
        self._lock = threading.Lock()
        self._retry_count = 0
        self._last_retry_time: Optional[float] = None
        self._current_retry_interval = self._initial_retry_interval
    
    @property
    def is_injected(self) -> bool:
        """TF2 是否已成功注入"""
        return self._injected_event.is_set()
    
    @property
    def injection_attempted(self) -> bool:
        """是否已尝试过注入"""
        return self._injection_attempted_event.is_set()
    
    @property
    def retry_count(self) -> int:
        """已重试次数"""
        with self._lock:
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
        
        self._injection_attempted_event.set()
        with self._lock:
            self._last_retry_time = self._get_time()
        
        # 注入回调
        if hasattr(coord_transformer, 'set_tf2_lookup_callback'):
            lookup_func = getattr(self._tf_bridge, 'lookup_transform', None)
            if lookup_func is not None:
                coord_transformer.set_tf2_lookup_callback(lookup_func)
                self._injected_event.set()
                self._reset_backoff()  # 成功后重置退避状态
                
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
    
    def _reset_backoff(self) -> None:
        """重置退避状态"""
        with self._lock:
            self._current_retry_interval = self._initial_retry_interval
    
    def _wait_for_tf2_ready(self, blocking: bool) -> bool:
        """等待 TF2 buffer 就绪"""
        can_transform_func = getattr(self._tf_bridge, 'can_transform', None)
        if can_transform_func is None:
            return False
        
        if blocking:
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
        
        使用指数退避策略，避免在 TF2 长时间不可用时频繁重试。
        
        Returns:
            是否执行了重新注入尝试
        """
        if self._injected_event.is_set():
            return False
        
        if not self._injection_attempted_event.is_set():
            return False
        
        if self._tf_bridge is None:
            return False
        
        if not getattr(self._tf_bridge, 'is_initialized', False):
            return False
        
        now = self._get_time()
        with self._lock:
            if self._max_retries >= 0 and self._retry_count >= self._max_retries:
                return False
            
            if self._last_retry_time is not None:
                elapsed = now - self._last_retry_time
                if elapsed < self._current_retry_interval:
                    return False
            
            # 更新重试状态
            self._last_retry_time = now
            self._retry_count += 1
            
            # 指数退避：增加下次重试间隔
            self._current_retry_interval = min(
                self._current_retry_interval * self._backoff_multiplier,
                self._max_retry_interval
            )
        
        # 执行重试（在锁外执行）
        success = self.inject(blocking=False)
        
        if not success:
            with self._lock:
                interval = self._current_retry_interval
            self._log_warn(
                f"TF2 reinjection attempt {self._retry_count} failed, "
                f"next retry in {interval:.1f}s"
            )
        
        return True
    
    def reset(self) -> None:
        """重置注入状态"""
        with self._lock:
            self._last_retry_time = None
            self._current_retry_interval = self._initial_retry_interval
            # 不重置 _injected_event 和 _injection_attempted_event
    
    def get_status(self) -> Dict[str, Any]:
        """获取注入状态"""
        with self._lock:
            return {
                'injected': self._injected_event.is_set(),
                'injection_attempted': self._injection_attempted_event.is_set(),
                'retry_count': self._retry_count,
                'current_retry_interval_sec': self._current_retry_interval,
                'source_frame': self._source_frame,
                'target_frame': self._target_frame,
            }
