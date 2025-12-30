"""
统一数据管理器

管理传感器数据的缓存和访问，支持 ROS1 和 ROS2。
将数据缓存逻辑从节点实现中抽离，避免代码重复。

生命周期说明：
- 实现 ILifecycle 接口（通过 LifecycleMixin）
- reset(): 清除所有缓存数据，重置时钟状态
- shutdown(): 清除数据并释放回调引用
- 支持健康状态查询
"""
from typing import Dict, Any, Optional, Callable, List
import threading
import time
import logging

from universal_controller.core.data_types import Odometry, Imu, Trajectory
from ..adapters import OdomAdapter, ImuAdapter, TrajectoryAdapter
from ..utils.ros_compat import ROS_VERSION
from ..lifecycle import LifecycleMixin, LifecycleState

logger = logging.getLogger(__name__)


class ClockJumpEvent:
    """时钟跳变事件"""
    
    def __init__(self, old_time: float, new_time: float, jump_delta: float):
        self.old_time = old_time
        self.new_time = new_time
        self.jump_delta = jump_delta  # 负值表示回退
        self.timestamp = time.monotonic()  # 使用单调时钟记录事件时间
    
    @property
    def is_backward(self) -> bool:
        """是否为时钟回退"""
        return self.jump_delta < 0
    
    def __repr__(self) -> str:
        direction = "backward" if self.is_backward else "forward"
        return f"ClockJumpEvent({direction}, delta={self.jump_delta:.3f}s)"


class DataManager(LifecycleMixin):
    """
    统一数据管理器
    
    职责:
    - 管理传感器数据的缓存
    - 提供线程安全的数据访问
    - 管理数据时间戳
    - 计算数据年龄
    - 检测时钟回退并安全处理
    
    生命周期:
    - 实现 ILifecycle 接口（通过 LifecycleMixin）
    - initialize(): 自动在构造时调用
    - reset(): 清除所有缓存数据，重置时钟状态（原 clear() 方法）
    - shutdown(): 清除数据并释放回调引用
    
    时钟回退处理策略:
    - 检测到时钟回退时，标记所有数据为"过时"（年龄设为 inf）
    - 每种数据类型独立跟踪有效性，只有收到该类型的新数据才恢复
    - 记录时钟跳变事件供上层查询
    - 提供回调机制通知上层
    
    时钟前跳处理策略 (可配置):
    - 默认: 只记录日志，不使数据无效 (适合仿真快进场景)
    - 可选: 大幅前跳时也使数据无效 (适合需要严格时间同步的场景)
    
    支持 ROS1 和 ROS2，通过注入时间获取函数实现。
    """
    
    # 时钟抖动容忍度（秒）
    CLOCK_JITTER_TOLERANCE = 0.001  # 1ms
    
    # 时钟大幅跳变阈值（秒）- 超过此值认为是仿真重置
    CLOCK_JUMP_THRESHOLD = 1.0
    
    def __init__(self, get_time_func: Optional[Callable[[], float]] = None,
                 on_clock_jump: Optional[Callable[[ClockJumpEvent], None]] = None,
                 invalidate_on_forward_jump: bool = False):
        """
        初始化数据管理器
        
        Args:
            get_time_func: 获取当前时间的函数（秒）。
                          如果为 None，使用 time.time()。
                          ROS 环境应传入支持仿真时间的函数。
            on_clock_jump: 时钟跳变回调函数，用于通知上层。
                          **重要**: 回调函数应快速返回，不应执行耗时操作。
                          回调在锁外执行，可以安全地调用 DataManager 的其他方法。
                          如果需要执行耗时操作，应在回调中启动新线程或使用队列。
            invalidate_on_forward_jump: 是否在大幅前跳时使数据无效。
                          默认 False (适合仿真快进场景)。
                          设为 True 可用于需要严格时间同步的场景。
        """
        # 初始化 LifecycleMixin
        super().__init__()
        
        # 适配器
        self._odom_adapter = OdomAdapter()
        self._imu_adapter = ImuAdapter()
        self._traj_adapter = TrajectoryAdapter()
        
        # 时间获取函数
        self._get_time_func = get_time_func or time.time
        
        # 时钟跳变回调
        self._on_clock_jump = on_clock_jump
        
        # 时钟前跳处理策略
        self._invalidate_on_forward_jump = invalidate_on_forward_jump
        
        # 线程安全的数据存储
        # 使用 RLock（可重入锁）允许回调函数安全地调用 DataManager 的其他方法
        self._lock = threading.RLock()
        self._latest_data: Dict[str, Any] = {}
        self._timestamps: Dict[str, float] = {}
        
        # 时钟回退检测
        self._last_time: float = 0.0
        self._clock_jumped_back: bool = False
        self._clock_jump_events: List[ClockJumpEvent] = []
        self._max_clock_jump_events = 10  # 最多保留的事件数
        
        # 数据有效性标记 - 时钟回退后各数据分别标记为无效
        # 设计说明：使用分离的有效性标记，确保每种数据只有在收到新数据后才恢复有效
        # 这避免了"只收到 odom 就恢复所有数据有效性"的问题
        self._data_invalidated: Dict[str, bool] = {'odom': False, 'imu': False, 'traj': False}
        
        # 统计信息
        # 注意：键名统一使用简短形式 'odom', 'imu', 'traj'
        self._update_counts: Dict[str, int] = {'odom': 0, 'imu': 0, 'traj': 0}
        
        # 自动初始化
        self.initialize()
    
    @property
    def odom_adapter(self) -> OdomAdapter:
        """获取里程计适配器"""
        return self._odom_adapter
    
    @property
    def imu_adapter(self) -> ImuAdapter:
        """获取 IMU 适配器"""
        return self._imu_adapter
    
    @property
    def traj_adapter(self) -> TrajectoryAdapter:
        """获取轨迹适配器"""
        return self._traj_adapter
    
    def set_clock_jump_callback(self, callback: Optional[Callable[[ClockJumpEvent], None]]) -> None:
        """
        设置时钟跳变回调
        
        Args:
            callback: 回调函数，接收 ClockJumpEvent 参数。
                     **重要**: 回调应快速返回（< 1ms），不应执行耗时操作。
                     如需执行耗时操作，请在回调中启动新线程或使用队列。
        """
        with self._lock:
            self._on_clock_jump = callback
    
    def _check_clock_jump(self, now: float) -> Optional[ClockJumpEvent]:
        """
        检查时钟跳变
        
        Args:
            now: 当前时间
        
        Returns:
            如果检测到跳变，返回 ClockJumpEvent；否则返回 None
        
        Note:
            此方法应在持有锁的情况下调用。
            调用者负责在检测到跳变后更新 _last_time。
        """
        # 忽略无效时间（仿真时间未启动时可能为 0 或负数）
        if now <= 0:
            return None
        
        if self._last_time <= 0:
            # 首次收到有效时间，初始化 _last_time 但不检测跳变
            self._last_time = now
            return None
        
        delta = now - self._last_time
        
        # 检测时钟回退
        if delta < -self.CLOCK_JITTER_TOLERANCE:
            event = ClockJumpEvent(self._last_time, now, delta)
            self._clock_jumped_back = True
            # 只标记已收到过数据的类型为无效
            # 从未收到过的数据不需要标记（它们本来就没有有效数据）
            for key in self._data_invalidated:
                if key in self._latest_data:
                    self._data_invalidated[key] = True
            
            # 记录事件
            self._clock_jump_events.append(event)
            if len(self._clock_jump_events) > self._max_clock_jump_events:
                self._clock_jump_events.pop(0)
            
            logger.warning(
                f"Clock jumped backward by {abs(delta):.3f}s "
                f"(from {self._last_time:.3f} to {now:.3f}). "
                f"All cached data marked as stale."
            )
            
            return event
        
        # 检测时钟大幅前跳（可能是仿真快进）
        if delta > self.CLOCK_JUMP_THRESHOLD:
            event = ClockJumpEvent(self._last_time, now, delta)
            
            # 记录事件
            self._clock_jump_events.append(event)
            if len(self._clock_jump_events) > self._max_clock_jump_events:
                self._clock_jump_events.pop(0)
            
            # 根据配置决定是否使数据无效
            if self._invalidate_on_forward_jump:
                # 只标记已收到过数据的类型为无效
                for key in self._data_invalidated:
                    if key in self._latest_data:
                        self._data_invalidated[key] = True
                logger.warning(
                    f"Clock jumped forward by {delta:.3f}s "
                    f"(from {self._last_time:.3f} to {now:.3f}). "
                    f"All cached data marked as stale (invalidate_on_forward_jump=True)."
                )
            else:
                logger.info(
                    f"Clock jumped forward by {delta:.3f}s "
                    f"(from {self._last_time:.3f} to {now:.3f})"
                )
            
            return event
        
        return None
    
    def _on_new_data(self, data_type: str, now: float) -> Optional[ClockJumpEvent]:
        """
        新数据到来时的处理
        
        Args:
            data_type: 数据类型 ('odom', 'imu', 'traj')
            now: 当前时间（秒）
        
        Returns:
            如果检测到时钟跳变，返回 ClockJumpEvent；否则返回 None
        
        Note:
            此方法应在持有锁的情况下调用
        """
        # 在数据更新时也检测时钟跳变
        # 这确保即使在等待数据阶段也能检测到时钟跳变
        event = self._check_clock_jump(now)
        if event is not None:
            # 更新 _last_time 以避免重复检测
            self._last_time = now
        
        # 只恢复当前数据类型的有效性
        # 设计说明：每种数据独立跟踪有效性，只有收到该类型的新数据才恢复
        if self._data_invalidated.get(data_type, False):
            self._data_invalidated[data_type] = False
            logger.info(f"Data validity restored for {data_type} after receiving new data")
        
        return event
    
    def _invoke_clock_jump_callback(self, event: ClockJumpEvent) -> None:
        """
        调用时钟跳变回调
        
        在锁外调用，避免死锁。包含超时警告检测。
        
        Args:
            event: 时钟跳变事件
        """
        if self._on_clock_jump is None:
            return
        
        try:
            start_time = time.monotonic()
            self._on_clock_jump(event)
            elapsed_ms = (time.monotonic() - start_time) * 1000
            
            # 警告慢回调（超过 10ms）
            if elapsed_ms > 10.0:
                logger.warning(
                    f"Clock jump callback took {elapsed_ms:.1f}ms (should be < 10ms). "
                    f"Consider using async processing for slow operations."
                )
        except Exception as e:
            logger.error(f"Clock jump callback failed: {e}")
    
    def update_odom(self, ros_msg: Any) -> Odometry:
        """
        更新里程计数据
        
        Args:
            ros_msg: ROS Odometry 消息
        
        Returns:
            转换后的 UC Odometry 数据
        """
        uc_odom = self._odom_adapter.to_uc(ros_msg)
        callback_event = None
        
        with self._lock:
            now = self._get_time_func()
            self._latest_data['odom'] = uc_odom
            self._timestamps['odom'] = now
            self._update_counts['odom'] += 1
            callback_event = self._on_new_data('odom', now)
        
        # 在锁外调用回调，避免死锁
        if callback_event is not None:
            self._invoke_clock_jump_callback(callback_event)
        
        return uc_odom
    
    def update_imu(self, ros_msg: Any) -> Imu:
        """
        更新 IMU 数据
        
        Args:
            ros_msg: ROS Imu 消息
        
        Returns:
            转换后的 UC Imu 数据
        """
        uc_imu = self._imu_adapter.to_uc(ros_msg)
        callback_event = None
        
        with self._lock:
            now = self._get_time_func()
            self._latest_data['imu'] = uc_imu
            self._timestamps['imu'] = now
            self._update_counts['imu'] += 1
            callback_event = self._on_new_data('imu', now)
        
        # 在锁外调用回调，避免死锁
        if callback_event is not None:
            self._invoke_clock_jump_callback(callback_event)
        
        return uc_imu
    
    def update_trajectory(self, ros_msg: Any) -> Trajectory:
        """
        更新轨迹数据
        
        Args:
            ros_msg: ROS LocalTrajectoryV4 消息
        
        Returns:
            转换后的 UC Trajectory 数据
        """
        uc_traj = self._traj_adapter.to_uc(ros_msg)
        callback_event = None
        
        with self._lock:
            now = self._get_time_func()
            self._latest_data['traj'] = uc_traj
            self._timestamps['traj'] = now
            self._update_counts['traj'] += 1
            callback_event = self._on_new_data('traj', now)
        
        # 在锁外调用回调，避免死锁
        if callback_event is not None:
            self._invoke_clock_jump_callback(callback_event)
        
        return uc_traj
    
    def get_latest_odom(self) -> Optional[Odometry]:
        """获取最新里程计数据"""
        with self._lock:
            return self._latest_data.get('odom')
    
    def get_latest_imu(self) -> Optional[Imu]:
        """获取最新 IMU 数据"""
        with self._lock:
            return self._latest_data.get('imu')
    
    def get_latest_trajectory(self) -> Optional[Trajectory]:
        """获取最新轨迹数据"""
        with self._lock:
            return self._latest_data.get('traj')
    
    def get_all_latest(self) -> Dict[str, Any]:
        """
        获取所有最新数据
        
        Returns:
            包含 'odom', 'imu', 'traj' 键的字典
        """
        with self._lock:
            return {
                'odom': self._latest_data.get('odom'),
                'imu': self._latest_data.get('imu'),
                'traj': self._latest_data.get('traj'),
            }
    
    def get_data_ages(self) -> Dict[str, float]:
        """
        获取各数据的年龄（秒）
        
        Returns:
            字典，键为数据名 ('odom', 'imu', 'traj')，
            值为距上次更新的秒数。未收到的数据返回 float('inf')。
            
        Note:
            - 如果时钟回退（如仿真时间重置），被标记为无效的数据年龄返回 inf
            - 每种数据独立跟踪有效性，只有收到该类型的新数据才恢复
            - 时钟跳变（回退或大幅前跳）会触发回调通知上层
            - 键名统一使用简短形式: 'odom', 'imu', 'traj'
        """
        now = self._get_time_func()
        callback_event = None
        
        with self._lock:
            # 检测时钟跳变
            event = self._check_clock_jump(now)
            # 所有跳变事件都触发回调，让上层决定如何处理
            if event is not None:
                callback_event = event
            
            self._last_time = now
            
            ages = {}
            # 统一使用简短键名: odom, imu, traj
            for key in ('odom', 'imu', 'traj'):
                if self._data_invalidated.get(key, False):
                    # 该数据被标记为无效，返回 inf
                    ages[key] = float('inf')
                elif key in self._timestamps:
                    # 正常计算年龄，防止负值
                    ages[key] = max(0.0, now - self._timestamps[key])
                else:
                    ages[key] = float('inf')
        
        # 在锁外调用回调，避免死锁
        if callback_event is not None:
            self._invoke_clock_jump_callback(callback_event)
        
        return ages
    
    def get_timestamps(self) -> Dict[str, float]:
        """
        获取各数据的时间戳
        
        Returns:
            字典，键为数据名，值为时间戳（秒）
        """
        with self._lock:
            return self._timestamps.copy()
    
    def is_data_fresh(self, max_ages: Dict[str, float]) -> bool:
        """
        检查数据是否新鲜
        
        Args:
            max_ages: 各数据的最大允许年龄（秒）
        
        Returns:
            所有指定数据都在允许年龄内返回 True
        """
        ages = self.get_data_ages()
        for key, max_age in max_ages.items():
            if ages.get(key, float('inf')) > max_age:
                return False
        return True
    
    def has_required_data(self) -> bool:
        """检查是否有必需的数据（odom 和 traj）"""
        with self._lock:
            return 'odom' in self._latest_data and 'traj' in self._latest_data
    
    def clear(self):
        """
        清除所有缓存数据
        
        .. deprecated:: 3.18
            使用 reset() 代替，clear() 将在未来版本中移除
        """
        import warnings
        warnings.warn(
            "clear() is deprecated, use reset() instead",
            DeprecationWarning,
            stacklevel=2
        )
        self.reset()
    
    # ==================== LifecycleMixin 实现 ====================
    
    def _do_initialize(self) -> bool:
        """初始化数据管理器"""
        # DataManager 不需要特殊的初始化逻辑
        # 所有资源在 __init__ 中已经创建
        return True
    
    def _do_shutdown(self) -> None:
        """关闭数据管理器，释放资源"""
        with self._lock:
            self._latest_data.clear()
            self._timestamps.clear()
            self._clock_jump_events.clear()
            self._on_clock_jump = None  # 释放回调引用
    
    def _do_reset(self) -> None:
        """重置数据管理器状态"""
        with self._lock:
            self._latest_data.clear()
            self._timestamps.clear()
            self._last_time = 0.0
            self._clock_jumped_back = False
            # 重置所有数据的有效性标记
            for key in self._data_invalidated:
                self._data_invalidated[key] = False
            self._clock_jump_events.clear()
            # 重置统计信息
            for key in self._update_counts:
                self._update_counts[key] = 0
    
    def _get_health_details(self) -> Dict[str, Any]:
        """获取详细健康信息"""
        with self._lock:
            ages = self.get_data_ages()
            # 检查是否所有数据都有效
            all_data_valid = not any(self._data_invalidated.values())
            return {
                'has_odom': 'odom' in self._latest_data,
                'has_imu': 'imu' in self._latest_data,
                'has_traj': 'traj' in self._latest_data,
                'data_valid': all_data_valid,
                'data_invalidated': self._data_invalidated.copy(),
                'clock_jumped_back': self._clock_jumped_back,
                'odom_age_ms': ages.get('odom', float('inf')) * 1000,
                'traj_age_ms': ages.get('traj', float('inf')) * 1000,
                'update_counts': self._update_counts.copy(),
            }
    
    def did_clock_jump_back(self) -> bool:
        """
        检查时钟是否发生过回退
        
        Returns:
            如果检测到时钟回退返回 True
        """
        with self._lock:
            return self._clock_jumped_back
    
    def clear_clock_jump_flag(self):
        """清除时钟回退标志"""
        with self._lock:
            self._clock_jumped_back = False
    
    def is_data_valid(self) -> bool:
        """
        检查所有数据是否有效
        
        Returns:
            如果所有数据都有效返回 True，任何数据无效则返回 False
        """
        with self._lock:
            return not any(self._data_invalidated.values())
    
    def is_data_type_valid(self, data_type: str) -> bool:
        """
        检查指定类型的数据是否有效
        
        Args:
            data_type: 数据类型 ('odom', 'imu', 'traj')
        
        Returns:
            如果该数据有效返回 True
        """
        with self._lock:
            return not self._data_invalidated.get(data_type, True)
    
    def get_clock_jump_events(self) -> List[ClockJumpEvent]:
        """
        获取时钟跳变事件历史
        
        Returns:
            时钟跳变事件列表（最多保留最近 10 个）
        """
        with self._lock:
            return self._clock_jump_events.copy()
    
    def get_last_clock_jump(self) -> Optional[ClockJumpEvent]:
        """
        获取最近一次时钟跳变事件
        
        Returns:
            最近的时钟跳变事件，如果没有则返回 None
        """
        with self._lock:
            if self._clock_jump_events:
                return self._clock_jump_events[-1]
            return None
