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
    - 记录时钟跳变事件供上层查询
    - 提供回调机制通知上层
    - 等待新数据到来后才恢复正常
    
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
        
        # 数据有效性标记 - 时钟回退后数据被标记为无效
        self._data_invalidated: bool = False
        
        # 统计信息
        self._update_counts: Dict[str, int] = {'odom': 0, 'imu': 0, 'trajectory': 0}
        
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
        """设置时钟跳变回调"""
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
            此方法应在持有锁的情况下调用
        """
        if self._last_time == 0.0:
            # 首次调用，不检测
            return None
        
        delta = now - self._last_time
        
        # 检测时钟回退
        if delta < -self.CLOCK_JITTER_TOLERANCE:
            event = ClockJumpEvent(self._last_time, now, delta)
            self._clock_jumped_back = True
            self._data_invalidated = True
            
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
                self._data_invalidated = True
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
    
    def _on_new_data(self, data_type: str) -> None:
        """
        新数据到来时的处理
        
        Args:
            data_type: 数据类型 ('odom', 'imu', 'trajectory')
        
        Note:
            此方法应在持有锁的情况下调用
        """
        # 如果数据被标记为无效，新数据到来后恢复有效性
        if self._data_invalidated:
            # 只有当所有必需数据都更新后才恢复
            # 这里简化处理：任何新数据都清除无效标记
            # 因为新数据的时间戳是在当前时钟下设置的
            self._data_invalidated = False
            logger.info(f"Data validity restored after receiving new {data_type} data")
    
    def update_odom(self, ros_msg: Any) -> Odometry:
        """
        更新里程计数据
        
        Args:
            ros_msg: ROS Odometry 消息
        
        Returns:
            转换后的 UC Odometry 数据
        """
        uc_odom = self._odom_adapter.to_uc(ros_msg)
        with self._lock:
            self._latest_data['odom'] = uc_odom
            self._timestamps['odom'] = self._get_time_func()
            self._update_counts['odom'] += 1
            self._on_new_data('odom')
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
        with self._lock:
            self._latest_data['imu'] = uc_imu
            self._timestamps['imu'] = self._get_time_func()
            self._update_counts['imu'] += 1
            self._on_new_data('imu')
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
        with self._lock:
            self._latest_data['trajectory'] = uc_traj
            self._timestamps['trajectory'] = self._get_time_func()
            self._update_counts['trajectory'] += 1
            self._on_new_data('trajectory')
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
            return self._latest_data.get('trajectory')
    
    def get_all_latest(self) -> Dict[str, Any]:
        """
        获取所有最新数据
        
        Returns:
            包含 'odom', 'imu', 'trajectory' 键的字典
        """
        with self._lock:
            return {
                'odom': self._latest_data.get('odom'),
                'imu': self._latest_data.get('imu'),
                'trajectory': self._latest_data.get('trajectory'),
            }
    
    def get_data_ages(self) -> Dict[str, float]:
        """
        获取各数据的年龄（秒）
        
        Returns:
            字典，键为数据名 ('odom', 'imu', 'traj')，
            值为距上次更新的秒数。未收到的数据返回 float('inf')。
            
        Note:
            - 如果时钟回退（如仿真时间重置），所有数据年龄返回 inf，
              表示数据已过时，需要等待新数据。
            - 时钟跳变（回退或大幅前跳）会触发回调通知上层。
            - 新数据到来后会自动恢复正常。
            - 内部使用 'trajectory' 存储，但返回 'traj' 以保持键名一致性
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
            # 键名映射: 内部 'trajectory' -> 外部 'traj'
            key_mapping = {'odom': 'odom', 'imu': 'imu', 'trajectory': 'traj'}
            for internal_key, external_key in key_mapping.items():
                if self._data_invalidated:
                    # 数据被标记为无效，返回 inf
                    ages[external_key] = float('inf')
                elif internal_key in self._timestamps:
                    # 正常计算年龄，防止负值
                    ages[external_key] = max(0.0, now - self._timestamps[internal_key])
                else:
                    ages[external_key] = float('inf')
        
        # 在锁外调用回调，避免死锁
        # 设计说明：
        # 1. 回调函数可能需要调用 DataManager 的其他方法（如 clear()）
        # 2. 如果在锁内调用回调，会导致死锁（即使使用 RLock，回调中的其他线程也可能阻塞）
        # 3. _data_invalidated 状态在锁内已经设置完成，回调只是通知上层
        # 4. 回调执行期间，其他线程可能更新数据，但这是预期行为（新数据会恢复有效性）
        if callback_event is not None and self._on_clock_jump is not None:
            try:
                self._on_clock_jump(callback_event)
            except Exception as e:
                logger.error(f"Clock jump callback failed: {e}")
        
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
        """检查是否有必需的数据（odom 和 trajectory）"""
        with self._lock:
            return 'odom' in self._latest_data and 'trajectory' in self._latest_data
    
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
            self._data_invalidated = False
            self._clock_jump_events.clear()
            # 重置统计信息
            for key in self._update_counts:
                self._update_counts[key] = 0
    
    def _get_health_details(self) -> Dict[str, Any]:
        """获取详细健康信息"""
        with self._lock:
            ages = self.get_data_ages()
            return {
                'has_odom': 'odom' in self._latest_data,
                'has_imu': 'imu' in self._latest_data,
                'has_trajectory': 'trajectory' in self._latest_data,
                'data_valid': not self._data_invalidated,
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
        检查数据是否有效
        
        Returns:
            如果数据有效返回 True，时钟回退后数据无效直到新数据到来
        """
        with self._lock:
            return not self._data_invalidated
    
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
