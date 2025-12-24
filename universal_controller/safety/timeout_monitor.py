"""超时监控器"""
from typing import Dict, Any, Optional

from ..core.data_types import TimeoutStatus
from ..core.ros_compat import get_monotonic_time


# 表示"从未收到消息"的超时值 (毫秒)
# 使用大的有限值代替无穷大，避免 JSON 序列化问题
# 1e9 ms ≈ 11.5 天，足够表示"非常长时间"
NEVER_RECEIVED_AGE_MS = 1e9


class TimeoutMonitor:
    """超时监控器
    
    注意：所有时间戳使用单调时钟 (time.monotonic())，避免系统时间跳变影响。
    消息的 stamp 字段仅用于日志记录，不参与超时计算。
    """
    
    def __init__(self, config: Dict[str, Any]):
        watchdog_config = config.get('watchdog', {})
        self.odom_timeout_ms = watchdog_config.get('odom_timeout_ms', 200)
        self.traj_timeout_ms = watchdog_config.get('traj_timeout_ms', 200)
        self.traj_grace_ms = watchdog_config.get('traj_grace_ms', 100)
        self.imu_timeout_ms = watchdog_config.get('imu_timeout_ms', 100)
        self.startup_grace_ms = watchdog_config.get('startup_grace_ms', 1000)
        
        self._startup_time: Optional[float] = None
        self._last_odom_time: Optional[float] = None
        self._last_traj_time: Optional[float] = None
        self._last_imu_time: Optional[float] = None
        self._traj_timeout_start: Optional[float] = None
    
    def update_odom(self) -> None:
        """
        更新 odom 接收时间
        
        Note:
            超时检测基于接收时间（单调时钟），而非消息时间戳，原因：
            1. 消息时间戳可能受系统时间同步影响
            2. 接收时间更能反映实际的数据新鲜度
            3. 使用单调时钟避免时间跳变导致的误判
        """
        receive_time = get_monotonic_time()
        self._last_odom_time = receive_time
        if self._startup_time is None:
            self._startup_time = receive_time
    
    def update_trajectory(self) -> None:
        """
        更新轨迹接收时间
        
        Note:
            参见 update_odom 的说明
        """
        receive_time = get_monotonic_time()
        self._last_traj_time = receive_time
        self._traj_timeout_start = None
        if self._startup_time is None:
            self._startup_time = receive_time
    
    def update_imu(self) -> None:
        """
        更新 IMU 接收时间
        
        Note:
            参见 update_odom 的说明
        """
        receive_time = get_monotonic_time()
        self._last_imu_time = receive_time
        if self._startup_time is None:
            self._startup_time = receive_time
    
    def check(self, current_time: float = None) -> TimeoutStatus:
        """检查所有超时状态
        
        Args:
            current_time: 当前时间（秒），如果为 None 则使用单调时钟
                         注意：为保持一致性，建议不传入此参数
        
        Note:
            超时阈值 <= 0 表示禁用该数据源的超时检测
        """
        # 使用单调时钟，忽略外部传入的时间
        monotonic_now = get_monotonic_time()
        
        in_startup_grace = False
        if self._startup_time is None:
            in_startup_grace = True
        else:
            startup_elapsed_ms = (monotonic_now - self._startup_time) * 1000
            in_startup_grace = startup_elapsed_ms < self.startup_grace_ms
        
        if in_startup_grace:
            return TimeoutStatus(
                odom_timeout=False, traj_timeout=False, traj_grace_exceeded=False,
                imu_timeout=False, last_odom_age_ms=0.0, last_traj_age_ms=0.0,
                last_imu_age_ms=0.0, in_startup_grace=True
            )
        
        odom_age_ms = self._compute_age_ms(self._last_odom_time, monotonic_now)
        traj_age_ms = self._compute_age_ms(self._last_traj_time, monotonic_now)
        imu_age_ms = self._compute_age_ms(self._last_imu_time, monotonic_now)
        
        # 超时阈值 <= 0 表示禁用超时检测
        odom_timeout = self.odom_timeout_ms > 0 and odom_age_ms > self.odom_timeout_ms
        imu_timeout = self.imu_timeout_ms > 0 and imu_age_ms > self.imu_timeout_ms
        
        # 轨迹超时检测（支持禁用）
        traj_timeout = self.traj_timeout_ms > 0 and traj_age_ms > self.traj_timeout_ms
        traj_grace_exceeded = False
        
        if traj_timeout:
            if self._traj_timeout_start is None:
                self._traj_timeout_start = monotonic_now
            grace_elapsed_ms = (monotonic_now - self._traj_timeout_start) * 1000
            traj_grace_exceeded = grace_elapsed_ms > self.traj_grace_ms
        else:
            self._traj_timeout_start = None
        
        return TimeoutStatus(
            odom_timeout=odom_timeout,
            traj_timeout=traj_timeout,
            traj_grace_exceeded=traj_grace_exceeded,
            imu_timeout=imu_timeout,
            last_odom_age_ms=odom_age_ms,
            last_traj_age_ms=traj_age_ms,
            last_imu_age_ms=imu_age_ms,
            in_startup_grace=False
        )
    
    def _compute_age_ms(self, last_time: Optional[float], current_time: float) -> float:
        if last_time is None:
            return NEVER_RECEIVED_AGE_MS
        return (current_time - last_time) * 1000
    
    def reset(self) -> None:
        self._last_odom_time = None
        self._last_traj_time = None
        self._last_imu_time = None
        self._traj_timeout_start = None
        self._startup_time = None
