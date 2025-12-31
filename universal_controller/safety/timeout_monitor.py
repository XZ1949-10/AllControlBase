"""超时监控器"""
from typing import Dict, Any, Optional

from ..core.data_types import TimeoutStatus
from ..core.ros_compat import get_monotonic_time
from ..core.constants import NEVER_RECEIVED_TIME_MS

# 向后兼容别名
NEVER_RECEIVED_AGE_MS = NEVER_RECEIVED_TIME_MS


class TimeoutMonitor:
    """超时监控器
    
    注意：所有时间戳使用单调时钟 (time.monotonic())，避免系统时间跳变影响。
    消息的 stamp 字段仅用于日志记录，不参与超时计算。
    
    超时配置说明:
    - xxx_timeout_ms > 0: 启用超时检测，超过此时间未收到数据则报告超时
    - xxx_timeout_ms <= 0: 禁用超时检测
    
    启动宽限期说明:
    - startup_grace_ms: 收到第一条数据后的宽限期，在此期间不报告超时
    - absolute_startup_timeout_ms: 从创建监控器开始的绝对超时
      如果超过此时间仍未收到任何数据，将报告超时
      设为 <= 0 表示禁用此检查（默认行为，向后兼容）
    
    典型配置示例:
        watchdog:
          odom_timeout_ms: 500      # 500ms 未收到 odom 则超时
          traj_timeout_ms: 2500     # 2.5s 未收到轨迹则超时
          imu_timeout_ms: -1        # 禁用 IMU 超时检测
          startup_grace_ms: 5000    # 启动后 5s 宽限期
          absolute_startup_timeout_ms: 10000  # 10s 内必须收到数据
    """
    
    def __init__(self, config: Dict[str, Any]):
        watchdog_config = config.get('watchdog', {})
        self.odom_timeout_ms = watchdog_config.get('odom_timeout_ms', 500)
        self.traj_timeout_ms = watchdog_config.get('traj_timeout_ms', 2500)
        self.traj_grace_ms = watchdog_config.get('traj_grace_ms', 1500)
        self.imu_timeout_ms = watchdog_config.get('imu_timeout_ms', -1)
        self.startup_grace_ms = watchdog_config.get('startup_grace_ms', 5000)
        
        # 绝对启动超时：从创建监控器开始，如果超过此时间仍未收到任何数据，报告超时
        # 默认 -1 表示禁用（与 WATCHDOG_CONFIG 一致）
        # 如需启用，建议设为 startup_grace_ms * 2
        self.absolute_startup_timeout_ms = watchdog_config.get('absolute_startup_timeout_ms', -1)
        
        # 记录监控器创建时间，用于绝对启动超时检测
        self._creation_time: float = get_monotonic_time()
        self._startup_time: Optional[float] = None
        self._last_odom_time: Optional[float] = None
        self._last_traj_time: Optional[float] = None
        self._last_imu_time: Optional[float] = None
        self._traj_timeout_start: Optional[float] = None
    
    @property
    def absolute_startup_timeout_enabled(self) -> bool:
        """是否启用绝对启动超时检测"""
        return self.absolute_startup_timeout_ms > 0
    
    @property
    def odom_timeout_enabled(self) -> bool:
        """是否启用 odom 超时检测"""
        return self.odom_timeout_ms > 0
    
    @property
    def traj_timeout_enabled(self) -> bool:
        """是否启用轨迹超时检测"""
        return self.traj_timeout_ms > 0
    
    @property
    def imu_timeout_enabled(self) -> bool:
        """是否启用 IMU 超时检测"""
        return self.imu_timeout_ms > 0
    
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
    
    def check(self) -> TimeoutStatus:
        """检查所有超时状态
        
        使用单调时钟 (time.monotonic()) 进行超时检测，避免系统时间跳变影响。
        
        Note:
            超时阈值 <= 0 表示禁用该数据源的超时检测
        """
        monotonic_now = get_monotonic_time()
        
        in_startup_grace = False
        if self._startup_time is None:
            # 从未收到任何数据
            # 检查是否超过绝对启动超时
            if self.absolute_startup_timeout_enabled:
                time_since_creation_ms = (monotonic_now - self._creation_time) * 1000
                if time_since_creation_ms > self.absolute_startup_timeout_ms:
                    # 超过绝对启动超时，报告 odom 和 traj 超时
                    # 这表明系统可能存在配置错误或硬件故障
                    return TimeoutStatus(
                        odom_timeout=True, 
                        traj_timeout=True, 
                        traj_grace_exceeded=True,
                        imu_timeout=self.imu_timeout_enabled,
                        last_odom_age_ms=NEVER_RECEIVED_AGE_MS,
                        last_traj_age_ms=NEVER_RECEIVED_AGE_MS,
                        last_imu_age_ms=NEVER_RECEIVED_AGE_MS,
                        in_startup_grace=False
                    )
            # 仍在等待第一条数据
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
        
        # 使用属性检查是否启用超时检测
        odom_timeout = self.odom_timeout_enabled and odom_age_ms > self.odom_timeout_ms
        imu_timeout = self.imu_timeout_enabled and imu_age_ms > self.imu_timeout_ms
        
        # 轨迹超时检测
        traj_timeout = self.traj_timeout_enabled and traj_age_ms > self.traj_timeout_ms
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
        # 重置创建时间，重新开始绝对启动超时计时
        self._creation_time = get_monotonic_time()
