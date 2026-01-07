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
    
    def check(self, data_ages: Optional[Dict[str, float]] = None) -> TimeoutStatus:
        """检查所有超时状态
        
        Args:
            data_ages: 数据年龄字典 {'odom': sec, 'trajectory': sec, 'imu': sec}
                      如果为 None，则假设数据无限久（从未收到）
        
        Returns:
            超时状态
        """
        monotonic_now = get_monotonic_time()
        
        # 1. 解析数据年龄 (秒 -> 毫秒)
        # 如果未提供 data_ages，默认为无穷大 (从未收到)
        current_ages_ms = {
            'odom': NEVER_RECEIVED_AGE_MS,
            'trajectory': NEVER_RECEIVED_AGE_MS,
            'imu': NEVER_RECEIVED_AGE_MS
        }
        
        if data_ages:
            for key, age_sec in data_ages.items():
                if age_sec != float('inf'):
                    current_ages_ms[key] = age_sec * 1000.0
        
        odom_age_ms = current_ages_ms['odom']
        traj_age_ms = current_ages_ms['trajectory']
        imu_age_ms = current_ages_ms['imu']
        
        # 2. 启动逻辑检测
        # 检测是否收到过任何有效数据
        has_any_data = (odom_age_ms < NEVER_RECEIVED_AGE_MS or 
                       traj_age_ms < NEVER_RECEIVED_AGE_MS or 
                       imu_age_ms < NEVER_RECEIVED_AGE_MS)
        
        if has_any_data:
            if self._startup_time is None:
                self._startup_time = monotonic_now
        
        # 3. 启动状态处理
        in_startup_grace = False
        if self._startup_time is None:
            # 尚未收到任何数据
            # 检查绝对启动超时
            if self.absolute_startup_timeout_enabled:
                time_since_creation_ms = (monotonic_now - self._creation_time) * 1000
                if time_since_creation_ms > self.absolute_startup_timeout_ms:
                    return TimeoutStatus(
                        odom_timeout=True, traj_timeout=True, traj_grace_exceeded=True,
                        imu_timeout=self.imu_timeout_enabled,
                        last_odom_age_ms=odom_age_ms,
                        last_traj_age_ms=traj_age_ms,
                        last_imu_age_ms=imu_age_ms,
                        in_startup_grace=False
                    )
            # 等待第一条数据，处于启动宽限期
            in_startup_grace = True
        else:
            # 已收到过数据，检查是否仍在启动宽限期内
            startup_elapsed_ms = (monotonic_now - self._startup_time) * 1000
            in_startup_grace = startup_elapsed_ms < self.startup_grace_ms
        
        if in_startup_grace:
            return TimeoutStatus(
                odom_timeout=False, traj_timeout=False, traj_grace_exceeded=False,
                imu_timeout=False, last_odom_age_ms=0.0, last_traj_age_ms=0.0,
                last_imu_age_ms=0.0, in_startup_grace=True
            )
        
        # 4. 超时检测
        odom_timeout = self.odom_timeout_enabled and odom_age_ms > self.odom_timeout_ms
        imu_timeout = self.imu_timeout_enabled and imu_age_ms > self.imu_timeout_ms
        
        # 轨迹超时与宽限期逻辑
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
    
    def reset(self) -> None:
        """
        完全重置监控器状态
        
        此方法会重置所有内部状态，包括：
        - 轨迹超时计时器
        - 启动时间（首次收到数据的时间）
        - 创建时间（用于绝对启动超时计算）
        
        设计意图：
            reset() 相当于重新创建一个监控器，所有超时计时将从零开始。
            这在以下场景中是正确的行为：
            - 控制器完全重启
            - 从 STOPPED 状态恢复
            - 长时间暂停后恢复（避免立即触发超时）
        
        Note:
            如果只需要清除临时状态而保留创建时间，请直接操作相应字段。
        """
        self._traj_timeout_start = None
        self._startup_time = None
        self._creation_time = get_monotonic_time()

