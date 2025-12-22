"""安全监控器"""
from typing import Dict, Any, Optional
from collections import deque
import numpy as np
import time

from ..core.interfaces import ISafetyMonitor
from ..core.data_types import ControlOutput, SafetyDecision
from ..core.enums import ControllerState, PlatformType


def _get_monotonic_time() -> float:
    """
    获取单调时钟时间（秒）
    
    使用 time.monotonic() 而非 time.time()，避免系统时间跳变
    （如 NTP 同步）导致的时间间隔计算错误。
    """
    return time.monotonic()


class BasicSafetyMonitor(ISafetyMonitor):
    """基础安全监控器实现"""
    
    def __init__(self, config: Dict[str, Any], platform_config: Dict[str, Any]):
        constraints = config.get('constraints', platform_config.get('constraints', {}))
        self.v_max = constraints.get('v_max', 2.0)
        self.vz_max = constraints.get('vz_max', 2.0)
        self.omega_max = constraints.get('omega_max', 2.0)
        self.a_max = constraints.get('a_max', 1.5)
        self.az_max = constraints.get('az_max', 1.0)
        self.alpha_max = constraints.get('alpha_max', 3.0)
        
        safety_config = config.get('safety', {})
        self.velocity_margin = safety_config.get('velocity_margin', 1.1)
        self.accel_margin = safety_config.get('accel_margin', 1.5)
        
        # 加速度滤波参数
        self.accel_filter_window = safety_config.get('accel_filter_window', 3)
        self.accel_filter_alpha = safety_config.get('accel_filter_alpha', 0.3)  # 低通滤波系数
        
        self._last_cmd: Optional[ControlOutput] = None
        self._last_time: Optional[float] = None
        self.is_3d = platform_config.get('type') == PlatformType.QUADROTOR
        
        # 加速度历史记录（用于滤波）
        self._accel_history_x: deque = deque(maxlen=self.accel_filter_window)
        self._accel_history_y: deque = deque(maxlen=self.accel_filter_window)
        self._accel_history_z: deque = deque(maxlen=self.accel_filter_window)
        self._alpha_history: deque = deque(maxlen=self.accel_filter_window)
        
        # 滤波后的加速度（None 表示未初始化）
        self._filtered_ax: Optional[float] = None
        self._filtered_ay: Optional[float] = None
        self._filtered_az: Optional[float] = None
        self._filtered_alpha: Optional[float] = None
    
    def _filter_acceleration(self, raw_ax: float, raw_ay: float, 
                            raw_az: float, raw_alpha: float) -> tuple:
        """
        使用低通滤波和滑动窗口平均来滤波加速度
        
        这可以减少由于速度测量噪声导致的加速度估计噪声
        
        初始化策略:
        - 第一次调用时，使用原始值初始化滤波器
        - 但如果原始值异常大（超过限制的 2 倍），使用限制值初始化
        - 这可以避免启动时的异常值影响后续滤波
        
        Args:
            raw_ax, raw_ay, raw_az: 原始加速度估计
            raw_alpha: 原始角加速度估计
        
        Returns:
            (filtered_ax, filtered_ay, filtered_az, filtered_alpha): 滤波后的加速度
        """
        # 添加到历史记录
        self._accel_history_x.append(raw_ax)
        self._accel_history_y.append(raw_ay)
        self._accel_history_z.append(raw_az)
        self._alpha_history.append(raw_alpha)
        
        # 计算滑动窗口平均
        if len(self._accel_history_x) > 0:
            avg_ax = np.mean(self._accel_history_x)
            avg_ay = np.mean(self._accel_history_y)
            avg_az = np.mean(self._accel_history_z)
            avg_alpha = np.mean(self._alpha_history)
        else:
            avg_ax = raw_ax
            avg_ay = raw_ay
            avg_az = raw_az
            avg_alpha = raw_alpha
        
        # 应用低通滤波（指数移动平均）
        if self._filtered_ax is None:
            # 第一次调用时初始化滤波器
            # 如果初始值异常大，使用限制值初始化，避免异常值影响
            INIT_CLAMP_FACTOR = 2.0  # 初始化时的限制因子
            init_ax = np.clip(avg_ax, -self.a_max * INIT_CLAMP_FACTOR, self.a_max * INIT_CLAMP_FACTOR)
            init_ay = np.clip(avg_ay, -self.a_max * INIT_CLAMP_FACTOR, self.a_max * INIT_CLAMP_FACTOR)
            init_az = np.clip(avg_az, -self.az_max * INIT_CLAMP_FACTOR, self.az_max * INIT_CLAMP_FACTOR)
            init_alpha = np.clip(avg_alpha, -self.alpha_max * INIT_CLAMP_FACTOR, self.alpha_max * INIT_CLAMP_FACTOR)
            
            self._filtered_ax = init_ax
            self._filtered_ay = init_ay
            self._filtered_az = init_az
            self._filtered_alpha = init_alpha
        else:
            self._filtered_ax = self.accel_filter_alpha * avg_ax + (1 - self.accel_filter_alpha) * self._filtered_ax
            self._filtered_ay = self.accel_filter_alpha * avg_ay + (1 - self.accel_filter_alpha) * self._filtered_ay
            self._filtered_az = self.accel_filter_alpha * avg_az + (1 - self.accel_filter_alpha) * self._filtered_az
            self._filtered_alpha = self.accel_filter_alpha * avg_alpha + (1 - self.accel_filter_alpha) * self._filtered_alpha
        
        return self._filtered_ax, self._filtered_ay, self._filtered_az, self._filtered_alpha
    
    def check(self, state: np.ndarray, cmd: ControlOutput, 
              diagnostics: Dict[str, Any]) -> SafetyDecision:
        """检查控制命令安全性"""
        reasons = []
        limited_cmd = cmd.copy()
        needs_limiting = False
        
        # 检查水平速度限制
        v_horizontal = cmd.v_horizontal
        if v_horizontal > self.v_max * self.velocity_margin:
            reasons.append(f"v_horizontal {v_horizontal:.2f} exceeds limit")
            if v_horizontal > 1e-6:
                scale = self.v_max / v_horizontal
                limited_cmd.vx = cmd.vx * scale
                limited_cmd.vy = cmd.vy * scale
                needs_limiting = True
        
        # 检查垂直速度限制 (无人机)
        if self.is_3d and abs(cmd.vz) > self.vz_max * self.velocity_margin:
            reasons.append(f"vz {cmd.vz:.2f} exceeds limit")
            limited_cmd.vz = np.clip(cmd.vz, -self.vz_max, self.vz_max)
            needs_limiting = True
        
        # 检查角速度限制
        if abs(cmd.omega) > self.omega_max * self.velocity_margin:
            reasons.append(f"omega {cmd.omega:.2f} exceeds limit")
            limited_cmd.omega = np.clip(cmd.omega, -self.omega_max, self.omega_max)
            needs_limiting = True
        
        # 检查加速度 (使用单调时钟避免时间跳变)
        current_time = _get_monotonic_time()
        if self._last_cmd is not None and self._last_time is not None:
            dt = current_time - self._last_time
            if dt > 0.001 and dt < 1.0:  # 添加上限检查，避免异常大的 dt
                # 计算原始加速度
                raw_ax = (cmd.vx - self._last_cmd.vx) / dt
                raw_ay = (cmd.vy - self._last_cmd.vy) / dt
                raw_az = (cmd.vz - self._last_cmd.vz) / dt if self.is_3d else 0.0
                raw_alpha = (cmd.omega - self._last_cmd.omega) / dt
                
                # 应用滤波
                ax, ay, az, alpha = self._filter_acceleration(raw_ax, raw_ay, raw_az, raw_alpha)
                
                accel_horizontal = np.sqrt(ax**2 + ay**2)
                
                if accel_horizontal > self.a_max * self.accel_margin:
                    reasons.append(f"a_horizontal {accel_horizontal:.2f} exceeds limit")
                    if accel_horizontal > 1e-6:
                        scale = self.a_max / accel_horizontal
                        limited_cmd.vx = self._last_cmd.vx + ax * scale * dt
                        limited_cmd.vy = self._last_cmd.vy + ay * scale * dt
                    needs_limiting = True
                
                if self.is_3d:
                    if abs(az) > self.az_max * self.accel_margin:
                        reasons.append(f"az {az:.2f} exceeds limit")
                        limited_cmd.vz = self._last_cmd.vz + np.clip(az, -self.az_max, self.az_max) * dt
                        needs_limiting = True
                
                if abs(alpha) > self.alpha_max * self.accel_margin:
                    reasons.append(f"alpha {alpha:.2f} exceeds limit")
                    limited_cmd.omega = self._last_cmd.omega + np.clip(alpha, -self.alpha_max, self.alpha_max) * dt
                    needs_limiting = True
        
        self._last_cmd = limited_cmd.copy() if needs_limiting else cmd.copy()
        self._last_time = current_time
        
        # 根据当前状态决定建议的新状态
        # 如果已经在 BACKUP_ACTIVE，不建议转换到 MPC_DEGRADED
        current_state = diagnostics.get('current_state', ControllerState.NORMAL)
        if reasons:
            if current_state == ControllerState.BACKUP_ACTIVE:
                # 已经在备用控制器，保持当前状态
                suggested_state = None
            else:
                suggested_state = ControllerState.MPC_DEGRADED
            
            return SafetyDecision(
                safe=False,
                new_state=suggested_state,
                reason="; ".join(reasons),
                limited_cmd=limited_cmd
            )
        
        return SafetyDecision(safe=True, new_state=None, reason="", limited_cmd=None)
    
    def reset(self) -> None:
        self._last_cmd = None
        self._last_time = None
        self._accel_history_x.clear()
        self._accel_history_y.clear()
        self._accel_history_z.clear()
        self._alpha_history.clear()
        self._filtered_ax = None
        self._filtered_ay = None
        self._filtered_az = None
        self._filtered_alpha = None
