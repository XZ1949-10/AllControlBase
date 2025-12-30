"""安全监控器"""
from typing import Dict, Any, Optional
from collections import deque
import numpy as np
import logging

from ..core.interfaces import ISafetyMonitor
from ..core.data_types import ControlOutput, SafetyDecision
from ..core.enums import ControllerState, PlatformType
from ..core.diagnostics_input import DiagnosticsInput
from ..core.ros_compat import get_monotonic_time
from ..core.constants import EPSILON

logger = logging.getLogger(__name__)


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
        
        # 紧急减速度配置
        # 用于安全违规时限制命令的减速度
        self.emergency_decel = safety_config.get('emergency_decel', 3.0)
        
        # 加速度滤波参数
        self.accel_filter_window = safety_config.get('accel_filter_window', 3)
        self.accel_filter_alpha = safety_config.get('accel_filter_alpha', 0.3)  # 低通滤波系数
        self.accel_filter_warmup_alpha = safety_config.get('accel_filter_warmup_alpha', 0.5)  # 预热系数
        self.accel_filter_warmup_period = safety_config.get('accel_filter_warmup_period', self.accel_filter_window)  # 预热期
        self.accel_warmup_margin_multiplier = safety_config.get('accel_warmup_margin_multiplier', 1.5)  # 预热期间裕度倍数
        # 预热期间裕度倍数的上限，防止配置错误导致安全检查过于宽松
        self.accel_warmup_margin_max = safety_config.get('accel_warmup_margin_max', 2.0)
        # 绝对加速度上限 - 即使在预热期间也不能超过此值
        # 这是一个硬性安全限制，防止任何情况下的危险加速度
        # 默认为 a_max 的 2 倍，确保即使滤波器未收敛也能捕获极端情况
        self.accel_absolute_max_multiplier = safety_config.get('accel_absolute_max_multiplier', 2.0)
        self.max_dt_for_accel = safety_config.get('max_dt_for_accel', 1.0)  # 加速度计算最大时间间隔
        self.min_dt_for_accel = safety_config.get('min_dt_for_accel', 0.001)  # 加速度计算最小时间间隔
        
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
        
        # 滤波器预热状态（在 __init__ 中初始化，避免动态创建属性）
        self._filter_warmup_count: int = 0
        self._filter_warmup_period: int = self.accel_filter_warmup_period
    
    def _filter_acceleration(self, raw_ax: float, raw_ay: float, 
                            raw_az: float, raw_alpha: float) -> tuple:
        """
        使用低通滤波和滑动窗口平均来滤波加速度
        
        这可以减少由于速度测量噪声导致的加速度估计噪声
        
        初始化策略:
        - 第一次调用时，使用限幅后的原始值初始化滤波器
        - 如果原始值异常大（超过限制的 2 倍），使用限制值初始化
        - 这可以避免启动时的异常值影响后续滤波，同时保持合理的初始估计
        
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
            # 
            # 初始化策略:
            # 1. 如果原始值在合理范围内 (< 2 * a_max)，使用原始值初始化
            #    这避免了零初始化导致的收敛延迟
            # 2. 如果原始值异常大 (>= 2 * a_max)，可能是传感器故障或启动瞬态
            #    使用零初始化，让滤波器从安全状态开始收敛
            # 3. 预热期间使用更宽松的安全阈值，避免误报
            
            max_init_accel = self.a_max * 2.0
            max_init_alpha = self.alpha_max * 2.0
            
            # 检测是否有异常大的初始值
            raw_accel_magnitude = np.sqrt(raw_ax**2 + raw_ay**2 + raw_az**2)
            is_anomalous = raw_accel_magnitude >= max_init_accel or abs(raw_alpha) >= max_init_alpha
            
            if is_anomalous:
                # 异常值：使用零初始化，更安全
                self._filtered_ax = 0.0
                self._filtered_ay = 0.0
                self._filtered_az = 0.0
                self._filtered_alpha = 0.0
                logger.debug(
                    f"Anomalous initial acceleration detected "
                    f"(magnitude={raw_accel_magnitude:.2f}, alpha={raw_alpha:.2f}), "
                    f"using zero initialization"
                )
            else:
                # 正常值：使用限幅后的原始值初始化
                self._filtered_ax = np.clip(raw_ax, -max_init_accel, max_init_accel)
                self._filtered_ay = np.clip(raw_ay, -max_init_accel, max_init_accel)
                self._filtered_az = np.clip(raw_az, -max_init_accel, max_init_accel)
                self._filtered_alpha = np.clip(raw_alpha, -max_init_alpha, max_init_alpha)
            
            # 重置预热计数
            self._filter_warmup_count = 0
        
        # 预热期间使用更大的滤波系数，加速收敛
        # 使用渐进过渡避免预热期结束时的跳变
        if self._filter_warmup_count < self._filter_warmup_period:
            # 预热期间：从 warmup_alpha 渐进过渡到 normal_alpha
            # 过渡因子从 0 增长到 1
            transition_factor = self._filter_warmup_count / self._filter_warmup_period
            # 线性插值：alpha = warmup_alpha * (1 - t) + normal_alpha * t
            current_alpha = (self.accel_filter_warmup_alpha * (1 - transition_factor) + 
                           self.accel_filter_alpha * transition_factor)
            self._filter_warmup_count += 1
        else:
            # 正常滤波
            current_alpha = self.accel_filter_alpha
        
        self._filtered_ax = current_alpha * avg_ax + (1 - current_alpha) * self._filtered_ax
        self._filtered_ay = current_alpha * avg_ay + (1 - current_alpha) * self._filtered_ay
        self._filtered_az = current_alpha * avg_az + (1 - current_alpha) * self._filtered_az
        self._filtered_alpha = current_alpha * avg_alpha + (1 - current_alpha) * self._filtered_alpha
        
        return self._filtered_ax, self._filtered_ay, self._filtered_az, self._filtered_alpha
    
    def is_filter_warmed_up(self) -> bool:
        """检查滤波器是否已完成预热"""
        return self._filter_warmup_count >= self._filter_warmup_period
    
    def check(self, state: np.ndarray, cmd: ControlOutput, 
              diagnostics: DiagnosticsInput) -> SafetyDecision:
        """检查控制命令安全性
        
        安全检查流程:
        1. 检查速度限制 (水平、垂直、角速度)
        2. 检查加速度限制 (使用滤波后的加速度估计)
        3. 如果违规，使用 emergency_decel 限制减速度
        
        emergency_decel 使用说明:
        - 当检测到安全违规时，限制命令的变化率不超过 emergency_decel
        - 这确保了即使在紧急情况下，机器人也能平滑减速而非急停
        """
        diag = diagnostics
        
        reasons = []
        limited_cmd = cmd.copy()
        needs_limiting = False
        
        # 检查水平速度限制
        v_horizontal = cmd.v_horizontal
        if v_horizontal > self.v_max * self.velocity_margin:
            reasons.append(f"v_horizontal {v_horizontal:.2f} exceeds limit")
            if v_horizontal > EPSILON:
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
        current_time = get_monotonic_time()
        if self._last_cmd is not None and self._last_time is not None:
            dt = current_time - self._last_time
            # 使用非严格不等式，确保边界值也被正确处理
            # min_dt_for_accel: 避免除零和数值不稳定
            # max_dt_for_accel: 避免长时间间隔导致的不准确加速度估计
            if self.min_dt_for_accel <= dt <= self.max_dt_for_accel:
                # 计算原始加速度
                raw_ax = (cmd.vx - self._last_cmd.vx) / dt
                raw_ay = (cmd.vy - self._last_cmd.vy) / dt
                raw_az = (cmd.vz - self._last_cmd.vz) / dt if self.is_3d else 0.0
                raw_alpha = (cmd.omega - self._last_cmd.omega) / dt
                
                # 应用滤波
                ax, ay, az, alpha = self._filter_acceleration(raw_ax, raw_ay, raw_az, raw_alpha)
                
                # 预热期间使用更宽松的阈值，而非完全跳过检查
                # 这样可以防止启动时的极端加速度，同时避免误报
                if self.is_filter_warmed_up():
                    accel_margin_effective = self.accel_margin
                else:
                    # 预热期间使用配置的裕度倍数，但不超过上限
                    # 这确保即使配置错误，安全检查也不会过于宽松
                    effective_multiplier = min(self.accel_warmup_margin_multiplier, self.accel_warmup_margin_max)
                    accel_margin_effective = self.accel_margin * effective_multiplier
                
                # 绝对加速度上限检查 - 无论预热状态如何都执行
                # 这是一个硬性安全限制，防止任何情况下的危险加速度
                accel_absolute_limit = self.a_max * self.accel_absolute_max_multiplier
                alpha_absolute_limit = self.alpha_max * self.accel_absolute_max_multiplier
                
                accel_horizontal = np.sqrt(ax**2 + ay**2)
                
                # 首先检查绝对上限（硬性限制）
                if accel_horizontal > accel_absolute_limit:
                    reasons.append(f"a_horizontal {accel_horizontal:.2f} exceeds absolute limit {accel_absolute_limit:.2f}")
                    # 使用 emergency_decel 限制减速度
                    limited_cmd = self._apply_emergency_decel_limit(
                        limited_cmd, self._last_cmd, dt, 'horizontal')
                    needs_limiting = True
                # 然后检查正常限制（考虑预热裕度）
                elif accel_horizontal > self.a_max * accel_margin_effective:
                    reasons.append(f"a_horizontal {accel_horizontal:.2f} exceeds limit")
                    # 使用 emergency_decel 限制减速度
                    limited_cmd = self._apply_emergency_decel_limit(
                        limited_cmd, self._last_cmd, dt, 'horizontal')
                    needs_limiting = True
                
                if self.is_3d:
                    # 垂直加速度绝对上限检查
                    az_absolute_limit = self.az_max * self.accel_absolute_max_multiplier
                    if abs(az) > az_absolute_limit:
                        reasons.append(f"az {az:.2f} exceeds absolute limit {az_absolute_limit:.2f}")
                        limited_cmd = self._apply_emergency_decel_limit(
                            limited_cmd, self._last_cmd, dt, 'vertical')
                        needs_limiting = True
                    elif abs(az) > self.az_max * accel_margin_effective:
                        reasons.append(f"az {az:.2f} exceeds limit")
                        limited_cmd = self._apply_emergency_decel_limit(
                            limited_cmd, self._last_cmd, dt, 'vertical')
                        needs_limiting = True
                
                # 角加速度绝对上限检查
                if abs(alpha) > alpha_absolute_limit:
                    reasons.append(f"alpha {alpha:.2f} exceeds absolute limit {alpha_absolute_limit:.2f}")
                    limited_cmd = self._apply_emergency_decel_limit(
                        limited_cmd, self._last_cmd, dt, 'angular')
                    needs_limiting = True
                elif abs(alpha) > self.alpha_max * accel_margin_effective:
                    reasons.append(f"alpha {alpha:.2f} exceeds limit")
                    limited_cmd = self._apply_emergency_decel_limit(
                        limited_cmd, self._last_cmd, dt, 'angular')
                    needs_limiting = True
        
        self._last_cmd = limited_cmd.copy() if needs_limiting else cmd.copy()
        self._last_time = current_time
        
        # 根据当前状态决定建议的新状态
        # 如果已经在 BACKUP_ACTIVE，不建议转换到 MPC_DEGRADED
        current_state = diag.current_state
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
    
    def _apply_emergency_decel_limit(self, cmd: ControlOutput, last_cmd: ControlOutput,
                                     dt: float, component: str) -> ControlOutput:
        """
        应用紧急减速度限制
        
        使用 emergency_decel 限制命令的变化率，确保平滑减速。
        
        设计说明:
        - 水平和垂直速度使用 emergency_decel (线加速度限制)
        - 角速度使用 emergency_decel 与 alpha_max 的比例关系
          即: emergency_alpha = emergency_decel * (alpha_max / a_max)
          这确保了线速度和角速度的紧急减速比例一致
        
        Args:
            cmd: 当前命令
            last_cmd: 上一次命令
            dt: 时间间隔
            component: 要限制的分量 ('horizontal', 'vertical', 'angular')
        
        Returns:
            限制后的命令
        """
        max_delta_v = self.emergency_decel * dt
        
        if component == 'horizontal':
            # 限制水平速度变化
            dvx = cmd.vx - last_cmd.vx
            dvy = cmd.vy - last_cmd.vy
            dv_mag = np.sqrt(dvx**2 + dvy**2)
            
            if dv_mag > max_delta_v and dv_mag > EPSILON:
                scale = max_delta_v / dv_mag
                cmd.vx = last_cmd.vx + dvx * scale
                cmd.vy = last_cmd.vy + dvy * scale
        
        elif component == 'vertical':
            # 限制垂直速度变化
            dvz = cmd.vz - last_cmd.vz
            if abs(dvz) > max_delta_v:
                cmd.vz = last_cmd.vz + np.sign(dvz) * max_delta_v
        
        elif component == 'angular':
            # 限制角速度变化
            # 使用与线速度相同的紧急减速比例
            # emergency_alpha = emergency_decel * (alpha_max / a_max)
            # 这确保了紧急情况下线速度和角速度的减速比例一致
            if self.a_max > EPSILON:
                emergency_alpha = self.emergency_decel * (self.alpha_max / self.a_max)
            else:
                emergency_alpha = self.alpha_max  # fallback
            max_delta_omega = emergency_alpha * dt
            domega = cmd.omega - last_cmd.omega
            if abs(domega) > max_delta_omega:
                cmd.omega = last_cmd.omega + np.sign(domega) * max_delta_omega
        
        return cmd
    
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
        # 重置预热状态
        self._filter_warmup_count = 0
