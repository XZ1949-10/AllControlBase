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
from ..core.constants import (
    EPSILON, 
    MIN_DT_FOR_ACCEL, 
    MAX_DT_FOR_ACCEL,
    SAFETY_VELOCITY_MARGIN,
    SAFETY_ACCEL_MARGIN,
    SAFETY_ACCEL_WARMUP_MARGIN_MAX,
    SAFETY_ACCEL_ABSOLUTE_MAX_MULTIPLIER,
)

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
        
        # 使用常量作为安全裕度（不再从配置读取）
        self.velocity_margin = SAFETY_VELOCITY_MARGIN
        self.accel_margin = SAFETY_ACCEL_MARGIN
        
        # 紧急减速度配置
        # 用于安全违规时限制命令的减速度
        self.emergency_decel = safety_config.get('emergency_decel', 3.0)
        
        # 加速度滤波参数
        self.accel_filter_window = safety_config.get('accel_filter_window', 3)
        self.accel_filter_alpha = safety_config.get('accel_filter_alpha', 0.3)  # 低通滤波系数
        self.accel_filter_warmup_alpha = safety_config.get('accel_filter_warmup_alpha', 0.5)  # 预热系数
        self.accel_filter_warmup_period = safety_config.get('accel_filter_warmup_period', self.accel_filter_window)  # 预热期
        self.accel_warmup_margin_multiplier = safety_config.get('accel_warmup_margin_multiplier', 1.5)  # 预热期间裕度倍数
        
        # 使用常量作为安全上限（不再从配置读取）
        # 预热期间裕度倍数的上限，防止配置错误导致安全检查过于宽松
        self.accel_warmup_margin_max = SAFETY_ACCEL_WARMUP_MARGIN_MAX
        # 绝对加速度上限倍数 - 硬性安全限制
        self.accel_absolute_max_multiplier = SAFETY_ACCEL_ABSOLUTE_MAX_MULTIPLIER
        
        # 使用常量作为加速度计算的时间间隔阈值
        self.max_dt_for_accel = MAX_DT_FOR_ACCEL
        self.min_dt_for_accel = MIN_DT_FOR_ACCEL
        
        self._last_cmd: Optional[ControlOutput] = None
        # _last_time removed as we now use external dt

        # 使用配置的 is_ground_vehicle，默认根据平台类型判断
        self.is_3d = not platform_config.get(
            'is_ground_vehicle', 
            platform_config.get('type') != PlatformType.QUADROTOR
        )
        
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
              diagnostics: DiagnosticsInput, dt: float) -> SafetyDecision:
        """检查控制命令安全性
        
        Args:
            state: 当前状态
            cmd: 待检查的命令
            diagnostics: 诊断输入
            dt: 时间步长（由 ControllerManager 传入，确保时间基准一致）
        
        安全检查流程:
        1. 检查命令值是否为 NaN/Inf (安全关键)
        2. 检查速度限制 (水平、垂直、角速度)
        3. 检查加速度限制 (使用滤波后的加速度估计)
        4. 如果违规，使用 emergency_decel 限制减速度
        """
        diag = diagnostics

        reasons = []
        limited_cmd = cmd.copy()
        needs_limiting = False

        # 安全关键: 检查命令值是否为 NaN/Inf
        # NaN 在比较时总是返回 False，会绕过所有限制检查
        # 必须在所有其他检查之前进行
        if not (np.isfinite(cmd.vx) and np.isfinite(cmd.vy) and
                np.isfinite(cmd.vz) and np.isfinite(cmd.omega)):
            logger.error(
                f"NaN/Inf detected in control command: "
                f"vx={cmd.vx}, vy={cmd.vy}, vz={cmd.vz}, omega={cmd.omega}. "
                f"Returning zero command for safety."
            )
            # 返回零速度命令，这是最安全的选择
            zero_cmd = ControlOutput(
                vx=0.0, vy=0.0, vz=0.0, omega=0.0,
                frame_id=cmd.frame_id, success=False,
                health_metrics={'error_type': 'nan_detected'}
            )
            # 更新内部状态，确保下次加速度计算正确
            # 使用零命令作为 last_cmd，避免 NaN 传播
            self._last_cmd = zero_cmd.copy()
            # self._last_time update removed

            return SafetyDecision(
                safe=False,
                new_state=ControllerState.MPC_DEGRADED,
                reason="NaN/Inf detected in control command",
                limited_cmd=zero_cmd
            )

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
        
        # 检查加速度 (使用传入的 dt，避免时间源不一致)
        # current_time = get_monotonic_time() # Removed internal time
        if self._last_cmd is not None:
             # dt is passed in argument

            
            # 初始化加速度值为 0 或上次的值
            ax, ay, az, alpha = 0.0, 0.0, 0.0, 0.0
            should_check_accel = False
            
            # 获取当前滤波器状态 (如果有)
            if hasattr(self, '_last_filtered_accel'):
                ax, ay, az, alpha = self._last_filtered_accel
            
            # 策略：
            # 1. 正常范围: 更新滤波器并检查
            # 2. 过小 (< min): 不更新滤波器（防止除零/噪声），但使用上次的值进行检查 (Zero-Order Hold)
            # 3. 过大 (> max): 可能是暂停或丢帧。不更新滤波器（防止极小加速度产生误导），也不检查（数据不可靠）
            
            # 策略优化:
            # 1. 正常范围 (dt >= min): 计算真实加速度并检查
            # 2. 过小 (dt < min): 可能是 Jitter 或离散更新。不计算加速度（数值不稳定），
            #    改为检查"速度跳变" (Velocity Jump)。如果 dv 超过了允许的瞬时突变量，则视为不安全。
            # 3. 过大 (dt > max): 可能是暂停或丢帧。不进行检查，但重置滤波器，避免历史数据污染
            
            if dt >= self.min_dt_for_accel and dt <= self.max_dt_for_accel:
                # 正常计算
                raw_ax = (cmd.vx - self._last_cmd.vx) / dt
                raw_ay = (cmd.vy - self._last_cmd.vy) / dt
                raw_az = (cmd.vz - self._last_cmd.vz) / dt if self.is_3d else 0.0
                raw_alpha = (cmd.omega - self._last_cmd.omega) / dt
                
                # 更新滤波器
                ax, ay, az, alpha = self._filter_acceleration(raw_ax, raw_ay, raw_az, raw_alpha)
                # 保存状态
                self._last_filtered_accel = (ax, ay, az, alpha)
                should_check_accel = True
                
            elif dt < self.min_dt_for_accel:
                # dt 过小 (Jitter/High-freq update)
                # 风险: 直接计算 a = dv/dt 会导致误报 (False Positive)
                # 风险: 如果沿用上次加速度，会漏报巨大的阶跃 (False Negative) - "Old Value Reuse" Bug
                # 解决方案: 检查速度增量 dv 是否超过 "最大允许跳变"
                # 最大允许跳变 = a_max * (min_dt_for_accel 或 一个合理的最小控制周期)
                # 这里我们使用 min_dt_for_accel 作为参考时间尺度
                
                # 计算速度增量范数
                dvx = cmd.vx - self._last_cmd.vx
                dvy = cmd.vy - self._last_cmd.vy
                dv_horiz = np.sqrt(dvx**2 + dvy**2)
                
                # 允许的跳变阈值 (放宽一点裕度，防止临界值误触)
                # 逻辑: 即使在极短时间内，速度变化也不应超过机器人全速加速 min_dt 既然的变化量
                # 使用 safety margins 放大一点
                jump_limit_horiz = self.a_max * self.min_dt_for_accel * 1.5
                
                if dv_horiz > jump_limit_horiz and dv_horiz > EPSILON:
                     reasons.append(f"Velocity jump detected (dt={dt*1000:.2f}ms): dv={dv_horiz:.2f} > limit {jump_limit_horiz:.2f}")
                     # 限制速度跳变
                     scale = jump_limit_horiz / dv_horiz
                     limited_cmd.vx = self._last_cmd.vx + dvx * scale
                     limited_cmd.vy = self._last_cmd.vy + dvy * scale
                     needs_limiting = True
                
                # 角速度跳变检查
                domega = abs(cmd.omega - self._last_cmd.omega)
                jump_limit_omega = self.alpha_max * self.min_dt_for_accel * 1.5
                
                if domega > jump_limit_omega:
                    reasons.append(f"Omega jump detected: domega={domega:.2f} > limit {jump_limit_omega:.2f}")
                    limited_cmd.omega = self._last_cmd.omega + np.sign(cmd.omega - self._last_cmd.omega) * jump_limit_omega
                    needs_limiting = True

                # 对于极小 dt，我们不更新滤波器（避免数值污染），也不进行常规加速度检查
                should_check_accel = False
            
            else:
                # dt 过大，重置或跳过
                logger.debug(f"dt too large ({dt:.3f}s), skipping accel check and resetting filter")
                # 重置滤波器以防下次使用过时的历史数据
                self._accel_history_x.clear()
                self._accel_history_y.clear()
                self._accel_history_z.clear()
                self._alpha_history.clear()
                self._filtered_ax = None # 标记为重新初始化
                should_check_accel = False

            if should_check_accel:
                # 预热期间使用更宽松的阈值
                if self.is_filter_warmed_up():
                    accel_margin_effective = self.accel_margin
                else:
                    effective_multiplier = min(self.accel_warmup_margin_multiplier, self.accel_warmup_margin_max)
                    accel_margin_effective = self.accel_margin * effective_multiplier
                
                # 绝对加速度上限检查 - 无论预热状态如何都执行
                accel_absolute_limit = self.a_max * self.accel_absolute_max_multiplier
                
                accel_horizontal = np.sqrt(ax**2 + ay**2)
                
                # 首先检查绝对上限（硬性限制）
                if accel_horizontal > accel_absolute_limit:
                    reasons.append(f"a_horizontal {accel_horizontal:.2f} exceeds absolute limit {accel_absolute_limit:.2f}")
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
                alpha_absolute_limit = self.alpha_max * self.accel_absolute_max_multiplier
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
        # self._last_time = current_time # Removed

        
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
        # self._last_time = None # Removed

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
