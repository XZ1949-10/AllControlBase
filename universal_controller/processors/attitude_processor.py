"""
无人机姿态内环处理器 (F14 Refactored)

实现 IControlProcessor 接口，为四旋翼平台提供姿态控制功能。
"""
from typing import Dict, Any, Optional, Tuple
import numpy as np

from ..core.interfaces import IControlProcessor
from ..core.data_types import ControlOutput, AttitudeCommand
from ..core.ros_compat import get_monotonic_time, normalize_angle
from ..core.constants import (
    EPSILON,
    DEFAULT_GRAVITY,
    ATTITUDE_MIN_THRUST_FACTOR,
    ATTITUDE_FACTOR_MIN,
)


class AttitudeProcessor(IControlProcessor):
    """
    四旋翼无人机姿态处理器
    
    将世界坐标系下的速度命令转换为姿态命令 (roll, pitch, yaw, thrust)，
    并将其作为 extra output 返回。
    """
    
    # 数值稳定性常量
    MIN_COS_ROLL_THRESH = 0.1
    LOW_THRUST_ATTITUDE_SIN_LIMIT = 0.5
    
    def __init__(self, config: Dict[str, Any]):
        attitude_config = config.get('attitude', {})
        
        # 物理参数
        self.mass = attitude_config.get('mass', 1.5)
        self.gravity = DEFAULT_GRAVITY
        
        # 姿态角速度限制
        self.roll_rate_max = attitude_config.get('roll_rate_max', 3.0)
        self.pitch_rate_max = attitude_config.get('pitch_rate_max', 3.0)
        self.yaw_rate_max = attitude_config.get('yaw_rate_max', 2.0)
        
        # 姿态角限制
        self.roll_max = attitude_config.get('roll_max', 0.5)
        self.pitch_max = attitude_config.get('pitch_max', 0.5)
        
        # 速度控制增益
        self.kp_vx = attitude_config.get('kp_vx', 0.5)
        self.kp_vy = attitude_config.get('kp_vy', 0.5)
        self.kp_vz = attitude_config.get('kp_vz', 1.0)
        
        # 悬停 yaw 漂移补偿
        self.hover_yaw_compensation_enabled = attitude_config.get(
            'hover_yaw_compensation', True)
        self.hover_speed_thresh = attitude_config.get('hover_speed_thresh', 0.1)
        self.hover_vz_thresh = attitude_config.get('hover_vz_thresh', 0.05)
        self._hover_yaw: Optional[float] = None
        self._hover_start_time: Optional[float] = None
        self._yaw_drift_rate = attitude_config.get('yaw_drift_rate', 0.001)
        self._accumulated_yaw_drift = 0.0
        
        # 悬停检测滞后参数
        self.hover_enter_factor = attitude_config.get('hover_enter_factor', 1.0)
        self.hover_exit_factor = attitude_config.get('hover_exit_factor', 1.5)
        self.hover_cmd_exit_factor = attitude_config.get('hover_cmd_exit_factor', 2.0)
        self.hover_debounce_time = attitude_config.get('hover_debounce_time', 0.1)
        
        self.position_attitude_decoupled = attitude_config.get(
            'position_attitude_decoupled', False)
        
        self.thrust_min = attitude_config.get('thrust_min', 0.1)
        self.thrust_max = attitude_config.get('thrust_max', 2.0)
        self.thrust_rate_max = attitude_config.get('thrust_rate_max', 2.0)
        
        self.min_thrust_factor = ATTITUDE_MIN_THRUST_FACTOR
        self.attitude_factor_min = ATTITUDE_FACTOR_MIN
        
        self.invert_pitch_sign = attitude_config.get('invert_pitch_sign', True)
        
        # 状态
        self._last_attitude: Optional[AttitudeCommand] = None
        self._last_time: Optional[float] = None
        self.dt = attitude_config.get('dt', 0.02)
        
        self._hover_debounce_start: Optional[float] = None
        self._hover_debounce_target: Optional[bool] = None
        
        # 用于参数获取的默认航向模式
        self._default_yaw_mode = 'velocity'

    def reset(self) -> None:
        self._last_attitude = None
        self._last_time = None
        self._hover_yaw = None
        self._hover_start_time = None
        self._accumulated_yaw_drift = 0.0
        self._hover_debounce_start = None
        self._hover_debounce_target = None

    def compute_extra(self, state: np.ndarray, cmd: ControlOutput) -> Dict[str, Any]:
        """
        计算额外的控制输出 (实现 IControlProcessor 接口)
        """
        if state is None or len(state) < 7:
            return {}
            
        # 确定 yaw mode (可以从 cmd.extras 或 config 获取，暂时默认为 velocity，
        # 实际逻辑可能需要从 ControllerNode 通过某种方式传递，但为了简化，
        # 我们假设 yaw_mode 在 velocity control 下主要是 velocity 或 fixed)
        # 
        # TODO: 如果需要支持 manual 等特定模式，可以通过 cmd.extras 传递参数
        yaw_mode = cmd.extras.get('yaw_mode', self._default_yaw_mode)
        
        attitude_cmd = self.compute_attitude(cmd, state, yaw_mode)
        
        return {
            'attitude_cmd': attitude_cmd,
            'is_hovering': self.is_hovering
        }

    # ==================== 原有逻辑 ====================

    def compute_attitude(self, velocity_cmd: ControlOutput,
                         current_state: np.ndarray,
                         yaw_mode: str = 'velocity') -> AttitudeCommand:
        """计算姿态命令 (内部逻辑保持不变)"""
        # 输入验证
        if velocity_cmd is None:
            raise ValueError("velocity_cmd cannot be None")
        if current_state is None or len(current_state) < 7:
            raise ValueError("current_state must have at least 7 elements")
        
        current_time = get_monotonic_time()
        
        # 当前状态
        vx_current = current_state[3]
        vy_current = current_state[4]
        vz_current = current_state[5]
        # yaw_current = current_state[6]
        
        # 速度误差
        vx_error = velocity_cmd.vx - vx_current
        vy_error = velocity_cmd.vy - vy_current
        vz_error = velocity_cmd.vz - vz_current
        
        # 计算期望加速度
        ax_desired = self.kp_vx * vx_error
        ay_desired = self.kp_vy * vy_error
        az_desired = self.kp_vz * vz_error
        
        # 计算目标 yaw
        yaw_desired = self._compute_yaw(
            velocity_cmd, current_state, yaw_mode, current_time)
        
        # 计算姿态和推力
        roll_desired, pitch_desired, thrust = self._compute_attitude_and_thrust(
            ax_desired, ay_desired, az_desired, yaw_desired
        )
        
        # 限制和后处理
        roll_desired = np.clip(roll_desired, -self.roll_max, self.roll_max)
        pitch_desired = np.clip(pitch_desired, -self.pitch_max, self.pitch_max)
        
        thrust = self._recompute_thrust_after_saturation(
            ax_desired, ay_desired, az_desired,
            roll_desired, pitch_desired, yaw_desired
        )
        
        thrust = np.clip(thrust, self.thrust_min, self.thrust_max)
        
        attitude_cmd = self._apply_rate_limits(
            roll_desired, pitch_desired, yaw_desired, thrust, current_time)
        
        self._last_attitude = attitude_cmd
        self._last_time = current_time
        
        return attitude_cmd

    def _compute_attitude_and_thrust(self, ax: float, ay: float, az: float,
                                      yaw: float) -> tuple:
        az_total = az + self.gravity
        az_total = max(az_total, self.min_thrust_factor * self.gravity)
        thrust_accel = np.sqrt(ax**2 + ay**2 + az_total**2)
        thrust = thrust_accel / self.gravity
        
        cos_yaw = np.cos(yaw)
        sin_yaw = np.sin(yaw)
        
        ax_body = ax * cos_yaw + ay * sin_yaw
        ay_body = -ax * sin_yaw + ay * cos_yaw
        
        MIN_THRUST_ACCEL = self.min_thrust_factor * self.gravity
        if thrust_accel < MIN_THRUST_ACCEL:
            if thrust_accel > EPSILON:
                sin_limit = self.LOW_THRUST_ATTITUDE_SIN_LIMIT
                roll = np.arcsin(np.clip(-ay_body / self.gravity, -sin_limit, sin_limit))
                pitch = -np.arcsin(np.clip(ax_body / self.gravity, -sin_limit, sin_limit))
            else:
                roll = 0.0
                pitch = 0.0
            return roll, pitch, thrust
        
        sin_roll = -ay_body / thrust_accel
        sin_roll = np.clip(sin_roll, -1.0, 1.0)
        roll = np.arcsin(sin_roll)
        cos_roll = np.cos(roll)
        
        if abs(cos_roll) > self.MIN_COS_ROLL_THRESH:
            denominator = thrust_accel * cos_roll
            sin_pitch = ax_body / denominator
            cos_pitch = az_total / denominator
            sin_pitch = np.clip(sin_pitch, -1.0, 1.0)
            cos_pitch = np.clip(cos_pitch, -1.0, 1.0)
            pitch_raw = np.arctan2(sin_pitch, cos_pitch)
            pitch = -pitch_raw if self.invert_pitch_sign else pitch_raw
        else:
            pitch = 0.0
        
        return roll, pitch, thrust

    def _recompute_thrust_after_saturation(self, ax: float, ay: float, az: float,
                                            roll: float, pitch: float,
                                            yaw: float) -> float:
        az_total = az + self.gravity
        az_total = max(az_total, self.min_thrust_factor * self.gravity)
        
        cos_roll = np.cos(roll)
        cos_pitch = np.cos(pitch)
        attitude_factor = cos_roll * cos_pitch
        
        if attitude_factor > self.attitude_factor_min:
            thrust = az_total / (attitude_factor * self.gravity)
        else:
            thrust_magnitude = np.sqrt(ax**2 + ay**2 + az_total**2)
            thrust = thrust_magnitude / self.gravity
        
        return thrust

    def _compute_yaw(self, velocity_cmd: ControlOutput, current_state: np.ndarray,
                     yaw_mode: str, current_time: float) -> float:
        theta_current = current_state[6]
        v_horizontal_current = np.sqrt(current_state[3]**2 + current_state[4]**2)
        v_horizontal_cmd = np.sqrt(velocity_cmd.vx**2 + velocity_cmd.vy**2)
        vz_current = abs(current_state[5]) if len(current_state) > 5 else 0.0
        vz_cmd = abs(velocity_cmd.vz)
        
        is_hovering = self._update_hover_state(
            v_horizontal_current, v_horizontal_cmd, vz_current, vz_cmd, current_time)
        
        if is_hovering and self.hover_yaw_compensation_enabled:
            yaw_with_compensation = self._apply_hover_yaw_compensation(
                theta_current, current_time)
            if yaw_with_compensation is not None:
                return yaw_with_compensation
        else:
            self._reset_hover_state()
            
        if yaw_mode == 'velocity':
            if v_horizontal_current > self.hover_speed_thresh:
                return np.arctan2(current_state[4], current_state[3])
            return theta_current
        elif yaw_mode == 'fixed':
            return theta_current
        elif yaw_mode == 'manual':
            if self._last_time is not None:
                dt_actual = current_time - self._last_time
                if 0 < dt_actual < 1.0:
                    return theta_current + velocity_cmd.omega * dt_actual
            return theta_current + velocity_cmd.omega * self.dt
        return theta_current

    def _update_hover_state(self, v_horizontal_current: float, v_horizontal_cmd: float,
                           vz_current: float, vz_cmd: float, 
                           current_time: float) -> bool:
        should_enter_hover = (
            v_horizontal_current < self.hover_speed_thresh * self.hover_enter_factor and
            vz_current < self.hover_vz_thresh * self.hover_enter_factor and
            v_horizontal_cmd < self.hover_speed_thresh * self.hover_enter_factor and
            vz_cmd < self.hover_vz_thresh * self.hover_enter_factor
        )
        should_exit_hover = (
            v_horizontal_current > self.hover_speed_thresh * self.hover_exit_factor or
            vz_current > self.hover_vz_thresh * self.hover_exit_factor or
            v_horizontal_cmd > self.hover_speed_thresh * self.hover_cmd_exit_factor or
            vz_cmd > self.hover_vz_thresh * self.hover_cmd_exit_factor
        )
        
        currently_hovering = self._hover_start_time is not None
        if currently_hovering:
            return self._handle_hover_exit(should_exit_hover, current_time)
        else:
            return self._handle_hover_enter(should_enter_hover, current_time)

    def _handle_hover_exit(self, should_exit: bool, current_time: float) -> bool:
        if should_exit:
            if self._hover_debounce_target is not False:
                self._hover_debounce_start = current_time
                self._hover_debounce_target = False
                return True
            elif (self._hover_debounce_start is not None and 
                  current_time - self._hover_debounce_start >= self.hover_debounce_time):
                return False
            else:
                return True
        else:
            self._hover_debounce_start = None
            self._hover_debounce_target = None
            return True

    def _handle_hover_enter(self, should_enter: bool, current_time: float) -> bool:
        if should_enter:
            if self._hover_debounce_target is not True:
                self._hover_debounce_start = current_time
                self._hover_debounce_target = True
                return False
            elif (self._hover_debounce_start is not None and 
                  current_time - self._hover_debounce_start >= self.hover_debounce_time):
                return True
            else:
                return False
        else:
            self._hover_debounce_start = None
            self._hover_debounce_target = None
            return False

    def _apply_hover_yaw_compensation(self, theta_current: float, 
                                      current_time: float) -> Optional[float]:
        if self._hover_start_time is None:
            self._hover_start_time = current_time
            self._hover_yaw = theta_current
            self._accumulated_yaw_drift = 0.0
        else:
            hover_duration = current_time - self._hover_start_time
            self._accumulated_yaw_drift = self._yaw_drift_rate * hover_duration
        
        if self._hover_yaw is not None:
            return self._hover_yaw - self._accumulated_yaw_drift
        return None

    def _reset_hover_state(self) -> None:
        self._hover_start_time = None
        self._hover_yaw = None
        self._accumulated_yaw_drift = 0.0

    def _apply_rate_limits(self, roll: float, pitch: float, yaw: float,
                           thrust: float, current_time: float) -> AttitudeCommand:
        if self._last_attitude is None or self._last_time is None:
            return AttitudeCommand(roll=roll, pitch=pitch, yaw=yaw, thrust=thrust)
        
        dt = current_time - self._last_time
        if dt <= 0 or dt > 1.0:
            dt = self.dt
        
        max_roll_change = self.roll_rate_max * dt
        max_pitch_change = self.pitch_rate_max * dt
        max_yaw_change = self.yaw_rate_max * dt
        
        roll_limited = np.clip(roll, self._last_attitude.roll - max_roll_change, self._last_attitude.roll + max_roll_change)
        pitch_limited = np.clip(pitch, self._last_attitude.pitch - max_pitch_change, self._last_attitude.pitch + max_pitch_change)
        
        yaw_error = normalize_angle(yaw - self._last_attitude.yaw)
        yaw_error_limited = np.clip(yaw_error, -max_yaw_change, max_yaw_change)
        yaw_limited = normalize_angle(self._last_attitude.yaw + yaw_error_limited)
        
        max_thrust_change = self.thrust_rate_max * dt
        thrust_limited = np.clip(thrust, self._last_attitude.thrust - max_thrust_change, self._last_attitude.thrust + max_thrust_change)
        
        return AttitudeCommand(roll=roll_limited, pitch=pitch_limited, yaw=yaw_limited, thrust=thrust_limited)

    # ==================== 接口扩展 ====================
    
    def set_hover_yaw(self, yaw: float) -> None:
        """设置悬停时的目标航向 (供外部调用)"""
        self._hover_yaw = yaw
        self._hover_start_time = get_monotonic_time()
        self._accumulated_yaw_drift = 0.0

    def get_attitude_rate_limits(self) -> Dict[str, float]:
        """获取姿态角速度限制 (供外部调用)"""
        return {
            'roll_rate_max': self.roll_rate_max,
            'pitch_rate_max': self.pitch_rate_max,
            'yaw_rate_max': self.yaw_rate_max
        }
        
    @property
    def is_hovering(self) -> bool:
        return self._hover_start_time is not None
