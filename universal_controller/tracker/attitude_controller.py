"""
无人机姿态内环控制器 (F14)

实现功能:
- F14.1: 独立的姿态内环接口
- F14.2: attitude rate 限制
- F14.3: 悬停时 yaw 漂移补偿
- F14.4: 位置-姿态解耦控制选项

坐标系约定:
- 世界坐标系: X-前(北), Y-左(西), Z-上 (NWU，类似 ENU 但 Y 轴相反)
  注意: 如果使用 ENU (X-东, Y-北, Z-上)，需要调整速度命令的解释
- 机体坐标系: X-前, Y-左, Z-上 (FLU)
- 欧拉角约定: ZYX (先 yaw, 再 pitch, 最后 roll)

姿态角符号约定 (与 PX4/ArduPilot 等常见飞控一致):
  - roll (φ):
    - 正 roll (φ > 0): 右翼向下，机体向右倾斜
    - 产生右向加速度 (世界坐标系 -Y 方向，如果 Y 轴向左)
  - pitch (θ):
    - 正 pitch (θ > 0): 机头向上，机体后仰
    - 负 pitch (θ < 0): 机头向下，机体前倾 -> 产生前向加速度
    - 注意: 本控制器输出的 pitch 符号已调整，使得正 pitch 命令产生前向加速度
      这与某些飞控的约定相反，使用时需要注意接口适配
  - yaw (ψ):
    - 正 yaw (ψ > 0): 机头向左旋转 (从上方看逆时针)

物理模型说明 (完整推导，无简化):
四旋翼在世界坐标系下的动力学方程为:
    m * a_world = R_body_to_world @ [0, 0, T]^T + [0, 0, -m*g]^T

其中:
- T 是总推力 (沿机体 z 轴向上)
- R_body_to_world 是从机体坐标系到世界坐标系的旋转矩阵
- 使用 ZYX 欧拉角约定: 先 yaw(ψ), 再 pitch(θ), 最后 roll(φ)

旋转矩阵 R_body_to_world (ZYX 顺序) 的第三列 (推力方向):
    R[:, 2] = [cos(ψ)*sin(θ)*cos(φ) + sin(ψ)*sin(φ),
               sin(ψ)*sin(θ)*cos(φ) - cos(ψ)*sin(φ),
               cos(θ)*cos(φ)]

展开后的加速度方程:
    a_x = T/m * (cos(ψ)*sin(θ)*cos(φ) + sin(ψ)*sin(φ))
    a_y = T/m * (sin(ψ)*sin(θ)*cos(φ) - cos(ψ)*sin(φ))
    a_z = T/m * cos(θ)*cos(φ) - g

给定期望加速度 (ax_d, ay_d, az_d) 和航向角 ψ，精确逆解 (φ, θ, T):

1. 定义 az_total = az_d + g
2. 计算推力大小: T/m = sqrt(ax_d^2 + ay_d^2 + az_total^2)
3. 转换到机体坐标系 (绕 z 轴旋转 -ψ):
   ax_body = ax_d*cos(ψ) + ay_d*sin(ψ) = T/m * sin(θ)*cos(φ)
   ay_body = -ax_d*sin(ψ) + ay_d*cos(ψ) = -T/m * sin(φ)
4. 精确求解姿态角:
   sin(φ) = -ay_body / (T/m)
   φ = arcsin(sin(φ))
   sin(θ) = ax_body / (T/m * cos(φ))
   cos(θ) = az_total / (T/m * cos(φ))
   θ_raw = atan2(sin(θ), cos(θ))
   θ = -θ_raw  (取负以匹配控制约定: 正 pitch 命令 = 前倾 = 前向加速度)
   
   注意: 这里的符号调整是为了使控制接口更直观:
   - 正的 pitch 命令 -> 产生正的前向加速度
   - 这与标准 ZYX 欧拉角约定相反（标准约定中正 pitch = 机头向上）
   - 如果需要与标准约定一致，可以在配置中禁用此调整
"""
from typing import Dict, Any, Optional
import numpy as np

from ..core.interfaces import IAttitudeController
from ..core.data_types import ControlOutput, AttitudeCommand
from ..core.ros_compat import get_monotonic_time, normalize_angle


class QuadrotorAttitudeController(IAttitudeController):
    """
    四旋翼无人机姿态控制器
    
    将世界坐标系下的速度命令转换为姿态命令 (roll, pitch, yaw, thrust)
    
    使用完整的四旋翼动力学模型进行精确计算。
    """
    
    # 数值稳定性常量
    MIN_COS_ROLL_THRESH = 0.1  # cos(roll) 最小阈值，用于避免奇异情况
    
    def __init__(self, config: Dict[str, Any]):
        attitude_config = config.get('attitude', {})
        
        # 物理参数
        self.mass = attitude_config.get('mass', 1.5)  # kg
        # 重力加速度 - 优先从 system.gravity 读取
        self.gravity = config.get('system', {}).get('gravity', 
                        attitude_config.get('gravity', 9.81))
        
        # 姿态角速度限制 (F14.2)
        self.roll_rate_max = attitude_config.get('roll_rate_max', 3.0)  # rad/s
        self.pitch_rate_max = attitude_config.get('pitch_rate_max', 3.0)  # rad/s
        self.yaw_rate_max = attitude_config.get('yaw_rate_max', 2.0)  # rad/s
        
        # 姿态角限制
        self.roll_max = attitude_config.get('roll_max', 0.5)  # rad (~30 deg)
        self.pitch_max = attitude_config.get('pitch_max', 0.5)  # rad (~30 deg)
        
        # 速度控制增益
        self.kp_vx = attitude_config.get('kp_vx', 0.5)
        self.kp_vy = attitude_config.get('kp_vy', 0.5)
        self.kp_vz = attitude_config.get('kp_vz', 1.0)
        
        # 悬停 yaw 漂移补偿 (F14.3)
        self.hover_yaw_compensation_enabled = attitude_config.get(
            'hover_yaw_compensation', True)
        self.hover_speed_thresh = attitude_config.get('hover_speed_thresh', 0.1)
        # 垂直速度悬停阈值通常应该更小，因为垂直运动对悬停状态影响更大
        self.hover_vz_thresh = attitude_config.get('hover_vz_thresh', 0.05)
        self._hover_yaw: Optional[float] = None
        self._hover_start_time: Optional[float] = None
        self._yaw_drift_rate = attitude_config.get('yaw_drift_rate', 0.001)
        self._accumulated_yaw_drift = 0.0
        
        # 悬停检测滞后参数 - 从配置读取
        self.hover_enter_factor = attitude_config.get('hover_enter_factor', 1.0)
        self.hover_exit_factor = attitude_config.get('hover_exit_factor', 1.5)
        self.hover_cmd_exit_factor = attitude_config.get('hover_cmd_exit_factor', 2.0)
        self.hover_debounce_time = attitude_config.get('hover_debounce_time', 0.1)
        
        # 位置-姿态解耦 (F14.4)
        self.position_attitude_decoupled = attitude_config.get(
            'position_attitude_decoupled', False)
        
        # 推力归一化参数 (相对于悬停推力 m*g)
        self.thrust_min = attitude_config.get('thrust_min', 0.1)
        self.thrust_max = attitude_config.get('thrust_max', 2.0)
        
        # 推力变化率限制 (每秒相对于悬停推力的变化)
        self.thrust_rate_max = attitude_config.get('thrust_rate_max', 2.0)
        
        # 最小推力加速度因子 (相对于重力)
        self.min_thrust_factor = attitude_config.get('min_thrust_factor', 0.1)
        
        # 姿态角饱和后推力重计算的最小因子
        self.attitude_factor_min = attitude_config.get('attitude_factor_min', 0.1)
        
        # pitch 符号反转选项
        # True (默认): 正 pitch 命令 -> 前向加速度 (更直观的控制接口)
        # False: 标准 ZYX 欧拉角约定 (正 pitch = 机头向上)
        self.invert_pitch_sign = attitude_config.get('invert_pitch_sign', True)
        
        # 状态
        self._last_attitude: Optional[AttitudeCommand] = None
        self._last_time: Optional[float] = None
        self.dt = attitude_config.get('dt', 0.02)
        
        # 悬停去抖动状态
        self._hover_debounce_start: Optional[float] = None
        self._hover_debounce_target: Optional[bool] = None

    def compute_attitude(self, velocity_cmd: ControlOutput,
                         current_state: np.ndarray,
                         yaw_mode: str = 'velocity') -> AttitudeCommand:
        """
        计算姿态命令
        
        使用完整的四旋翼动力学模型，将期望加速度转换为姿态和推力命令。
        
        Args:
            velocity_cmd: 速度命令 (世界坐标系)
            current_state: 当前状态 [px, py, pz, vx, vy, vz, theta, omega]
            yaw_mode: 航向模式 ('velocity', 'fixed', 'manual')
        
        Returns:
            AttitudeCommand: 姿态命令 (roll, pitch, yaw, thrust)
        
        Raises:
            ValueError: 如果输入参数无效
        """
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
        yaw_current = current_state[6]
        
        # 速度误差
        vx_error = velocity_cmd.vx - vx_current
        vy_error = velocity_cmd.vy - vy_current
        vz_error = velocity_cmd.vz - vz_current
        
        # 计算期望加速度 (世界坐标系)
        ax_desired = self.kp_vx * vx_error
        ay_desired = self.kp_vy * vy_error
        az_desired = self.kp_vz * vz_error  # 重力在推力计算中补偿
        
        # 计算目标 yaw
        yaw_desired = self._compute_yaw(
            velocity_cmd, current_state, yaw_mode, current_time)
        
        # 使用完整动力学模型计算姿态和推力
        roll_desired, pitch_desired, thrust = self._compute_attitude_and_thrust(
            ax_desired, ay_desired, az_desired, yaw_desired
        )
        
        # 限制姿态角
        roll_desired = np.clip(roll_desired, -self.roll_max, self.roll_max)
        pitch_desired = np.clip(pitch_desired, -self.pitch_max, self.pitch_max)
        
        # 如果姿态角被限制，需要重新计算推力以保持垂直加速度
        thrust = self._recompute_thrust_after_saturation(
            ax_desired, ay_desired, az_desired,
            roll_desired, pitch_desired, yaw_desired
        )
        
        # 限制推力范围
        thrust = np.clip(thrust, self.thrust_min, self.thrust_max)
        
        # 应用姿态角速度限制 (F14.2)
        attitude_cmd = self._apply_rate_limits(
            roll_desired, pitch_desired, yaw_desired, thrust, current_time)
        
        self._last_attitude = attitude_cmd
        self._last_time = current_time
        
        return attitude_cmd

    def _compute_attitude_and_thrust(self, ax: float, ay: float, az: float,
                                      yaw: float) -> tuple:
        """
        使用完整动力学模型计算姿态角和推力 (无简化)
        
        基于四旋翼动力学方程的精确逆解:
        给定期望加速度 (ax, ay, az) 和航向角 yaw，求解 roll, pitch, thrust
        
        完整物理模型 (ZYX 欧拉角约定):
        旋转矩阵 R_body_to_world 的第三列 (推力方向在世界坐标系的投影):
            R[:, 2] = [cos(ψ)*sin(θ)*cos(φ) + sin(ψ)*sin(φ),
                       sin(ψ)*sin(θ)*cos(φ) - cos(ψ)*sin(φ),
                       cos(θ)*cos(φ)]
        
        加速度方程:
            ax = T/m * (cos(ψ)*sin(θ)*cos(φ) + sin(ψ)*sin(φ))
            ay = T/m * (sin(ψ)*sin(θ)*cos(φ) - cos(ψ)*sin(φ))
            az = T/m * cos(θ)*cos(φ) - g
        
        逆解过程:
        1. 定义 az_total = az + g = T/m * cos(θ)*cos(φ)
        2. 将世界坐标系加速度转换到以 yaw 为参考的中间坐标系:
           ax_body = ax*cos(ψ) + ay*sin(ψ) = T/m * sin(θ)*cos(φ)
           ay_body = -ax*sin(ψ) + ay*cos(ψ) = T/m * (-sin(φ))
        3. 从 ay_body 和 az_total 求 roll:
           tan(φ) = -ay_body / (az_total / cos(θ))
           但 cos(θ) 未知，需要迭代或使用近似
        4. 精确解法: 使用推力向量的几何关系
        
        Args:
            ax, ay, az: 期望加速度 (世界坐标系, m/s^2)
            yaw: 目标航向角 (rad)
        
        Returns:
            (roll, pitch, thrust): 姿态角 (rad) 和归一化推力 (相对于悬停推力)
        """
        # 总推力向量的垂直分量 (世界坐标系)
        az_total = az + self.gravity  # 补偿重力
        
        # 防止 az_total 过小 (避免倒飞或自由落体)
        # 使用配置的最小推力因子
        az_total = max(az_total, self.min_thrust_factor * self.gravity)
        
        # 总推力大小 (加速度量纲)
        # T/m = sqrt(ax^2 + ay^2 + az_total^2)
        thrust_accel = np.sqrt(ax**2 + ay**2 + az_total**2)
        
        # 归一化推力 (相对于悬停推力 m*g)
        thrust = thrust_accel / self.gravity
        
        # 将世界坐标系加速度转换到以 yaw 为参考的中间坐标系
        # 这相当于绕 z 轴旋转 -yaw (从世界坐标系到机体 yaw 对齐坐标系)
        cos_yaw = np.cos(yaw)
        sin_yaw = np.sin(yaw)
        
        # ax_body = T/m * sin(θ)*cos(φ)
        # ay_body = T/m * (-sin(φ)) = -T/m * sin(φ)
        ax_body = ax * cos_yaw + ay * sin_yaw
        ay_body = -ax * sin_yaw + ay * cos_yaw
        
        # ========== 精确求解 roll 和 pitch ==========
        # 
        # 已知:
        #   ax_body = T/m * sin(θ) * cos(φ)
        #   ay_body = -T/m * sin(φ)
        #   az_total = T/m * cos(θ) * cos(φ)
        #   thrust_accel = T/m = sqrt(ax_body^2 + ay_body^2 + az_total^2)
        #
        # 从 ay_body 求 sin(φ):
        #   sin(φ) = -ay_body / (T/m)
        #
        # 从 az_total 和 ax_body 求 θ:
        #   tan(θ) = ax_body / az_total  (当 cos(φ) ≠ 0)
        #   或者使用: sin(θ) = ax_body / (T/m * cos(φ))
        
        # 防止除以零和数值不稳定
        # 当推力加速度很小时（接近自由落体或悬停），使用保守的姿态
        MIN_THRUST_ACCEL = self.min_thrust_factor * self.gravity  # 使用配置的最小推力因子
        if thrust_accel < MIN_THRUST_ACCEL:
            # 推力很小，使用简化计算
            # 假设小姿态角近似: roll ≈ -ay/g, pitch ≈ ax/g
            if thrust_accel > 1e-6:
                roll = np.arcsin(np.clip(-ay_body / self.gravity, -0.5, 0.5))
                pitch = -np.arcsin(np.clip(ax_body / self.gravity, -0.5, 0.5))
            else:
                roll = 0.0
                pitch = 0.0
            return roll, pitch, thrust
        
        # 精确计算 roll (φ)
        # sin(φ) = -ay_body / thrust_accel
        sin_roll = -ay_body / thrust_accel
        sin_roll = np.clip(sin_roll, -1.0, 1.0)  # 数值稳定性
        roll = np.arcsin(sin_roll)
        
        # 计算 cos(φ)
        cos_roll = np.cos(roll)
        
        # 精确计算 pitch (θ)
        # 使用: ax_body = thrust_accel * sin(θ) * cos(φ)
        #       az_total = thrust_accel * cos(θ) * cos(φ)
        # 
        # 当 cos(φ) ≠ 0 时:
        #   tan(θ) = ax_body / az_total
        #   θ = atan2(ax_body, az_total)
        #
        # 当 cos(φ) ≈ 0 时 (roll ≈ ±90°，奇异情况):
        #   此时 ax_body ≈ 0, az_total ≈ 0，pitch 不确定
        #   使用备用计算方法
        
        if abs(cos_roll) > self.MIN_COS_ROLL_THRESH:
            # 正常情况: 使用精确公式
            # sin(θ) = ax_body / (thrust_accel * cos(φ))
            # cos(θ) = az_total / (thrust_accel * cos(φ))
            # 使用更保守的阈值 0.1 避免数值不稳定
            denominator = thrust_accel * cos_roll
            sin_pitch = ax_body / denominator
            cos_pitch = az_total / denominator
            
            # 数值稳定性处理
            sin_pitch = np.clip(sin_pitch, -1.0, 1.0)
            cos_pitch = np.clip(cos_pitch, -1.0, 1.0)
            
            # 使用 atan2 获得正确象限
            pitch_raw = np.arctan2(sin_pitch, cos_pitch)
            
            # 根据配置决定是否反转 pitch 符号
            # 反转后: 正 pitch 命令 -> 前倾 -> 前向加速度 (更直观)
            # 不反转: 标准 ZYX 约定，正 pitch = 机头向上
            if self.invert_pitch_sign:
                pitch = -pitch_raw
            else:
                pitch = pitch_raw
        else:
            # 奇异情况: roll ≈ ±90°
            # 此时推力几乎完全在水平面内，pitch 的定义变得模糊
            # 使用简化计算: pitch = 0
            pitch = 0.0
        
        return roll, pitch, thrust

    def _recompute_thrust_after_saturation(self, ax: float, ay: float, az: float,
                                            roll: float, pitch: float,
                                            yaw: float) -> float:
        """
        当姿态角被限制后，重新计算推力以尽可能保持期望的垂直加速度
        
        由于水平加速度受限于姿态角限制，我们优先保证垂直加速度。
        
        Args:
            ax, ay, az: 原始期望加速度
            roll, pitch, yaw: 限制后的姿态角
        
        Returns:
            重新计算的归一化推力
        """
        az_total = az + self.gravity
        # 使用配置的最小推力因子
        az_total = max(az_total, self.min_thrust_factor * self.gravity)
        
        cos_roll = np.cos(roll)
        cos_pitch = np.cos(pitch)
        
        # 推力的垂直分量 = T * cos(roll) * cos(pitch)
        # 为了达到期望的垂直加速度: T * cos(roll) * cos(pitch) = az_total
        attitude_factor = cos_roll * cos_pitch
        
        if attitude_factor > self.attitude_factor_min:  # 使用配置的最小因子
            thrust = az_total / (attitude_factor * self.gravity)
        else:
            # 姿态角过大，使用总加速度计算
            thrust_magnitude = np.sqrt(ax**2 + ay**2 + az_total**2)
            thrust = thrust_magnitude / self.gravity
        
        return thrust

    def _compute_yaw(self, velocity_cmd: ControlOutput, current_state: np.ndarray,
                     yaw_mode: str, current_time: float) -> float:
        """
        计算目标 yaw 角
        
        Args:
            velocity_cmd: 速度命令
            current_state: 当前状态
            yaw_mode: 航向模式
            current_time: 当前时间
        
        Returns:
            目标 yaw 角 (rad)
        """
        theta_current = current_state[6]
        v_horizontal_current = np.sqrt(current_state[3]**2 + current_state[4]**2)
        v_horizontal_cmd = np.sqrt(velocity_cmd.vx**2 + velocity_cmd.vy**2)
        vz_current = abs(current_state[5]) if len(current_state) > 5 else 0.0
        vz_cmd = abs(velocity_cmd.vz)
        
        # 悬停检测
        is_hovering = self._update_hover_state(
            v_horizontal_current, v_horizontal_cmd, vz_current, vz_cmd, current_time)
        
        # 悬停 yaw 漂移补偿 (F14.3)
        if is_hovering and self.hover_yaw_compensation_enabled:
            yaw_with_compensation = self._apply_hover_yaw_compensation(
                theta_current, current_time)
            if yaw_with_compensation is not None:
                return yaw_with_compensation
        else:
            self._reset_hover_state()
        
        # 根据模式计算 yaw
        return self._compute_yaw_by_mode(
            yaw_mode, theta_current, v_horizontal_current, 
            current_state, velocity_cmd, current_time)
    
    def _update_hover_state(self, v_horizontal_current: float, v_horizontal_cmd: float,
                           vz_current: float, vz_cmd: float, 
                           current_time: float) -> bool:
        """
        更新悬停状态检测
        
        使用滞后逻辑和时间去抖动避免频繁切换
        
        Returns:
            是否处于悬停状态
        """
        # 判断是否应该进入悬停状态
        should_enter_hover = (
            v_horizontal_current < self.hover_speed_thresh * self.hover_enter_factor and
            vz_current < self.hover_vz_thresh * self.hover_enter_factor and
            v_horizontal_cmd < self.hover_speed_thresh * self.hover_enter_factor and
            vz_cmd < self.hover_vz_thresh * self.hover_enter_factor
        )
        
        # 判断是否应该退出悬停状态
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
        """处理悬停退出逻辑"""
        if should_exit:
            if self._hover_debounce_target != False:
                self._hover_debounce_start = current_time
                self._hover_debounce_target = False
                return True  # 去抖动期间保持悬停
            elif (self._hover_debounce_start is not None and 
                  current_time - self._hover_debounce_start >= self.hover_debounce_time):
                return False  # 确认退出悬停
            else:
                return True  # 去抖动中，保持悬停
        else:
            self._hover_debounce_start = None
            self._hover_debounce_target = None
            return True
    
    def _handle_hover_enter(self, should_enter: bool, current_time: float) -> bool:
        """处理悬停进入逻辑"""
        if should_enter:
            if self._hover_debounce_target != True:
                self._hover_debounce_start = current_time
                self._hover_debounce_target = True
                return False  # 去抖动期间保持非悬停
            elif (self._hover_debounce_start is not None and 
                  current_time - self._hover_debounce_start >= self.hover_debounce_time):
                return True  # 确认进入悬停
            else:
                return False  # 去抖动中，保持非悬停
        else:
            self._hover_debounce_start = None
            self._hover_debounce_target = None
            return False
    
    def _apply_hover_yaw_compensation(self, theta_current: float, 
                                      current_time: float) -> Optional[float]:
        """
        应用悬停 yaw 漂移补偿
        
        Returns:
            补偿后的 yaw 角，如果无法补偿则返回 None
        """
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
        """重置悬停状态"""
        self._hover_start_time = None
        self._hover_yaw = None
        self._accumulated_yaw_drift = 0.0
    
    def _compute_yaw_by_mode(self, yaw_mode: str, theta_current: float,
                            v_horizontal_current: float, current_state: np.ndarray,
                            velocity_cmd: ControlOutput, current_time: float) -> float:
        """根据模式计算 yaw"""
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

    def _apply_rate_limits(self, roll: float, pitch: float, yaw: float,
                           thrust: float, current_time: float) -> AttitudeCommand:
        """
        应用姿态角速度限制 (F14.2)
        
        Args:
            roll, pitch, yaw: 目标姿态角 (rad)
            thrust: 目标推力 (归一化)
            current_time: 当前时间
        
        Returns:
            限制后的姿态命令
        """
        if self._last_attitude is None or self._last_time is None:
            return AttitudeCommand(roll=roll, pitch=pitch, yaw=yaw, thrust=thrust)
        
        dt = current_time - self._last_time
        if dt <= 0 or dt > 1.0:  # 时间间隔异常，使用默认值
            dt = self.dt
        
        # 计算角速度限制
        max_roll_change = self.roll_rate_max * dt
        max_pitch_change = self.pitch_rate_max * dt
        max_yaw_change = self.yaw_rate_max * dt
        
        # 限制角度变化率
        roll_limited = np.clip(
            roll,
            self._last_attitude.roll - max_roll_change,
            self._last_attitude.roll + max_roll_change
        )
        pitch_limited = np.clip(
            pitch,
            self._last_attitude.pitch - max_pitch_change,
            self._last_attitude.pitch + max_pitch_change
        )
        
        # yaw 需要处理角度环绕 (-pi, pi]
        yaw_error = yaw - self._last_attitude.yaw
        yaw_error = normalize_angle(yaw_error)
        yaw_error_limited = np.clip(yaw_error, -max_yaw_change, max_yaw_change)
        yaw_limited = self._last_attitude.yaw + yaw_error_limited
        yaw_limited = normalize_angle(yaw_limited)
        
        # 推力变化率限制 (可配置)
        max_thrust_change = self.thrust_rate_max * dt
        thrust_limited = np.clip(
            thrust,
            self._last_attitude.thrust - max_thrust_change,
            self._last_attitude.thrust + max_thrust_change
        )
        
        return AttitudeCommand(
            roll=roll_limited,
            pitch=pitch_limited,
            yaw=yaw_limited,
            thrust=thrust_limited
        )

    def set_hover_yaw(self, yaw: float) -> None:
        """设置悬停时的目标航向"""
        self._hover_yaw = yaw
        self._hover_start_time = get_monotonic_time()
        self._accumulated_yaw_drift = 0.0

    def get_attitude_rate_limits(self) -> Dict[str, float]:
        """获取姿态角速度限制"""
        return {
            'roll_rate_max': self.roll_rate_max,
            'pitch_rate_max': self.pitch_rate_max,
            'yaw_rate_max': self.yaw_rate_max
        }

    def reset(self) -> None:
        """重置控制器状态"""
        self._last_attitude = None
        self._last_time = None
        self._hover_yaw = None
        self._hover_start_time = None
        self._accumulated_yaw_drift = 0.0
        # 重置悬停去抖动状态
        self._hover_debounce_start = None
        self._hover_debounce_target = None
