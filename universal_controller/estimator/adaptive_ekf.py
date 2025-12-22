"""自适应 EKF 状态估计器"""
from typing import Dict, Any, Optional, List
from collections import deque
import numpy as np

from ..core.interfaces import IStateEstimator
from ..core.data_types import EstimatorOutput, Odometry, Imu
from ..config.default_config import PLATFORM_CONFIG
from ..core.ros_compat import euler_from_quaternion, get_monotonic_time, normalize_angle


# 四元数有效性检查常量
# 单位四元数的范数平方应该为 1.0
# 使用宽松的边界来容忍数值误差和传感器噪声
QUATERNION_NORM_SQ_MIN = 0.5  # 范数平方下界 (范数 > 0.707)
QUATERNION_NORM_SQ_MAX = 2.0  # 范数平方上界 (范数 < 1.414)


class AdaptiveEKFEstimator(IStateEstimator):
    """自适应 EKF 状态估计器"""
    
    def __init__(self, config: Dict[str, Any]):
        ekf_config = config.get('ekf', config)
        
        # 初始协方差值
        covariance_config = ekf_config.get('covariance', {})
        self.initial_covariance = covariance_config.get('initial_value', 0.1)
        
        # 状态向量 [px, py, pz, vx, vy, vz, θ, ω, bias_ax, bias_ay, bias_az]
        self.x = np.zeros(11)
        self.P = np.eye(11) * self.initial_covariance
        
        # 平台类型
        platform_name = config.get('system', {}).get('platform', 'differential')
        self.platform_config = PLATFORM_CONFIG.get(platform_name, PLATFORM_CONFIG['differential'])
        self.velocity_heading_coupled = self.platform_config.get('velocity_heading_coupled', True)
        
        # 重力加速度 - 优先从 system.gravity 读取，其次从 attitude.gravity
        self.gravity = config.get('system', {}).get('gravity', 
                        config.get('attitude', {}).get('gravity', 9.81))
        
        # 自适应参数
        adaptive = ekf_config.get('adaptive', {})
        self.base_slip_thresh = adaptive.get('base_slip_thresh', 2.0)
        self.slip_velocity_factor = adaptive.get('slip_velocity_factor', 0.5)
        self.slip_covariance_scale = adaptive.get('slip_covariance_scale', 10.0)
        self.stationary_covariance_scale = adaptive.get('stationary_covariance_scale', 0.1)
        self.stationary_thresh = adaptive.get('stationary_thresh', 0.05)
        self.slip_probability_k_factor = adaptive.get('slip_probability_k_factor', 5.0)
        self.slip_history_window = adaptive.get('slip_history_window', 20)
        
        # IMU 相关参数
        self.max_tilt_angle = ekf_config.get('max_tilt_angle', np.pi / 3)  # ~60°
        self.accel_freshness_thresh = ekf_config.get('accel_freshness_thresh', 0.1)  # 100ms
        
        # Jacobian 计算参数
        self.min_velocity_for_jacobian = ekf_config.get('min_velocity_for_jacobian', 0.01)
        
        # 航向备选参数
        self.use_odom_orientation_fallback = ekf_config.get('use_odom_orientation_fallback', True)
        self.theta_covariance_fallback_thresh = ekf_config.get('theta_covariance_fallback_thresh', 0.5)
        self._last_odom_orientation = None
        
        # 测量噪声
        meas_noise = ekf_config.get('measurement_noise', {})
        self.R_odom_base = np.diag([
            meas_noise.get('odom_position', 0.01)] * 3 +
            [meas_noise.get('odom_velocity', 0.1)] * 3
        )
        self.R_odom_current = self.R_odom_base.copy()
        
        self.R_imu = np.diag([
            meas_noise.get('imu_accel', 0.5)] * 3 +
            [meas_noise.get('imu_gyro', 0.01)]
        )
        
        # 过程噪声
        proc_noise = ekf_config.get('process_noise', {})
        self.Q = np.diag([
            proc_noise.get('position', 0.001)] * 3 +
            [proc_noise.get('velocity', 0.1)] * 3 +
            [proc_noise.get('orientation', 0.01),
             proc_noise.get('angular_velocity', 0.1)] +
            [proc_noise.get('imu_bias', 0.0001)] * 3
        )
        
        # 异常检测阈值
        anomaly = ekf_config.get('anomaly_detection', {})
        self.drift_thresh = anomaly.get('drift_thresh', 0.1)
        self.jump_thresh = anomaly.get('jump_thresh', 0.5)
        self.min_eigenvalue = ekf_config.get('covariance', {}).get('min_eigenvalue', 1e-6)
        
        # IMU 运动加速度补偿
        self.imu_motion_compensation = ekf_config.get('imu_motion_compensation', False)
        
        # 状态变量
        self.slip_detected = False
        self.slip_probability = 0.0
        self.last_innovation_norm = 0.0
        self.last_position = np.zeros(3)
        self.position_jump = 0.0
        self.gyro_z = 0.0
        self._imu_drift_detected = False
        
        # 打滑概率计算
        self.slip_history = deque(maxlen=self.slip_history_window)
        
        # 世界坐标系加速度
        self.last_world_velocity = np.zeros(2)
        self.current_world_velocity = np.zeros(2)
        self.last_odom_time: Optional[float] = None
        self.world_accel_vec = np.zeros(2)
        self._world_accel_initialized = False
        
        self.last_imu_time: Optional[float] = None
        self._imu_available = True
    
    def set_imu_available(self, available: bool) -> None:
        self._imu_available = available
    
    def _get_theta_for_transform(self) -> float:
        """获取用于坐标变换的航向角"""
        theta_var = self.P[6, 6]
        
        if (self.use_odom_orientation_fallback and 
            theta_var > self.theta_covariance_fallback_thresh and
            self._last_odom_orientation is not None):
            _, _, yaw = euler_from_quaternion(self._last_odom_orientation)
            return yaw
        
        return self.x[6]
    
    def apply_drift_correction(self, dx: float, dy: float, dtheta: float) -> None:
        """应用外部漂移校正"""
        self.x[0] += dx
        self.x[1] += dy
        self.x[6] += dtheta
        self.x[6] = normalize_angle(self.x[6])
        
        if self.velocity_heading_coupled:
            v_body = np.sqrt(self.x[3]**2 + self.x[4]**2)
            new_theta = self.x[6]
            self.x[3] = v_body * np.cos(new_theta)
            self.x[4] = v_body * np.sin(new_theta)
        
        self.P[0, 0] += abs(dx) * 0.1
        self.P[1, 1] += abs(dy) * 0.1
        self.P[6, 6] += abs(dtheta) * 0.1
    
    def predict(self, dt: float) -> None:
        """
        运动学预测
        
        标准 EKF 预测步骤:
        1. 在当前状态上计算 Jacobian F
        2. 更新状态 x = f(x)
        3. 更新协方差 P = F @ P @ F.T + Q
        
        注意: Jacobian 必须在状态更新之前计算，使用预测前的状态值
        """
        if dt <= 0:
            return
        
        # 1. 先在当前状态上计算 Jacobian (预测前的状态)
        theta_before = self.x[6]
        omega_before = self.x[7]
        vx_before = self.x[3]
        vy_before = self.x[4]
        F = self._compute_jacobian(dt, theta_before, omega_before, vx_before, vy_before)
        
        # 2. 更新状态
        self.x[0] += self.x[3] * dt
        self.x[1] += self.x[4] * dt
        self.x[2] += self.x[5] * dt
        self.x[6] += self.x[7] * dt
        self.x[6] = normalize_angle(self.x[6])
        
        # 对于速度-航向耦合平台，更新速度方向
        if self.velocity_heading_coupled and abs(self.x[7]) > 1e-6:
            v_body = np.sqrt(self.x[3]**2 + self.x[4]**2)
            self.x[3] = v_body * np.cos(self.x[6])
            self.x[4] = v_body * np.sin(self.x[6])
        
        # 3. 更新协方差
        self.P = F @ self.P @ F.T + self.Q * dt
        self._ensure_positive_definite()

    
    def update_odom(self, odom: Odometry) -> None:
        """Odom 更新"""
        current_time = get_monotonic_time()
        
        self._last_odom_orientation = odom.pose_orientation
        
        new_position = np.array([
            odom.pose_position.x,
            odom.pose_position.y,
            odom.pose_position.z
        ])
        self.position_jump = np.linalg.norm(new_position - self.last_position)
        self.last_position = new_position
        
        vx_body = odom.twist_linear[0]
        vy_body = odom.twist_linear[1]
        vz_body = odom.twist_linear[2]
        v_body = np.sqrt(vx_body**2 + vy_body**2)
        
        # 获取航向角用于坐标变换
        # theta 定义: 机器人在世界坐标系中的航向角
        # - 从世界坐标系 X 轴正方向逆时针测量
        # - theta = 0 时，机器人朝向世界 X 轴正方向
        # - theta = π/2 时，机器人朝向世界 Y 轴正方向
        theta = self._get_theta_for_transform()
        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)
        
        # 机体坐标系到世界坐标系的速度变换
        # v_world = R(theta) @ v_body
        # 其中 R(theta) 是绕 Z 轴旋转 theta 的旋转矩阵
        # [vx_world]   [cos(θ)  -sin(θ)] [vx_body]
        # [vy_world] = [sin(θ)   cos(θ)] [vy_body]
        vx_world = vx_body * cos_theta - vy_body * sin_theta
        vy_world = vx_body * sin_theta + vy_body * cos_theta
        vz_world = vz_body  # Z 轴速度不受 yaw 旋转影响
        
        self.last_world_velocity = self.current_world_velocity.copy()
        self.current_world_velocity = np.array([vx_world, vy_world])
        
        if self.last_odom_time is not None:
            dt = current_time - self.last_odom_time
            if dt > 0:
                self.world_accel_vec = (self.current_world_velocity - self.last_world_velocity) / dt
                self._world_accel_initialized = True
            else:
                self.world_accel_vec = np.zeros(2)
        else:
            self.world_accel_vec = np.zeros(2)
        
        self.last_odom_time = current_time
        self._update_odom_covariance(v_body)
        
        z = np.array([
            odom.pose_position.x,
            odom.pose_position.y,
            odom.pose_position.z,
            vx_world,
            vy_world,
            vz_world,
        ])
        
        H = np.zeros((6, 11))
        H[0:3, 0:3] = np.eye(3)
        H[3:6, 3:6] = np.eye(3)
        
        self._kalman_update(z, H, self.R_odom_current)
    
    def update_imu(self, imu: Imu) -> None:
        """
        IMU 观测更新
        
        IMU 测量模型 (比力模型):
        - 加速度计测量的是比力 (specific force): a_measured = a_true - g_body + bias + noise
        - 其中 a_true 是真实加速度，g_body 是重力在机体坐标系的投影
        - 静止时: a_measured = -g_body + bias ≈ [0, 0, +g] + bias (假设水平)
        - 陀螺仪测量: omega_measured = omega + bias + noise
        
        坐标系约定 (ENU):
        - 世界坐标系: X-东, Y-北, Z-上
        - 重力向量 (世界坐标系): g_world = [0, 0, -9.81] m/s²
        
        重力在机体坐标系的投影 (ZYX 欧拉角约定):
        - g_body = R_world_to_body @ [0, 0, -g]
        - g_body_x = -g * sin(pitch)
        - g_body_y = g * sin(roll) * cos(pitch)  
        - g_body_z = -g * cos(roll) * cos(pitch)
        
        加速度计静止时的期望测量值 (比力 = -g_body):
        - a_expected_x = g * sin(pitch)
        - a_expected_y = -g * sin(roll) * cos(pitch)
        - a_expected_z = g * cos(roll) * cos(pitch)  (约 +9.81 当水平时)
        """
        if not self._imu_available:
            return
        
        self.gyro_z = imu.angular_velocity[2]
        theta = self.x[6]  # yaw 角
        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)
        
        # 首先计算重力在机体坐标系的期望测量值（比力）
        # 对于地面车辆，假设 roll 和 pitch 接近 0
        # 对于无人机，使用 IMU 提供的姿态信息
        g = self.gravity  # 使用配置的重力加速度
        use_imu_orientation = False
        
        if imu.orientation is not None:
            # 检查四元数有效性
            q = imu.orientation
            if hasattr(q, '__len__') and len(q) == 4:
                qx, qy, qz, qw = q[0], q[1], q[2], q[3]
                
                # 检查各分量是否为 NaN 或 Inf
                if not (np.isfinite(qx) and np.isfinite(qy) and 
                        np.isfinite(qz) and np.isfinite(qw)):
                    # 四元数包含无效值
                    use_imu_orientation = False
                else:
                    norm_sq = qx*qx + qy*qy + qz*qz + qw*qw
                    
                    # 四元数有效性检查:
                    # 1. 范数不能太小（接近零向量）
                    # 2. 范数应该接近 1（单位四元数）
                    is_valid = (
                        norm_sq > QUATERNION_NORM_SQ_MIN and
                        norm_sq < QUATERNION_NORM_SQ_MAX
                    )
                    
                    if is_valid:
                        roll, pitch, _ = euler_from_quaternion(q)
                        
                        # 额外检查: roll 和 pitch 应该在合理范围内
                        # 使用配置的最大倾斜角阈值
                        if abs(roll) < self.max_tilt_angle and abs(pitch) < self.max_tilt_angle:
                            use_imu_orientation = True
        
        if use_imu_orientation:
            # 加速度计静止时的期望测量值 (比力 = -g_body)
            g_measured_x = g * np.sin(pitch)
            g_measured_y = -g * np.sin(roll) * np.cos(pitch)
            g_measured_z = g * np.cos(roll) * np.cos(pitch)
        else:
            # 没有有效姿态信息，假设水平
            g_measured_x = 0.0
            g_measured_y = 0.0
            g_measured_z = g
        
        # 从 IMU 测量值中提取真实加速度（去除重力和 bias）
        # a_true_body = a_measured - g_measured - bias
        ax_body_true = imu.linear_acceleration[0] - g_measured_x - self.x[8]
        ay_body_true = imu.linear_acceleration[1] - g_measured_y - self.x[9]
        
        # 将真实加速度转换到世界坐标系（只考虑 yaw 旋转，因为地面车辆 roll/pitch 接近 0）
        ax_world = ax_body_true * cos_theta - ay_body_true * sin_theta
        ay_world = ax_body_true * sin_theta + ay_body_true * cos_theta
        
        # 打滑检测：比较 IMU 测量的加速度与 odom 计算的加速度
        # 添加时间窗口检查，确保 odom 加速度数据是新鲜的
        current_imu_time = get_monotonic_time()
        
        odom_accel_fresh = (
            self._world_accel_initialized and 
            self.last_odom_time is not None and
            (current_imu_time - self.last_odom_time) < self.accel_freshness_thresh
        )
        
        if odom_accel_fresh:
            imu_accel_world = np.array([ax_world, ay_world])
            accel_diff = np.linalg.norm(imu_accel_world - self.world_accel_vec)
            self.slip_probability = self._compute_slip_probability(accel_diff)
            self.slip_detected = self.slip_probability > 0.5
        else:
            # odom 加速度数据过旧，不进行打滑检测
            # 保持上一次的打滑状态，但逐渐衰减概率
            if self.slip_probability > 0:
                self.slip_probability = max(0, self.slip_probability - 0.05)
            self.slip_detected = self.slip_probability > 0.5
        
        self.last_imu_time = current_imu_time
        
        # IMU 观测向量
        z = np.array([
            imu.linear_acceleration[0],
            imu.linear_acceleration[1],
            imu.linear_acceleration[2],
            imu.angular_velocity[2],
        ])
        
        # 观测矩阵：IMU 测量与状态的关系
        # 加速度计测量 = bias + 重力期望值 (+ 运动加速度，如果启用补偿)
        # 陀螺仪测量 = 角速度
        H = np.zeros((4, 11))
        H[0, 8] = 1.0  # ax 与 bias_ax
        H[1, 9] = 1.0  # ay 与 bias_ay
        H[2, 10] = 1.0  # az 与 bias_az
        H[3, 7] = 1.0  # wz 与 omega
        
        # 计算期望的 IMU 测量值
        if self.imu_motion_compensation and self._world_accel_initialized:
            # 运动加速度从世界坐标系转换到机体坐标系
            ax_motion_body = self.world_accel_vec[0] * cos_theta + self.world_accel_vec[1] * sin_theta
            ay_motion_body = -self.world_accel_vec[0] * sin_theta + self.world_accel_vec[1] * cos_theta
            z_expected = np.array([
                self.x[8] + ax_motion_body + g_measured_x,
                self.x[9] + ay_motion_body + g_measured_y,
                self.x[10] + g_measured_z,
                self.x[7],
            ])
        else:
            z_expected = np.array([
                self.x[8] + g_measured_x,
                self.x[9] + g_measured_y,
                self.x[10] + g_measured_z,
                self.x[7]
            ])
        
        self._kalman_update_with_expected(z, z_expected, H, self.R_imu)
    
    def _kalman_update(self, z: np.ndarray, H: np.ndarray, R: np.ndarray) -> None:
        y = z - H @ self.x
        self._apply_kalman_gain(y, H, R)
    
    def _kalman_update_with_expected(self, z: np.ndarray, z_expected: np.ndarray,
                                     H: np.ndarray, R: np.ndarray):
        y = z - z_expected
        self._apply_kalman_gain(y, H, R)
    
    def _apply_kalman_gain(self, y: np.ndarray, H: np.ndarray, R: np.ndarray) -> None:
        S = H @ self.P @ H.T + R
        try:
            L = np.linalg.cholesky(S)
            K = self.P @ H.T @ np.linalg.solve(L.T, np.linalg.solve(L, np.eye(len(S))))
        except np.linalg.LinAlgError:
            K = self.P @ H.T @ np.linalg.pinv(S)
        
        self.x = self.x + K @ y
        I_KH = np.eye(11) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T
        self._ensure_positive_definite()
        self.last_innovation_norm = np.linalg.norm(y)
        
        self.x[6] = normalize_angle(self.x[6])
    
    def _compute_jacobian(self, dt: float, theta: float, omega: float, 
                          vx: float, vy: float) -> np.ndarray:
        """
        计算状态转移 Jacobian
        
        重要: 此函数应在状态更新之前调用，使用预测前的状态值
        
        对于速度-航向耦合的平台 (差速车/阿克曼车):
        - vx_world = v_body * cos(theta)
        - vy_world = v_body * sin(theta)
        
        状态转移方程:
        - px_new = px + vx * dt
        - py_new = py + vy * dt
        - pz_new = pz + vz * dt
        - theta_new = theta + omega * dt
        - 对于耦合平台: vx_new = v_body * cos(theta_new), vy_new = v_body * sin(theta_new)
        
        Jacobian 在预测前状态点计算 (标准 EKF 做法)
        
        Args:
            dt: 时间步长
            theta: 预测前的航向角
            omega: 预测前的角速度
            vx: 预测前的 x 方向速度
            vy: 预测前的 y 方向速度
        
        Returns:
            状态转移 Jacobian 矩阵 F [11x11]
        """
        F = np.eye(11)
        F[0, 3] = dt  # ∂px/∂vx
        F[1, 4] = dt  # ∂py/∂vy
        F[2, 5] = dt  # ∂pz/∂vz
        F[6, 7] = dt  # ∂theta/∂omega
        
        if self.velocity_heading_coupled:
            # 使用传入的预测前速度计算 v_body
            v_body = np.sqrt(vx**2 + vy**2)
            cos_theta = np.cos(theta)
            sin_theta = np.sin(theta)
            
            # 使用平滑过渡避免 Jacobian 跳变
            # 当 v_body 很小时，使用最小值来保证协方差传播
            # 
            # 物理意义：即使车辆静止，航向角的不确定性仍然会影响
            # 速度估计的不确定性（因为一旦开始运动，速度方向取决于航向）
            
            # 使用平滑的 max 函数避免不连续
            # effective_v = sqrt(v_body^2 + MIN_V^2) 近似于 max(v_body, MIN_V)
            # 但在 v_body ≈ MIN_V 附近更平滑
            effective_v = np.sqrt(v_body**2 + self.min_velocity_for_jacobian**2)
            
            # ∂vx_world/∂theta = -v_body * sin(theta)
            # ∂vy_world/∂theta = v_body * cos(theta)
            F[3, 6] = -effective_v * sin_theta
            F[4, 6] = effective_v * cos_theta
            
            # ∂vx_world/∂omega 和 ∂vy_world/∂omega
            # 由于 theta_new = theta + omega * dt
            # vx_new = v_body * cos(theta_new)
            # 使用链式法则: ∂vx_new/∂omega = ∂vx_new/∂theta_new * ∂theta_new/∂omega
            #                              = -v_body * sin(theta) * dt
            F[3, 7] = -effective_v * sin_theta * dt
            F[4, 7] = effective_v * cos_theta * dt
        
        return F
    
    def _ensure_positive_definite(self) -> None:
        """确保协方差矩阵正定"""
        self.P = (self.P + self.P.T) / 2
        try:
            eigenvalues, eigenvectors = np.linalg.eigh(self.P)
            if np.any(eigenvalues < self.min_eigenvalue):
                eigenvalues = np.maximum(eigenvalues, self.min_eigenvalue)
                self.P = eigenvectors @ np.diag(eigenvalues) @ eigenvectors.T
        except np.linalg.LinAlgError:
            # 使用配置的初始协方差值而非硬编码
            self.P = np.eye(11) * self.initial_covariance
    
    def _get_adaptive_slip_threshold(self) -> float:
        current_velocity = np.linalg.norm(self.x[3:6])
        return self.base_slip_thresh + self.slip_velocity_factor * current_velocity
    
    def _is_stationary(self) -> bool:
        return np.linalg.norm(self.x[3:6]) < self.stationary_thresh
    
    def _update_odom_covariance(self, v_body: float) -> None:
        """更新 odom 测量噪声协方差"""
        if self.slip_detected:
            self.R_odom_current = self.R_odom_base * self.slip_covariance_scale
        elif self._is_stationary():
            self.R_odom_current = self.R_odom_base * self.stationary_covariance_scale
        else:
            self.R_odom_current = self.R_odom_base.copy()
        
        # 当速度较高时，航向角不确定性会影响速度估计的不确定性
        # 使用 stationary_thresh 作为阈值，保持一致性
        theta_var = self.P[6, 6]
        if theta_var > 0 and v_body > self.stationary_thresh * 2:
            velocity_transform_var = (v_body ** 2) * theta_var
            self.R_odom_current[3, 3] += velocity_transform_var
            self.R_odom_current[4, 4] += velocity_transform_var
    
    def _compute_slip_probability(self, accel_diff: float) -> float:
        slip_thresh = self._get_adaptive_slip_threshold()
        k = self.slip_probability_k_factor / slip_thresh
        probability = 1.0 / (1.0 + np.exp(-k * (accel_diff - slip_thresh * 0.8)))
        self.slip_history.append(probability)
        return np.mean(self.slip_history)
    
    def detect_anomalies(self) -> List[str]:
        anomalies = []
        if self.slip_detected:
            anomalies.append("SLIP_DETECTED")
        if self._is_stationary() and abs(self.gyro_z) > self.drift_thresh:
            anomalies.append("IMU_DRIFT")
            self._imu_drift_detected = True
        else:
            self._imu_drift_detected = False
        if self.position_jump > self.jump_thresh:
            anomalies.append("ODOM_JUMP")
        if not self._imu_available:
            anomalies.append("IMU_UNAVAILABLE")
        return anomalies
    
    def get_state(self) -> EstimatorOutput:
        return EstimatorOutput(
            state=self.x[:8].copy(),
            covariance=self.P[:8, :8].copy(),
            covariance_norm=np.linalg.norm(self.P[:8, :8]),
            innovation_norm=self.last_innovation_norm,
            imu_bias=self.x[8:11].copy(),
            slip_probability=self.slip_probability,
            anomalies=self.detect_anomalies(),
            imu_available=self._imu_available,
            imu_drift_detected=self._imu_drift_detected
        )
    
    def reset(self) -> None:
        self.x = np.zeros(11)
        self.P = np.eye(11) * self.initial_covariance
        self.slip_detected = False
        self.slip_probability = 0.0
        self.last_innovation_norm = 0.0
        self.slip_history.clear()
        self.last_imu_time = None
        self.last_odom_time = None
        self.last_world_velocity = np.zeros(2)
        self.current_world_velocity = np.zeros(2)
        self.world_accel_vec = np.zeros(2)
        self._world_accel_initialized = False
        self.last_position = np.zeros(3)
        self.position_jump = 0.0
        self.gyro_z = 0.0
        self._imu_available = True
        self._imu_drift_detected = False
        self._last_odom_orientation = None
