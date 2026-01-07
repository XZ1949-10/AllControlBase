"""自适应 EKF 状态估计器"""
from typing import Dict, Any, Optional, List
from collections import deque
import numpy as np
import logging
import threading

from ..core.interfaces import IStateEstimator
from ..core.data_types import EstimatorOutput, Odometry, Imu
from ..config.default_config import PLATFORM_CONFIG
from ..core.ros_compat import euler_from_quaternion, get_monotonic_time, normalize_angle
from ..core.indices import StateIdx
from ..core.constants import (
    QUATERNION_NORM_SQ_MIN,
    QUATERNION_NORM_SQ_MAX,
    COVARIANCE_MIN_EIGENVALUE,
    COVARIANCE_INITIAL_VALUE,
    DEFAULT_GRAVITY,
    EKF_MAX_TILT_ANGLE,
    EKF_MIN_VELOCITY_FOR_JACOBIAN,
    EKF_COVARIANCE_EXPLOSION_THRESH,
    EKF_INNOVATION_ANOMALY_THRESH,
)

logger = logging.getLogger(__name__)


class AdaptiveEKFEstimator(IStateEstimator):
    """自适应 EKF 状态估计器

    线程安全性:
        此类提供可选的线程安全支持：
        - 默认情况下不启用锁（thread_safe=False），以获得最佳性能
        - 设置 thread_safe=True 启用内部锁保护
        - 启用后，predict(), update_odom(), update_imu(), get_state() 方法是线程安全的
        - 如果不启用内部锁，调用者应确保在单线程中调用或在外部加锁

    典型调用顺序:
        1. predict(dt)      - 运动学预测
        2. update_odom(odom) - 里程计更新
        3. update_imu(imu)   - IMU 更新（可选）
        4. get_state()       - 获取估计结果
    """

    def __init__(self, config: Dict[str, Any], thread_safe: bool = False):
        """
        初始化 EKF 估计器

        Args:
            config: 配置字典
            thread_safe: 是否启用线程安全模式（默认 False 以获得最佳性能）
            
        Note:
            当 thread_safe=False 时，调用者必须确保在单线程中调用，
            或在外部加锁。在多线程环境中使用非线程安全模式可能导致数据竞争。
        """
        # 线程安全支持 - 优化：避免在非线程安全模式下的检查开销
        self._thread_safe = thread_safe
        if thread_safe:
            self._lock = threading.RLock()
            self._acquire_lock = self._lock.acquire
            self._release_lock = self._lock.release
        else:
            self._lock = None
            # 零开销的 No-op，返回 True 以匹配 RLock.acquire() 的返回类型
            self._acquire_lock = lambda: True
            self._release_lock = lambda: None
        
        # 记录调用线程 ID，用于检测多线程误用
        self._creation_thread_id = threading.get_ident()
        self._thread_warning_logged = False

        ekf_config = config.get('ekf', config)
        
        # 初始协方差值 - 使用常量
        self.initial_covariance = COVARIANCE_INITIAL_VALUE
        
        # 状态向量 [px, py, pz, vx, vy, vz, θ, ω, bias_ax, bias_ay, bias_az]
        self.x = np.zeros(11)
        self.P = np.eye(11) * self.initial_covariance
        
        # 平台类型
        platform_name = config.get('system', {}).get('platform', 'differential')
        self.platform_config = PLATFORM_CONFIG.get(platform_name, PLATFORM_CONFIG['differential'])
        self.velocity_heading_coupled = self.platform_config.get('velocity_heading_coupled', True)
        
        # 重力加速度 - 使用物理常量
        self.gravity = DEFAULT_GRAVITY
        
        # 自适应参数
        adaptive = ekf_config.get('adaptive', {})
        self.base_slip_thresh = adaptive.get('base_slip_thresh', 2.0)
        self.slip_velocity_factor = adaptive.get('slip_velocity_factor', 0.5)
        self.slip_covariance_scale = adaptive.get('slip_covariance_scale', 10.0)
        self.stationary_covariance_scale = adaptive.get('stationary_covariance_scale', 0.1)
        self.stationary_thresh = adaptive.get('stationary_thresh', 0.05)
        self.slip_probability_k_factor = adaptive.get('slip_probability_k_factor', 5.0)
        # 打滑概率衰减参数
        self.slip_decay_rate = adaptive.get('slip_decay_rate', 2.0)  # 每秒衰减量
        self.slip_history_window = adaptive.get('slip_history_window', 20)
        
        # IMU 相关参数 - 使用常量
        self.max_tilt_angle = EKF_MAX_TILT_ANGLE
        self.accel_freshness_thresh = ekf_config.get('accel_freshness_thresh', 0.1)  # 100ms
        
        # Jacobian 计算参数 - 使用常量
        self.min_velocity_for_jacobian = EKF_MIN_VELOCITY_FOR_JACOBIAN
        self.jacobian_smooth_epsilon = ekf_config.get('jacobian_smooth_epsilon', 0.1)
        
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
        
        # 航向角测量噪声
        self._odom_orientation_noise = meas_noise.get('odom_orientation', 0.01)
        
        # 角速度测量噪声
        self._odom_angular_velocity_noise = meas_noise.get('odom_angular_velocity', 0.05)
        
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
        # 使用常量作为协方差最小特征值
        self.min_eigenvalue = COVARIANCE_MIN_EIGENVALUE
        # 协方差爆炸检测阈值 - 使用常量
        self.covariance_explosion_thresh = EKF_COVARIANCE_EXPLOSION_THRESH
        # 创新度异常阈值 - 使用常量
        self.innovation_anomaly_thresh = EKF_INNOVATION_ANOMALY_THRESH
        
        # IMU 运动加速度补偿
        self.imu_motion_compensation = ekf_config.get('imu_motion_compensation', False)
        
        # 约束配置
        constraints = ekf_config.get('constraints', {})
        self.enable_non_holonomic_constraint = constraints.get('non_holonomic', True)
        self.non_holonomic_slip_threshold = constraints.get('non_holonomic_slip_threshold', 0.5)
        
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
        
        # Body Frame 加速度 (用于打滑检测)
        self.last_body_velocity = np.zeros(2)
        self.current_body_velocity = np.zeros(2)
        self._raw_odom_twist_norm = 0.0
        self._raw_odom_angular_velocity = 0.0  # 新增: 确保在 reset 前也有定义
        self.last_odom_time: Optional[float] = None
        self.body_accel_vec = np.zeros(2) # Derived from Odom
        self._body_accel_initialized = False
        self._initialized = False
        
        self.last_imu_time: Optional[float] = None
        self._imu_available = True
        
        # 日志节流标志 - 避免重复警告
        # 使用字典统一管理，便于 reset() 时清理
        self._warning_logged = {
            'quaternion_error': False,
            'euler_nan': False,
            'tilt_exceeded': False,
            'accel_nan': False,
            'gyro_nan': False,
        }

        # ----------------------------------------------------------------------
        # 性能优化: 预分配矩阵和向量
        # ----------------------------------------------------------------------
        # EKF 矩阵 (均在 __init__ 一次性分配)
        self._F = np.eye(11)
        self._H_odom = np.zeros((8, 11))
        # 预设固定 H_odom 元素
        self._H_odom[0:3, 0:3] = np.eye(3)
        self._H_odom[3:6, 3:6] = np.eye(3)
        self._H_odom[StateIdx.YAW, StateIdx.YAW] = 1.0
        self._H_odom[7, StateIdx.YAW_RATE] = 1.0
        
        self._R_odom = np.zeros((8, 8))
        self._z_odom = np.zeros(8)
        self._y_odom = np.zeros(8) # Innovation buffer
        
        self._H_imu = np.zeros((4, 11))
        # 预设固定 H_imu 元素
        self._H_imu[0, StateIdx.ACCEL_BIAS_X] = 1.0  # bias_ax
        self._H_imu[1, StateIdx.ACCEL_BIAS_Y] = 1.0  # bias_ay
        self._H_imu[2, StateIdx.ACCEL_BIAS_Z] = 1.0 # bias_az
        self._H_imu[3, StateIdx.YAW_RATE] = 1.0  # omega
        
        self._z_imu = np.zeros(4)
        self._y_imu = np.zeros(4) # Innovation buffer
        self._z_imu_expected = np.zeros(4)
        self._I_11 = np.eye(11) # 恒等矩阵

        # ----------------------------------------------------------------------
        # 优化: 中间计算 Buffer (避免运行时 malloc)
        # 命名约定: _temp_{rows}_{cols}_{usage}
        # ----------------------------------------------------------------------
        # Predict 阶段: P = F @ P @ F.T + Q * dt
        self._temp_F_P = np.zeros((11, 11))        # F @ P
        self._temp_P_FT = np.zeros((11, 11))       # P @ F.T (Not used directly if using F@P@FT) -> F@P -> (F@P)@F.T
        self._temp_FP_FT = np.zeros((11, 11))      # (F @ P) @ F.T
        
        # Update 阶段 (Odom 8维): 
        # y = z - Hx
        # S = H @ P @ H.T + R
        # K = P @ H.T @ S_inv
        self._temp_H8_P = np.zeros((8, 11))        # H @ P
        self._temp_H8_P_HT = np.zeros((8, 8))      # (H @ P) @ H.T
        self._temp_PHt_8 = np.zeros((11, 8))       # P @ H.T
        self._temp_K_8 = np.zeros((11, 8))         # K (Gain)
        self._temp_K_y_8 = np.zeros(11)            # K @ y
        self._temp_K_S_8 = np.zeros((11, 8))       # K @ S
        self._temp_P_update_11 = np.zeros((11, 11)) # K @ S @ K.T (Joseph term)

        # Update 阶段 (IMU 4维):
        self._temp_H4_P = np.zeros((4, 11))        # H @ P
        self._temp_H4_P_HT = np.zeros((4, 4))      # (H @ P) @ H.T
        self._temp_PHt_4 = np.zeros((11, 4))       # P @ H.T
        self._temp_K_4 = np.zeros((11, 4))         # K
        self._temp_K_y_4 = np.zeros(11)            # K @ y
        self._temp_K_S_4 = np.zeros((11, 4))       # K @ S

    # _acquire_lock 和 _release_lock 现在是动态分配的，移除默认实现
    # def _acquire_lock(self): ...
    # def _release_lock(self): ...

    def set_imu_available(self, available: bool) -> None:
        """设置 IMU 可用状态（线程安全）"""
        self._acquire_lock()
        try:
            self._imu_available = available
        finally:
            self._release_lock()
    
    def _get_theta_for_transform(self) -> float:
        """获取用于坐标变换的航向角"""
        theta_var = self.P[StateIdx.YAW, StateIdx.YAW]
        
        if (self.use_odom_orientation_fallback and 
            theta_var > self.theta_covariance_fallback_thresh and
            self._last_odom_orientation is not None):
            _, _, yaw = euler_from_quaternion(self._last_odom_orientation)
            return yaw
        
        return self.x[6]
    
    def apply_drift_correction(self, dx: float, dy: float, dtheta: float) -> None:
        """应用外部漂移校正（线程安全）

        对位置和航向进行外部校正（如来自 SLAM 或 GPS 的修正）。
        对于速度-航向耦合平台，同时更新速度方向以保持与新航向一致。

        Args:
            dx: X 方向位置校正量（米）
            dy: Y 方向位置校正量（米）
            dtheta: 航向角校正量（弧度）
        """
        self._acquire_lock()
        try:
            self.x[StateIdx.X] += dx
            self.x[StateIdx.Y] += dy
            self.x[StateIdx.YAW] += dtheta
            self.x[StateIdx.YAW] = normalize_angle(self.x[StateIdx.YAW])

            if self.velocity_heading_coupled:
                # 优雅修复: 使用旋转矩阵对速度向量进行旋转
                # 之前的方法 (project -> rotate) 会丢失横向速度分量 (Lateral Velocity)
                # 正确做法是保留 Body Frame 下的速度不变，随着 Yaw 的变化旋转 World Frame 速度
                
                # 1. 计算旋转前的 World 速度
                vx_old = self.x[StateIdx.VX]
                vy_old = self.x[StateIdx.VY]
                
                # 2. 旋转速度向量 (Rotate by dtheta)
                # [vx_new]   [cos(dtheta)  -sin(dtheta)] [vx_old]
                # [vy_new] = [sin(dtheta)   cos(dtheta)] [vy_old]
                cos_d = np.cos(dtheta)
                sin_d = np.sin(dtheta)
                
                self.x[StateIdx.VX] = vx_old * cos_d - vy_old * sin_d
                self.x[StateIdx.VY] = vx_old * sin_d + vy_old * cos_d

            self.P[StateIdx.X, StateIdx.X] += abs(dx) * 0.1
            self.P[StateIdx.Y, StateIdx.Y] += abs(dy) * 0.1
            self.P[StateIdx.YAW, StateIdx.YAW] += abs(dtheta) * 0.1
        finally:
            self._release_lock()
    
    def predict(self, dt: float) -> None:
        """
        运动学预测

        标准 EKF 预测步骤:
        1. 在当前状态上计算 Jacobian F
        2. 更新状态 x = f(x)
        3. 更新协方差 P = F @ P @ F.T + Q

        注意: Jacobian 必须在状态更新之前计算，使用预测前的状态值

        Args:
            dt: 时间步长（秒），必须为正有限值
        """
        # 验证 dt 是有效的正有限值
        # - dt <= 0: 无效的时间步长
        # - np.isnan(dt): NaN 值（NaN <= 0 返回 False，需要单独检查）
        # - np.isinf(dt): 无穷大值
        if dt <= 0 or not np.isfinite(dt):
            return

        self._acquire_lock()
        try:
            # 资源有效性检查 (防止 Shutdown 竞态)
            if self._F is None:
                return

            # 1. 先在当前状态上计算 Jacobian (预测前的状态)
            # 使用 StateIdx 访问状态
            
            # Fix: 如果尚未初始化，不进行预测，防止基于全零状态的发散
            if not self._initialized:
                return

            theta_before = self.x[StateIdx.YAW]
            omega_before = self.x[StateIdx.YAW_RATE]
            vx_before = self.x[StateIdx.VX]
            vy_before = self.x[StateIdx.VY]
            
            # 使用预分配的 _F 矩阵，并在 _update_jacobian_F 中填充
            # 注意：原代码逻辑是 _compute_jacobian 返回新的 F
            # 这里我们需要改为原地更新 _F
            self._update_jacobian_F(dt, theta_before, omega_before, vx_before, vy_before)

            # 2. 更新状态 (x = f(x))
            self.x[StateIdx.X] += self.x[StateIdx.VX] * dt
            self.x[StateIdx.Y] += self.x[StateIdx.VY] * dt
            self.x[StateIdx.Z] += self.x[StateIdx.VZ] * dt
            self.x[StateIdx.YAW] += self.x[StateIdx.YAW_RATE] * dt
            self.x[StateIdx.YAW] = normalize_angle(self.x[StateIdx.YAW])

            # 对于速度-航向耦合平台（差速车/阿克曼车），强制速度方向与航向一致
            if self.velocity_heading_coupled and self.enable_non_holonomic_constraint:
                v_magnitude = np.sqrt(self.x[StateIdx.VX]**2 + self.x[StateIdx.VY]**2)
                
                # 低速时不强制约束
                if v_magnitude > self.stationary_thresh:
                    if self.slip_probability < self.non_holonomic_slip_threshold:
                        yaw = self.x[StateIdx.YAW]
                        # 使用点积计算带符号的投影速度
                        v_signed = self.x[StateIdx.VX] * np.cos(yaw) + self.x[StateIdx.VY] * np.sin(yaw)
                        self.x[StateIdx.VX] = v_signed * np.cos(yaw)
                        self.x[StateIdx.VY] = v_signed * np.sin(yaw)
                    else:
                        # 打滑时允许横向滑移
                        pass

            # 3. 更新协方差 P = F @ P @ F.T + Q * dt
            # 优化: 使用原地操作避免分配
            # Step 3.1: _temp_F_P = F @ P
            np.matmul(self._F, self.P, out=self._temp_F_P)
            
            # Step 3.2: P_pred = _temp_F_P @ F.T
            # 注意: self._F 是 array, .T 是属性，不是拷贝
            np.matmul(self._temp_F_P, self._F.T, out=self._temp_FP_FT)
            
            # Step 3.3: P = P_pred + Q * dt
            # Q 是对角阵，Q*dt 也是对角阵，直接加到 P_pred 上
            # 为了极致性能，我们可以预先计算 Q_dt (如果 dt 不变)，但 dt 是变化的
            # 这里做 inplace add: P = _temp_FP_FT + Q*dt
            # 由于 Q 是稀疏的 (对角)，全矩阵加法略显浪费，但 numpy 如果不做 view 优化也不大
            # 安全做法: P = _temp_FP_FT + Q * dt
            # In-place: self.P[:] = ...
            np.multiply(self.Q, dt, out=self._temp_F_P) # 复用 _temp_F_P 暂存 Q*dt
            np.add(self._temp_FP_FT, self._temp_F_P, out=self.P)
            
            self._ensure_positive_definite()
        finally:
            self._release_lock()

    
    def update_odom(self, odom: Odometry, current_time: float) -> None:
        """Odom 更新
        
        Args:
            odom: 里程计数据
            current_time: 当前时间戳 (使用与 predict 一致的时间源)
        """
        self._acquire_lock()
        try:
            # 资源有效性检查
            if self._F is None:
                return


            self._last_odom_orientation = odom.pose_orientation

            new_position = np.array([
                odom.pose_position.x,
                odom.pose_position.y,
                odom.pose_position.z
            ])

            # ------------------------------------------------------------------
            # 初始化逻辑: 如果是 Reset 后的第一帧，直接硬初始化状态
            # 避免 "Zero State" 与 "Actual Position" 之间的巨大差异触发跳变检测 (Phantom Jump)
            # ------------------------------------------------------------------
            if not self._initialized:
                self.x[StateIdx.X] = new_position[0]
                self.x[StateIdx.Y] = new_position[1]
                self.x[StateIdx.Z] = new_position[2]
                
                _, _, yaw = euler_from_quaternion(odom.pose_orientation)
                self.x[StateIdx.YAW] = normalize_angle(yaw)
                self.x[StateIdx.YAW_RATE] = odom.twist_angular[2]
                
                vx_b, vy_b = odom.twist_linear[0], odom.twist_linear[1]
                c, s = np.cos(yaw), np.sin(yaw)
                self.x[StateIdx.VX] = vx_b * c - vy_b * s
                self.x[StateIdx.VY] = vx_b * s + vy_b * c
                self.x[StateIdx.VZ] = odom.twist_linear[2]

                self.last_position = new_position
                self.position_jump = 0.0
                
                # 初始化缓存变量，防止虚假打滑
                self.current_body_velocity[0] = vx_b
                self.current_body_velocity[1] = vy_b
                self.last_body_velocity[:] = self.current_body_velocity
                self.last_odom_time = current_time
                
                self._initialized = True
                logger.info(f"EKF state hard-initialized from Odom at {current_time:.3f}s")
                return

            self.position_jump = np.linalg.norm(new_position - self.last_position)
            self.last_position = new_position

            vx_body = odom.twist_linear[0]
            vy_body = odom.twist_linear[1]
            vz_body = odom.twist_linear[2]
            v_body = np.sqrt(vx_body**2 + vy_body**2)
            self._raw_odom_twist_norm = v_body  # Store raw for drift detection

            # 从 odom 获取角速度 (z 轴)
            omega_z = odom.twist_angular[2]
            self._raw_odom_angular_velocity = omega_z

            # 从 odom 四元数提取航向角
            _, _, odom_yaw = euler_from_quaternion(odom.pose_orientation)

            # 获取航向角用于坐标变换
            # 使用 odom 的航向角，而不是 EKF 估计的航向角
            # 这确保速度变换使用正确的航向
             # 机体坐标系速度 (直接从 Odom 获取)
            vx_body = odom.twist_linear[0]
            vy_body = odom.twist_linear[1]
            # vz_body = odom.twist_linear[2] # Unused for 2D slip
            
            # 使用 Odom 提供的角速度
            omega_z = odom.twist_angular[2]
            
            # 记录用于后续步骤的全局转换
            _, _, odom_yaw = euler_from_quaternion(odom.pose_orientation)
            
            # ------------------------------------------------------------------
            # 打滑检测专用: 计算 Body Frame 下的加速度
            # a_body = dv_body/dt + omega x v_body
            # ------------------------------------------------------------------
            # ------------------------------------------------------------------
            # 打滑检测专用: 计算 Body Frame 下的加速度
            # a_body = dv_body/dt + omega x v_body
            # ------------------------------------------------------------------
            self.last_body_velocity[:] = self.current_body_velocity
            self.current_body_velocity[0] = vx_body
            self.current_body_velocity[1] = vy_body
            
            if self.last_odom_time is not None:
                dt_odom = current_time - self.last_odom_time
                if dt_odom > 0.0001:
                    # 1. 线性加速度项: (v_curr - v_last) / dt
                    dv_dt_x = (vx_body - self.last_body_velocity[0]) / dt_odom
                    dv_dt_y = (vy_body - self.last_body_velocity[1]) / dt_odom
                    
                    # 2. 科氏/向心项: omega x v
                    # a_cor_x = -v_y * omega
                    # a_cor_y = +v_x * omega
                    # 使用当前和上一时刻速度的平均值会让积分更准，但这里用当前值即可
                    a_cor_x = -vy_body * omega_z
                    a_cor_y =  vx_body * omega_z
                    
                    self.body_accel_vec[0] = dv_dt_x + a_cor_x
                    self.body_accel_vec[1] = dv_dt_y + a_cor_y
                    self._body_accel_initialized = True
                else:
                    self.body_accel_vec = np.zeros(2)
            else:
                self.body_accel_vec = np.zeros(2)
             
            # ------------------------------------------------------------------
            # 观测向量构建
            # ------------------------------------------------------------------
            theta = odom_yaw
            cos_theta = np.cos(theta)
            sin_theta = np.sin(theta)

            # 转换到世界系用于 EKF 更新 (观测方程 z = Hx, x 是世界系状态)
            vx_world = vx_body * cos_theta - vy_body * sin_theta
            vy_world = vx_body * sin_theta + vy_body * cos_theta
            vz_world = vz_body

            self.last_odom_time = current_time
            self._update_odom_covariance(v_body)

            # 观测向量 z (复用预分配数组)
            self._z_odom[0] = odom.pose_position.x
            self._z_odom[1] = odom.pose_position.y
            self._z_odom[2] = odom.pose_position.z
            self._z_odom[3] = vx_world
            self._z_odom[4] = vy_world
            self._z_odom[5] = vz_world
            self._z_odom[6] = odom_yaw
            self._z_odom[7] = omega_z

            # H 矩阵在 __init__ 中已初始化 (self._H_odom)
            
            # R 矩阵 (复用预分配矩阵)
            self._R_odom.fill(0.0)
            
            # 填充对角线
            self._R_odom[0:6, 0:6] = self.R_odom_current
            self._R_odom[6, 6] = self._odom_orientation_noise
            self._R_odom[7, 7] = self._odom_angular_velocity_noise

            # 里程计跳变处理
            if self.position_jump > self.jump_thresh:
                jump_scale = min(self.position_jump / self.jump_thresh, 10.0)
                # 原地修改 R 的前三行前三列
                self._R_odom[0:3, 0:3] *= (jump_scale ** 2)
                logger.warning(
                    f"Odom position jump detected: {self.position_jump:.3f}m > {self.jump_thresh}m, "
                    f"increasing position measurement noise by {jump_scale**2:.1f}x"
                )

            self._kalman_update(self._z_odom, self._H_odom, self._R_odom, 
                              angle_indices=[StateIdx.YAW],
                              buffers=(self._temp_H8_P, self._temp_H8_P_HT, self._temp_K_8, self._temp_K_y_8, self._temp_PHt_8, self._temp_K_S_8, self._temp_P_update_11),
                              y_buffer=self._y_odom)
        finally:
            self._release_lock()
    
    def update_imu(self, imu: Imu, current_time: float) -> None:
        """
        IMU 观测更新
        
        Args:
           imu: IMU 数据
           current_time: 当前时间戳
           
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

        # 安全关键: 检查 IMU 数据有效性
        # 传感器故障可能返回 NaN/Inf，必须在处理前检测
        accel = imu.linear_acceleration
        gyro = imu.angular_velocity

        if not (np.isfinite(accel[0]) and np.isfinite(accel[1]) and np.isfinite(accel[2])):
            if not self._warning_logged.get('accel_nan', False):
                logger.warning(
                    f"Invalid IMU acceleration data (NaN/Inf): "
                    f"ax={accel[0]}, ay={accel[1]}, az={accel[2]}. Skipping IMU update."
                )
                self._warning_logged['accel_nan'] = True
            return

        if not (np.isfinite(gyro[0]) and np.isfinite(gyro[1]) and np.isfinite(gyro[2])):
            if not self._warning_logged.get('gyro_nan', False):
                logger.warning(
                    f"Invalid IMU angular velocity data (NaN/Inf): "
                    f"wx={gyro[0]}, wy={gyro[1]}, wz={gyro[2]}. Skipping IMU update."
                )
                self._warning_logged['gyro_nan'] = True
            return

        self._acquire_lock()
        try:
            # 资源有效性检查
            if self._F is None:
                return
            
            self._update_imu_internal(imu, accel, gyro, current_time)
        finally:
            self._release_lock()

    def _update_imu_internal(self, imu: Imu, accel, gyro, current_time: float) -> None:
        """IMU 更新的内部实现（已在锁保护下调用）"""
        self.gyro_z = imu.angular_velocity[2]
        theta = self.x[StateIdx.YAW]  # yaw 角
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
            try:
                # 尝试提取四元数分量
                # 支持多种数据格式：tuple, list, numpy array, 或带索引的自定义对象
                if hasattr(q, '__len__') and len(q) == 4:
                    qx, qy, qz, qw = float(q[0]), float(q[1]), float(q[2]), float(q[3])
                elif hasattr(q, 'x') and hasattr(q, 'y') and hasattr(q, 'z') and hasattr(q, 'w'):
                    # 支持 geometry_msgs/Quaternion 风格的对象
                    qx, qy, qz, qw = float(q.x), float(q.y), float(q.z), float(q.w)
                else:
                    # 不支持的格式
                    qx, qy, qz, qw = None, None, None, None
                
                if qx is not None:
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
                            roll, pitch, _ = euler_from_quaternion((qx, qy, qz, qw))
                            
                            # 检查 euler_from_quaternion 返回值是否有效
                            # 虽然 euler_from_quaternion 内部有归一化，但仍可能返回 NaN
                            if not (np.isfinite(roll) and np.isfinite(pitch)):
                                # 欧拉角计算失败，使用默认姿态
                                if not self._warning_logged['euler_nan']:
                                    logger.warning(
                                        f"euler_from_quaternion returned NaN/Inf: "
                                        f"roll={roll}, pitch={pitch}, q=({qx},{qy},{qz},{qw})"
                                    )
                                    self._warning_logged['euler_nan'] = True
                                use_imu_orientation = False
                            # 额外检查: roll 和 pitch 应该在合理范围内
                            # 使用配置的最大倾斜角阈值
                            elif abs(roll) < self.max_tilt_angle and abs(pitch) < self.max_tilt_angle:
                                use_imu_orientation = True
                            else:
                                # roll/pitch 超出最大倾斜角阈值，使用默认姿态
                                # 这可能是传感器故障或机器人翻倒
                                if not self._warning_logged['tilt_exceeded']:
                                    logger.warning(
                                        f"IMU tilt angle exceeded max_tilt_angle ({np.degrees(self.max_tilt_angle):.1f}°): "
                                        f"roll={np.degrees(roll):.1f}°, pitch={np.degrees(pitch):.1f}°. "
                                        f"Using default horizontal orientation."
                                    )
                                    self._warning_logged['tilt_exceeded'] = True
                                use_imu_orientation = False
            except (TypeError, ValueError, IndexError, AttributeError) as e:
                # 四元数数据格式错误，使用默认姿态
                # 只在首次遇到时记录警告，避免日志泛滥
                if not self._warning_logged['quaternion_error']:
                    logger.warning(f"Invalid quaternion format in IMU data: {e}")
                    self._warning_logged['quaternion_error'] = True
                use_imu_orientation = False
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
        ax_body_true = imu.linear_acceleration[0] - g_measured_x - self.x[StateIdx.ACCEL_BIAS_X]
        ay_body_true = imu.linear_acceleration[1] - g_measured_y - self.x[StateIdx.ACCEL_BIAS_Y]
        az_body_true = imu.linear_acceleration[2] - g_measured_z - self.x[StateIdx.ACCEL_BIAS_Z]
        
        # 将真实加速度转换到世界坐标系
        # 修复: 使用完整的四元数进行 3D 旋转，以支持无人机 (Quadrotor) 或倾斜地形
        # 之前的实现假设 Roll/Pitch ~ 0，这会导致 3D 平台下的严重偏差
        
        # 优化: 使用显式代数运算代替 NumPy 数组分配和 np.cross，显著减少高频 IMU 更新的开销
        
        # 提取四元数分量并归一化
        # 防御性编程: 确保四元数是单位四元数，避免非单位四元数导致的旋转缩放误差
        qx, qy, qz, qw = imu.orientation[0], imu.orientation[1], imu.orientation[2], imu.orientation[3]
        
        # 高性能归一化: 使用平方和避免 sqrt 两次调用
        q_norm_sq = qx*qx + qy*qy + qz*qz + qw*qw
        # 仅在明显偏离单位四元数时进行归一化 (容差 0.1%)
        # 这避免了正常情况下的除法开销
        if abs(q_norm_sq - 1.0) > 1e-3:
            if q_norm_sq > 1e-10:
                q_norm_inv = 1.0 / np.sqrt(q_norm_sq)
                qx, qy, qz, qw = qx * q_norm_inv, qy * q_norm_inv, qz * q_norm_inv, qw * q_norm_inv
            else:
                # 四元数接近零，使用单位四元数 (无旋转)
                logger.warning("IMU quaternion near zero, using identity rotation")
                qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
        
        # 目标向量 v = [ax_body, ay_body, az_body]
        vx, vy, vz = ax_body_true, ay_body_true, az_body_true
        
        # 四元数旋转向量 v' = v + 2 * cross(q_xyz, cross(q_xyz, v) + qw * v)
        # 展开计算以避免分配:
        # t = 2 * cross(q_xyz, v)
        # v' = v + qw * t + cross(q_xyz, t)
        
        # t = 2 * cross(q_xyz, v)
        tx = 2.0 * (qy * vz - qz * vy)
        ty = 2.0 * (qz * vx - qx * vz)
        tz = 2.0 * (qx * vy - qy * vx)
        
        # v_prime = v + qw * t + cross(q_xyz, t)
        # cross(q_xyz, t)
        # cx = qy * tz - qz * ty
        # cy = qz * tx - qx * tz
        # cz = qx * ty - qy * tx
        
        ax_world = vx + qw * tx + (qy * tz - qz * ty)
        ay_world = vy + qw * ty + (qz * tx - qx * tz)
        # az_world = vz + qw * tz + (qx * ty - qy * tx)
        
        # 打滑检测：比较 IMU 测量的加速度 (Body Frame) 与 Odom 计算的加速度 (Body Frame)
        # 修正: 统一在 Body Frame 下进行比较，避免 Yaw 漂移 带来的虚假打滑报警
        
        odom_accel_fresh = (
            self._body_accel_initialized and 
            self.last_odom_time is not None and
            (current_time - self.last_odom_time) < self.accel_freshness_thresh
        )
        
        if odom_accel_fresh:
            # 1. 获取 IMU 真实加速度 (Body Frame, 去重力和 Bias)
            # ax_body_true, ay_body_true 已经在上面计算好了
            imu_accel_body_vec = np.array([ax_body_true, ay_body_true])
            
            # 2. 获取 Odom 加速度 (Body Frame)
            # self.body_accel_vec 已经在 update_odom 中计算好
            
            # 3. 比较模长差
            accel_diff = np.linalg.norm(imu_accel_body_vec - self.body_accel_vec)
            
            self.slip_probability = self._compute_slip_probability(accel_diff)
            self.slip_detected = self.slip_probability > 0.5
        else:
            # odom 加速度数据过旧，不进行打滑检测
            # 保持上一次的打滑状态，但基于时间间隔衰减概率
            if self.slip_probability > 0 and self.last_imu_time is not None:
                dt_decay = current_time - self.last_imu_time
                if dt_decay > 0:
                    # 基于时间的指数衰减: P(t) = P(0) * exp(-decay_rate * dt)
                    decay_factor = np.exp(-self.slip_decay_rate * dt_decay)
                    self.slip_probability *= decay_factor
            self.slip_detected = self.slip_probability > 0.5
        
        self.last_imu_time = current_time
        
        # 复用预分配的观测向量
        self._z_imu[0] = imu.linear_acceleration[0]
        self._z_imu[1] = imu.linear_acceleration[1]
        self._z_imu[2] = imu.linear_acceleration[2]
        self._z_imu[3] = imu.angular_velocity[2]
        
        # H 矩阵是 constant 且已预分配 (self._H_imu)
        
        # 计算期望值
        bias_x = self.x[StateIdx.ACCEL_BIAS_X]
        bias_y = self.x[StateIdx.ACCEL_BIAS_Y]
        bias_z = self.x[StateIdx.ACCEL_BIAS_Z]
        omega_current = self.x[StateIdx.YAW_RATE]
        
        if self.imu_motion_compensation and self._body_accel_initialized:
            # Motion Compensation in Body Frame
            # Accel_measured_expected = a_kinematic_body + gravity_body
            # a_kinematic_body is approximated by Odom derived body acceleration
            
            self._z_imu_expected[0] = bias_x + self.body_accel_vec[0] + g_measured_x
            self._z_imu_expected[1] = bias_y + self.body_accel_vec[1] + g_measured_y
            self._z_imu_expected[2] = bias_z + g_measured_z
            self._z_imu_expected[3] = omega_current
        else:
            self._z_imu_expected[0] = bias_x + g_measured_x
            self._z_imu_expected[1] = bias_y + g_measured_y
            self._z_imu_expected[2] = bias_z + g_measured_z
            self._z_imu_expected[3] = omega_current
        
        self._kalman_update_with_expected(self._z_imu, self._z_imu_expected, self._H_imu, self.R_imu,
                                        buffers=(self._temp_H4_P, self._temp_H4_P_HT, self._temp_K_4, self._temp_K_y_4, self._temp_PHt_4, self._temp_K_S_4, self._temp_P_update_11),
                                        y_buffer=self._y_imu)
    
    def _kalman_update(self, z: np.ndarray, H: np.ndarray, R: np.ndarray, 
                       angle_indices: list = None,
                       buffers: tuple = None,
                       y_buffer: np.ndarray = None) -> None:
        """
        卡尔曼更新
        
        Args:
            z: 观测向量
            H: 观测矩阵
            R: 测量噪声矩阵
            angle_indices: 观测向量中角度元素的索引列表，用于角度归一化
            buffers: 预分配的 Buffer 元组，用于性能优化
            y_buffer: 创新 (Innovation) 向量 y 的 buffer，避免 H @ x 分配
        """
        if y_buffer is not None:
            # Zero-alloc impl
            # 1. Hx = H @ x (Store in y_buffer temporarily)
            np.matmul(H, self.x, out=y_buffer)
            # 2. y = z - Hx (Store result in y_buffer)
            np.subtract(z, y_buffer, out=y_buffer)
            y = y_buffer
        else:
            # Fallback
            y = z - H @ self.x 
        
        # 对角度元素进行归一化
        if angle_indices:
            for idx in angle_indices:
                y[idx] = normalize_angle(y[idx])
        
        self._apply_kalman_gain(y, H, R, buffers)
    
    def _kalman_update_with_expected(self, z: np.ndarray, z_expected: np.ndarray,
                                     H: np.ndarray, R: np.ndarray,
                                     buffers: tuple = None,
                                     y_buffer: np.ndarray = None):
        if y_buffer is not None:
            np.subtract(z, z_expected, out=y_buffer)
            y = y_buffer
        else:
            y = z - z_expected
             
        self._apply_kalman_gain(y, H, R, buffers)
    
    def _apply_kalman_gain(self, y: np.ndarray, H: np.ndarray, R: np.ndarray,
                          buffers: tuple = None) -> None:
        """应用卡尔曼增益 (零内存分配版)"""
        
        if buffers is None:
            # Fallback for simplicity or legacy calls
            self._apply_kalman_gain_legacy(y, H, R)
            return

        # 解包 Buffers
        # buffers=(temp_H_P, temp_H_P_HT, temp_K, temp_K_y, temp_PHt, temp_K_S, temp_P_update)
        # 注意: 传入的 buffers 需要根据 H 的维度匹配
        (t_HP, t_HPHT, t_K, t_Ky, t_PHt, t_KS, t_P_upd) = buffers
        
        # 1. S = H @ P @ H.T + R
        # Step 1.1: t_HP = H @ P
        np.matmul(H, self.P, out=t_HP)
        # Step 1.2: t_HPHT = t_HP @ H.T
        np.matmul(t_HP, H.T, out=t_HPHT)
        # Step 1.3: S = t_HPHT + R (S reuse t_HPHT)
        # 注意: R 是对角/对称阵，这里直接加
        np.add(t_HPHT, R, out=t_HPHT) # Now t_HPHT holds S
        S = t_HPHT
        
        # 2. 计算 Kalman Gain K
        # K = P @ H.T @ S_inv
        
        # Step 2.1: t_PHt = P @ H.T
        # 这个其实就是 t_HP.T，因为 P 对称。但为了稳健还是乘一下
        # 优化: t_PHt = t_HP.T (Zero copy view if shape matches?) 
        # t_HP shape (M, N), t_PHt shape (N, M). 
        # 直接拿 t_HP.T 并在后续使用 solve 可能更快，但这里为了利用 buffer显式计算
        np.matmul(self.P, H.T, out=t_PHt)
        
        # Step 2.2: Compute K
        # 由于无法做 inplace solve (S_inv)，我们仍需解方程
        # K = t_PHt @ inv(S) -> K @ S = t_PHt -> K = solve(S.T, t_PHt.T).T
        # 这里的 solve 依然会分配内存，这很难避免，除非手写 Cholesky
        try:
            # 使用 Cholesky 分解 (比 inv 快)
            # L = cholesky(S)
            # K = t_PHt @ inv(S)
            # solve is: x = solve(a, b) -> ax = b
            # We want K = P H' S^-1 => K S = P H'
            # S is symmetric
            # scipy.linalg.solve(S, (P H').T, assume_a='pos').T
            # 这里用 numpy standard solve: K = solve(S, t_PHt.T).T
            # 分配不可避免: S_inv or intermediate
            K_trans = np.linalg.solve(S, t_PHt.T)
            t_K[:] = K_trans.T # Store in t_K
        except np.linalg.LinAlgError:
            # Fallback
            K_legacy = self.P @ H.T @ np.linalg.pinv(S)
            t_K[:] = K_legacy
             
        # 3. 更新状态 x = x + K @ y
        np.matmul(t_K, y, out=t_Ky)
        np.add(self.x, t_Ky, out=self.x)
        
        # 4. 更新协方差 P (Joseph Form: P = (I - KH)P(I-KH)' + KRK')
        # Simplified: P = P - K @ S @ K.T (Optimal for symmetric)
        # Step 4.1: t_KS = K @ S
        np.matmul(t_K, S, out=t_KS)
        # Step 4.2: term = t_KS @ K.T
        np.matmul(t_KS, t_K.T, out=t_P_upd)
        
        # Step 4.3: P = P - term
        np.subtract(self.P, t_P_upd, out=self.P)
        
        self._ensure_positive_definite()
        self.last_innovation_norm = np.linalg.norm(y)
        self.x[StateIdx.YAW] = normalize_angle(self.x[StateIdx.YAW])

    
    def _apply_kalman_gain_legacy(self, y: np.ndarray, H: np.ndarray, R: np.ndarray) -> None:
        """应用卡尔曼增益 (legacy implementation)"""
        # S = H P H.T + R
        S = H @ self.P @ H.T + R
        
        try:
            # 使用 Cholesky 分解求逆更加数值稳定且快
            L = np.linalg.cholesky(S)
            # K = P H.T S^-1
            S_inv = np.linalg.solve(L.T, np.linalg.solve(L, np.eye(len(S))))
            K = self.P @ H.T @ S_inv
        except np.linalg.LinAlgError:
            # 退化回伪逆
            K = self.P @ H.T @ np.linalg.pinv(S)
        
        # 更新状态 x = x + K y
        self.x = self.x + K @ y
        
        # 更新协方差 P = P - K @ S @ K.T
        term = K @ S @ K.T
        self.P = self.P - term
        
        self._ensure_positive_definite()
        self.last_innovation_norm = np.linalg.norm(y)
        
        self.x[StateIdx.YAW] = normalize_angle(self.x[StateIdx.YAW])
    
    def _update_jacobian_F(self, dt: float, theta: float, omega: float, 
                          vx: float, vy: float) -> None:
        """
        更新状态转移 Jacobian 矩阵 (原地修改 self._F)
        """
        # 我们只修改变动的项。_F 在 init 中被设为 eye(11)
        
        # 必须先清除上一帧可能设置的值（特别是那些根据条件分支设置的值）
        self._F[StateIdx.VX, StateIdx.YAW] = 0.0
        self._F[StateIdx.VY, StateIdx.YAW] = 0.0
        self._F[StateIdx.VX, StateIdx.YAW_RATE] = 0.0
        self._F[StateIdx.VY, StateIdx.YAW_RATE] = 0.0
        
        # 更新标准项
        self._F[StateIdx.X, StateIdx.VX] = dt  # px/vx
        self._F[StateIdx.Y, StateIdx.VY] = dt  # py/vy
        self._F[StateIdx.Z, StateIdx.VZ] = dt  # pz/vz
        self._F[StateIdx.YAW, StateIdx.YAW_RATE] = dt  # theta/omega
        
        if self.velocity_heading_coupled:
            cos_theta = np.cos(theta)
            sin_theta = np.sin(theta)
            
            # 计算带符号的速度投影
            v_signed = vx * cos_theta + vy * sin_theta
            
            # 平滑符号函数处理
            epsilon = self.jacobian_smooth_epsilon
            smooth_sign = np.tanh(v_signed / epsilon)
            v_magnitude_smooth = np.sqrt(v_signed**2 + self.min_velocity_for_jacobian**2)
            effective_v = smooth_sign * v_magnitude_smooth
            
            # ∂vx_world/∂theta
            self._F[StateIdx.VX, StateIdx.YAW] = -effective_v * sin_theta
            self._F[StateIdx.VY, StateIdx.YAW] = effective_v * cos_theta
            
            # ∂vx_world/∂omega
            self._F[StateIdx.VX, StateIdx.YAW_RATE] = -effective_v * sin_theta * dt
            self._F[StateIdx.VY, StateIdx.YAW_RATE] = effective_v * cos_theta * dt
    
    def _ensure_positive_definite(self) -> None:
        """确保协方差矩阵正定
        
        优化策略:
        1. 首先强制对称性（低成本操作）
        2. 尝试 Cholesky 分解检测正定性（比特征值分解快）
        3. 只有在 Cholesky 失败时才进行特征值分解修复
        
        这种分层策略在正常情况下避免了昂贵的特征值分解，
        只在协方差矩阵出现问题时才进行完整修复。
        """
        # 强制对称性（低成本操作，总是执行）
        self.P = (self.P + self.P.T) / 2
        
        # 尝试 Cholesky 分解检测正定性
        # Cholesky 分解比特征值分解快约 3 倍
        try:
            np.linalg.cholesky(self.P)
            # Cholesky 成功，矩阵已经正定，无需进一步处理
            return
        except np.linalg.LinAlgError:
            # Cholesky 失败，需要修复
            pass
        
        # 使用特征值分解修复非正定矩阵
        try:
            eigenvalues, eigenvectors = np.linalg.eigh(self.P)
            if np.any(eigenvalues < self.min_eigenvalue):
                eigenvalues = np.maximum(eigenvalues, self.min_eigenvalue)
                self.P = eigenvectors @ np.diag(eigenvalues) @ eigenvectors.T
        except np.linalg.LinAlgError:
            # 特征值分解也失败，重置为初始协方差
            self.P = np.eye(11) * self.initial_covariance
    
    def _get_adaptive_slip_threshold(self) -> float:
        current_velocity = np.linalg.norm(self.x[StateIdx.VX : StateIdx.VZ + 1])
        return self.base_slip_thresh + self.slip_velocity_factor * current_velocity
    
    def _is_stationary(self) -> bool:
        return np.linalg.norm(self.x[StateIdx.VX : StateIdx.VZ + 1]) < self.stationary_thresh
    
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
        theta_var = self.P[StateIdx.YAW, StateIdx.YAW]
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

    def _detect_anomalies_unlocked(self) -> List[str]:
        """检测估计器异常（内部无锁版本）

        检测以下异常情况:
        - SLIP_DETECTED: 检测到轮子打滑
        - IMU_DRIFT: IMU 漂移（静止时陀螺仪有输出）
        - ODOM_JUMP: 里程计位置跳变
        - IMU_UNAVAILABLE: IMU 数据不可用
        - COVARIANCE_EXPLOSION: 协方差矩阵发散
        - INNOVATION_ANOMALY: 测量创新度异常大

        Returns:
            异常标识列表
        """
        anomalies = []

        if self.slip_detected:
            anomalies.append("SLIP_DETECTED")

        # Use raw odom velocity for drift detection to avoid circular dependency
        # If EKF is drifting, _is_stationary() (based on self.x) might return False, masking the drift.
        # Fix: Check both linear and angular velocity for stationarity to support in-place rotation
        is_linear_stationary = self._raw_odom_twist_norm < self.stationary_thresh
        is_angular_stationary = True
        if hasattr(self, '_raw_odom_angular_velocity'):
             is_angular_stationary = abs(self._raw_odom_angular_velocity) < self.stationary_thresh
             
        is_physically_stationary = is_linear_stationary and is_angular_stationary
        
        if is_physically_stationary and abs(self.gyro_z) > self.drift_thresh:
            anomalies.append("IMU_DRIFT")
            self._imu_drift_detected = True
        else:
            self._imu_drift_detected = False

        if self.position_jump > self.jump_thresh:
            anomalies.append("ODOM_JUMP")

        if not self._imu_available:
            anomalies.append("IMU_UNAVAILABLE")

        # 协方差爆炸检测
        covariance_norm = np.linalg.norm(self.P[:8, :8])
        if covariance_norm > self.covariance_explosion_thresh:
            anomalies.append("COVARIANCE_EXPLOSION")
            logger.warning(f"Covariance explosion detected: norm={covariance_norm:.2f}")

        # 创新度异常检测
        if self.last_innovation_norm > self.innovation_anomaly_thresh:
            anomalies.append("INNOVATION_ANOMALY")

        return anomalies

    def detect_anomalies(self) -> List[str]:
        """检测估计器异常（线程安全）

        检测以下异常情况:
        - SLIP_DETECTED: 检测到轮子打滑
        - IMU_DRIFT: IMU 漂移（静止时陀螺仪有输出）
        - ODOM_JUMP: 里程计位置跳变
        - IMU_UNAVAILABLE: IMU 数据不可用
        - COVARIANCE_EXPLOSION: 协方差矩阵发散
        - INNOVATION_ANOMALY: 测量创新度异常大

        Returns:
            异常标识列表
        """
        self._acquire_lock()
        try:
            return self._detect_anomalies_unlocked()
        finally:
            self._release_lock()

    def get_state(self) -> EstimatorOutput:
        """获取当前状态估计结果（线程安全）"""
        self._acquire_lock()
        try:
            if self.P is None:  # Shutdown check
                return EstimatorOutput(
                    state=np.zeros(3), covariance=np.zeros((3,3)), covariance_norm=0, 
                    innovation_norm=0, imu_bias=np.zeros(3), slip_probability=0, anomalies=['SHUTDOWN']
                )

            return EstimatorOutput(
                state=self.x[:StateIdx.ACCEL_BIAS_X].copy(),
                covariance=self.P[:StateIdx.ACCEL_BIAS_X, :StateIdx.ACCEL_BIAS_X].copy(),
                covariance_norm=np.linalg.norm(self.P[:StateIdx.ACCEL_BIAS_X, :StateIdx.ACCEL_BIAS_X]),
                innovation_norm=self.last_innovation_norm,
                imu_bias=self.x[StateIdx.ACCEL_BIAS_X : StateIdx.ACCEL_BIAS_Z + 1].copy(),
                slip_probability=self.slip_probability,
                anomalies=self._detect_anomalies_unlocked(),
                imu_available=self._imu_available,
                imu_drift_detected=self._imu_drift_detected
            )
        finally:
            self._release_lock()

    def reset(self, initial_state: Optional[np.ndarray] = None) -> None:
        """重置估计器状态（线程安全）
        
        Args:
            initial_state: 可选的初始状态向量 (11维). 如果提供，将立即初始化为该状态;
                          否则将重置为 False 等待第一次 Odom 更新.
        """
        self._acquire_lock()
        try:
            self.x = np.zeros(11)
            self.P = np.eye(11) * self.initial_covariance
            self.slip_detected = False
            self.slip_probability = 0.0
            self.last_innovation_norm = 0.0
            self.slip_history.clear()
            self.last_imu_time = None
            self.last_odom_time = None
            
            # 修复: 变量名必须与 __init__ 和 update_odom 中使用的 Body Frame 变量一致
            # 之前错误地使用了 world_ 前缀，导致 reset 无效，引发暂停后的打滑误报
            self.last_body_velocity = np.zeros(2)
            self.current_body_velocity = np.zeros(2)
            self._raw_odom_twist_norm = 0.0  # 必须重置，否则 static check 可能失效
            self._raw_odom_angular_velocity = 0.0 # Fix: 确保状态完全重置
            self.body_accel_vec = np.zeros(2)
            self._body_accel_initialized = False
            
            if initial_state is not None and len(initial_state) == 11:
                self.x = initial_state.copy()
                self._initialized = True
            else:
                self._initialized = False
            
            self.last_position = np.zeros(3)
            self.position_jump = 0.0
            self.gyro_z = 0.0
            self._imu_available = True
            self._imu_drift_detected = False
            self._last_odom_orientation = None
            # 重置日志节流标志，允许重置后再次记录警告
            for key in self._warning_logged:
                self._warning_logged[key] = False
        finally:
            self._release_lock()

    def shutdown(self) -> None:
        """关闭并释放所有资源"""
        self._acquire_lock()
        try:
            # 清理状态
            self.reset()
            # 释放大数组引用，帮助 GC
            self._F = None
            self._H_odom = None
            self._temp_F_P = None
            self._temp_FP_FT = None
            self._temp_P_update_11 = None
            # ... 其他 buffers
            logger.info("AdaptiveEKFEstimator shutdown complete")
        finally:
            self._release_lock()
