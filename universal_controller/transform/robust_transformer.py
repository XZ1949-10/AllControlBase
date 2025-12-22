"""
鲁棒坐标变换器

实现功能:
- F3.1: 补偿网络推理延迟
- F3.2: 使用 TF2 lookupTransform 获取坐标变换
- F3.3: TF2 降级策略 (降级到 odom 积分，记录持续时间，恢复后校正)
- F3.4: TF2 降级超过 1000ms 触发 MPC_DEGRADED 状态
- F3.5: TF2 恢复时校正状态估计器的漂移
"""
from typing import Dict, Any, Tuple, Optional
import numpy as np
import logging

from ..core.interfaces import ICoordinateTransformer, IStateEstimator
from ..core.data_types import Trajectory, Point3D
from ..core.enums import TransformStatus
from ..core.ros_compat import (
    tf2_ros, tft, ROS_AVAILABLE, TF2_AVAILABLE,
    TF2LookupException, TF2ExtrapolationException, TF2ConnectivityException,
    get_current_time, create_time, create_duration,
    euler_from_quaternion, quaternion_from_euler,
    normalize_angle,
    get_monotonic_time
)

logger = logging.getLogger(__name__)


class RobustCoordinateTransformer(ICoordinateTransformer):
    """
    带降级时限和恢复校正的坐标变换器
    
    TF2 集成说明:
    - 优先使用 TF2 进行坐标变换
    - TF2 不可用时降级到 odom 积分
    - 降级超过阈值时触发警告/临界状态
    - TF2 恢复时执行漂移校正
    """
    
    def __init__(self, config: Dict[str, Any]):
        transform_config = config.get('transform', config)
        
        # 配置参数
        self.fallback_duration_limit = transform_config.get('fallback_duration_limit_ms', 500) / 1000.0
        self.fallback_critical_limit = transform_config.get('fallback_critical_limit_ms', 1000) / 1000.0
        self.tf2_timeout = transform_config.get('tf2_timeout_ms', 10) / 1000.0
        self.drift_estimation_enabled = transform_config.get('drift_estimation_enabled', True)
        self.recovery_correction_enabled = transform_config.get('recovery_correction_enabled', True)
        self.drift_rate = transform_config.get('drift_rate', 0.01)
        self.max_drift_dt = transform_config.get('max_drift_dt', 0.5)  # 从配置读取
        self.source_frame = transform_config.get('source_frame', 'base_link')
        
        # 状态变量
        self.fallback_start_time: Optional[float] = None
        self.accumulated_drift = 0.0
        self.state_estimator: Optional[IStateEstimator] = None
        
        # TF2 缓存
        self._last_tf2_position: Optional[np.ndarray] = None
        self._last_tf2_yaw: float = 0.0
        self._fallback_start_tf2_position: Optional[np.ndarray] = None
        self._fallback_start_tf2_yaw: float = 0.0
        self._fallback_start_estimator_position: Optional[np.ndarray] = None
        self._fallback_start_estimator_theta: float = 0.0
        self._last_status = TransformStatus.TF2_OK
        self._last_fallback_update_time: Optional[float] = None  # 漂移估计时间跟踪
        
        # TF2 Buffer 和 Listener
        self._tf_buffer: Optional[Any] = None
        self._tf_listener: Optional[Any] = None
        self._tf2_initialized = False
        self._tf2_force_disabled = False  # 用于测试时强制禁用 TF2
        
        # 初始化 TF2
        self._initialize_tf2()
    
    def _initialize_tf2(self) -> None:
        """初始化 TF2 Buffer 和 Listener"""
        if TF2_AVAILABLE and not self._tf2_force_disabled:
            try:
                self._tf_buffer = tf2_ros.Buffer()
                self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
                self._tf2_initialized = True
                logger.info("TF2 initialized successfully")
            except Exception as e:
                logger.warning(f"TF2 initialization failed: {e}")
                self._tf2_initialized = False
        else:
            # 非 ROS 环境，使用模拟 Buffer
            self._tf_buffer = tf2_ros.Buffer()
            self._tf2_initialized = True
            logger.debug("Using mock TF2 buffer")
    
    def set_tf2_available(self, available: bool) -> None:
        """
        设置 TF2 可用性 (用于测试)
        
        注意: 这会强制禁用/启用 TF2，即使真实 TF2 可用
        """
        self._tf2_force_disabled = not available
        if not available:
            logger.debug("TF2 force disabled for testing")
    
    def set_transform(self, parent_frame: str, child_frame: str,
                     x: float, y: float, z: float, yaw: float,
                     stamp: Optional[float] = None) -> None:
        """
        设置变换 (用于测试或手动设置)
        
        Args:
            parent_frame: 父坐标系
            child_frame: 子坐标系
            x, y, z: 平移
            yaw: 航向角 (弧度)
            stamp: 时间戳 (秒)，None 表示当前时间
        """
        if self._tf_buffer is None:
            return
        
        from ..mock.data_mock import MockTransformStamped, MockHeader, MockTransform, MockVector3, MockQuaternion
        
        transform = MockTransformStamped()
        transform.header = MockHeader()
        transform.header.frame_id = parent_frame
        transform.header.stamp = create_time(stamp if stamp is not None else get_current_time())
        transform.child_frame_id = child_frame
        
        q = quaternion_from_euler(0, 0, yaw)
        transform.transform = MockTransform()
        transform.transform.translation = MockVector3(x, y, z)
        transform.transform.rotation = MockQuaternion(q[0], q[1], q[2], q[3])
        
        self._tf_buffer.set_transform(transform, "manual")
    
    def _try_tf2_lookup(self, target_frame: str, source_frame: str,
                       target_time: float) -> Tuple[Optional[np.ndarray], Optional[float], bool]:
        """
        尝试 TF2 查找
        
        Returns:
            (position, yaw, success): 位置 [x, y, z], 航向角, 是否成功
        """
        if self._tf_buffer is None or self._tf2_force_disabled:
            return None, None, False
        
        try:
            # 创建时间对象
            lookup_time = create_time(target_time)
            timeout = create_duration(self.tf2_timeout)
            
            # 查找变换
            transform = self._tf_buffer.lookup_transform(
                target_frame, source_frame, lookup_time, timeout)
            
            # 提取位置
            t = transform.transform.translation
            position = np.array([t.x, t.y, t.z])
            
            # 提取航向角
            r = transform.transform.rotation
            q = (r.x, r.y, r.z, r.w)
            _, _, yaw = euler_from_quaternion(q)
            
            # 更新缓存
            self._last_tf2_position = position.copy()
            self._last_tf2_yaw = yaw
            
            return position, yaw, True
            
        except (TF2LookupException, TF2ExtrapolationException, TF2ConnectivityException) as e:
            # TF2 查找失败
            return None, None, False
        except Exception as e:
            logger.warning(f"TF2 lookup error: {e}")
            return None, None, False
    
    def transform_trajectory(self, traj: Trajectory, target_frame: str, 
                            target_time: float) -> Tuple[Trajectory, TransformStatus]:
        """
        变换轨迹到目标坐标系
        
        实现 F3.2: 使用 TF2 lookupTransform 获取坐标变换
        实现 F3.3: TF2 降级策略
        """
        source_frame = traj.header.frame_id if traj.header.frame_id else self.source_frame
        
        # 尝试 TF2 查找
        position, yaw, tf2_success = self._try_tf2_lookup(target_frame, source_frame, target_time)
        
        if tf2_success:
            # TF2 可用
            if self.fallback_start_time is not None:
                # 从降级状态恢复
                if self.recovery_correction_enabled:
                    self._apply_recovery_correction(position, yaw)
                self.fallback_start_time = None
                self.accumulated_drift = 0.0
                # 重置漂移估计时间跟踪
                if hasattr(self, '_last_fallback_update_time'):
                    self._last_fallback_update_time = None
                logger.info("TF2 recovered, drift corrected")
            
            self._last_status = TransformStatus.TF2_OK
            
            # 应用变换
            transformed_traj = self._apply_tf2_transform(traj, position, yaw, target_frame)
            return transformed_traj, TransformStatus.TF2_OK
        else:
            # TF2 不可用，降级处理
            return self._handle_fallback(traj, target_time, target_frame)
    
    def _apply_tf2_transform(self, traj: Trajectory, position: np.ndarray,
                            yaw: float, target_frame: str) -> Trajectory:
        """应用 TF2 变换到轨迹"""
        cos_yaw = np.cos(yaw)
        sin_yaw = np.sin(yaw)
        
        transformed_points = []
        for p in traj.points:
            # 旋转 + 平移
            new_x = p.x * cos_yaw - p.y * sin_yaw + position[0]
            new_y = p.x * sin_yaw + p.y * cos_yaw + position[1]
            new_z = p.z + position[2]
            transformed_points.append(Point3D(new_x, new_y, new_z))
        
        # 变换速度 (如果有)
        new_velocities = None
        if traj.velocities is not None and len(traj.velocities) > 0:
            new_velocities = traj.velocities.copy()
            for i in range(len(new_velocities)):
                vx, vy = new_velocities[i, 0], new_velocities[i, 1]
                new_velocities[i, 0] = vx * cos_yaw - vy * sin_yaw
                new_velocities[i, 1] = vx * sin_yaw + vy * cos_yaw
        
        new_traj = traj.copy()
        new_traj.points = transformed_points
        new_traj.velocities = new_velocities
        new_traj.header.frame_id = target_frame
        return new_traj
    
    def _handle_fallback(self, traj: Trajectory, target_time: float,
                        target_frame: str) -> Tuple[Trajectory, TransformStatus]:
        """
        处理 TF2 不可用时的降级
        
        实现 F3.3: TF2 降级策略
        - 降级到 odom 积分时记录持续时间
        - 降级超过 500ms 触发更高级别警告
        - 降级超过 1000ms 触发临界状态 (F3.4)
        """
        current_time = get_monotonic_time()
        
        if self.fallback_start_time is None:
            # 开始降级
            self.fallback_start_time = current_time
            logger.warning("TF2 fallback started")
            
            if self.state_estimator is not None:
                state = self.state_estimator.get_state()
                self._fallback_start_estimator_position = state.state[:3].copy()
                self._fallback_start_estimator_theta = state.state[6]
                if self._last_tf2_position is not None:
                    self._fallback_start_tf2_position = self._last_tf2_position.copy()
                    self._fallback_start_tf2_yaw = self._last_tf2_yaw
        
        fallback_duration = current_time - self.fallback_start_time
        
        # 累积漂移估计
        # 使用实际时间间隔而非假设的固定频率
        if self.drift_estimation_enabled:
            # 计算自上次更新以来的时间间隔
            if self._last_fallback_update_time is None:
                self._last_fallback_update_time = self.fallback_start_time
            
            dt = current_time - self._last_fallback_update_time
            # 限制 dt 在合理范围内，避免异常值 (使用配置值)
            dt = np.clip(dt, 0.0, self.max_drift_dt)
            
            self.accumulated_drift += self.drift_rate * dt
            self._last_fallback_update_time = current_time
        
        # 确定降级状态
        if fallback_duration > self.fallback_critical_limit:
            status = TransformStatus.FALLBACK_CRITICAL
        elif fallback_duration > self.fallback_duration_limit:
            status = TransformStatus.FALLBACK_WARNING
        else:
            status = TransformStatus.FALLBACK_OK
        
        self._last_status = status
        
        # 使用 odom 积分计算变换
        # 
        # 注意坐标系处理:
        # - _fallback_start_tf2_position 和 _fallback_start_tf2_yaw 是 TF2 变换
        #   表示 source_frame 到 target_frame 的变换
        # - delta_position 是状态估计器在世界坐标系 (odom) 下的位移
        # - 如果 TF2 变换包含旋转，需要将 delta_position 旋转到正确的坐标系
        #
        # 对于典型的 base_link -> odom 变换:
        # - TF2 变换表示机器人在 odom 坐标系下的位姿
        # - 状态估计器的位移也是在 odom 坐标系下
        # - 因此可以直接相加
        #
        # 对于更复杂的变换链，需要考虑旋转
        if (self.state_estimator is not None and 
            self._fallback_start_estimator_position is not None and
            self._fallback_start_tf2_position is not None):
            
            state = self.state_estimator.get_state()
            current_estimator_position = state.state[:3]
            current_estimator_theta = state.state[6]
            
            # 计算相对位移（在状态估计器的坐标系下，通常是 odom）
            delta_position = current_estimator_position - self._fallback_start_estimator_position
            delta_theta = current_estimator_theta - self._fallback_start_estimator_theta
            delta_theta = normalize_angle(delta_theta)
            
            # 如果 TF2 变换的 yaw 不为零，需要将 delta_position 旋转到 TF2 坐标系
            # 这是因为 TF2 变换可能包含一个初始旋转
            # 
            # 对于 base_link -> odom 的情况:
            # - _fallback_start_tf2_yaw 是机器人在 odom 坐标系下的初始航向
            # - delta_position 已经是在 odom 坐标系下的位移
            # - 不需要额外旋转
            #
            # 对于其他情况（如 base_link -> map，其中 map 和 odom 有旋转偏移）:
            # - 需要将 delta_position 从 odom 坐标系旋转到 map 坐标系
            # - 但这需要知道 odom -> map 的旋转，这里没有这个信息
            #
            # 因此，这里假设 TF2 变换的目标坐标系与状态估计器的坐标系一致
            # 这是大多数实际应用的情况
            
            # 估计当前变换
            estimated_position = self._fallback_start_tf2_position + delta_position
            estimated_yaw = self._fallback_start_tf2_yaw + delta_theta
            estimated_yaw = normalize_angle(estimated_yaw)
            
            transformed_traj = self._apply_tf2_transform(
                traj, estimated_position, estimated_yaw, target_frame)
            
            return transformed_traj, status
        
        # 无法估计变换，返回原轨迹
        return traj, status
    
    def _apply_recovery_correction(self, current_tf2_position: Optional[np.ndarray] = None,
                                   current_tf2_yaw: Optional[float] = None) -> None:
        """
        计算并应用漂移校正 (F3.5)
        
        漂移校正逻辑:
        - 在 TF2 恢复时，比较 fallback 期间的 odom 积分与实际 TF2 位移
        - 差值即为需要校正的漂移
        - 通过 state_estimator.apply_drift_correction() 应用校正
        
        Args:
            current_tf2_position: TF2 恢复后的真实位置
            current_tf2_yaw: TF2 恢复后的真实航向
        """
        if self.state_estimator is None:
            return
        
        if self._fallback_start_tf2_position is None or self._fallback_start_estimator_position is None:
            return
        
        # 获取当前状态估计位置
        state = self.state_estimator.get_state()
        current_estimator_position = state.state[:3]
        current_estimator_theta = state.state[6]
        
        # 计算 fallback 期间 odom 积分的位移
        odom_displacement = current_estimator_position - self._fallback_start_estimator_position
        odom_theta_change = current_estimator_theta - self._fallback_start_estimator_theta
        odom_theta_change = normalize_angle(odom_theta_change)
        
        # 如果有真实 TF2 位置，计算精确漂移
        if current_tf2_position is not None and current_tf2_yaw is not None:
            # 计算 TF2 实际位移
            tf2_displacement = current_tf2_position - self._fallback_start_tf2_position
            tf2_theta_change = current_tf2_yaw - self._fallback_start_tf2_yaw
            tf2_theta_change = normalize_angle(tf2_theta_change)
            
            # 漂移 = odom 积分 - TF2 实际
            drift_x = odom_displacement[0] - tf2_displacement[0]
            drift_y = odom_displacement[1] - tf2_displacement[1]
            drift_theta = odom_theta_change - tf2_theta_change
            drift_theta = normalize_angle(drift_theta)
            
            # 应用校正 (反向)
            if abs(drift_x) > 0.01 or abs(drift_y) > 0.01 or abs(drift_theta) > 0.01:
                self.state_estimator.apply_drift_correction(-drift_x, -drift_y, -drift_theta)
                logger.info(f"Applied precise drift correction: "
                      f"dx={-drift_x:.4f}, dy={-drift_y:.4f}, dtheta={-drift_theta:.4f}")
        else:
            # 没有真实 TF2 位置，无法计算精确漂移
            # 使用保守策略：不做位置校正，只记录警告
            # 
            # 原因：漂移方向是未知的，盲目猜测可能使情况更糟
            # 例如：如果漂移实际上与运动方向相同，反向校正会加倍误差
            # 
            # 如果需要启用估计校正，应该：
            # 1. 使用更复杂的漂移模型（如基于历史数据的统计模型）
            # 2. 或者使用外部定位系统（如 GPS、UWB）提供真实位置
            drift_magnitude = self.accumulated_drift
            
            if drift_magnitude > 0.02:  # 只有漂移超过 2cm 才记录警告
                logger.warning(f"Accumulated drift estimate: {drift_magnitude:.4f}m, "
                      f"but no TF2 position available for precise correction. "
                      f"Consider enabling external localization for drift correction.")
        
        # 清理状态
        self._fallback_start_tf2_position = None
        self._fallback_start_estimator_position = None
        self._fallback_start_tf2_yaw = 0.0
        self._fallback_start_estimator_theta = 0.0
    
    def set_state_estimator(self, estimator: IStateEstimator) -> None:
        """设置状态估计器"""
        self.state_estimator = estimator
    
    def get_status(self) -> Dict[str, Any]:
        """获取变换器状态"""
        fallback_duration_ms = 0.0
        if self.fallback_start_time is not None:
            fallback_duration_ms = (get_monotonic_time() - self.fallback_start_time) * 1000
        
        return {
            'tf2_available': self.fallback_start_time is None,
            'fallback_duration_ms': fallback_duration_ms,
            'accumulated_drift': self.accumulated_drift,
            'is_critical': self._last_status.is_critical(),
            'status': self._last_status.name,
            'tf2_initialized': self._tf2_initialized
        }
    
    def reset(self) -> None:
        """重置变换器状态"""
        self.fallback_start_time = None
        self.accumulated_drift = 0.0
        self._last_tf2_position = None
        self._last_tf2_yaw = 0.0
        self._fallback_start_tf2_position = None
        self._fallback_start_tf2_yaw = 0.0
        self._fallback_start_estimator_position = None
        self._fallback_start_estimator_theta = 0.0
        self._last_status = TransformStatus.TF2_OK
        # 重置漂移估计时间跟踪
        self._last_fallback_update_time = None
