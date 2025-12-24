"""
鲁棒坐标变换器

实现功能:
- F3.1: 补偿网络推理延迟
- F3.2: 使用 TF2 lookupTransform 获取坐标变换
- F3.3: TF2 降级策略 (降级到 odom 积分，记录持续时间，恢复后校正)
- F3.4: TF2 降级超过 1000ms 触发 MPC_DEGRADED 状态
- F3.5: TF2 恢复时校正状态估计器的漂移

坐标系约定 (不需要建图):
- base_link: 机体坐标系，原点在机器人中心，X轴朝前
- odom: 里程计坐标系，从启动位置开始累积 (这就是"世界坐标系")

数据流:
- 网络输出轨迹: base_link (局部坐标系，当前位置为原点)
- 控制器工作坐标系: odom (里程计坐标系)
- 变换方向: base_link -> odom

延迟补偿说明:
- 网络推理有延迟，输出的轨迹是基于推理开始时刻的机体坐标系
- 控制执行时机器人已经移动，需要将轨迹变换到当前的里程计坐标系
- TF2 提供 base_link -> odom 的变换，即机器人在里程计坐标系中的位姿
"""
from typing import Dict, Any, Tuple, Optional, List
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
    
    坐标系处理 (不需要建图):
    - 网络输出轨迹在 base_link 坐标系下 (局部坐标，当前位置为原点)
    - 变换后轨迹在 odom 坐标系下 (里程计坐标系)
    - 控制器使用 odom 坐标系下的轨迹进行跟踪
    
    注意: 这里的 odom 就是你的"世界坐标系"，不需要建图或定位。
    """
    
    def __init__(self, config: Dict[str, Any]):
        transform_config = config.get('transform', config)
        
        # 配置参数
        self.fallback_duration_limit = transform_config.get('fallback_duration_limit_ms', 500) / 1000.0
        self.fallback_critical_limit = transform_config.get('fallback_critical_limit_ms', 1000) / 1000.0
        # 支持两种键名: timeout_ms (新) 和 tf2_timeout_ms (旧，向后兼容)
        self.tf2_timeout = transform_config.get('timeout_ms', 
                           transform_config.get('tf2_timeout_ms', 10)) / 1000.0
        self.drift_estimation_enabled = transform_config.get('drift_estimation_enabled', True)
        self.recovery_correction_enabled = transform_config.get('recovery_correction_enabled', True)
        self.drift_rate = transform_config.get('drift_rate', 0.01)
        self.max_drift_dt = transform_config.get('max_drift_dt', 0.5)
        self.source_frame = transform_config.get('source_frame', 'base_link')
        self.drift_correction_thresh = transform_config.get('drift_correction_thresh', 0.01)
        
        # 坐标系验证配置
        self.expected_source_frames: List[str] = transform_config.get(
            'expected_source_frames', ['base_link', 'base_link_0', ''])
        self.warn_unexpected_frame = transform_config.get('warn_unexpected_frame', True)
        self._warned_frames: set = set()  # 已警告过的坐标系，避免重复警告
        
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
        
        # 外部 TF2 查找回调 (用于 ROS 胶水层注入)
        self._external_tf2_lookup: Optional[callable] = None
        
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
            # 非 ROS 环境，使用独立运行模式的 TF2 Buffer
            self._tf_buffer = tf2_ros.Buffer()
            self._tf2_initialized = True
            logger.debug("Using standalone TF2 buffer (non-ROS mode)")
    
    def set_tf2_available(self, available: bool) -> None:
        """
        设置 TF2 可用性 (用于测试)
        
        注意: 这会强制禁用/启用 TF2，即使真实 TF2 可用
        """
        self._tf2_force_disabled = not available
        if not available:
            logger.debug("TF2 force disabled for testing")
    
    def set_tf2_lookup_callback(self, callback: callable) -> None:
        """
        设置外部 TF2 查找回调
        
        用于 ROS 胶水层注入真实的 TF2 查找功能。
        
        Args:
            callback: 回调函数，签名为:
                callback(target_frame: str, source_frame: str, 
                        time: Optional[float], timeout_sec: float) -> Optional[dict]
                返回 {'translation': (x, y, z), 'rotation': (x, y, z, w)} 或 None
        """
        self._external_tf2_lookup = callback
        if callback is not None:
            logger.info("External TF2 lookup callback set")
        else:
            logger.info("External TF2 lookup callback cleared")
    
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
        
        from ..core.data_types import TransformStamped, Header, Transform, Vector3, Quaternion
        
        transform = TransformStamped()
        transform.header = Header()
        transform.header.frame_id = parent_frame
        transform.header.stamp = stamp if stamp is not None else get_current_time()
        transform.child_frame_id = child_frame
        
        q = quaternion_from_euler(0, 0, yaw)
        transform.transform = Transform()
        transform.transform.translation = Vector3(x, y, z)
        transform.transform.rotation = Quaternion(q[0], q[1], q[2], q[3])
        
        self._tf_buffer.set_transform(transform, "manual")
    
    def _try_tf2_lookup(self, target_frame: str, source_frame: str,
                       target_time: float) -> Tuple[Optional[np.ndarray], Optional[float], bool]:
        """
        尝试 TF2 查找
        
        优先使用外部 TF2 回调（如果设置），否则使用内部 TF2 Buffer。
        
        Returns:
            (position, yaw, success): 位置 [x, y, z], 航向角, 是否成功
        """
        # 优先使用外部 TF2 回调 (ROS 胶水层注入)
        if self._external_tf2_lookup is not None and not self._tf2_force_disabled:
            try:
                result = self._external_tf2_lookup(
                    target_frame, source_frame, target_time, self.tf2_timeout
                )
                if result is not None:
                    position = np.array(result['translation'])
                    q = result['rotation']
                    _, _, yaw = euler_from_quaternion(q)
                    
                    # 更新缓存
                    self._last_tf2_position = position.copy()
                    self._last_tf2_yaw = yaw
                    
                    return position, yaw, True
            except Exception as e:
                logger.debug(f"External TF2 lookup failed: {e}")
            
            return None, None, False
        
        # 使用内部 TF2 Buffer
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
        
        实现 F3.1: 补偿网络推理延迟
        实现 F3.2: 使用 TF2 lookupTransform 获取坐标变换
        实现 F3.3: TF2 降级策略
        
        坐标变换说明:
        - 输入轨迹在 source_frame (通常是 base_link) 坐标系下
        - 轨迹点是相对于推理时刻机器人位置的局部坐标 (原点在机器人位置)
        - 输出轨迹在 target_frame (通常是 odom) 坐标系下
        - 变换使用 TF2 获取的 base_link -> odom 变换
        
        特殊情况:
        - 如果轨迹已经在目标坐标系 (world/odom)，则不进行变换
        
        延迟补偿:
        - traj.header.stamp 是轨迹生成时刻 (网络推理时刻)
        - target_time 是当前控制时刻
        - 理想情况下应使用 traj.header.stamp 时刻的 TF 变换
        - 但由于 TF2 缓存限制，可能需要使用最新的变换
        
        Args:
            traj: 输入轨迹 (局部坐标系)
            target_frame: 目标坐标系 (通常是 'odom')
            target_time: 目标时间戳 (当前控制时刻)
        
        Returns:
            (transformed_traj, status): 变换后的轨迹和状态
        """
        # 确定源坐标系
        source_frame = traj.header.frame_id if traj.header.frame_id else self.source_frame
        
        # 坐标系验证
        self._validate_source_frame(source_frame)
        
        # 如果轨迹已经在目标坐标系 (odom)，不需要变换
        if source_frame == target_frame or source_frame == 'odom':
            # 轨迹已经在里程计坐标系，直接返回
            new_traj = traj.copy()
            new_traj.header.frame_id = target_frame
            return new_traj, TransformStatus.TF2_OK
        
        # 确定查询时间：优先使用轨迹时间戳进行延迟补偿
        # 如果轨迹时间戳有效且不太旧，使用它来获取更准确的变换
        lookup_time = target_time
        traj_stamp = traj.header.stamp
        if traj_stamp > 0:
            time_diff = target_time - traj_stamp
            # 如果轨迹时间戳在合理范围内 (0-500ms 延迟)，使用它
            if 0 <= time_diff <= 0.5:
                lookup_time = traj_stamp
        
        # 尝试 TF2 查找
        position, yaw, tf2_success = self._try_tf2_lookup(target_frame, source_frame, lookup_time)
        
        if tf2_success:
            # TF2 可用
            if self.fallback_start_time is not None:
                # 从降级状态恢复
                if self.recovery_correction_enabled:
                    self._apply_recovery_correction(position, yaw)
                self.fallback_start_time = None
                self.accumulated_drift = 0.0
                # 重置漂移估计时间跟踪
                self._last_fallback_update_time = None
                logger.info("TF2 recovered, drift corrected")
            
            self._last_status = TransformStatus.TF2_OK
            
            # 应用变换
            transformed_traj = self._apply_tf2_transform(traj, position, yaw, target_frame)
            return transformed_traj, TransformStatus.TF2_OK
        else:
            # TF2 不可用，降级处理
            return self._handle_fallback(traj, target_time, target_frame)
    
    def _validate_source_frame(self, source_frame: str) -> None:
        """
        验证源坐标系是否符合预期
        
        网络输出的轨迹应该在 base_link 或 base_link_0 坐标系下。
        如果收到其他坐标系的轨迹，发出警告。
        """
        if not self.warn_unexpected_frame:
            return
        
        if source_frame not in self.expected_source_frames:
            if source_frame not in self._warned_frames:
                self._warned_frames.add(source_frame)
                logger.warning(
                    f"Unexpected trajectory frame_id: '{source_frame}'. "
                    f"Expected one of {self.expected_source_frames}. "
                    f"Network output should be in local (base_link) frame. "
                    f"If this is intentional, add '{source_frame}' to "
                    f"transform.expected_source_frames config."
                )
    
    def _apply_tf2_transform(self, traj: Trajectory, position: np.ndarray,
                            yaw: float, target_frame: str) -> Trajectory:
        """
        应用 TF2 变换到轨迹
        
        变换说明:
        - position 和 yaw 表示 base_link 在 odom 中的位姿
        - 即机器人在里程计坐标系中的位置和航向
        - 轨迹点是相对于机器人的局部坐标
        - 变换公式: p_odom = R(yaw) @ p_local + position
        
        Args:
            traj: 输入轨迹 (局部坐标系 base_link)
            position: 机器人在 odom 坐标系中的位置 [x, y, z]
            yaw: 机器人在 odom 坐标系中的航向角
            target_frame: 目标坐标系名称 (通常是 'odom')
        
        Returns:
            变换后的轨迹 (odom 坐标系)
        """
        cos_yaw = np.cos(yaw)
        sin_yaw = np.sin(yaw)
        
        transformed_points = []
        for p in traj.points:
            # 旋转 + 平移: p_world = R(yaw) @ p_local + position
            # 这里 p.x, p.y 是局部坐标 (相对于机器人)
            # new_x, new_y 是世界坐标
            new_x = p.x * cos_yaw - p.y * sin_yaw + position[0]
            new_y = p.x * sin_yaw + p.y * cos_yaw + position[1]
            new_z = p.z + position[2]
            transformed_points.append(Point3D(new_x, new_y, new_z))
        
        # 变换速度 (如果有)
        # 速度是向量，只需要旋转，不需要平移
        new_velocities = None
        if traj.velocities is not None and len(traj.velocities) > 0:
            new_velocities = traj.velocities.copy()
            for i in range(len(new_velocities)):
                vx, vy = new_velocities[i, 0], new_velocities[i, 1]
                # 旋转速度向量: v_world = R(yaw) @ v_local
                new_velocities[i, 0] = vx * cos_yaw - vy * sin_yaw
                new_velocities[i, 1] = vx * sin_yaw + vy * cos_yaw
                # vz 不受 yaw 旋转影响
        
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
        # 使用实际时间间隔和速度相关的漂移率
        if self.drift_estimation_enabled:
            # 计算自上次更新以来的时间间隔
            if self._last_fallback_update_time is None:
                self._last_fallback_update_time = self.fallback_start_time
            
            dt_raw = current_time - self._last_fallback_update_time
            # 限制 dt 在合理范围内，避免异常值 (使用配置值)
            dt = np.clip(dt_raw, 0.0, self.max_drift_dt)
            
            # 如果 dt 被截断，记录警告（可能是系统暂停）
            if dt_raw > self.max_drift_dt:
                logger.debug(f"Drift estimation dt clamped: {dt_raw:.3f}s -> {dt:.3f}s (possible system pause)")
            
            # 使用速度相关的漂移率
            # 高速运动时漂移更大（轮子打滑、IMU 积分误差等）
            effective_drift_rate = self.drift_rate
            if self.state_estimator is not None:
                state = self.state_estimator.get_state()
                velocity = np.linalg.norm(state.state[3:6])
                # 漂移率 = 基础漂移率 * (1 + 速度因子)
                # 速度因子使用 0.1，即每 1 m/s 增加 10% 的漂移率
                effective_drift_rate = self.drift_rate * (1.0 + velocity * 0.1)
            
            self.accumulated_drift += effective_drift_rate * dt
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
            # 使用配置的漂移校正阈值
            if (abs(drift_x) > self.drift_correction_thresh or 
                abs(drift_y) > self.drift_correction_thresh or 
                abs(drift_theta) > self.drift_correction_thresh):
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
            
            # 使用配置的漂移校正阈值判断是否记录警告
            if drift_magnitude > self.drift_correction_thresh:
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
        
        tf2_injected = self._external_tf2_lookup is not None
        
        return {
            'tf2_available': self.fallback_start_time is None,
            'tf2_injected': tf2_injected,
            'fallback_duration_ms': fallback_duration_ms,
            'accumulated_drift': self.accumulated_drift,
            'is_critical': self._last_status.is_critical(),
            'status': self._last_status.name,
            'tf2_initialized': self._tf2_initialized,
            'external_tf2_callback': tf2_injected  # 保持向后兼容
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
        # 重置警告记录
        self._warned_frames.clear()
