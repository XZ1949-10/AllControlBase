"""
轨迹适配器

ROS 消息: controller_ros/LocalTrajectoryV4
UC 数据类型: universal_controller.core.data_types.Trajectory

轨迹模式映射:
- ROS: 0=MODE_TRACK, 1=MODE_STOP, 2=MODE_HOLD
- UC:  0=MODE_TRACK, 1=MODE_STOP, 2=MODE_HOVER, 3=MODE_EMERGENCY
- 注意: ROS 的 MODE_HOLD 对应 UC 的 MODE_HOVER (数值相同，语义相近)

配置说明:
- 配置参数通过 TrajectoryConfig 实例获取
- TrajectoryConfig 从配置字典创建，避免全局状态
"""
from typing import Any, Optional, Tuple, Dict
import logging
import numpy as np

from universal_controller.core.data_types import (
    Trajectory as UcTrajectory, Header, Point3D, TrajectoryConfig
)
from universal_controller.core.data_types import (
    Trajectory as UcTrajectory, Header, Point3D, TrajectoryConfig
)
from universal_controller.core.enums import TrajectoryMode
from .base import IMsgConverter

try:
    from controller_ros.msg import LocalTrajectoryV4
    from geometry_msgs.msg import Point
except ImportError:
    LocalTrajectoryV4 = None
    Point = None

logger = logging.getLogger(__name__)

# ============================================================================
# 协议常量 (不应修改)
# ============================================================================

# 速度向量维度 [vx, vy, vz, wz]
VELOCITY_DIMENSION = 4

# 默认坐标系
DEFAULT_TRAJECTORY_FRAME_ID = 'base_link'


class TrajectoryAdapter(IMsgConverter):
    """
    轨迹消息适配器
    
    将 ROS LocalTrajectoryV4 转换为 UC Trajectory 数据类型。
    
    性能优化说明 (v2):
    - 使用 Numpy 进行批量数据提取和校验 (Vectorized Operations)
    - 避免 Python 层面的逐点循环验证
    - 仅在数据有效时创建 Point3D 对象列表
    
    速度数组处理策略 (严格校验):
    - 速度点数 != 位置点数: 丢弃速度数据，禁用 soft 模式
    - 速度点数 = 0 且 soft_enabled=True: 禁用 soft 模式
    
    坐标系处理:
    - frame_id 必须显式指定，空值会抛出 ValueError
    """
    
    def __init__(self, config: Dict[str, Any] = None):
        """
        初始化轨迹适配器
        
        Args:
            config: 配置字典，用于创建 TrajectoryConfig
        """
        super().__init__()
        self._config = TrajectoryConfig.from_dict(config) if config else TrajectoryConfig()
    
    def _validate_dt_sec(self, dt_sec: float) -> float:
        """
        验证 dt_sec 值
        
        策略: 严格校验模式。
        如果 dt 非法 (<=0 或 超出范围)，抛出 ValueError。
        适配器不应擅自修改导致物理行为改变的关键参数。
        """
        from universal_controller.core.constants import TRAJECTORY_MIN_DT_SEC, TRAJECTORY_MAX_DT_SEC
        
        # 快速检查范围
        if TRAJECTORY_MIN_DT_SEC <= dt_sec <= TRAJECTORY_MAX_DT_SEC:
            return dt_sec
            
        # 错误处理
        if dt_sec <= 0:
            raise ValueError(f"Invalid trajectory dt_sec={dt_sec} (must be > 0)")
        else:
            # 对于超出范围但正值的情况，给出警告并截断，保持系统的连续性
            # 这是为了容忍上游轻微的计算误差
            clipped = max(TRAJECTORY_MIN_DT_SEC, min(dt_sec, TRAJECTORY_MAX_DT_SEC))
            logger.warning(f"dt_sec={dt_sec} out of range, clipped to {clipped}")
            return clipped
    
    def _process_velocities(self, velocities_flat: list, total_points: int, 
                           valid_mask: np.ndarray, soft_enabled: bool) -> Tuple[Optional[np.ndarray], bool]:
        """
        处理速度数组 (高性能版 + 同步过滤)
        """
        if not soft_enabled:
            return None, False
        
        flat_len = len(velocities_flat)
        if flat_len == 0:
            return None, False
        
        # 维度检查
        if flat_len % VELOCITY_DIMENSION != 0:
            logger.warning(
                f"velocities_flat length {flat_len} is not a multiple of {VELOCITY_DIMENSION}, "
                f"discarding velocity data."
            )
            return None, False
        
        # 数量一致性检查 (基于原始点数)
        expected_size = total_points * VELOCITY_DIMENSION
        if flat_len != expected_size:
             logger.warning(
                f"Velocity size {flat_len} != expected {expected_size} (points={total_points}). "
                f"Discarding velocity data to prevent behavioral ambiguity."
            )
             return None, False
             
        # 转换为 Numpy 数组并重塑
        velocities = np.array(velocities_flat, dtype=np.float64)
        velocities = velocities.reshape(total_points, VELOCITY_DIMENSION)
        
        # 应用掩码进行过滤
        if valid_mask is not None:
            velocities = velocities[valid_mask]
            
        return velocities, True
    
    def to_uc(self, ros_msg: Any) -> UcTrajectory:
        """ROS LocalTrajectoryV4 → UC Trajectory (Vectorized & Robust)"""
        # 1. 严格校验 frame_id
        frame_id = ros_msg.header.frame_id
        if not frame_id:
            # 鲁棒性改进: 不要崩溃，而是返回停止轨迹并报错
            logger.error("Trajectory message valid frame_id missing! Returning SAFE STOP trajectory.")
            return UcTrajectory(
                header=Header(
                    stamp=self._ros_time_to_sec(ros_msg.header.stamp),
                    frame_id='base_link' # Fallback to base_link for safety
                ),
                points=np.zeros((0, 3)),
                velocities=None,
                dt_sec=self._config.dt_sec, # Use default dt for safe stop
                confidence=0.0,
                mode=TrajectoryMode.MODE_STOP,
                soft_enabled=False
            )
            
        try:
            dt_sec = self._validate_dt_sec(ros_msg.dt_sec)
        except ValueError as e:
            logger.error(f"Trajectory validation failed: {e}. Returning SAFE STOP trajectory.")
            return UcTrajectory(
                header=Header(
                    stamp=self._ros_time_to_sec(ros_msg.header.stamp),
                    frame_id=frame_id
                ),
                points=np.zeros((0, 3)),
                velocities=None,
                dt_sec=self._config.dt_sec,
                confidence=0.0,
                mode=TrajectoryMode.MODE_STOP,
                soft_enabled=False
            )
            
        # 2. 批量提取点坐标 (Bottleneck optimization)
        # 3. 批量提取点坐标 (Bottleneck optimization)
        # 优先使用 points_flat (Zero-Copy approx)
        if hasattr(ros_msg, 'points_flat') and len(ros_msg.points_flat) > 0:
            flat_pts = ros_msg.points_flat
            
            # 校验长度
            if len(flat_pts) % 3 != 0:
                 logger.warning(f"points_flat length {len(flat_pts)} is not multiple of 3, falling back to points")
                 use_flat = False
            else:
                 # 将 array.array 或 list 转为 numpy
                 # 这是一个非常快的操作 (C-level copy)
                 points_arr = np.array(flat_pts, dtype=np.float64).reshape(-1, 3)
                 num_raw = len(points_arr)
                 use_flat = True
        else:
            use_flat = False
            
        if not use_flat:
            raw_points = ros_msg.points
            num_raw = len(raw_points)
            
            # Performance Optimization: Early Truncation
            # 如果使用 Python 对象列表且点数远超配置上限，提前截断以避免昂贵的转换开销
            if self._config.max_points > 0:
                limit = int(self._config.max_points * 2)  # 留出余量，避免过度截断影响逻辑
                if num_raw > limit:
                     from ..utils.ros_compat import log_warn_throttle
                     log_warn_throttle(
                        10.0,
                        f"Performance Protection: Truncating large legacy trajectory "
                        f"({num_raw} > {limit}) to prevent CPU stall. "
                        f"Please use 'points_flat' for large paths."
                     )
                     raw_points = raw_points[:limit]
                     num_raw = limit
            
            if num_raw == 0:
                # 快速路径：空轨迹
                logger.warning("Received empty trajectory (0 points), returning MODE_STOP trajectory.")
                return UcTrajectory(
                    header=Header(
                        stamp=self._ros_time_to_sec(ros_msg.header.stamp),
                        frame_id=frame_id
                    ),
                    points=np.zeros((0, 3)),
                    velocities=None,
                    dt_sec=self._validate_dt_sec(ros_msg.dt_sec),
                    confidence=0.0,
                    mode=TrajectoryMode.MODE_STOP,
                    soft_enabled=False
                )
    
            # 构建 numpy 数组进行向量化校验 N x 3
            try:
                # 性能警告: 进入此路径意味着上游没有填充 points_flat
                # 简单的 List Comprehension // Python Loop 是这里的瓶颈
                if num_raw > 50: # 仅对长轨迹警告
                    from ..utils.ros_compat import log_warn_throttle
                    log_warn_throttle(
                        5.0, 
                        f"Performance Warning: Received trajectory with {num_raw} points as object list. "
                        f"Adapter is falling back to slow Python iteration. "
                        f"Please populate 'points_flat' in LocalTrajectoryV4 for zero-copy performance."
                    )
                
                # 列表推导提取数据 (Python 层面最快的方式)
                # Optimization: 使用 Flat List 生成器大大减少中间小列表对象的创建开销
                # [x, y, z, x, y, z...] then reshape -> faster than [[x, y, z], ...]
                points_flat_py = [val for p in raw_points for val in (p.x, p.y, p.z)]
                points_arr = np.array(points_flat_py, dtype=np.float64).reshape(-1, 3)
            except AttributeError:
                 raise ValueError("Trajectory points elements must have x, y, z attributes")

        # 3. 向量化校验 (Vectorized Validation)
        from universal_controller.core.constants import TRAJECTORY_MAX_COORD
        
        # 检查 NaN/Inf
        is_finite = np.all(np.isfinite(points_arr), axis=1)
        # 检查范围
        is_in_range = np.all(np.abs(points_arr) <= TRAJECTORY_MAX_COORD, axis=1)
        
        valid_mask = is_finite & is_in_range
        
        # 统计无效点
        n_invalid = num_raw - np.sum(valid_mask)
        if n_invalid > 0:
            logger.warning(f"Trajectory: {n_invalid} invalid points (NaN/Inf or OutOfRange) removed")
            
        # 4. 同步过滤位置和速度
        
        # 4.1 处理速度 (传入原始点数和掩码)
        velocities, soft_enabled = self._process_velocities(
            ros_msg.velocities_flat, 
            num_raw, 
            valid_mask, 
            ros_msg.soft_enabled
        )
        
        # 4.2 处理位置 (应用掩码)
        points_arr_filtered = points_arr[valid_mask]
        
        if len(points_arr_filtered) == 0:
             return UcTrajectory(
                header=Header(stamp=self._ros_time_to_sec(ros_msg.header.stamp), frame_id=frame_id),
                points=np.zeros((0, 3)), velocities=None, dt_sec=self._validate_dt_sec(ros_msg.dt_sec),
                confidence=0.0, mode=TrajectoryMode.MODE_STOP, soft_enabled=False
            )

        # 5. 直接传递 Numpy 数组 (不再创建 Point3D 对象列表)
        final_points = points_arr_filtered
        
        # 6. 处理模式
        try:
            mode = TrajectoryMode(ros_msg.mode)
        except ValueError:
            logger.warning(f"Unknown trajectory mode {ros_msg.mode}, using MODE_TRACK")
            mode = TrajectoryMode.MODE_TRACK
            
        # 7. 处理置信度
        confidence = float(np.clip(ros_msg.confidence, 0.0, 1.0))

        return UcTrajectory(
            header=Header(
                stamp=self._ros_time_to_sec(ros_msg.header.stamp),
                frame_id=frame_id
            ),
            points=final_points,
            velocities=velocities,
            dt_sec=self._validate_dt_sec(ros_msg.dt_sec),
            confidence=confidence,
            mode=mode,
            soft_enabled=soft_enabled
        )
    
    def to_ros(self, uc_data: UcTrajectory) -> Any:
        """UC Trajectory → ROS LocalTrajectoryV4"""
        if LocalTrajectoryV4 is None:
            raise ImportError("ROS messages not available")
            
        ros_msg = LocalTrajectoryV4()
        ros_msg.header.stamp = self._sec_to_ros_time(uc_data.header.stamp)
        ros_msg.header.frame_id = uc_data.header.frame_id or self._config.default_frame_id
        
        if isinstance(uc_data.mode, TrajectoryMode):
            ros_msg.mode = uc_data.mode.value
        else:
            ros_msg.mode = int(uc_data.mode)
            
        ros_msg.dt_sec = float(uc_data.dt_sec)
        ros_msg.confidence = float(uc_data.confidence)
        ros_msg.soft_enabled = uc_data.soft_enabled
        
        # 将 Point3D 列表转换为 ROS Point 列表
        # 处理 numpy 数组或 Point3D 列表
        if isinstance(uc_data.points, np.ndarray):
            # Numpy 路径
            # 1. 填充 points_flat (高性能)
            ros_msg.points_flat = uc_data.points.flatten().tolist()
            
            # 2. 填充 legacy points (兼容性)
            # 仅当配置明确启用时才填充，避免昂贵的 Python 循环
            if self._config.legacy_points_enabled:
                ros_msg.points = [
                    Point(x=float(row[0]), y=float(row[1]), z=float(row[2]))
                    for row in uc_data.points
                ]
            else:
                ros_msg.points = []
        else:
            # Legacy List[Point3D] 路径
            ros_msg.points = [
                Point(x=float(p.x), y=float(p.y), z=float(p.z))
                for p in uc_data.points
            ]
            # 尝试也填充 flat
            try:
                ros_msg.points_flat = [coord for p in uc_data.points for coord in (p.x, p.y, p.z)]
            except Exception:
                pass
        
        if uc_data.velocities is not None and uc_data.velocities.size > 0:
            ros_msg.velocities_flat = uc_data.velocities.flatten().tolist()
        else:
            ros_msg.velocities_flat = []
            
        return ros_msg
