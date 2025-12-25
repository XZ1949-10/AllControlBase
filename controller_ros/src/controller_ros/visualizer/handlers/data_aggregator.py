"""
数据聚合器

聚合来自多个 ROS 话题的数据，提供统一的数据访问接口。
线程安全设计，支持 GUI 线程和 ROS 回调线程并发访问。
"""
from typing import Dict, Any, Optional, List, Callable
from dataclasses import dataclass, field
from collections import deque
import threading
import time
import math
import logging

from ..models import (
    VisualizerData, VelocityData, TrajectoryData, RobotPose,
    JoystickState, ControllerStatus, ControlMode, TrajectoryMode, Point3D
)

logger = logging.getLogger(__name__)


@dataclass
class VelocityHistory:
    """速度历史记录"""
    max_duration_sec: float = 10.0
    timestamps: deque = field(default_factory=lambda: deque(maxlen=1000))
    linear_x: deque = field(default_factory=lambda: deque(maxlen=1000))
    angular_z: deque = field(default_factory=lambda: deque(maxlen=1000))
    target_linear_x: deque = field(default_factory=lambda: deque(maxlen=1000))
    target_angular_z: deque = field(default_factory=lambda: deque(maxlen=1000))
    
    def add_actual(self, velocity: VelocityData):
        """添加实际速度"""
        self.timestamps.append(velocity.timestamp)
        self.linear_x.append(velocity.linear_x)
        self.angular_z.append(velocity.angular_z)
    
    def add_target(self, velocity: VelocityData):
        """添加目标速度"""
        self.target_linear_x.append(velocity.linear_x)
        self.target_angular_z.append(velocity.angular_z)
    
    def get_recent(self, duration_sec: float = None) -> Dict[str, List[float]]:
        """获取最近的历史数据"""
        duration = duration_sec or self.max_duration_sec
        now = time.time()
        cutoff = now - duration
        
        # 找到截止时间点
        start_idx = 0
        for i, t in enumerate(self.timestamps):
            if t >= cutoff:
                start_idx = i
                break
        
        return {
            'timestamps': list(self.timestamps)[start_idx:],
            'linear_x': list(self.linear_x)[start_idx:],
            'angular_z': list(self.angular_z)[start_idx:],
            'target_linear_x': list(self.target_linear_x)[start_idx:],
            'target_angular_z': list(self.target_angular_z)[start_idx:],
        }


class DataAggregator:
    """
    数据聚合器
    
    职责:
    - 聚合来自多个数据源的数据
    - 提供线程安全的数据访问
    - 维护速度历史记录
    - 计算派生数据 (如机器人位姿)
    """
    
    def __init__(self, velocity_history_sec: float = 10.0):
        """
        初始化数据聚合器
        
        Args:
            velocity_history_sec: 速度历史记录时长 (秒)
        """
        self._lock = threading.RLock()
        
        # 核心数据
        self._data = VisualizerData()
        
        # 速度历史
        self._velocity_history = VelocityHistory(max_duration_sec=velocity_history_sec)
        
        # 数据更新时间戳
        self._last_odom_time = 0.0
        self._last_traj_time = 0.0
        self._last_cmd_time = 0.0
        self._last_diag_time = 0.0
        self._last_joy_time = 0.0
        
        # 数据更新回调
        self._on_data_updated: Optional[Callable[[], None]] = None
    
    def set_data_updated_callback(self, callback: Callable[[], None]):
        """设置数据更新回调"""
        self._on_data_updated = callback
    
    def _notify_update(self):
        """通知数据更新"""
        if self._on_data_updated:
            try:
                self._on_data_updated()
            except Exception as e:
                logger.debug(f"Data update callback failed: {e}")
    
    # ==================== 数据更新方法 ====================
    
    def update_odom(self, position: tuple, orientation: tuple, 
                    velocity: VelocityData, timestamp: float):
        """
        更新里程计数据
        
        Args:
            position: (x, y, z) 位置
            orientation: (x, y, z, w) 四元数
            velocity: 速度数据
            timestamp: 时间戳
        """
        with self._lock:
            # 更新机器人位姿
            self._data.robot_pose = RobotPose(
                x=position[0],
                y=position[1],
                z=position[2],
                yaw=self._quaternion_to_yaw(orientation),
                timestamp=timestamp,
            )
            
            # 更新实际速度
            self._data.actual_velocity = velocity
            
            # 添加到历史
            self._velocity_history.add_actual(velocity)
            
            self._last_odom_time = timestamp
            self._data.ros_connected = True
        
        self._notify_update()
    
    def update_trajectory(self, points: List[tuple], mode: int, 
                          confidence: float, dt_sec: float,
                          frame_id: str, timestamp: float):
        """
        更新轨迹数据
        
        Args:
            points: [(x, y, z), ...] 轨迹点列表
            mode: 轨迹模式
            confidence: 置信度
            dt_sec: 时间间隔
            frame_id: 坐标系
            timestamp: 时间戳
        """
        with self._lock:
            self._data.trajectory = TrajectoryData(
                points=[Point3D(x=p[0], y=p[1], z=p[2]) for p in points],
                mode=TrajectoryMode(mode) if mode in [0, 1, 2, 3] else TrajectoryMode.TRACK,
                confidence=confidence,
                dt_sec=dt_sec,
                frame_id=frame_id,
                timestamp=timestamp,
            )
            self._last_traj_time = timestamp
        
        self._notify_update()
    
    def update_target_velocity(self, velocity: VelocityData):
        """更新目标速度"""
        with self._lock:
            self._data.target_velocity = velocity
            self._velocity_history.add_target(velocity)
            self._last_cmd_time = velocity.timestamp
        
        self._notify_update()
    
    def update_controller_status(self, status: ControllerStatus):
        """更新控制器状态"""
        with self._lock:
            self._data.controller_status = status
            self._last_diag_time = time.time()
        
        self._notify_update()
    
    def update_joystick(self, state: JoystickState):
        """更新手柄状态"""
        with self._lock:
            self._data.joystick = state
            self._last_joy_time = time.time()
        
        self._notify_update()
    
    def update_control_mode(self, mode: ControlMode):
        """更新控制模式"""
        with self._lock:
            self._data.control_mode = mode
        
        self._notify_update()
    
    def update_camera_image(self, image):
        """更新相机图像"""
        with self._lock:
            self._data.camera_image = image
        
        self._notify_update()
    
    # ==================== 数据获取方法 ====================
    
    def get_data(self) -> VisualizerData:
        """获取聚合数据的副本"""
        with self._lock:
            # 返回浅拷贝，避免长时间持有锁
            return VisualizerData(
                target_velocity=self._data.target_velocity,
                actual_velocity=self._data.actual_velocity,
                trajectory=self._data.trajectory,
                robot_pose=self._data.robot_pose,
                joystick=self._data.joystick,
                controller_status=self._data.controller_status,
                control_mode=self._data.control_mode,
                camera_image=self._data.camera_image,
                ros_connected=self._data.ros_connected,
            )
    
    def get_velocity_history(self, duration_sec: float = None) -> Dict[str, List[float]]:
        """获取速度历史"""
        with self._lock:
            return self._velocity_history.get_recent(duration_sec)
    
    def get_connection_status(self) -> Dict[str, bool]:
        """获取连接状态"""
        now = time.time()
        timeout = 2.0  # 2 秒超时
        
        with self._lock:
            return {
                'odom': (now - self._last_odom_time) < timeout if self._last_odom_time > 0 else False,
                'trajectory': (now - self._last_traj_time) < timeout if self._last_traj_time > 0 else False,
                'cmd': (now - self._last_cmd_time) < timeout if self._last_cmd_time > 0 else False,
                'diagnostics': (now - self._last_diag_time) < timeout if self._last_diag_time > 0 else False,
                'joystick': (now - self._last_joy_time) < timeout if self._last_joy_time > 0 else False,
            }
    
    # ==================== 工具方法 ====================
    
    @staticmethod
    def _quaternion_to_yaw(q: tuple) -> float:
        """四元数转航向角"""
        x, y, z, w = q
        # yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def reset(self):
        """重置数据聚合器状态"""
        with self._lock:
            self._data = VisualizerData()
            self._velocity_history = VelocityHistory(
                max_duration_sec=self._velocity_history.max_duration_sec
            )
            self._last_odom_time = 0.0
            self._last_traj_time = 0.0
            self._last_cmd_time = 0.0
            self._last_diag_time = 0.0
            self._last_joy_time = 0.0
        logger.info("DataAggregator reset")
