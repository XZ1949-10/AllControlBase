"""
统一数据管理器

管理传感器数据的缓存和访问，支持 ROS1 和 ROS2。
将数据缓存逻辑从节点实现中抽离，避免代码重复。
"""
from typing import Dict, Any, Optional, Callable
import threading
import time

from universal_controller.core.data_types import Odometry, Imu, Trajectory
from ..adapters import OdomAdapter, ImuAdapter, TrajectoryAdapter
from ..utils.ros_compat import ROS_VERSION


class DataManager:
    """
    统一数据管理器
    
    职责:
    - 管理传感器数据的缓存
    - 提供线程安全的数据访问
    - 管理数据时间戳
    - 计算数据年龄
    
    支持 ROS1 和 ROS2，通过注入时间获取函数实现。
    """
    
    def __init__(self, get_time_func: Optional[Callable[[], float]] = None):
        """
        初始化数据管理器
        
        Args:
            get_time_func: 获取当前时间的函数（秒）。
                          如果为 None，使用 time.time()。
                          ROS 环境应传入支持仿真时间的函数。
        """
        # 适配器
        self._odom_adapter = OdomAdapter()
        self._imu_adapter = ImuAdapter()
        self._traj_adapter = TrajectoryAdapter()
        
        # 时间获取函数
        self._get_time_func = get_time_func or time.time
        
        # 线程安全的数据存储
        self._lock = threading.Lock()
        self._latest_data: Dict[str, Any] = {}
        self._timestamps: Dict[str, float] = {}
    
    @property
    def odom_adapter(self) -> OdomAdapter:
        """获取里程计适配器"""
        return self._odom_adapter
    
    @property
    def imu_adapter(self) -> ImuAdapter:
        """获取 IMU 适配器"""
        return self._imu_adapter
    
    @property
    def traj_adapter(self) -> TrajectoryAdapter:
        """获取轨迹适配器"""
        return self._traj_adapter
    
    def update_odom(self, ros_msg: Any) -> Odometry:
        """
        更新里程计数据
        
        Args:
            ros_msg: ROS Odometry 消息
        
        Returns:
            转换后的 UC Odometry 数据
        """
        uc_odom = self._odom_adapter.to_uc(ros_msg)
        with self._lock:
            self._latest_data['odom'] = uc_odom
            self._timestamps['odom'] = self._get_time_func()
        return uc_odom
    
    def update_imu(self, ros_msg: Any) -> Imu:
        """
        更新 IMU 数据
        
        Args:
            ros_msg: ROS Imu 消息
        
        Returns:
            转换后的 UC Imu 数据
        """
        uc_imu = self._imu_adapter.to_uc(ros_msg)
        with self._lock:
            self._latest_data['imu'] = uc_imu
            self._timestamps['imu'] = self._get_time_func()
        return uc_imu
    
    def update_trajectory(self, ros_msg: Any) -> Trajectory:
        """
        更新轨迹数据
        
        Args:
            ros_msg: ROS LocalTrajectoryV4 消息
        
        Returns:
            转换后的 UC Trajectory 数据
        """
        uc_traj = self._traj_adapter.to_uc(ros_msg)
        with self._lock:
            self._latest_data['trajectory'] = uc_traj
            self._timestamps['trajectory'] = self._get_time_func()
        return uc_traj
    
    def get_latest_odom(self) -> Optional[Odometry]:
        """获取最新里程计数据"""
        with self._lock:
            return self._latest_data.get('odom')
    
    def get_latest_imu(self) -> Optional[Imu]:
        """获取最新 IMU 数据"""
        with self._lock:
            return self._latest_data.get('imu')
    
    def get_latest_trajectory(self) -> Optional[Trajectory]:
        """获取最新轨迹数据"""
        with self._lock:
            return self._latest_data.get('trajectory')
    
    def get_all_latest(self) -> Dict[str, Any]:
        """
        获取所有最新数据
        
        Returns:
            包含 'odom', 'imu', 'trajectory' 键的字典
        """
        with self._lock:
            return {
                'odom': self._latest_data.get('odom'),
                'imu': self._latest_data.get('imu'),
                'trajectory': self._latest_data.get('trajectory'),
            }
    
    def get_data_ages(self) -> Dict[str, float]:
        """
        获取各数据的年龄（秒）
        
        Returns:
            字典，键为数据名 ('odom', 'imu', 'trajectory')，
            值为距上次更新的秒数。未收到的数据返回 float('inf')。
        """
        now = self._get_time_func()
        with self._lock:
            ages = {}
            for key in ['odom', 'imu', 'trajectory']:
                if key in self._timestamps:
                    ages[key] = now - self._timestamps[key]
                else:
                    ages[key] = float('inf')
            return ages
    
    def get_timestamps(self) -> Dict[str, float]:
        """
        获取各数据的时间戳
        
        Returns:
            字典，键为数据名，值为时间戳（秒）
        """
        with self._lock:
            return self._timestamps.copy()
    
    def is_data_fresh(self, max_ages: Dict[str, float]) -> bool:
        """
        检查数据是否新鲜
        
        Args:
            max_ages: 各数据的最大允许年龄（秒）
        
        Returns:
            所有指定数据都在允许年龄内返回 True
        """
        ages = self.get_data_ages()
        for key, max_age in max_ages.items():
            if ages.get(key, float('inf')) > max_age:
                return False
        return True
    
    def has_required_data(self) -> bool:
        """检查是否有必需的数据（odom 和 trajectory）"""
        with self._lock:
            return 'odom' in self._latest_data and 'trajectory' in self._latest_data
    
    def clear(self):
        """清除所有缓存数据"""
        with self._lock:
            self._latest_data.clear()
            self._timestamps.clear()
