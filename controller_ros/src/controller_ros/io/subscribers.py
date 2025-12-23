"""
订阅管理器

管理所有 ROS 订阅，提供线程安全的数据访问。
"""
from typing import Dict, Any, Optional
import threading
import time

from rclpy.node import Node
from rclpy.callback_groups import CallbackGroup
from nav_msgs.msg import Odometry as RosOdometry
from sensor_msgs.msg import Imu as RosImu

from universal_controller.core.data_types import Odometry, Imu, Trajectory
from ..adapters import OdomAdapter, ImuAdapter, TrajectoryAdapter


class SubscriberManager:
    """
    订阅管理器
    
    职责:
    - 创建和管理所有订阅
    - 线程安全的数据缓存
    - 提供最新数据访问
    - 数据时间戳管理
    """
    
    def __init__(self, node: Node, topics: Dict[str, str],
                 callback_group: Optional[CallbackGroup] = None):
        """
        初始化订阅管理器
        
        Args:
            node: ROS2 节点
            topics: 话题配置字典 {'odom': '/odom', 'imu': '/imu', ...}
            callback_group: 回调组 (用于并发控制)
        """
        self._node = node
        self._topics = topics
        self._callback_group = callback_group
        
        # 适配器
        self._odom_adapter = OdomAdapter()
        self._imu_adapter = ImuAdapter()
        self._traj_adapter = TrajectoryAdapter()
        
        # 线程安全的数据存储
        self._lock = threading.Lock()
        self._latest_data: Dict[str, Any] = {}
        self._timestamps: Dict[str, float] = {}
        
        # 创建订阅
        self._create_subscriptions()
    
    def _create_subscriptions(self):
        """创建所有订阅"""
        # 里程计订阅
        odom_topic = self._topics.get('odom', '/odom')
        self._odom_sub = self._node.create_subscription(
            RosOdometry,
            odom_topic,
            self._odom_callback,
            10,
            callback_group=self._callback_group
        )
        self._node.get_logger().info(f"Subscribed to odom: {odom_topic}")
        
        # IMU 订阅
        imu_topic = self._topics.get('imu', '/imu')
        self._imu_sub = self._node.create_subscription(
            RosImu,
            imu_topic,
            self._imu_callback,
            10,
            callback_group=self._callback_group
        )
        self._node.get_logger().info(f"Subscribed to imu: {imu_topic}")
        
        # 轨迹订阅 (需要自定义消息类型)
        traj_topic = self._topics.get('trajectory', '/nn/local_trajectory')
        try:
            from controller_ros.msg import LocalTrajectoryV4
            self._traj_sub = self._node.create_subscription(
                LocalTrajectoryV4,
                traj_topic,
                self._traj_callback,
                10,
                callback_group=self._callback_group
            )
            self._node.get_logger().info(f"Subscribed to trajectory: {traj_topic}")
        except ImportError:
            self._node.get_logger().warn(
                f"LocalTrajectoryV4 message not available, trajectory subscription disabled"
            )
            self._traj_sub = None
    
    def _odom_callback(self, msg: RosOdometry):
        """里程计回调"""
        uc_odom = self._odom_adapter.to_uc(msg)
        with self._lock:
            self._latest_data['odom'] = uc_odom
            self._timestamps['odom'] = time.time()
    
    def _imu_callback(self, msg: RosImu):
        """IMU 回调"""
        uc_imu = self._imu_adapter.to_uc(msg)
        with self._lock:
            self._latest_data['imu'] = uc_imu
            self._timestamps['imu'] = time.time()
    
    def _traj_callback(self, msg):
        """轨迹回调"""
        uc_traj = self._traj_adapter.to_uc(msg)
        with self._lock:
            self._latest_data['trajectory'] = uc_traj
            self._timestamps['trajectory'] = time.time()
    
    def get_latest_odom(self) -> Optional[Odometry]:
        """获取最新里程计"""
        with self._lock:
            return self._latest_data.get('odom')
    
    def get_latest_imu(self) -> Optional[Imu]:
        """获取最新 IMU"""
        with self._lock:
            return self._latest_data.get('imu')
    
    def get_latest_trajectory(self) -> Optional[Trajectory]:
        """获取最新轨迹"""
        with self._lock:
            return self._latest_data.get('trajectory')
    
    def get_data_ages(self) -> Dict[str, float]:
        """
        获取各数据的年龄 (秒)
        
        Returns:
            字典，键为数据名，值为距上次更新的秒数
        """
        now = time.time()
        with self._lock:
            return {k: now - v for k, v in self._timestamps.items()}
    
    def is_data_fresh(self, max_ages: Dict[str, float]) -> bool:
        """
        检查数据是否新鲜
        
        Args:
            max_ages: 各数据的最大允许年龄 (秒)
        
        Returns:
            所有数据都在允许年龄内返回 True
        """
        ages = self.get_data_ages()
        for key, max_age in max_ages.items():
            if ages.get(key, float('inf')) > max_age:
                return False
        return True
    
    def has_required_data(self) -> bool:
        """检查是否有必需的数据 (odom 和 trajectory)"""
        with self._lock:
            return 'odom' in self._latest_data and 'trajectory' in self._latest_data
