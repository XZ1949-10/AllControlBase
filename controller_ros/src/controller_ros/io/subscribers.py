"""
订阅管理器

管理所有 ROS2 订阅，提供线程安全的数据访问。
基于 DataManager 实现，避免代码重复。
"""
from typing import Dict, Any, Optional

from rclpy.node import Node
from rclpy.callback_groups import CallbackGroup
from nav_msgs.msg import Odometry as RosOdometry
from sensor_msgs.msg import Imu as RosImu

from universal_controller.core.data_types import Odometry, Imu, Trajectory
from .data_manager import DataManager
from ..utils.ros_compat import get_time_sec


class SubscriberManager:
    """
    订阅管理器 (ROS2)
    
    职责:
    - 创建和管理所有 ROS2 订阅
    - 委托 DataManager 进行数据缓存和访问
    
    注意：此类是 ROS2 特定的。ROS1 节点直接使用 DataManager。
    """
    
    def __init__(self, node: Node, topics: Dict[str, str],
                 callback_group: Optional[CallbackGroup] = None):
        """
        初始化订阅管理器
        
        Args:
            node: ROS2 节点
            topics: 话题配置字典 {'odom': '/odom', 'imu': '/imu', ...}
            callback_group: 回调组（用于并发控制）
        """
        self._node = node
        self._topics = topics
        self._callback_group = callback_group
        
        # 使用 DataManager 管理数据
        self._data_manager = DataManager(
            get_time_func=lambda: get_time_sec(node)
        )
        
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
        
        # 轨迹订阅（需要自定义消息类型）
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
        self._data_manager.update_odom(msg)
    
    def _imu_callback(self, msg: RosImu):
        """IMU 回调"""
        self._data_manager.update_imu(msg)
    
    def _traj_callback(self, msg):
        """轨迹回调"""
        self._data_manager.update_trajectory(msg)
    
    # ==================== 数据访问（委托给 DataManager）====================
    
    def get_latest_odom(self) -> Optional[Odometry]:
        """获取最新里程计"""
        return self._data_manager.get_latest_odom()
    
    def get_latest_imu(self) -> Optional[Imu]:
        """获取最新 IMU"""
        return self._data_manager.get_latest_imu()
    
    def get_latest_trajectory(self) -> Optional[Trajectory]:
        """获取最新轨迹"""
        return self._data_manager.get_latest_trajectory()
    
    def get_data_ages(self) -> Dict[str, float]:
        """
        获取各数据的年龄（秒）
        
        Returns:
            字典，键为数据名 ('odom', 'imu', 'trajectory')，
            值为距上次更新的秒数。未收到的数据返回 float('inf')。
        """
        return self._data_manager.get_data_ages()
    
    def is_data_fresh(self, max_ages: Dict[str, float]) -> bool:
        """
        检查数据是否新鲜
        
        Args:
            max_ages: 各数据的最大允许年龄（秒）
        
        Returns:
            所有数据都在允许年龄内返回 True
        """
        return self._data_manager.is_data_fresh(max_ages)
    
    def has_required_data(self) -> bool:
        """检查是否有必需的数据（odom 和 trajectory）"""
        return self._data_manager.has_required_data()
    
    @property
    def data_manager(self) -> DataManager:
        """获取底层 DataManager（用于高级操作）"""
        return self._data_manager
