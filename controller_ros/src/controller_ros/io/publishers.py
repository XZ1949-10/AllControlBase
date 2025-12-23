"""
发布管理器

管理所有 ROS 发布器。
"""
from typing import Dict, Any, Optional

from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from universal_controller.core.data_types import ControlOutput, Trajectory
from ..adapters import OutputAdapter
from ..utils.ros_compat import get_time_sec

# 默认诊断发布降频率 (每 N 次控制循环发布一次)
DEFAULT_DIAG_PUBLISH_RATE = 5


class PublisherManager:
    """
    发布管理器
    
    职责:
    - 创建和管理所有发布器
    - 发布统一控制命令和诊断
    
    注意: 不做平台特定转换，只发布 UnifiedCmd
    """
    
    def __init__(self, node: Node, topics: Dict[str, str],
                 default_frame_id: str = 'base_link',
                 diag_publish_rate: int = DEFAULT_DIAG_PUBLISH_RATE):
        """
        初始化发布管理器
        
        Args:
            node: ROS2 节点
            topics: 话题配置字典
            default_frame_id: 默认输出坐标系
            diag_publish_rate: 诊断发布降频率 (每 N 次控制循环发布一次)
        """
        self._node = node
        self._topics = topics
        
        # 创建时间获取函数，支持仿真时间
        self._output_adapter = OutputAdapter(
            default_frame_id, 
            get_time_func=lambda: get_time_sec(node)
        )
        
        # 创建发布器
        self._create_publishers()
        
        # 诊断降频计数器
        # 初始化为 publish_rate - 1，确保首次调用时立即发布
        self._diag_publish_rate = max(1, diag_publish_rate)  # 至少为 1
        self._diag_counter = self._diag_publish_rate - 1
        
        # 上一次发布的状态，用于检测状态变化
        self._last_state: Optional[int] = None
    
    def _create_publishers(self):
        """创建所有发布器"""
        # 统一控制命令发布
        cmd_topic = self._topics.get('cmd_unified', '/cmd_unified')
        try:
            from controller_ros.msg import UnifiedCmd
            self._cmd_pub = self._node.create_publisher(
                UnifiedCmd, cmd_topic, 10
            )
            self._node.get_logger().info(f"Publishing cmd to: {cmd_topic}")
        except ImportError:
            self._node.get_logger().warn("UnifiedCmd message not available")
            self._cmd_pub = None
        
        # 诊断发布
        diag_topic = self._topics.get('diagnostics', '/controller/diagnostics')
        try:
            from controller_ros.msg import DiagnosticsV2
            self._diag_pub = self._node.create_publisher(
                DiagnosticsV2, diag_topic, 10
            )
            self._node.get_logger().info(f"Publishing diagnostics to: {diag_topic}")
        except ImportError:
            self._node.get_logger().warn("DiagnosticsV2 message not available")
            self._diag_pub = None
        
        # 调试路径发布
        self._path_pub = self._node.create_publisher(
            Path, '/controller/debug_path', 1
        )
    
    def publish_cmd(self, cmd: ControlOutput):
        """发布统一控制命令"""
        if self._cmd_pub is None:
            return
        
        ros_msg = self._output_adapter.to_ros(cmd)
        self._cmd_pub.publish(ros_msg)
    
    def publish_stop_cmd(self):
        """发布停止命令"""
        if self._cmd_pub is None:
            return
        
        ros_msg = self._output_adapter.create_stop_cmd()
        self._cmd_pub.publish(ros_msg)
    
    def publish_diagnostics(self, diag: Dict[str, Any], force: bool = False):
        """
        发布诊断信息
        
        Args:
            diag: 诊断数据字典
            force: 是否强制发布 (忽略降频)
        """
        if self._diag_pub is None:
            return
        
        # 检测状态变化，状态变化时强制发布
        current_state = diag.get('state', 0)
        state_changed = self._last_state is not None and current_state != self._last_state
        self._last_state = current_state
        
        # 降频发布，但状态变化时强制发布
        self._diag_counter += 1
        if not force and not state_changed and self._diag_counter < self._diag_publish_rate:
            return
        self._diag_counter = 0
        
        try:
            from controller_ros.msg import DiagnosticsV2
            from ..utils.diag_filler import fill_diagnostics_msg
            
            msg = DiagnosticsV2()
            fill_diagnostics_msg(
                msg, diag,
                get_time_func=lambda: self._node.get_clock().now().to_msg()
            )
            
            self._diag_pub.publish(msg)
        except Exception as e:
            self._node.get_logger().warn(f"Failed to publish diagnostics: {e}")
    
    def publish_debug_path(self, trajectory: Trajectory):
        """发布调试路径"""
        if self._path_pub is None:
            return
        
        # 检查轨迹点是否有效
        if trajectory.points is None or len(trajectory.points) == 0:
            return
        
        path = Path()
        path.header.stamp = self._node.get_clock().now().to_msg()
        path.header.frame_id = trajectory.header.frame_id or 'odom'
        
        for p in trajectory.points:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = p.x
            pose.pose.position.y = p.y
            pose.pose.position.z = p.z
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        
        self._path_pub.publish(path)
