"""
发布管理器

管理所有 ROS2 发布器。
"""
from typing import Dict, Any, Optional

from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32

from universal_controller.core.data_types import ControlOutput, Trajectory, AttitudeCommand
from ..adapters import OutputAdapter, AttitudeAdapter
from ..utils.ros_compat import get_time_sec
from ..utils.diagnostics_publisher import fill_diagnostics_msg, DiagnosticsThrottler
from ..utils.param_loader import TOPICS_DEFAULTS


class PublisherManager:
    """
    发布管理器 (ROS2)
    
    职责:
    - 创建和管理所有发布器
    - 发布统一控制命令和诊断
    - 支持四旋翼平台姿态命令发布
    
    注意: 不做平台特定转换，只发布 UnifiedCmd
    """
    
    def __init__(self, node: Node, topics: Dict[str, str],
                 default_frame_id: str = 'base_link',
                 diag_publish_rate: int = 10,
                 is_quadrotor: bool = False):
        """
        初始化发布管理器
        
        Args:
            node: ROS2 节点
            topics: 话题配置字典
            default_frame_id: 默认输出坐标系
            diag_publish_rate: 诊断发布降频率 (每 N 次控制循环发布一次)
            is_quadrotor: 是否为四旋翼平台
        """
        self._node = node
        self._topics = topics
        self._is_quadrotor = is_quadrotor
        
        # 创建时间获取函数，支持仿真时间
        self._get_time_func = lambda: get_time_sec(node)
        self._output_adapter = OutputAdapter(default_frame_id, get_time_func=self._get_time_func)
        
        # 姿态适配器 (四旋翼平台)
        if is_quadrotor:
            self._attitude_adapter = AttitudeAdapter(get_time_func=self._get_time_func)
        else:
            self._attitude_adapter = None
        
        # 创建发布器
        self._create_publishers()
        
        # 使用统一的诊断发布节流器
        self._diag_throttler = DiagnosticsThrottler(publish_rate=diag_publish_rate)
    
    def _create_publishers(self):
        """创建所有发布器"""
        # 统一控制命令发布
        cmd_topic = self._topics.get('cmd_unified', TOPICS_DEFAULTS['cmd_unified'])
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
        diag_topic = self._topics.get('diagnostics', TOPICS_DEFAULTS['diagnostics'])
        try:
            from controller_ros.msg import DiagnosticsV2
            self._diag_pub = self._node.create_publisher(
                DiagnosticsV2, diag_topic, 10
            )
            self._node.get_logger().info(f"Publishing diagnostics to: {diag_topic}")
        except ImportError:
            self._node.get_logger().warn("DiagnosticsV2 message not available")
            self._diag_pub = None
        
        # 状态发布器 (需求文档要求)
        state_topic = self._topics.get('state', TOPICS_DEFAULTS['state'])
        self._state_pub = self._node.create_publisher(
            Int32, state_topic, 1
        )
        self._node.get_logger().info(f"Publishing state to: {state_topic}")
        
        # 调试路径发布
        debug_path_topic = self._topics.get('debug_path', TOPICS_DEFAULTS['debug_path'])
        self._path_pub = self._node.create_publisher(
            Path, debug_path_topic, 1
        )
        
        # 姿态命令发布器 (四旋翼平台)
        if self._is_quadrotor:
            attitude_topic = self._topics.get('attitude_cmd', TOPICS_DEFAULTS['attitude_cmd'])
            try:
                from controller_ros.msg import AttitudeCmd
                self._attitude_pub = self._node.create_publisher(
                    AttitudeCmd, attitude_topic, 1
                )
                self._node.get_logger().info(f"Publishing attitude to: {attitude_topic}")
            except ImportError:
                self._attitude_pub = None
                self._node.get_logger().warn("AttitudeCmd message not available")
        else:
            self._attitude_pub = None
    
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
        # 始终发布状态话题 (不降频)
        current_state = diag.get('state', 0)
        state_msg = Int32()
        state_msg.data = current_state
        self._state_pub.publish(state_msg)
        
        # 使用节流器判断是否发布诊断
        if not self._diag_throttler.should_publish(diag, force=force):
            return
        
        if self._diag_pub is None:
            return
        
        try:
            from controller_ros.msg import DiagnosticsV2
            
            msg = DiagnosticsV2()
            fill_diagnostics_msg(
                msg, diag,
                get_time_func=lambda: self._node.get_clock().now().to_msg()
            )
            
            self._diag_pub.publish(msg)
        except ImportError:
            # 消息类型不可用，静默忽略（已在初始化时记录警告）
            pass
        except AttributeError as e:
            # 消息字段访问错误，可能是消息定义不匹配
            self._node.get_logger().warn(
                f"Diagnostics message attribute error (possible version mismatch): {e}",
                throttle_duration_sec=10.0
            )
        except TypeError as e:
            # 类型转换错误
            self._node.get_logger().warn(
                f"Diagnostics type conversion error: {e}",
                throttle_duration_sec=10.0
            )
    
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
    
    def publish_attitude_cmd(self, attitude_cmd: AttitudeCommand,
                             yaw_mode: int = 0, is_hovering: bool = False):
        """
        发布姿态命令 (四旋翼平台)
        
        Args:
            attitude_cmd: 姿态命令
            yaw_mode: 航向模式
            is_hovering: 是否悬停
        """
        if self._attitude_pub is None or self._attitude_adapter is None:
            return
        
        ros_msg = self._attitude_adapter.to_ros(
            attitude_cmd,
            yaw_mode=yaw_mode,
            is_hovering=is_hovering
        )
        self._attitude_pub.publish(ros_msg)
    
    @property
    def output_adapter(self) -> OutputAdapter:
        """获取输出适配器"""
        return self._output_adapter
    
    @property
    def attitude_adapter(self) -> Optional[AttitudeAdapter]:
        """获取姿态适配器"""
        return self._attitude_adapter

    def shutdown(self) -> None:
        """
        关闭发布管理器，释放资源
        
        清理所有发布器引用。在 ROS2 中，发布器会在节点销毁时自动清理，
        但显式清理可以更早释放资源。
        """
        self._cmd_pub = None
        self._diag_pub = None
        self._state_pub = None
        self._path_pub = None
        self._attitude_pub = None
        self._output_adapter = None
        self._attitude_adapter = None
        self._diag_throttler = None
