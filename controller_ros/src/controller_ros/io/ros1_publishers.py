"""
ROS1 发布管理器

管理所有 ROS1 发布器，与 ROS2 的 PublisherManager 接口对齐。
"""
from typing import Dict, Any, Optional, Callable
import logging

try:
    import rospy
    from std_msgs.msg import Int32
    from nav_msgs.msg import Path
    from geometry_msgs.msg import PoseStamped
    ROS1_AVAILABLE = True
except ImportError:
    ROS1_AVAILABLE = False

from universal_controller.core.data_types import ControlOutput, AttitudeCommand, Trajectory
from ..adapters import OutputAdapter, AttitudeAdapter
from ..utils.diagnostics_publisher import fill_diagnostics_msg, DiagnosticsThrottler
from ..utils.param_loader import TOPICS_DEFAULTS

logger = logging.getLogger(__name__)


class ROS1PublisherManager:
    """
    ROS1 发布管理器
    
    职责:
    - 创建和管理所有 ROS1 发布器
    - 发布统一控制命令和诊断
    - 与 ROS2 的 PublisherManager 接口对齐
    """
    
    def __init__(self, topics: Dict[str, str],
                 default_frame_id: str = 'base_link',
                 diag_publish_rate: int = 10,
                 get_time_func: Optional[Callable[[], float]] = None,
                 is_quadrotor: bool = False):
        """
        初始化发布管理器
        
        Args:
            topics: 话题配置字典
            default_frame_id: 默认输出坐标系
            diag_publish_rate: 诊断发布降频率
            get_time_func: 时间获取函数
            is_quadrotor: 是否为四旋翼平台
        """
        if not ROS1_AVAILABLE:
            raise RuntimeError("ROS1 (rospy) not available")
        
        self._topics = topics
        self._get_time_func = get_time_func or (lambda: rospy.Time.now().to_sec())
        self._is_quadrotor = is_quadrotor
        
        # 创建适配器
        self._output_adapter = OutputAdapter(default_frame_id, get_time_func=self._get_time_func)
        
        if is_quadrotor:
            self._attitude_adapter = AttitudeAdapter(get_time_func=self._get_time_func)
        else:
            self._attitude_adapter = None
        
        # 消息类型缓存 (在 _create_publishers 中设置)
        self._DiagnosticsV2 = None
        self._UnifiedCmd = None
        self._AttitudeCmd = None
        
        # 创建发布器 (会设置消息类型缓存)
        self._create_publishers()
        
        # 诊断节流器
        self._diag_throttler = DiagnosticsThrottler(publish_rate=diag_publish_rate)
    
    def _create_publishers(self):
        """创建所有发布器"""
        # 尝试导入自定义消息
        try:
            from controller_ros.msg import UnifiedCmd, DiagnosticsV2
            self._UnifiedCmd = UnifiedCmd
            self._DiagnosticsV2 = DiagnosticsV2
            # 预创建可复用的消息对象
            self._reusable_diag_msg = DiagnosticsV2()
        except ImportError:
            self._UnifiedCmd = None
            self._DiagnosticsV2 = None
            self._reusable_diag_msg = None
            rospy.logwarn("Custom messages (UnifiedCmd, DiagnosticsV2) not available")
        
        # 控制命令发布器
        cmd_topic = self._topics.get('cmd_unified', TOPICS_DEFAULTS['cmd_unified'])
        if self._UnifiedCmd is not None:
            self._cmd_pub = rospy.Publisher(cmd_topic, self._UnifiedCmd, queue_size=1)
            rospy.loginfo(f"Publishing cmd to: {cmd_topic}")
        else:
            self._cmd_pub = None
            rospy.logerr(f"Cannot create command publisher: UnifiedCmd not available")
        
        # 诊断发布器
        diag_topic = self._topics.get('diagnostics', TOPICS_DEFAULTS['diagnostics'])
        if self._DiagnosticsV2 is not None:
            self._diag_pub = rospy.Publisher(diag_topic, self._DiagnosticsV2, queue_size=10)
            rospy.loginfo(f"Publishing diagnostics to: {diag_topic}")
        else:
            self._diag_pub = None
            rospy.logwarn(f"Cannot create diagnostics publisher: DiagnosticsV2 not available")
        
        # 状态发布器 (标准消息，始终可用)
        state_topic = self._topics.get('state', TOPICS_DEFAULTS['state'])
        self._state_pub = rospy.Publisher(state_topic, Int32, queue_size=1)
        rospy.loginfo(f"Publishing state to: {state_topic}")
        
        # 调试路径发布器
        debug_path_topic = self._topics.get('debug_path', TOPICS_DEFAULTS['debug_path'])
        self._path_pub = rospy.Publisher(debug_path_topic, Path, queue_size=1)
        rospy.loginfo(f"Publishing debug_path to: {debug_path_topic}")
        
        # 姿态命令发布器 (四旋翼平台)
        if self._is_quadrotor:
            attitude_topic = self._topics.get('attitude_cmd', TOPICS_DEFAULTS['attitude_cmd'])
            try:
                from controller_ros.msg import AttitudeCmd
                self._AttitudeCmd = AttitudeCmd
                self._attitude_pub = rospy.Publisher(attitude_topic, AttitudeCmd, queue_size=1)
                rospy.loginfo(f"Publishing attitude to: {attitude_topic}")
            except ImportError:
                self._attitude_pub = None
                rospy.logwarn("AttitudeCmd message not available")
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
        """发布诊断信息"""
        # 始终发布状态话题
        current_state = diag.get('state', 0)
        state_msg = Int32()
        state_msg.data = current_state
        self._state_pub.publish(state_msg)
        
        # 使用节流器判断是否发布诊断
        if not self._diag_throttler.should_publish(diag, force=force):
            return
        
        if self._diag_pub is None:
            rospy.logwarn_throttle(10.0, "Diagnostics publisher is None, cannot publish")
            return
        
        if self._reusable_diag_msg is None:
            rospy.logwarn_throttle(10.0, "DiagnosticsV2 message not available, cannot publish")
            return
        
        try:
            # 复用消息对象，避免每次创建新对象
            fill_diagnostics_msg(self._reusable_diag_msg, diag, get_time_func=lambda: rospy.Time.now())
            self._diag_pub.publish(self._reusable_diag_msg)
        except Exception as e:
            rospy.logwarn_throttle(10.0, f"Failed to publish diagnostics: {e}")
    
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
    
    def publish_debug_path(self, trajectory: Trajectory):
        """
        发布调试路径
        
        将轨迹转换为 nav_msgs/Path 消息发布，用于 RViz 可视化。
        
        Args:
            trajectory: UC 轨迹数据
        """
        if self._path_pub is None:
            return
        
        # 检查轨迹点是否有效
        if trajectory.points is None or len(trajectory.points) == 0:
            return
        
        path = Path()
        path.header.stamp = rospy.Time.now()
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
        
        清理所有发布器引用。在 ROS1 中，发布器会在节点关闭时自动清理，
        但显式清理可以更早释放资源。
        """
        # 注销发布器
        if self._cmd_pub is not None:
            self._cmd_pub.unregister()
            self._cmd_pub = None
        
        if self._diag_pub is not None:
            self._diag_pub.unregister()
            self._diag_pub = None
        
        if self._state_pub is not None:
            self._state_pub.unregister()
            self._state_pub = None
        
        if self._path_pub is not None:
            self._path_pub.unregister()
            self._path_pub = None
        
        if self._attitude_pub is not None:
            self._attitude_pub.unregister()
            self._attitude_pub = None
        
        self._output_adapter = None
        self._attitude_adapter = None
        self._diag_throttler = None
