"""
ROS1 发布管理器

管理所有 ROS1 发布器，与 ROS2 的 PublisherManager 接口对齐。
"""
from typing import Dict, Any, Optional, Callable
import logging

try:
    import rospy
    from std_msgs.msg import Int32
    ROS1_AVAILABLE = True
except ImportError:
    ROS1_AVAILABLE = False

from universal_controller.core.data_types import ControlOutput, AttitudeCommand
from ..adapters import OutputAdapter, AttitudeAdapter
from ..utils.diagnostics_publisher import fill_diagnostics_msg, DiagnosticsThrottler

logger = logging.getLogger(__name__)

# 默认话题名常量
DEFAULT_STATE_TOPIC = '/controller/state'
DEFAULT_ATTITUDE_CMD_TOPIC = '/controller/attitude_cmd'


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
        except ImportError:
            self._UnifiedCmd = None
            self._DiagnosticsV2 = None
            rospy.logwarn("Custom messages (UnifiedCmd, DiagnosticsV2) not available")
        
        # 控制命令发布器
        cmd_topic = self._topics.get('cmd_unified', '/cmd_unified')
        if self._UnifiedCmd is not None:
            self._cmd_pub = rospy.Publisher(cmd_topic, self._UnifiedCmd, queue_size=1)
            rospy.loginfo(f"Publishing cmd to: {cmd_topic}")
        else:
            self._cmd_pub = None
            rospy.logerr(f"Cannot create command publisher: UnifiedCmd not available")
        
        # 诊断发布器
        diag_topic = self._topics.get('diagnostics', '/controller/diagnostics')
        if self._DiagnosticsV2 is not None:
            self._diag_pub = rospy.Publisher(diag_topic, self._DiagnosticsV2, queue_size=10)
            rospy.loginfo(f"Publishing diagnostics to: {diag_topic}")
        else:
            self._diag_pub = None
            rospy.logwarn(f"Cannot create diagnostics publisher: DiagnosticsV2 not available")
        
        # 状态发布器 (标准消息，始终可用)
        state_topic = self._topics.get('state', DEFAULT_STATE_TOPIC)
        self._state_pub = rospy.Publisher(state_topic, Int32, queue_size=1)
        rospy.loginfo(f"Publishing state to: {state_topic}")
        
        # 姿态命令发布器 (四旋翼平台)
        if self._is_quadrotor:
            attitude_topic = self._topics.get('attitude_cmd', DEFAULT_ATTITUDE_CMD_TOPIC)
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
        
        if self._DiagnosticsV2 is None:
            rospy.logwarn_throttle(10.0, "DiagnosticsV2 message type is None, cannot publish")
            return
        
        try:
            msg = self._DiagnosticsV2()
            fill_diagnostics_msg(msg, diag, get_time_func=lambda: rospy.Time.now())
            self._diag_pub.publish(msg)
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
    
    @property
    def output_adapter(self) -> OutputAdapter:
        """获取输出适配器"""
        return self._output_adapter
    
    @property
    def attitude_adapter(self) -> Optional[AttitudeAdapter]:
        """获取姿态适配器"""
        return self._attitude_adapter
