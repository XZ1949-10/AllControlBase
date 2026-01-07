"""
Controller ROS1 Node

.. deprecated::
    This module is deprecated. Use scripts/controller_node.py instead.
    This file is kept for backward compatibility but is no longer maintained.
    The scripts/controller_node.py implementation is more complete and correct.

Main node implementation for ROS1 (Noetic).
Inherits from ControllerNodeBase for shared logic.

Usage:
    rosrun controller_ros controller_node.py  # Use scripts/controller_node.py instead
"""
import warnings
warnings.warn(
    "controller_ros.node.controller_node_ros1 is deprecated. "
    "Use scripts/controller_node.py instead.",
    DeprecationWarning,
    stacklevel=2
)
from typing import Dict, Any, Optional
import threading

try:
    import rospy
    from nav_msgs.msg import Odometry as RosOdometry
    from sensor_msgs.msg import Imu as RosImu
    from std_msgs.msg import Empty, Int32
    from std_srvs.srv import Empty as EmptySrv, EmptyResponse
    from std_srvs.srv import SetBool, SetBoolResponse
    ROS1_AVAILABLE = True
except ImportError:
    ROS1_AVAILABLE = False

from universal_controller.core.data_types import ControlOutput, AttitudeCommand

from .base_node import ControllerNodeBase
from ..bridge import TFBridge
from ..io.ros1_publishers import ROS1PublisherManager
from ..io.ros1_services import ROS1ServiceManager
from ..utils import ParamLoader
from ..utils.param_loader import TOPICS_DEFAULTS
from ..utils.msg_availability import (
    CUSTOM_MSGS_AVAILABLE,
    LocalTrajectoryV4,
    UnifiedCmd,
    DiagnosticsV2,
    AttitudeCmd,
)


class ControllerNodeROS1(ControllerNodeBase):
    """
    Controller Main Node (ROS1)
    
    Inherits ControllerNodeBase for shared logic,
    implements ROS1 specific interfaces.
    
    Outputs:
    - /cmd_unified (UnifiedCmd)
    - /controller/diagnostics (DiagnosticsV2)
    - /controller/state (Int32)
    - /controller/attitude_cmd (AttitudeCmd, quadrotor only)
    """
    
    def __init__(self):
        if not ROS1_AVAILABLE:
            raise RuntimeError("ROS1 (rospy) not available")
        
        # Initialize ROS1 node first
        rospy.init_node('universal_controller_node', anonymous=False)
        
        # Initialize base class
        ControllerNodeBase.__init__(self)
        
        # 1. Load parameters
        self._params = ParamLoader.load(node=None)
        self._topics = ParamLoader.get_topics(node=None)
        
        # 1.1 Load clock config and merge into params
        clock_config = ParamLoader.get_clock_config(node=None)
        self._params['clock'] = clock_config
        
        # 2. Initialize core components (base class method)
        self._initialize()
        
        # 3. Create TF bridge
        self._tf_bridge = TFBridge(node=None)  # ROS1 doesn't need node
        
        # 4. Inject TF2 to coordinate transformer (base class method)
        self._inject_tf2_to_controller()
        
        # 5. Create ROS1 interfaces
        self._create_ros_interfaces()
        
        # 6. Create control timer
        control_rate = self._params.get('system', {}).get('ctrl_freq', 50)
        control_period = 1.0 / control_rate
        self._control_timer = rospy.Timer(
            rospy.Duration(control_period),
            self._control_callback
        )
        
        # Register shutdown callback
        rospy.on_shutdown(self.shutdown)
        
        # Check trajectory message availability
        self._traj_msg_available = CUSTOM_MSGS_AVAILABLE
        
        rospy.loginfo(
            f'Controller node initialized (platform={self._platform_type}, '
            f'rate={control_rate}Hz, tf2={self._tf_bridge.is_initialized}, '
            f'quadrotor={self._is_quadrotor})'
        )
    
    def _create_ros_interfaces(self):
        """Create ROS1 specific interfaces"""
        # Create publisher manager
        # 同时设置 _publishers 以兼容基类 _publish_extra_outputs() 方法
        diag_publish_rate = self._params.get('diagnostics', {}).get('publish_rate', 10)
        self._publishers = ROS1PublisherManager(
            topics=self._topics,
            default_frame_id=self._default_frame_id,
            diag_publish_rate=diag_publish_rate,
            is_quadrotor=self._is_quadrotor,
        )
        # 向后兼容别名
        self._publisher_manager = self._publishers
        
        # Create service manager
        # 同时设置 _services 以保持与基类命名一致
        self._services = ROS1ServiceManager(
            reset_callback=self._handle_reset,
            get_diagnostics_callback=self._handle_get_diagnostics,
            set_state_callback=self._handle_set_state,
            set_hover_yaw_callback=self._handle_set_hover_yaw if self._is_quadrotor else None,
            get_attitude_rate_limits_callback=self._handle_get_attitude_rate_limits if self._is_quadrotor else None,
            is_quadrotor=self._is_quadrotor,
        )
        # 向后兼容别名
        self._service_manager = self._services
        
        # Create subscribers
        self._create_subscriptions()
    
    def _create_subscriptions(self):
        """Create ROS1 subscriptions"""
        # Odometry subscription (required)
        odom_topic = self._topics.get('odom', TOPICS_DEFAULTS['odom'])
        self._odom_sub = rospy.Subscriber(
            odom_topic, RosOdometry, self._odom_callback, queue_size=10
        )
        
        # IMU subscription (optional)
        imu_topic = self._topics.get('imu', '')
        if imu_topic:
            self._imu_sub = rospy.Subscriber(
                imu_topic, RosImu, self._imu_callback, queue_size=10
            )
        else:
            self._imu_sub = None
        
        # Trajectory subscription (required)
        traj_topic = self._topics.get('trajectory', TOPICS_DEFAULTS['trajectory'])
        if CUSTOM_MSGS_AVAILABLE and LocalTrajectoryV4 is not None:
            self._traj_sub = rospy.Subscriber(
                traj_topic, LocalTrajectoryV4, self._traj_callback, queue_size=10
            )
        else:
            self._traj_sub = None
            rospy.logwarn("LocalTrajectoryV4 message not available, trajectory subscription disabled")
        
        # Emergency stop subscription (required)
        emergency_topic = self._topics.get('emergency_stop', TOPICS_DEFAULTS['emergency_stop'])
        self._emergency_stop_sub = rospy.Subscriber(
            emergency_topic, Empty, self._emergency_stop_callback, queue_size=1
        )
    
    # ==================== Callbacks ====================
    
    def _odom_callback(self, msg):
        """Odometry callback"""
        self._data_manager.update_odom(msg)
    
    def _imu_callback(self, msg):
        """IMU callback"""
        self._data_manager.update_imu(msg)
    
    def _traj_callback(self, msg):
        """Trajectory callback"""
        self._data_manager.update_trajectory(msg)
    
    def _emergency_stop_callback(self, msg):
        """Emergency stop callback"""
        self._handle_emergency_stop()
    
    def _control_callback(self, event):
        """Control timer callback"""
        cmd = self._control_loop_core()
        if cmd is not None:
            self._publish_cmd(cmd)
    
    # ==================== Abstract Method Implementations ====================
    
    def _get_time(self) -> float:
        """Get current ROS time (seconds)"""
        # Always use ROS time.
        # If rospy is not initialized, this will raise, which is correct (Fail Fast).
        return rospy.Time.now().to_sec()
    
    def _log_info(self, msg: str):
        rospy.loginfo(msg)
    
    def _log_warn(self, msg: str):
        rospy.logwarn(msg)
    
    def _log_warn_throttle(self, period: float, msg: str):
        rospy.logwarn_throttle(period, msg)
    
    def _log_error(self, msg: str):
        rospy.logerr(msg)
    
    def _publish_cmd(self, cmd: ControlOutput):
        """Publish control command"""
        if self._publishers is not None:
            self._publishers.publish_cmd(cmd)
    
    def _publish_stop_cmd(self):
        """Publish stop command"""
        if self._publishers is not None:
            self._publishers.publish_stop_cmd()
    
    def _publish_diagnostics(self, diag: Dict[str, Any], force: bool = False):
        """Publish diagnostics"""
        if self._publishers is not None:
            self._publishers.publish_diagnostics(diag, force=force)
    
    def _publish_attitude_cmd(self, attitude_cmd: AttitudeCommand):
        """Publish attitude command (quadrotor only)"""
        if self._publishers is not None and self._is_quadrotor:
            self._publishers.publish_attitude_cmd(attitude_cmd)
    
    def _publish_debug_path(self, trajectory):
        """Publish debug path for RViz visualization"""
        if self._publishers is not None:
            self._publishers.publish_debug_path(trajectory)
    
    # ==================== Lifecycle ====================
    
    def shutdown(self):
        """
        清理资源 (ROS1 版本)
        
        完整的资源清理流程:
        1. 设置关闭标志，阻止控制循环继续执行
        2. 取消控制定时器
        3. 发送最终停止命令
        4. 注销订阅器
        5. 关闭发布器和服务
        6. 关闭 TF 桥接
        7. 调用基类清理
        """
        # 1. 设置关闭标志
        self._shutting_down.set()
        
        # 2. 取消控制定时器
        if self._control_timer is not None:
            self._control_timer.shutdown()
            self._control_timer = None
        
        # 3. 发送最终停止命令
        try:
            if self._publishers is not None:
                self._publishers.publish_stop_cmd()
                rospy.loginfo("Final stop command sent")
        except Exception as e:
            rospy.logwarn(f"Failed to send final stop command: {e}")
        
        # 4. 注销订阅器 (ROS1 使用 unregister)
        if self._odom_sub is not None:
            self._odom_sub.unregister()
            self._odom_sub = None
        
        if self._imu_sub is not None:
            self._imu_sub.unregister()
            self._imu_sub = None
        
        if self._traj_sub is not None:
            self._traj_sub.unregister()
            self._traj_sub = None
        
        if self._emergency_stop_sub is not None:
            self._emergency_stop_sub.unregister()
            self._emergency_stop_sub = None
        
        # 5. 关闭发布器和服务
        if self._publishers is not None:
            self._publishers.shutdown()
            self._publishers = None
        
        if self._services is not None:
            self._services.shutdown()
            self._services = None
        
        # 6. 关闭 TF 桥接
        if self._tf_bridge is not None:
            self._tf_bridge.shutdown()
            self._tf_bridge = None
        
        # 7. 调用基类清理
        ControllerNodeBase.shutdown(self)
        
        rospy.loginfo("Controller node shutdown complete")
    
    def run(self):
        """Run the node"""
        rospy.loginfo("Controller node running...")
        rospy.spin()


def main():
    """Main entry point"""
    try:
        node = ControllerNodeROS1()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        if ROS1_AVAILABLE:
            rospy.logerr(f"Controller node exception: {e}")
        raise


if __name__ == '__main__':
    main()
