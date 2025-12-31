"""
Controller ROS1 Node

Main node implementation for ROS1 (Noetic).
Inherits from ControllerNodeBase for shared logic.

Usage:
    rosrun controller_ros controller_node_ros1.py
    roslaunch controller_ros controller.launch
"""
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
        self._publisher_manager = ROS1PublisherManager(
            topics=self._topics,
            default_frame_id=self._default_frame_id,
            is_quadrotor=self._is_quadrotor,
        )
        
        # Create service manager
        self._service_manager = ROS1ServiceManager(
            reset_callback=self._handle_reset,
            get_diagnostics_callback=self._handle_get_diagnostics,
            set_state_callback=self._handle_set_state,
            is_quadrotor=self._is_quadrotor,
        )
        
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
        self._notify_odom_received()
    
    def _imu_callback(self, msg):
        """IMU callback"""
        self._data_manager.update_imu(msg)
        self._notify_imu_received()
    
    def _traj_callback(self, msg):
        """Trajectory callback"""
        self._data_manager.update_trajectory(msg)
        self._notify_trajectory_received()
    
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
        try:
            t = rospy.Time.now().to_sec()
            return t if t > 0 else rospy.get_time()
        except Exception:
            import time
            return time.time()
    
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
        if self._publisher_manager is not None:
            self._publisher_manager.publish_cmd(cmd)
    
    def _publish_stop_cmd(self):
        """Publish stop command"""
        if self._publisher_manager is not None:
            self._publisher_manager.publish_stop_cmd()
    
    def _publish_diagnostics(self, diag: Dict[str, Any], force: bool = False):
        """Publish diagnostics"""
        if self._publisher_manager is not None:
            self._publisher_manager.publish_diagnostics(diag, force=force)
    
    def _publish_attitude_cmd(self, attitude_cmd: AttitudeCommand):
        """Publish attitude command (quadrotor only)"""
        if self._publisher_manager is not None and self._is_quadrotor:
            self._publisher_manager.publish_attitude_cmd(attitude_cmd)
    
    def _publish_debug_path(self, trajectory):
        """Publish debug path for RViz visualization"""
        if self._publisher_manager is not None:
            self._publisher_manager.publish_debug_path(trajectory)
    
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
