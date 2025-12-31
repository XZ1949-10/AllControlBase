#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Trajectory Publisher - Subscribe /waypoint and convert to LocalTrajectoryV4

Features:
- Subscribe /waypoint (std_msgs/Float32MultiArray) with N points [x, y] coordinates
- Convert to controller_ros/LocalTrajectoryV4 format
- Publish to /controller/input/trajectory

Usage:
    rosrun controller_ros trajectory_publisher.py
    rosrun controller_ros trajectory_publisher.py --stop  # Publish stop trajectory

Topics:
    Subscribe: /waypoint (std_msgs/Float32MultiArray)
               data format: [x0, y0, x1, y1, x2, y2, ..., xN, yN]
    Publish: /controller/input/trajectory (controller_ros/LocalTrajectoryV4)

Note:
    - Input trajectory coordinates assumed in base_link frame
    - Differential drive only uses x, y coordinates
"""

import rospy
import numpy as np
from typing import Optional, Dict, Any
from geometry_msgs.msg import Point
from std_msgs.msg import Header, Float32MultiArray

from controller_ros.utils.param_loader import ParamLoader, TOPICS_DEFAULTS
from controller_ros.lifecycle import LifecycleMixin

try:
    from controller_ros.msg import LocalTrajectoryV4
except ImportError:
    rospy.logerr("Cannot import controller_ros.msg, ensure controller_ros is built")
    raise

try:
    from universal_controller.core.enums import TrajectoryMode
    MODE_TRACK = TrajectoryMode.MODE_TRACK.value
    MODE_STOP = TrajectoryMode.MODE_STOP.value
    MODE_HOVER = TrajectoryMode.MODE_HOVER.value
    MODE_EMERGENCY = TrajectoryMode.MODE_EMERGENCY.value
except ImportError:
    MODE_TRACK = 0
    MODE_STOP = 1
    MODE_HOVER = 2
    MODE_EMERGENCY = 3


class TrajectoryPublisher(LifecycleMixin):
    """
    Trajectory Publisher Class
    
    Subscribe /waypoint topic and convert to LocalTrajectoryV4 format.
    
    Lifecycle:
    - Implements ILifecycleComponent via LifecycleMixin
    - initialize(): Setup ROS interfaces
    - reset(): Reset statistics
    - shutdown(): Publish stop trajectory and cleanup
    """
    
    # Validation constants
    MAX_COORD = 100.0  # Maximum reasonable coordinate (meters)
    
    def __init__(self):
        LifecycleMixin.__init__(self)
        rospy.init_node('trajectory_publisher', anonymous=False)
        
        # Load configuration using unified ParamLoader
        config = ParamLoader.load(node=None, validate=False)
        topics = ParamLoader.get_topics(node=None)
        
        # Get trajectory parameters from config
        traj_config = config.get('trajectory', {})
        transform_config = config.get('transform', {})
        mpc_config = config.get('mpc', {})
        
        # dt_sec priority: trajectory.default_dt_sec > mpc.dt > default
        self._dt = traj_config.get('default_dt_sec', mpc_config.get('dt', 0.1))
        self._confidence = rospy.get_param('~confidence', 1.0)
        self._soft_enabled = rospy.get_param('~soft_enabled', False)
        
        # Frame ID: use transform.source_frame for consistency
        private_frame = rospy.get_param('~frame_id', None)
        if private_frame:
            self._frame_id = private_frame
        else:
            self._frame_id = transform_config.get('source_frame', 'base_link')
        
        # Topics: use unified topic configuration
        self._input_topic = rospy.get_param('~input_topic', '/waypoint')
        self._output_topic = rospy.get_param(
            '~output_topic', 
            topics.get('trajectory', TOPICS_DEFAULTS['trajectory'])
        )
        
        # State
        self._last_waypoint = None
        self._publish_count = 0
        self._receive_count = 0
        self._invalid_count = 0
        
        # ROS interfaces
        self._pub = None
        self._sub = None
        
        rospy.on_shutdown(self.shutdown)
        
        if not self.initialize():
            raise RuntimeError(f"Initialization failed: {self._last_error}")
    
    def _do_initialize(self) -> bool:
        """Initialize ROS interfaces"""
        try:
            self._pub = rospy.Publisher(self._output_topic, LocalTrajectoryV4, queue_size=1)
            self._sub = rospy.Subscriber(self._input_topic, Float32MultiArray, self._waypoint_callback)
            
            rospy.loginfo("=" * 50)
            rospy.loginfo("TrajectoryPublisher initialized")
            rospy.loginfo(f"  Subscribe: {self._input_topic} (Float32MultiArray)")
            rospy.loginfo(f"  Publish: {self._output_topic} (LocalTrajectoryV4)")
            rospy.loginfo(f"  dt: {self._dt} s")
            rospy.loginfo(f"  Frame: {self._frame_id}")
            rospy.loginfo(f"  Soft constraint: {'enabled' if self._soft_enabled else 'disabled'}")
            rospy.loginfo("=" * 50)
            return True
        except Exception as e:
            rospy.logerr(f"TrajectoryPublisher initialization error: {e}")
            return False
    
    def _do_shutdown(self) -> None:
        """Shutdown and publish stop trajectory"""
        rospy.loginfo("TrajectoryPublisher shutting down...")
        
        # Publish stop trajectory for safety
        try:
            if self._pub is not None:
                stop_msg = self._create_stop_trajectory()
                self._pub.publish(stop_msg)
                rospy.loginfo("Published stop trajectory on shutdown")
        except Exception as e:
            rospy.logwarn(f"Failed to publish stop trajectory: {e}")
        
        if self._sub is not None:
            self._sub.unregister()
            self._sub = None
    
    def _do_reset(self) -> None:
        """Reset statistics"""
        self._last_waypoint = None
        self._publish_count = 0
        self._receive_count = 0
        self._invalid_count = 0
        rospy.loginfo("TrajectoryPublisher state reset")
    
    def _get_health_details(self) -> Dict[str, Any]:
        """Get health details"""
        return {
            'publish_count': self._publish_count,
            'receive_count': self._receive_count,
            'invalid_count': self._invalid_count,
            'has_waypoint': self._last_waypoint is not None,
        }
    
    def _waypoint_callback(self, msg: Float32MultiArray):
        """Waypoint topic callback"""
        self._receive_count += 1
        
        data = list(msg.data)
        if len(data) < 2:
            rospy.logwarn("Received empty trajectory data")
            return
        
        if len(data) % 2 != 0:
            rospy.logwarn(f"Trajectory data length {len(data)} not even, truncating")
            data = data[:-1]
        
        num_points = len(data) // 2
        positions = np.array(data).reshape(num_points, 2)
        
        # Validate data
        if not np.all(np.isfinite(positions)):
            self._invalid_count += 1
            rospy.logwarn_throttle(1.0, "Trajectory contains NaN/Inf, ignoring")
            return
        
        if np.any(np.abs(positions) > self.MAX_COORD):
            self._invalid_count += 1
            rospy.logwarn_throttle(1.0, f"Trajectory coordinates exceed {self.MAX_COORD}m")
            return
        
        # Create and publish message
        traj_msg = self._create_trajectory_msg(positions)
        self._pub.publish(traj_msg)
        
        self._publish_count += 1
        self._last_waypoint = positions
        
        if self._publish_count % 50 == 0:
            rospy.loginfo(f"Published {self._publish_count} trajectories ({num_points} points)")
    
    def _create_trajectory_msg(self, positions: np.ndarray, 
                               velocities: np.ndarray = None,
                               mode: int = MODE_TRACK) -> LocalTrajectoryV4:
        """Create LocalTrajectoryV4 message"""
        msg = LocalTrajectoryV4()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self._frame_id
        msg.mode = mode
        
        msg.points = []
        for i in range(len(positions)):
            p = Point()
            p.x = float(positions[i, 0])
            p.y = float(positions[i, 1])
            p.z = 0.0
            msg.points.append(p)
        
        if velocities is not None and len(velocities) > 0 and self._soft_enabled:
            msg.velocities_flat = []
            for i in range(len(velocities)):
                if velocities.shape[1] >= 4:
                    msg.velocities_flat.extend([
                        float(velocities[i, 0]),
                        float(velocities[i, 1]),
                        float(velocities[i, 2]),
                        float(velocities[i, 3]),
                    ])
                else:
                    msg.velocities_flat.extend([
                        float(velocities[i, 0]),
                        float(velocities[i, 1]),
                        0.0, 0.0,
                    ])
            msg.soft_enabled = True
        else:
            msg.velocities_flat = []
            msg.soft_enabled = False
        
        msg.dt_sec = self._dt
        msg.confidence = self._confidence
        return msg
    
    def _create_stop_trajectory(self) -> LocalTrajectoryV4:
        """Create stop trajectory message"""
        msg = LocalTrajectoryV4()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self._frame_id
        msg.mode = MODE_STOP
        msg.points = [Point(x=0.0, y=0.0, z=0.0)]
        msg.velocities_flat = []
        msg.soft_enabled = False
        msg.dt_sec = self._dt
        msg.confidence = 1.0
        return msg
    
    def run(self):
        """Main loop"""
        rospy.loginfo("TrajectoryPublisher running...")
        rospy.spin()


class StopTrajectoryPublisher:
    """Publish stop trajectory (one-shot)"""
    
    def __init__(self):
        rospy.init_node('stop_trajectory_publisher', anonymous=True)
        
        # Use unified configuration
        config = ParamLoader.load(node=None, validate=False)
        topics = ParamLoader.get_topics(node=None)
        transform_config = config.get('transform', {})
        
        self._output_topic = rospy.get_param(
            '~output_topic',
            topics.get('trajectory', TOPICS_DEFAULTS['trajectory'])
        )
        
        private_frame = rospy.get_param('~frame_id', None)
        self._frame_id = private_frame or transform_config.get('source_frame', 'base_link')
        
        self._pub = rospy.Publisher(self._output_topic, LocalTrajectoryV4, queue_size=1)
        rospy.sleep(0.5)
    
    def publish_stop(self):
        """Publish stop trajectory"""
        msg = LocalTrajectoryV4()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self._frame_id
        msg.mode = MODE_STOP
        msg.points = [Point(x=0.0, y=0.0, z=0.0)]
        msg.velocities_flat = []
        msg.soft_enabled = False
        msg.dt_sec = 0.1
        msg.confidence = 1.0
        
        self._pub.publish(msg)
        rospy.loginfo("Published stop trajectory")


def main():
    try:
        publisher = TrajectoryPublisher()
        publisher.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"TrajectoryPublisher exception: {e}")
        raise


def main_stop():
    try:
        publisher = StopTrajectoryPublisher()
        publisher.publish_stop()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"StopTrajectoryPublisher exception: {e}")
        raise


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='Trajectory Publisher')
    parser.add_argument('--stop', action='store_true', help='Publish stop trajectory')
    args = parser.parse_args()
    
    if args.stop:
        main_stop()
    else:
        main()
