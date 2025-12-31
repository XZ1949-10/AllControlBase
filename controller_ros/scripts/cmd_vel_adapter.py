#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
cmd_vel Adapter - Convert UnifiedCmd to TurtleBot cmd_vel

Features:
- Subscribe /cmd_unified (controller_ros/UnifiedCmd) - controller output
- Subscribe /joy_cmd_vel (geometry_msgs/Twist) - joystick control output
- Subscribe /visualizer/control_mode (std_msgs/Bool) - control mode switch
- Publish /cmd_vel (geometry_msgs/Twist)
- Apply velocity limits to protect the robot
- Publish commands at fixed rate to prevent chassis timeout

Architecture:
- This node is a platform adapter responsible for final command processing
- Publishes at fixed rate (publish_rate) to solve chassis cmd_vel timeout
- Command timeout detection for both controller and joystick modes

Control Modes:
- control_mode = False: Use controller output (/cmd_unified)
- control_mode = True:  Use joystick output (/joy_cmd_vel)

Configuration (from ROS parameter server):
    # Constraint parameters (shared with controller for consistency)
    constraints/v_max: Maximum linear velocity (m/s)
    constraints/v_min: Minimum linear velocity (m/s) - negative allows reverse
    constraints/omega_max: Maximum angular velocity (rad/s)
    constraints/a_max: Maximum linear acceleration (m/s²)
    constraints/alpha_max: Maximum angular acceleration (rad/s²)
    
    # cmd_vel_adapter specific parameters
    cmd_vel_adapter/publish_rate: Publish rate (Hz), default 20
    cmd_vel_adapter/cmd_timeout: Command timeout (s), default 0.5
    cmd_vel_adapter/enable_rate_limit: Enable rate limiting, default true
"""

import rospy
import math
import threading
from typing import Optional, Dict, Any
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_srvs.srv import Empty, EmptyResponse

from controller_ros.utils.param_loader import ParamLoader
from controller_ros.lifecycle import LifecycleMixin

try:
    from controller_ros.msg import UnifiedCmd
except ImportError:
    rospy.logerr("Cannot import controller_ros.msg, ensure controller_ros is built")
    raise


class CmdVelAdapter(LifecycleMixin):
    """
    cmd_vel Adapter Class
    
    Responsibilities:
    - Select command source based on control mode (controller/joystick)
    - Apply velocity/acceleration limits to protect robot
    - Publish cmd_vel to chassis at fixed rate
    - Handle command timeout for both modes
    
    Lifecycle:
    - Implements ILifecycleComponent via LifecycleMixin
    - initialize(): Setup ROS interfaces
    - reset(): Reset internal state
    - shutdown(): Stop timer and publish zero velocity
    """
    
    def __init__(self):
        LifecycleMixin.__init__(self)
        rospy.init_node('cmd_vel_adapter', anonymous=False)
        self._shutting_down = threading.Event()
        
        # Load configuration
        # 注意：不需要完整的控制器配置验证，只需要 constraints 验证
        controller_config = ParamLoader.load(node=None, validate=False)
        constraints = controller_config.get('constraints', {})
        
        # 验证约束配置（这是安全关键参数）
        constraint_errors = ParamLoader.validate_constraints(constraints)
        if constraint_errors:
            for err in constraint_errors:
                rospy.logerr(f"Configuration error: {err}")
            raise RuntimeError(f"Constraints validation failed: {constraint_errors}")
        
        topics = ParamLoader.get_topics(node=None)
        adapter_config = ParamLoader.get_cmd_vel_adapter_config(node=None)
        
        # Constraints
        self._v_max = constraints.get('v_max', 2.0)
        self._v_min = constraints.get('v_min', 0.0)
        self._omega_max = constraints.get('omega_max', 2.0)
        self._a_max = constraints.get('a_max', 1.5)
        self._alpha_max = constraints.get('alpha_max', 3.0)
        
        # Adapter config
        self._publish_rate = adapter_config.get('publish_rate', 20.0)
        self._cmd_timeout = adapter_config.get('cmd_timeout', 0.5)
        self._enable_rate_limit = adapter_config.get('enable_rate_limit', True)
        
        # Topics
        self._input_topic = topics.get('cmd_unified', '/cmd_unified')
        self._joy_topic = adapter_config.get('joy_topic', '/joy_cmd_vel')
        self._mode_topic = adapter_config.get('mode_topic', '/visualizer/control_mode')
        self._output_topic = adapter_config.get('output_topic', '/cmd_vel')
        
        # State (protected by lock)
        self._lock = threading.RLock()
        self._joystick_mode = False
        self._latest_controller_cmd = Twist()
        self._latest_joy_cmd = Twist()
        self._controller_cmd_time: Optional[rospy.Time] = None
        self._joy_cmd_time: Optional[rospy.Time] = None
        self._last_linear_x = 0.0
        self._last_angular_z = 0.0
        self._last_time: Optional[rospy.Time] = None
        
        # Statistics
        self._published_cmd_count = 0
        self._velocity_clamped_count = 0
        self._rate_limited_cmd_count = 0
        self._timeout_count = 0
        
        # ROS interfaces
        self._sub = None
        self._joy_sub = None
        self._mode_sub = None
        self._pub = None
        self._timer = None
        self._reset_srv = None
        
        rospy.on_shutdown(self.shutdown)
        
        if not self.initialize():
            raise RuntimeError(f"Initialization failed: {self._last_error}")

    def _do_initialize(self) -> bool:
        """Initialize ROS interfaces"""
        try:
            self._pub = rospy.Publisher(self._output_topic, Twist, queue_size=1)
            self._sub = rospy.Subscriber(
                self._input_topic, UnifiedCmd, self._controller_callback, queue_size=1)
            self._joy_sub = rospy.Subscriber(
                self._joy_topic, Twist, self._joy_callback, queue_size=1)
            self._mode_sub = rospy.Subscriber(
                self._mode_topic, Bool, self._mode_callback, queue_size=1)
            self._timer = rospy.Timer(
                rospy.Duration(1.0 / self._publish_rate), self._timer_callback)
            self._reset_srv = rospy.Service('~reset', Empty, self._reset_service_callback)
            
            rospy.loginfo("CmdVelAdapter initialized:")
            rospy.loginfo(f"  Controller input: {self._input_topic}")
            rospy.loginfo(f"  Joystick input: {self._joy_topic}")
            rospy.loginfo(f"  Output: {self._output_topic}")
            rospy.loginfo(f"  Publish rate: {self._publish_rate} Hz")
            rospy.loginfo(f"  Command timeout: {self._cmd_timeout} s")
            rospy.loginfo(f"  Velocity range: [{self._v_min}, {self._v_max}] m/s")
            rospy.loginfo(f"  Max angular: {self._omega_max} rad/s")
            rospy.loginfo(f"  Rate limit: {'enabled' if self._enable_rate_limit else 'disabled'}")
            return True
        except Exception as e:
            rospy.logerr(f"CmdVelAdapter initialization error: {e}")
            return False
    
    def _do_shutdown(self) -> None:
        """Shutdown and cleanup"""
        if self._shutting_down.is_set():
            return
        self._shutting_down.set()
        rospy.loginfo("CmdVelAdapter shutting down...")
        
        if self._timer is not None:
            self._timer.shutdown()
            self._timer = None
        
        try:
            if self._pub is not None:
                self._pub.publish(Twist())
        except Exception as e:
            rospy.logwarn(f"Failed to publish zero velocity: {e}")
        
        for sub in [self._sub, self._joy_sub, self._mode_sub]:
            if sub is not None:
                sub.unregister()
        self._sub = self._joy_sub = self._mode_sub = None
    
    def _do_reset(self) -> None:
        """Reset internal state"""
        with self._lock:
            self._last_linear_x = 0.0
            self._last_angular_z = 0.0
            self._last_time = None
            self._joystick_mode = False
            self._latest_controller_cmd = Twist()
            self._latest_joy_cmd = Twist()
            self._controller_cmd_time = None
            self._joy_cmd_time = None
            self._published_cmd_count = 0
            self._velocity_clamped_count = 0
            self._rate_limited_cmd_count = 0
            self._timeout_count = 0
        rospy.loginfo("CmdVelAdapter state reset")
    
    def _get_health_details(self) -> Dict[str, Any]:
        """Get health details"""
        with self._lock:
            return {
                'joystick_mode': self._joystick_mode,
                'published_count': self._published_cmd_count,
                'timeout_count': self._timeout_count,
                'has_controller_cmd': self._controller_cmd_time is not None,
                'has_joy_cmd': self._joy_cmd_time is not None,
            }
    
    def _reset_service_callback(self, req):
        self.reset()
        return EmptyResponse()
    
    def _mode_callback(self, msg: Bool):
        with self._lock:
            if msg.data != self._joystick_mode:
                self._joystick_mode = msg.data
                mode = "Joystick" if self._joystick_mode else "Controller"
                rospy.loginfo(f"Control mode: {mode}")
    
    def _joy_callback(self, msg: Twist):
        with self._lock:
            self._latest_joy_cmd = msg
            self._joy_cmd_time = rospy.Time.now()
    
    def _controller_callback(self, msg: UnifiedCmd):
        with self._lock:
            twist = Twist()
            twist.linear.x = msg.vx
            twist.angular.z = msg.omega
            self._latest_controller_cmd = twist
            self._controller_cmd_time = rospy.Time.now()
    
    def _timer_callback(self, event):
        """Timer callback: publish at fixed rate with timeout protection"""
        if self._shutting_down.is_set():
            return
        
        try:
            linear_x = 0.0
            angular_z = 0.0
            is_timeout = False
            
            with self._lock:
                now = rospy.Time.now()
                if self._joystick_mode:
                    if self._joy_cmd_time is not None:
                        if (now - self._joy_cmd_time).to_sec() <= self._cmd_timeout:
                            linear_x = self._latest_joy_cmd.linear.x
                            angular_z = self._latest_joy_cmd.angular.z
                        else:
                            is_timeout = True
                    else:
                        is_timeout = True
                else:
                    # Controller mode with timeout protection
                    if self._controller_cmd_time is not None:
                        if (now - self._controller_cmd_time).to_sec() <= self._cmd_timeout:
                            linear_x = self._latest_controller_cmd.linear.x
                            angular_z = self._latest_controller_cmd.angular.z
                        else:
                            is_timeout = True
                    else:
                        is_timeout = True
            
            if is_timeout:
                self._timeout_count += 1
                if self._timeout_count % 100 == 1:
                    mode = "joystick" if self._joystick_mode else "controller"
                    rospy.logwarn_throttle(5.0, f"Command timeout ({mode} mode)")
            
            self._publish_twist(linear_x, angular_z)
        except Exception as e:
            rospy.logerr_throttle(1.0, f"Timer callback error: {e}")
            try:
                if self._pub is not None:
                    self._pub.publish(Twist())
            except Exception:
                pass

    def _publish_twist(self, linear_x: float, angular_z: float):
        """Publish Twist with safety checks"""
        if self._shutting_down.is_set() or self._pub is None:
            return
        
        twist = Twist()
        original_linear = linear_x
        original_angular = angular_z
        
        if not (math.isfinite(linear_x) and math.isfinite(angular_z)):
            rospy.logwarn_throttle(1.0, f"Invalid velocity: vx={linear_x}, omega={angular_z}")
            self._pub.publish(twist)
            with self._lock:
                self._last_linear_x = 0.0
                self._last_angular_z = 0.0
            return
        
        if self._enable_rate_limit:
            linear_x, angular_z = self._apply_rate_limit(linear_x, angular_z)
        
        linear_x = self._clamp(linear_x, self._v_min, self._v_max)
        angular_z = self._clamp(angular_z, -self._omega_max, self._omega_max)
        
        if abs(linear_x - original_linear) > 0.001 or abs(angular_z - original_angular) > 0.001:
            self._velocity_clamped_count += 1
            if self._velocity_clamped_count % 100 == 1:
                rospy.logwarn(f"Velocity clamped: vx {original_linear:.2f}->{linear_x:.2f}")
        
        with self._lock:
            self._last_linear_x = linear_x
            self._last_angular_z = angular_z
        
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self._pub.publish(twist)
        self._published_cmd_count += 1
    
    def _apply_rate_limit(self, linear_x: float, angular_z: float) -> tuple:
        """Apply velocity rate limiting"""
        now = rospy.Time.now()
        with self._lock:
            if self._last_time is None:
                self._last_time = now
                return linear_x, angular_z
            
            dt = (now - self._last_time).to_sec()
            self._last_time = now
            if dt <= 0:
                return linear_x, angular_z
            
            last_linear = self._last_linear_x
            last_angular = self._last_angular_z
        
        rate_limited = False
        if self._a_max > 0:
            max_delta = self._a_max * dt
            delta = linear_x - last_linear
            if abs(delta) > max_delta:
                linear_x = last_linear + max_delta * (1 if delta > 0 else -1)
                rate_limited = True
        
        if self._alpha_max > 0:
            max_delta = self._alpha_max * dt
            delta = angular_z - last_angular
            if abs(delta) > max_delta:
                angular_z = last_angular + max_delta * (1 if delta > 0 else -1)
                rate_limited = True
        
        if rate_limited:
            self._rate_limited_cmd_count += 1
        return linear_x, angular_z
    
    def _clamp(self, value: float, min_val: float, max_val: float) -> float:
        return max(min_val, min(max_val, value))
    
    def run(self):
        rospy.loginfo("CmdVelAdapter running...")
        rospy.spin()


def main():
    try:
        adapter = CmdVelAdapter()
        adapter.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"CmdVelAdapter exception: {e}")
        raise


if __name__ == '__main__':
    main()
