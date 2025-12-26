#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Full System Diagnostics and Auto-Tuning Tool

Comprehensive diagnostics for:
  1. Chassis characteristics (max velocity, acceleration, response time)
  2. Sensor rates and latency (odometry, IMU, trajectory)
  3. Trajectory quality (points, dt, velocity info, jitter)
  4. Network latency and timing
  5. TF tree and coordinate frames
  6. System load and MPC solver performance

Usage:
  rosrun controller_ros full_diagnostics.py
  rosrun controller_ros full_diagnostics.py --output optimized.yaml
  rosrun controller_ros full_diagnostics.py --test-chassis  # Test chassis limits
"""

import rospy
import sys
import time
import yaml
import argparse
import numpy as np
from collections import deque
from datetime import datetime
from threading import Lock, Thread
import subprocess

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

try:
    from controller_ros.msg import LocalTrajectoryV4, DiagnosticsV2
    HAS_CUSTOM_MSG = True
except ImportError:
    HAS_CUSTOM_MSG = False


class Colors:
    RED = '\033[0;31m'
    GREEN = '\033[0;32m'
    YELLOW = '\033[1;33m'
    BLUE = '\033[0;34m'
    CYAN = '\033[0;36m'
    MAGENTA = '\033[0;35m'
    NC = '\033[0m'


class ControllerDiagnosticsMonitor:
    """Monitor controller diagnostics for runtime tuning"""
    
    def __init__(self, topic='/controller/diagnostics'):
        self.topic = topic
        self.sub = None
        self.lock = Lock()
        self.msg_count = 0
        
        # MPC statistics
        self.mpc_solve_times = deque(maxlen=500)
        self.mpc_successes = deque(maxlen=500)
        self.mpc_kkt_residuals = deque(maxlen=500)
        
        # Tracking errors
        self.lateral_errors = deque(maxlen=500)
        self.longitudinal_errors = deque(maxlen=500)
        self.heading_errors = deque(maxlen=500)
        
        # Consistency
        self.alpha_values = deque(maxlen=500)
        
        # States
        self.states = deque(maxlen=500)
        self.backup_active_count = 0
        
    def start(self):
        if not HAS_CUSTOM_MSG:
            return False
        try:
            self.sub = rospy.Subscriber(self.topic, DiagnosticsV2, self._cb, queue_size=10)
            return True
        except:
            return False
    
    def stop(self):
        if self.sub:
            self.sub.unregister()
    
    def _cb(self, msg):
        with self.lock:
            self.msg_count += 1
            
            # MPC stats
            self.mpc_solve_times.append(msg.mpc_solve_time_ms)
            self.mpc_successes.append(msg.mpc_success)
            self.mpc_kkt_residuals.append(msg.mpc_health_kkt_residual)
            
            # Tracking errors
            self.lateral_errors.append(abs(msg.tracking_lateral_error))
            self.longitudinal_errors.append(abs(msg.tracking_longitudinal_error))
            self.heading_errors.append(abs(msg.tracking_heading_error))
            
            # Consistency
            self.alpha_values.append(msg.consistency_alpha_soft)
            
            # States
            self.states.append(msg.state)
            if msg.backup_active:
                self.backup_active_count += 1
    
    def get_stats(self):
        with self.lock:
            if self.msg_count < 10:
                return None
            
            mpc_times = list(self.mpc_solve_times)
            mpc_success_list = list(self.mpc_successes)
            
            stats = {
                'msg_count': self.msg_count,
                # MPC performance
                'mpc_solve_time_avg_ms': np.mean(mpc_times) if mpc_times else 0,
                'mpc_solve_time_max_ms': np.max(mpc_times) if mpc_times else 0,
                'mpc_solve_time_std_ms': np.std(mpc_times) if mpc_times else 0,
                'mpc_success_rate': np.mean(mpc_success_list) if mpc_success_list else 0,
                'mpc_kkt_residual_avg': np.mean(list(self.mpc_kkt_residuals)) if self.mpc_kkt_residuals else 0,
                # Tracking errors
                'lateral_error_avg': np.mean(list(self.lateral_errors)) if self.lateral_errors else 0,
                'lateral_error_max': np.max(list(self.lateral_errors)) if self.lateral_errors else 0,
                'longitudinal_error_avg': np.mean(list(self.longitudinal_errors)) if self.longitudinal_errors else 0,
                'heading_error_avg': np.mean(list(self.heading_errors)) if self.heading_errors else 0,
                # Consistency
                'alpha_avg': np.mean(list(self.alpha_values)) if self.alpha_values else 0,
                'alpha_min': np.min(list(self.alpha_values)) if self.alpha_values else 0,
                # Backup usage
                'backup_active_ratio': self.backup_active_count / self.msg_count if self.msg_count > 0 else 0,
            }
            
            return stats


class TopicMonitor:
    """Monitor a single topic for rate and latency"""
    
    def __init__(self, topic, msg_type, window_size=200):
        self.topic = topic
        self.msg_type = msg_type
        self.timestamps = deque(maxlen=window_size)
        self.latencies = deque(maxlen=window_size)
        self.msg_count = 0
        self.last_msg = None
        self.lock = Lock()
        self.sub = None
        
    def start(self):
        try:
            self.sub = rospy.Subscriber(self.topic, self.msg_type, self._cb, queue_size=10)
            return True
        except:
            return False
    
    def stop(self):
        if self.sub:
            self.sub.unregister()
    
    def _cb(self, msg):
        now = time.time()
        with self.lock:
            self.timestamps.append(now)
            self.last_msg = msg
            self.msg_count += 1
            
            # Calculate latency if message has header
            if hasattr(msg, 'header') and msg.header.stamp.to_sec() > 0:
                msg_time = msg.header.stamp.to_sec()
                latency = now - msg_time
                if latency > 0 and latency < 10:  # Sanity check
                    self.latencies.append(latency * 1000)  # ms
    
    def get_stats(self):
        with self.lock:
            if len(self.timestamps) < 2:
                return {'rate': 0, 'latency_ms': 0, 'jitter_ms': 0, 'count': self.msg_count}
            
            # Calculate rate
            dts = np.diff(list(self.timestamps))
            rate = 1.0 / np.mean(dts) if len(dts) > 0 and np.mean(dts) > 0 else 0
            jitter = np.std(dts) * 1000 if len(dts) > 1 else 0
            
            # Calculate latency
            latency = np.mean(list(self.latencies)) if self.latencies else 0
            
            return {
                'rate': rate,
                'latency_ms': latency,
                'jitter_ms': jitter,
                'count': self.msg_count
            }


class OdometryAnalyzer(TopicMonitor):
    """Analyze odometry for chassis characteristics"""
    
    def __init__(self, topic):
        super().__init__(topic, Odometry)
        self.velocities = deque(maxlen=500)
        self.accelerations = deque(maxlen=500)
        self.last_vel = None
        self.last_time = None
        
    def _cb(self, msg):
        super()._cb(msg)
        now = time.time()
        
        with self.lock:
            vx = msg.twist.twist.linear.x
            vy = msg.twist.twist.linear.y
            wz = msg.twist.twist.angular.z
            v = np.sqrt(vx**2 + vy**2)
            
            self.velocities.append((now, vx, vy, wz, v))
            
            # Calculate acceleration
            if self.last_vel is not None and self.last_time is not None:
                dt = now - self.last_time
                if dt > 0.001:
                    ax = (vx - self.last_vel[0]) / dt
                    ay = (vy - self.last_vel[1]) / dt
                    alpha = (wz - self.last_vel[2]) / dt
                    self.accelerations.append((now, ax, ay, alpha))
            
            self.last_vel = (vx, vy, wz)
            self.last_time = now
    
    def get_chassis_stats(self):
        with self.lock:
            if len(self.velocities) < 10:
                return None
            
            vels = np.array([(v[1], v[2], v[3], v[4]) for v in self.velocities])
            
            stats = {
                'max_vx': float(np.max(np.abs(vels[:, 0]))),
                'max_vy': float(np.max(np.abs(vels[:, 1]))),
                'max_wz': float(np.max(np.abs(vels[:, 2]))),
                'max_speed': float(np.max(vels[:, 3])),
                'avg_speed': float(np.mean(vels[:, 3])),
            }
            
            if len(self.accelerations) > 10:
                accels = np.array([(a[1], a[2], a[3]) for a in self.accelerations])
                stats['max_ax'] = float(np.max(np.abs(accels[:, 0])))
                stats['max_ay'] = float(np.max(np.abs(accels[:, 1])))
                stats['max_alpha'] = float(np.max(np.abs(accels[:, 2])))
                stats['avg_ax'] = float(np.mean(np.abs(accels[:, 0])))
            
            return stats


class TrajectoryAnalyzer(TopicMonitor):
    """Analyze trajectory characteristics"""
    
    def __init__(self, topic):
        # Use LocalTrajectoryV4 if available for proper message parsing
        if HAS_CUSTOM_MSG:
            super().__init__(topic, LocalTrajectoryV4)
        else:
            # Fallback to AnyMsg (won't parse fields properly)
            from rospy.msg import AnyMsg
            super().__init__(topic, AnyMsg)
            rospy.logwarn("LocalTrajectoryV4 not available, trajectory analysis will be limited")
        self.traj_info = {
            'num_points': 0,
            'dt_sec': 0.1,
            'has_velocities': False,
            'confidence': 0.9,
            'frame_id': '',
            'total_length': 0,
            'max_curvature': 0,
        }
        self.point_counts = deque(maxlen=100)
        self.dt_values = deque(maxlen=100)
        
    def _cb(self, msg):
        super()._cb(msg)
        
        try:
            # Try to parse trajectory info
            if hasattr(msg, 'points'):
                self.traj_info['num_points'] = len(msg.points)
                self.point_counts.append(len(msg.points))
            if hasattr(msg, 'dt_sec'):
                self.traj_info['dt_sec'] = msg.dt_sec
                self.dt_values.append(msg.dt_sec)
            if hasattr(msg, 'velocities_flat'):
                self.traj_info['has_velocities'] = len(msg.velocities_flat) > 0
            if hasattr(msg, 'confidence'):
                self.traj_info['confidence'] = msg.confidence
            if hasattr(msg, 'header'):
                self.traj_info['frame_id'] = msg.header.frame_id
                
            # Calculate trajectory length and curvature
            if hasattr(msg, 'points') and len(msg.points) >= 2:
                points = [(p.x, p.y) for p in msg.points]
                length = sum(np.sqrt((points[i+1][0]-points[i][0])**2 + 
                                    (points[i+1][1]-points[i][1])**2) 
                            for i in range(len(points)-1))
                self.traj_info['total_length'] = length
                
                # Estimate max curvature
                if len(points) >= 3:
                    curvatures = []
                    for i in range(1, len(points)-1):
                        p0, p1, p2 = points[i-1], points[i], points[i+1]
                        # Menger curvature
                        area = abs((p1[0]-p0[0])*(p2[1]-p0[1]) - (p2[0]-p0[0])*(p1[1]-p0[1])) / 2
                        d01 = np.sqrt((p1[0]-p0[0])**2 + (p1[1]-p0[1])**2)
                        d12 = np.sqrt((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2)
                        d02 = np.sqrt((p2[0]-p0[0])**2 + (p2[1]-p0[1])**2)
                        if d01 * d12 * d02 > 1e-6:
                            kappa = 4 * area / (d01 * d12 * d02)
                            curvatures.append(kappa)
                    if curvatures:
                        self.traj_info['max_curvature'] = max(curvatures)
        except:
            pass
    
    def get_trajectory_stats(self):
        stats = self.traj_info.copy()
        
        with self.lock:
            if self.point_counts:
                stats['avg_points'] = np.mean(list(self.point_counts))
                stats['point_variance'] = np.std(list(self.point_counts))
            if self.dt_values:
                stats['avg_dt'] = np.mean(list(self.dt_values))
                stats['dt_variance'] = np.std(list(self.dt_values))
        
        return stats


class ChassisTestRunner:
    """Run chassis capability tests"""
    
    def __init__(self, cmd_topic, odom_analyzer):
        self.cmd_topic = cmd_topic
        self.odom = odom_analyzer
        self.cmd_pub = None
        self.results = {}
        
    def setup(self):
        self.cmd_pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=1)
        time.sleep(0.5)
        
    def test_max_velocity(self, target_v=1.0, duration=3.0):
        """Test maximum achievable velocity"""
        print(f"  Testing max velocity (target: {target_v} m/s)...")
        
        cmd = Twist()
        cmd.linear.x = target_v
        
        start = time.time()
        max_v = 0
        
        while time.time() - start < duration:
            self.cmd_pub.publish(cmd)
            stats = self.odom.get_chassis_stats()
            if stats:
                max_v = max(max_v, stats['max_speed'])
            time.sleep(0.05)
        
        # Stop
        self.cmd_pub.publish(Twist())
        time.sleep(0.5)
        
        self.results['max_velocity_achieved'] = max_v
        return max_v
    
    def test_acceleration(self, target_v=0.5):
        """Test acceleration capability"""
        print(f"  Testing acceleration (target: {target_v} m/s)...")
        
        # Start from stop
        self.cmd_pub.publish(Twist())
        time.sleep(1.0)
        
        # Clear old data
        self.odom.accelerations.clear()
        
        # Accelerate
        cmd = Twist()
        cmd.linear.x = target_v
        
        start = time.time()
        while time.time() - start < 2.0:
            self.cmd_pub.publish(cmd)
            time.sleep(0.02)
        
        # Stop
        self.cmd_pub.publish(Twist())
        time.sleep(0.5)
        
        stats = self.odom.get_chassis_stats()
        if stats and 'max_ax' in stats:
            self.results['max_acceleration'] = stats['max_ax']
            return stats['max_ax']
        return 0
    
    def test_angular_velocity(self, target_w=1.0, duration=2.0):
        """Test maximum angular velocity"""
        print(f"  Testing angular velocity (target: {target_w} rad/s)...")
        
        cmd = Twist()
        cmd.angular.z = target_w
        
        start = time.time()
        max_w = 0
        
        while time.time() - start < duration:
            self.cmd_pub.publish(cmd)
            stats = self.odom.get_chassis_stats()
            if stats:
                max_w = max(max_w, stats['max_wz'])
            time.sleep(0.05)
        
        self.cmd_pub.publish(Twist())
        time.sleep(0.5)
        
        self.results['max_angular_velocity'] = max_w
        return max_w
    
    def test_response_time(self, step_v=0.3):
        """Test chassis response time"""
        print(f"  Testing response time...")
        
        # Start from stop
        self.cmd_pub.publish(Twist())
        time.sleep(1.0)
        
        # Record start
        start_time = time.time()
        threshold = step_v * 0.63  # 63% of target (time constant)
        
        cmd = Twist()
        cmd.linear.x = step_v
        
        response_time = None
        while time.time() - start_time < 3.0:
            self.cmd_pub.publish(cmd)
            stats = self.odom.get_chassis_stats()
            if stats and stats['max_speed'] >= threshold and response_time is None:
                response_time = time.time() - start_time
            time.sleep(0.01)
        
        self.cmd_pub.publish(Twist())
        time.sleep(0.5)
        
        self.results['response_time'] = response_time or 1.0
        return response_time


class FullDiagnostics:
    """Complete system diagnostics"""
    
    def __init__(self, args):
        self.args = args
        self.monitors = {}
        self.results = {}
        self.recommended = {}
        
        # Topic configuration
        self.topics = {
            'odom': args.odom_topic,
            'imu': args.imu_topic,
            'trajectory': args.traj_topic,
            'cmd_vel': args.cmd_vel_topic,
            'diagnostics': getattr(args, 'diag_topic', '/controller/diagnostics'),
        }
    
    def run(self):
        print(f"\n{Colors.GREEN}{'='*70}")
        print("  Full System Diagnostics and Auto-Tuning Tool")
        print(f"{'='*70}{Colors.NC}\n")
        
        rospy.init_node('full_diagnostics', anonymous=True)
        
        # Phase 1: Topic monitoring
        self._run_topic_monitoring()
        
        # Phase 2: Chassis testing (optional)
        if self.args.test_chassis:
            self._run_chassis_tests()
        
        # Phase 3: Controller runtime diagnostics (if controller is running)
        if getattr(self.args, 'runtime_tuning', False):
            self._run_controller_diagnostics()
        
        # Phase 4: Calculate recommendations
        self._calculate_all_recommendations()
        
        # Phase 5: Show results
        self._show_results()
        
        # Phase 6: Generate config
        if self.args.output:
            self._generate_full_config(self.args.output)
    
    def _run_topic_monitoring(self):
        print(f"{Colors.BLUE}Phase 1: Topic Monitoring ({self.args.duration}s){Colors.NC}\n")
        
        # Setup monitors
        self.monitors['odom'] = OdometryAnalyzer(self.topics['odom'])
        self.monitors['imu'] = TopicMonitor(self.topics['imu'], Imu)
        self.monitors['trajectory'] = TrajectoryAnalyzer(self.topics['trajectory'])
        
        # Start all monitors
        for name, mon in self.monitors.items():
            if mon.start():
                print(f"  {Colors.GREEN}[OK]{Colors.NC} Subscribed to {mon.topic}")
            else:
                print(f"  {Colors.RED}[FAIL]{Colors.NC} Cannot subscribe to {mon.topic}")
        
        # Wait for data
        print(f"\n  Collecting data for {self.args.duration} seconds...")
        time.sleep(self.args.duration)
        
        # Collect results
        for name, mon in self.monitors.items():
            self.results[name] = mon.get_stats()
            if isinstance(mon, OdometryAnalyzer):
                chassis = mon.get_chassis_stats()
                if chassis:
                    self.results['chassis'] = chassis
            elif isinstance(mon, TrajectoryAnalyzer):
                self.results['trajectory_info'] = mon.get_trajectory_stats()
        
        # Stop monitors
        for mon in self.monitors.values():
            mon.stop()
    
    def _run_controller_diagnostics(self):
        """Run controller runtime diagnostics for MPC tuning"""
        print(f"\n{Colors.BLUE}Phase 3: Controller Runtime Diagnostics ({self.args.duration}s){Colors.NC}\n")
        
        if not HAS_CUSTOM_MSG:
            print(f"  {Colors.YELLOW}[WARN]{Colors.NC} controller_ros messages not available, skipping")
            return
        
        # Setup controller diagnostics monitor
        diag_monitor = ControllerDiagnosticsMonitor(self.topics['diagnostics'])
        
        if diag_monitor.start():
            print(f"  {Colors.GREEN}[OK]{Colors.NC} Subscribed to {self.topics['diagnostics']}")
        else:
            print(f"  {Colors.RED}[FAIL]{Colors.NC} Cannot subscribe to {self.topics['diagnostics']}")
            print(f"  {Colors.YELLOW}[INFO]{Colors.NC} Is the controller running?")
            return
        
        # Wait for data
        print(f"\n  Collecting controller diagnostics for {self.args.duration} seconds...")
        print(f"  {Colors.YELLOW}[INFO]{Colors.NC} Move the robot to generate tracking data!")
        time.sleep(self.args.duration)
        
        # Collect results
        controller_stats = diag_monitor.get_stats()
        diag_monitor.stop()
        
        if controller_stats:
            self.results['controller'] = controller_stats
            print(f"  {Colors.GREEN}[OK]{Colors.NC} Collected {controller_stats['msg_count']} diagnostics messages")
        else:
            print(f"  {Colors.YELLOW}[WARN]{Colors.NC} No controller diagnostics data received")
    
    def _run_chassis_tests(self):
        print(f"\n{Colors.BLUE}Phase 2: Chassis Capability Tests{Colors.NC}\n")
        print(f"  {Colors.YELLOW}WARNING: Robot will move! Ensure clear space.{Colors.NC}")
        
        input("  Press Enter to start chassis tests (Ctrl+C to skip)...")
        
        tester = ChassisTestRunner(self.topics['cmd_vel'], self.monitors['odom'])
        tester.setup()
        
        # Run tests
        tester.test_max_velocity(target_v=0.5)
        tester.test_acceleration(target_v=0.3)
        tester.test_angular_velocity(target_w=1.0)
        tester.test_response_time(step_v=0.3)
        
        self.results['chassis_tests'] = tester.results
    
    def _calculate_all_recommendations(self):
        print(f"\n{Colors.BLUE}Phase 3: Calculating Recommendations{Colors.NC}\n")
        
        odom = self.results.get('odom', {})
        traj = self.results.get('trajectory', {})
        traj_info = self.results.get('trajectory_info', {})
        imu = self.results.get('imu', {})
        chassis = self.results.get('chassis', {})
        chassis_tests = self.results.get('chassis_tests', {})
        
        odom_rate = odom.get('rate', 0)
        traj_rate = traj.get('rate', 0)
        imu_rate = imu.get('rate', 0)
        
        # ===== System Configuration =====
        # Control frequency: based on odom rate
        if odom_rate >= 100:
            ctrl_freq = 50
        elif odom_rate >= 50:
            ctrl_freq = 40
        elif odom_rate >= 20:
            ctrl_freq = 20
        else:
            ctrl_freq = max(10, int(odom_rate / 2))
        
        self.recommended['system'] = {
            'ctrl_freq': ctrl_freq,
            'platform': 'differential',
        }
        
        # ===== Watchdog Configuration =====
        self.recommended['watchdog'] = {
            'odom_timeout_ms': int(3000 / max(odom_rate, 1)) if odom_rate > 0 else 500,
            'traj_timeout_ms': int(2000 / max(traj_rate, 1)) if traj_rate > 0 else 1000,
            'traj_grace_ms': int(1000 / max(traj_rate, 1)) if traj_rate > 0 else 500,
            'imu_timeout_ms': int(3000 / max(imu_rate, 1)) if imu_rate > 0 else -1,
            'startup_grace_ms': 5000,
        }
        
        # ===== MPC Configuration =====
        num_points = traj_info.get('num_points', 8)
        dt_sec = traj_info.get('dt_sec', 0.1)
        
        mpc_horizon = min(max(num_points - 1, 3), 30)
        mpc_dt = dt_sec if dt_sec > 0 else 0.1
        
        # MPC timing based on control frequency
        ctrl_period_ms = 1000 / ctrl_freq
        
        self.recommended['mpc'] = {
            'horizon': mpc_horizon,
            'horizon_degraded': max(mpc_horizon // 2, 3),
            'dt': mpc_dt,
            'weights': {
                'position': 10.0,
                'velocity': 1.0,
                'heading': 5.0,
                'control_accel': 0.2,
                'control_alpha': 0.2,
            },
            'solver': {
                'nlp_max_iter': 50,
                'qp_solver': 'PARTIAL_CONDENSING_HPIPM',
                'integrator_type': 'ERK',
                'nlp_solver_type': 'SQP_RTI',
            },
            'health_monitor': {
                'time_warning_thresh_ms': int(ctrl_period_ms * 0.4),
                'time_critical_thresh_ms': int(ctrl_period_ms * 0.8),
                'time_recovery_thresh_ms': int(ctrl_period_ms * 0.3),
                'condition_number_thresh': 1e8,
                'condition_number_recovery': 1e5,
                'kkt_residual_thresh': 1e-3,
                'consecutive_warning_limit': 10,
                'consecutive_recovery_limit': 5,
                'recovery_multiplier': 2.0,
                'consecutive_good_for_decay': 2,
                'timeout_decay_rate': 2,
            },
            'fallback': {
                'lookahead_steps': 3,
                'heading_kp': 1.5,
                'max_curvature': 5.0,
                'min_distance_thresh': 0.1,
                'min_turn_speed': 0.1,
                'default_speed_ratio': 0.5,
            },
        }

        # ===== Constraints Configuration =====
        # Use measured chassis capabilities or defaults
        max_v = chassis_tests.get('max_velocity_achieved', chassis.get('max_speed', 0.5))
        max_a = chassis_tests.get('max_acceleration', chassis.get('max_ax', 0.5))
        max_w = chassis_tests.get('max_angular_velocity', chassis.get('max_wz', 1.0))
        max_alpha = chassis.get('max_alpha', 1.0)
        
        # Apply safety margin (80% of measured max)
        safety_margin = 0.8
        
        self.recommended['constraints'] = {
            'v_max': round(max_v * safety_margin, 2) if max_v > 0 else 0.5,
            'v_min': -0.2,
            'omega_max': round(max_w * safety_margin, 2) if max_w > 0 else 1.0,
            'omega_max_low': round(max_w * safety_margin * 0.5, 2) if max_w > 0 else 0.5,
            'a_max': round(max_a * safety_margin, 2) if max_a > 0 else 0.5,
            'alpha_max': round(max_alpha * safety_margin, 2) if max_alpha > 0 else 1.0,
            'v_low_thresh': 0.1,
        }
        
        # ===== Safety Configuration =====
        response_time = chassis_tests.get('response_time', 0.2)
        
        self.recommended['safety'] = {
            'v_stop_thresh': 0.05,
            'vz_stop_thresh': 0.1,
            'stopping_timeout': 5.0,
            'emergency_decel': round(max_a * 1.5, 2) if max_a > 0 else 1.5,
            'low_speed': {
                'threshold': 0.1,
                'omega_limit': round(max_w * 0.5, 2) if max_w > 0 else 0.5,
            },
            'margins': {
                'velocity': 1.1,
                'acceleration': 1.5,
            },
            'accel_filter_window': 3,
            'accel_filter_alpha': 0.3,
            'accel_filter_warmup_alpha': 0.5,
            'accel_filter_warmup_period': 3,
            'min_dt_for_accel': 0.001,
            'max_dt_for_accel': 1.0,
            'state_machine': {
                'alpha_disable_thresh': 0.1 if traj_info.get('has_velocities') else 0.0,
                'alpha_recovery_value': 0.3 if traj_info.get('has_velocities') else 0.0,
                'alpha_recovery_thresh': 5,
                'mpc_recovery_thresh': 5,
                'mpc_fail_window_size': 10,
                'mpc_fail_thresh': 3,
                'mpc_fail_ratio_thresh': 0.5,
                'mpc_recovery_history_min': 3,
                'mpc_recovery_recent_count': 5,
                'mpc_recovery_tolerance': 0,
                'mpc_recovery_success_ratio': 0.8,
            },
        }
        
        # ===== EKF Configuration =====
        # Adjust noise based on sensor quality
        odom_jitter = odom.get('jitter_ms', 10)
        odom_latency = odom.get('latency_ms', 0)
        
        # Higher jitter = higher noise
        odom_noise_factor = 1.0 + (odom_jitter / 50.0)
        
        self.recommended['ekf'] = {
            'use_odom_orientation_fallback': imu_rate == 0,
            'imu_motion_compensation': imu_rate > 0,
            'process_noise': {
                'position': round(0.01 * odom_noise_factor, 4),
                'velocity': round(0.1 * odom_noise_factor, 3),
                'orientation': round(0.05 * odom_noise_factor, 4),
                'angular_velocity': 0.1,
            },
            'measurement_noise': {
                'odom_position': round(0.01 * odom_noise_factor, 4),
                'odom_velocity': round(0.1 * odom_noise_factor, 3),
            },
        }
        
        # ===== Consistency Configuration =====
        traj_jitter = traj.get('jitter_ms', 0)
        
        self.recommended['consistency'] = {
            'kappa_thresh': 0.5,
            'v_dir_thresh': 0.8,
            'temporal_smooth_thresh': 0.5 if traj_jitter < 50 else 0.3,
            'alpha_min': 0.1,
            'max_curvature': 10.0,
            'temporal_window_size': 10,
            'weights': {
                'kappa': 0.3,
                'velocity': 0.3,
                'temporal': 0.4,
            },
        }
        
        # ===== Backup Controller Configuration =====
        # Lookahead based on velocity and response time
        lookahead = max(0.3, min(max_v * response_time * 2, 2.0)) if max_v > 0 else 0.5
        
        self.recommended['backup'] = {
            'lookahead_dist': round(lookahead, 2),
            'min_lookahead': round(lookahead * 0.6, 2),
            'max_lookahead': round(lookahead * 3, 2),
            'lookahead_ratio': 0.5,
            'kp_z': 1.0,
            'kp_heading': 1.5,
            'heading_mode': 'follow_velocity',
            'dt': round(1.0 / ctrl_freq, 3),
            'heading_error_thresh': 1.047,
            'pure_pursuit_angle_thresh': 1.047,
            'heading_control_angle_thresh': 1.571,
            'max_curvature': 5.0,
            'min_turn_speed': 0.1,
            'default_speed_ratio': 0.5,
            'low_speed_transition_factor': 0.5,
            'curvature_speed_limit_thresh': 0.1,
            'min_distance_thresh': 0.1,
            'omega_rate_limit': None,
        }
        
        # ===== Transform Configuration =====
        self.recommended['transform'] = {
            'target_frame': 'odom',
            'source_frame': traj_info.get('frame_id', 'base_link') or 'base_link',
            'timeout_ms': max(50, int(odom_latency * 2 + 20)),
            'fallback_duration_limit_ms': 500,
            'fallback_critical_limit_ms': 1000,
            'drift_estimation_enabled': False,
            'recovery_correction_enabled': True,
            'drift_rate': 0.01,
            'drift_velocity_factor': 0.1,
            'max_drift_dt': 0.5,
            'drift_correction_thresh': 0.01,
            'expected_source_frames': ['base_link', 'base_footprint', 'base_link_0', '', 'odom'],
            'warn_unexpected_frame': True,
        }
        
        # ===== Transition Configuration =====
        ctrl_period = 1.0 / ctrl_freq
        self.recommended['transition'] = {
            'type': 'exponential',
            'tau': round(ctrl_period * 2, 3),
            'max_duration': 0.5,
            'completion_threshold': 0.95,
            'duration': 0.2,
        }
        
        # ===== TF Configuration =====
        self.recommended['tf'] = {
            'source_frame': traj_info.get('frame_id', 'base_link') or 'base_link',
            'target_frame': 'odom',
            'timeout_ms': max(50, int(odom_latency * 2 + 20)),
            'buffer_warmup_timeout_sec': 5.0,
            'buffer_warmup_interval_sec': 0.2,
            'expected_source_frames': ['base_link', 'base_footprint', ''],
        }
        
        # ===== Tracking Quality Configuration =====
        # Based on trajectory characteristics
        traj_length = traj_info.get('total_length', 1.0)
        
        self.recommended['tracking'] = {
            'lateral_thresh': min(0.3, traj_length * 0.1),
            'longitudinal_thresh': min(0.5, traj_length * 0.15),
            'heading_thresh': 0.5,
            'prediction_thresh': 0.5,
            'weights': {
                'lateral': 0.4,
                'longitudinal': 0.4,
                'heading': 0.2,
            },
            'rating': {
                'excellent': 90,
                'good': 70,
                'fair': 50,
            },
        }
        
        # ===== cmd_vel Adapter Configuration =====
        self.recommended['cmd_vel_adapter'] = {
            'publish_rate': float(ctrl_freq),
            'joy_timeout': 0.5,
            'max_linear': self.recommended['constraints']['v_max'],
            'max_angular': self.recommended['constraints']['omega_max'],
            'max_linear_accel': 0.0,
            'max_angular_accel': 0.0,
            'input_topic': '/cmd_unified',
            'joy_topic': '/joy_cmd_vel',
            'mode_topic': '/visualizer/control_mode',
            'output_topic': self.topics['cmd_vel'],
        }

    def _show_results(self):
        print(f"\n{Colors.BLUE}{'='*70}")
        print("  Diagnostics Results")
        print(f"{'='*70}{Colors.NC}\n")
        
        # Sensor Status
        print(f"{Colors.CYAN}Sensor Status:{Colors.NC}")
        for name in ['odom', 'imu', 'trajectory']:
            stats = self.results.get(name, {})
            rate = stats.get('rate', 0)
            latency = stats.get('latency_ms', 0)
            jitter = stats.get('jitter_ms', 0)
            status = f"{Colors.GREEN}[OK]{Colors.NC}" if rate > 0 else f"{Colors.RED}[NO DATA]{Colors.NC}"
            print(f"  {status} {name}: {rate:.1f} Hz, latency: {latency:.1f}ms, jitter: {jitter:.1f}ms")
        
        # Trajectory Info
        traj_info = self.results.get('trajectory_info', {})
        if traj_info:
            print(f"\n{Colors.CYAN}Trajectory Characteristics:{Colors.NC}")
            print(f"  Points: {traj_info.get('num_points', 'N/A')}")
            print(f"  Time step: {traj_info.get('dt_sec', 'N/A')} sec")
            print(f"  Has velocity: {'Yes' if traj_info.get('has_velocities') else 'No'}")
            print(f"  Frame: {traj_info.get('frame_id', 'N/A')}")
            print(f"  Length: {traj_info.get('total_length', 0):.2f} m")
            print(f"  Max curvature: {traj_info.get('max_curvature', 0):.2f} 1/m")
        
        # Chassis Stats
        chassis = self.results.get('chassis', {})
        if chassis:
            print(f"\n{Colors.CYAN}Chassis Characteristics (from odometry):{Colors.NC}")
            print(f"  Max speed: {chassis.get('max_speed', 0):.2f} m/s")
            print(f"  Max vx: {chassis.get('max_vx', 0):.2f} m/s")
            print(f"  Max wz: {chassis.get('max_wz', 0):.2f} rad/s")
            if 'max_ax' in chassis:
                print(f"  Max accel: {chassis.get('max_ax', 0):.2f} m/s^2")
                print(f"  Max alpha: {chassis.get('max_alpha', 0):.2f} rad/s^2")
        
        # Chassis Test Results
        tests = self.results.get('chassis_tests', {})
        if tests:
            print(f"\n{Colors.CYAN}Chassis Test Results:{Colors.NC}")
            print(f"  Max velocity achieved: {tests.get('max_velocity_achieved', 0):.2f} m/s")
            print(f"  Max acceleration: {tests.get('max_acceleration', 0):.2f} m/s^2")
            print(f"  Max angular velocity: {tests.get('max_angular_velocity', 0):.2f} rad/s")
            print(f"  Response time: {tests.get('response_time', 0):.3f} sec")
        
        # Recommendations
        print(f"\n{Colors.CYAN}Recommended Parameters:{Colors.NC}")
        print(f"  Control frequency: {self.recommended['system']['ctrl_freq']} Hz")
        print(f"  MPC horizon: {self.recommended['mpc']['horizon']}")
        print(f"  MPC dt: {self.recommended['mpc']['dt']} sec")
        print(f"  v_max: {self.recommended['constraints']['v_max']} m/s")
        print(f"  omega_max: {self.recommended['constraints']['omega_max']} rad/s")
        print(f"  a_max: {self.recommended['constraints']['a_max']} m/s^2")
        print(f"  Odom timeout: {self.recommended['watchdog']['odom_timeout_ms']} ms")
        print(f"  Traj timeout: {self.recommended['watchdog']['traj_timeout_ms']} ms")
        print(f"  Lookahead: {self.recommended['backup']['lookahead_dist']} m")
        
        # Warnings
        print(f"\n{Colors.CYAN}Optimization Suggestions:{Colors.NC}")
        
        odom_rate = self.results.get('odom', {}).get('rate', 0)
        traj_rate = self.results.get('trajectory', {}).get('rate', 0)
        
        if odom_rate > 50:
            print(f"  {Colors.GREEN}[OK]{Colors.NC} Odometry rate is good for high-frequency control")
        elif odom_rate > 0:
            print(f"  {Colors.YELLOW}[WARN]{Colors.NC} Odometry rate is low, control frequency limited")
        else:
            print(f"  {Colors.RED}[ERROR]{Colors.NC} No odometry data!")
        
        if traj_rate > 0 and traj_rate < 2:
            print(f"  {Colors.YELLOW}[WARN]{Colors.NC} Trajectory rate is low, increase timeout values")
        
        traj_info = self.results.get('trajectory_info', {})
        if traj_info.get('num_points', 0) < 5:
            print(f"  {Colors.YELLOW}[WARN]{Colors.NC} Few trajectory points, MPC prediction limited")
        
        if not traj_info.get('has_velocities'):
            print(f"  {Colors.YELLOW}[WARN]{Colors.NC} No velocity in trajectory, alpha check disabled")
        
        odom_jitter = self.results.get('odom', {}).get('jitter_ms', 0)
        if odom_jitter > 20:
            print(f"  {Colors.YELLOW}[WARN]{Colors.NC} High odometry jitter ({odom_jitter:.1f}ms), increased EKF noise")
        
        # Controller Runtime Stats
        controller = self.results.get('controller')
        if controller:
            print(f"\n{Colors.CYAN}Controller Runtime Statistics:{Colors.NC}")
            print(f"  MPC solve time: {controller['mpc_solve_time_avg_ms']:.2f}ms avg, {controller['mpc_solve_time_max_ms']:.2f}ms max")
            print(f"  MPC success rate: {controller['mpc_success_rate']*100:.1f}%")
            print(f"  Backup active ratio: {controller['backup_active_ratio']*100:.1f}%")
            print(f"  Lateral error: {controller['lateral_error_avg']*100:.1f}cm avg, {controller['lateral_error_max']*100:.1f}cm max")
            print(f"  Heading error: {np.degrees(controller['heading_error_avg']):.1f}° avg")
            print(f"  Alpha (consistency): {controller['alpha_avg']:.2f} avg, {controller['alpha_min']:.2f} min")
            
            # Runtime tuning suggestions
            print(f"\n{Colors.MAGENTA}Runtime Tuning Suggestions:{Colors.NC}")
            
            ctrl_freq = self.recommended.get('system', {}).get('ctrl_freq', 20)
            ctrl_period_ms = 1000 / ctrl_freq
            
            # MPC timing
            if controller['mpc_solve_time_avg_ms'] > ctrl_period_ms * 0.5:
                print(f"  {Colors.RED}[CRITICAL]{Colors.NC} MPC solve time too high!")
                print(f"    → Reduce mpc.horizon (current recommendation: {self.recommended.get('mpc', {}).get('horizon', 7)})")
                print(f"    → Or reduce system.ctrl_freq")
            elif controller['mpc_solve_time_avg_ms'] > ctrl_period_ms * 0.3:
                print(f"  {Colors.YELLOW}[WARN]{Colors.NC} MPC solve time is high")
                print(f"    → Consider reducing mpc.horizon")
            else:
                print(f"  {Colors.GREEN}[OK]{Colors.NC} MPC solve time is good ({controller['mpc_solve_time_avg_ms']:.1f}ms < {ctrl_period_ms*0.3:.1f}ms)")
            
            # MPC success rate
            if controller['mpc_success_rate'] < 0.9:
                print(f"  {Colors.RED}[CRITICAL]{Colors.NC} MPC success rate too low ({controller['mpc_success_rate']*100:.0f}%)")
                print(f"    → Check trajectory quality")
                print(f"    → Reduce mpc.horizon")
                print(f"    → Increase mpc.solver.nlp_max_iter")
            elif controller['mpc_success_rate'] < 0.98:
                print(f"  {Colors.YELLOW}[WARN]{Colors.NC} MPC success rate could be better")
            else:
                print(f"  {Colors.GREEN}[OK]{Colors.NC} MPC success rate is good ({controller['mpc_success_rate']*100:.0f}%)")
            
            # Backup usage
            if controller['backup_active_ratio'] > 0.1:
                print(f"  {Colors.YELLOW}[WARN]{Colors.NC} Backup controller used frequently ({controller['backup_active_ratio']*100:.0f}%)")
                print(f"    → Check MPC solver health")
                print(f"    → Verify trajectory consistency")
            
            # Tracking error
            if controller['lateral_error_avg'] > 0.1:
                print(f"  {Colors.YELLOW}[WARN]{Colors.NC} High lateral tracking error ({controller['lateral_error_avg']*100:.1f}cm)")
                print(f"    → Increase mpc.weights.position (try 15-20)")
                print(f"    → Decrease mpc.weights.control_accel (try 0.1)")
            else:
                print(f"  {Colors.GREEN}[OK]{Colors.NC} Lateral tracking error is acceptable")
            
            if controller['heading_error_avg'] > 0.3:
                print(f"  {Colors.YELLOW}[WARN]{Colors.NC} High heading error ({np.degrees(controller['heading_error_avg']):.1f}°)")
                print(f"    → Increase mpc.weights.heading (try 8-10)")
            
            # Alpha (consistency)
            if controller['alpha_min'] < 0.3:
                print(f"  {Colors.YELLOW}[WARN]{Colors.NC} Low alpha values detected (min: {controller['alpha_min']:.2f})")
                print(f"    → Trajectory consistency is poor")
                print(f"    → Check network output quality")

    def _generate_full_config(self, output_file):
        print(f"\n{Colors.BLUE}Generating configuration file: {output_file}{Colors.NC}")
        
        # Build complete config
        config = {
            'system': self.recommended['system'],
            'topics': {
                'odom': self.topics['odom'],
                'imu': self.topics['imu'] if self.results.get('imu', {}).get('rate', 0) > 0 else '',
                'trajectory': self.topics['trajectory'],
                'emergency_stop': '/controller/emergency_stop',
                'cmd_unified': '/cmd_unified',
                'diagnostics': '/controller/diagnostics',
                'state': '/controller/state',
            },
            'tf': self.recommended['tf'],
            'watchdog': self.recommended['watchdog'],
            'mpc': self.recommended['mpc'],
            'constraints': self.recommended['constraints'],
            'safety': self.recommended['safety'],
            'consistency': self.recommended['consistency'],
            'ekf': self.recommended['ekf'],
            'transform': self.recommended['transform'],
            'transition': self.recommended['transition'],
            'backup': self.recommended['backup'],
            'tracking': self.recommended['tracking'],
            'cmd_vel_adapter': self.recommended['cmd_vel_adapter'],
            'diagnostics': {'publish_rate': 10},
        }
        
        # Generate header
        odom = self.results.get('odom', {})
        traj = self.results.get('trajectory', {})
        traj_info = self.results.get('trajectory_info', {})
        chassis = self.results.get('chassis', {})
        tests = self.results.get('chassis_tests', {})
        
        header = f"""# ============================================================================
# Auto-generated Optimized Configuration
# Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}
# ============================================================================
#
# Detection Results:
#   Odometry: {odom.get('rate', 0):.1f} Hz, latency {odom.get('latency_ms', 0):.1f}ms, jitter {odom.get('jitter_ms', 0):.1f}ms
#   Trajectory: {traj.get('rate', 0):.1f} Hz, {traj_info.get('num_points', 0)} points, dt={traj_info.get('dt_sec', 0.1)}s
#   IMU: {self.results.get('imu', {}).get('rate', 0):.1f} Hz
#   Has velocity info: {traj_info.get('has_velocities', False)}
#
# Chassis Characteristics:
#   Max speed: {chassis.get('max_speed', 0):.2f} m/s
#   Max angular: {chassis.get('max_wz', 0):.2f} rad/s
#   Max accel: {chassis.get('max_ax', 0):.2f} m/s^2
"""
        
        if tests:
            header += f"""#
# Chassis Test Results:
#   Tested max velocity: {tests.get('max_velocity_achieved', 0):.2f} m/s
#   Tested max accel: {tests.get('max_acceleration', 0):.2f} m/s^2
#   Response time: {tests.get('response_time', 0):.3f} sec
"""
        
        header += """#
# ============================================================================

"""
        
        with open(output_file, 'w') as f:
            f.write(header)
            yaml.dump(config, f, default_flow_style=False, allow_unicode=True, sort_keys=False)
        
        print(f"  {Colors.GREEN}[OK]{Colors.NC} Configuration saved to: {output_file}")
        print(f"\n{Colors.CYAN}Usage:{Colors.NC}")
        print(f"  roslaunch controller_ros controller.launch config:=$(pwd)/{output_file}")


def main():
    parser = argparse.ArgumentParser(
        description='Full System Diagnostics and Auto-Tuning Tool',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Basic diagnostics (only needs turtlebot_bringup + network trajectory)
  rosrun controller_ros full_diagnostics.py
  
  # Generate optimized config
  rosrun controller_ros full_diagnostics.py --output my_robot.yaml
  
  # Include chassis capability tests (robot will move!)
  rosrun controller_ros full_diagnostics.py --test-chassis --output my_robot.yaml
  
  # Runtime tuning (requires controller to be running)
  # Start: turtlebot_bringup + network + controller_ros
  rosrun controller_ros full_diagnostics.py --runtime-tuning --duration 30
  
  # Full diagnostics with everything
  rosrun controller_ros full_diagnostics.py --test-chassis --runtime-tuning --output tuned.yaml

Modes:
  Basic mode (default):
    - Needs: turtlebot_bringup + network trajectory
    - Diagnoses: odom rate, trajectory characteristics, chassis from odom
    
  --test-chassis mode:
    - Needs: turtlebot_bringup (robot will move!)
    - Diagnoses: actual max velocity, acceleration, response time
    
  --runtime-tuning mode:
    - Needs: turtlebot_bringup + network + controller_ros running
    - Diagnoses: MPC solve time, success rate, tracking errors
    - Provides: specific tuning suggestions for MPC weights
"""
    )
    
    parser.add_argument('--odom-topic', default='/odom', help='Odometry topic')
    parser.add_argument('--traj-topic', default='/nn/local_trajectory', help='Trajectory topic')
    parser.add_argument('--imu-topic', default='/imu', help='IMU topic')
    parser.add_argument('--cmd-vel-topic', default='/cmd_vel', help='Command velocity topic')
    parser.add_argument('--duration', type=float, default=5.0, help='Monitoring duration (sec)')
    parser.add_argument('--output', '-o', help='Output config file')
    parser.add_argument('--test-chassis', action='store_true', help='Run chassis capability tests')
    parser.add_argument('--runtime-tuning', action='store_true', 
                        help='Run controller runtime diagnostics (requires controller to be running)')
    parser.add_argument('--diag-topic', default='/controller/diagnostics', 
                        help='Controller diagnostics topic')
    
    args = parser.parse_args()
    
    try:
        diag = FullDiagnostics(args)
        diag.run()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("\nDiagnostics cancelled")


if __name__ == '__main__':
    main()
