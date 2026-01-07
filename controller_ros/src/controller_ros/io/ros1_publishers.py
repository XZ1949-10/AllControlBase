"""
ROS1 发布管理器

管理所有 ROS1 发布器，与 ROS2 的 PublisherManager 接口对齐。
优化说明:
- 修复了 Numpy 数组属性访问导致的崩溃问题 (Critical Bug Fix)
- 引入了异步可视化发布线程，避免阻塞控制主循环 (High Performance)
"""
from typing import Dict, Any, Optional, Callable, List
import logging
import threading
import queue
import time
import numpy as np
import math

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
from ..utils.msg_availability import (
    CUSTOM_MSGS_AVAILABLE,
    UnifiedCmd,
    DiagnosticsV2,
    AttitudeCmd,
)

logger = logging.getLogger(__name__)


class ROS1PublisherManager:
    """
    ROS1 发布管理器
    
    职责:
    - 创建和管理所有 ROS1 发布器
    - 发布统一控制命令和诊断
    - 异步发布可视化消息 (Debug Path)
    """
    
    def __init__(self, topics: Dict[str, str],
                 default_frame_id: str = 'base_link',
                 diag_publish_rate: int = 10,
                 get_time_func: Optional[Callable[[], float]] = None,
                 is_quadrotor: bool = False):
        """
        初始化发布管理器
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
        
        # 创建发布器
        self._create_publishers()
        
        # 诊断节流器
        self._diag_throttler = DiagnosticsThrottler(publish_rate=diag_publish_rate)

        # -----------------------------------------------------------
        # 异步可视化工作线程 (Async Visualization)
        # -----------------------------------------------------------
        self._vis_queue = queue.Queue(maxsize=2)  # 小缓冲区实现自动背压
        self._vis_running = True
        self._vis_thread = threading.Thread(
            target=self._vis_worker_loop,
            daemon=True,
            name="VisWorkerROS1"
        )
        self._vis_thread.start()
        
        # 节流控制
        self._last_debug_path_time = 0.0
        self._last_mpc_path_time = 0.0  # MPC 路径发布节流
        self._debug_path_interval = 0.5  # 2Hz Limit

    def _create_publishers(self):
        """创建所有发布器"""
        # 预创建可复用的诊断消息对象
        if CUSTOM_MSGS_AVAILABLE and DiagnosticsV2 is not None:
            self._reusable_diag_msg = DiagnosticsV2()
        else:
            self._reusable_diag_msg = None
        
        # 控制命令发布器
        cmd_topic = self._topics.get('cmd_unified', TOPICS_DEFAULTS['cmd_unified'])
        if CUSTOM_MSGS_AVAILABLE and UnifiedCmd is not None:
            self._cmd_pub = rospy.Publisher(cmd_topic, UnifiedCmd, queue_size=1)
            rospy.loginfo(f"Publishing cmd to: {cmd_topic}")
        else:
            self._cmd_pub = None
            rospy.logerr("Cannot create command publisher: UnifiedCmd not available")
        
        # 诊断发布器
        diag_topic = self._topics.get('diagnostics', TOPICS_DEFAULTS['diagnostics'])
        if CUSTOM_MSGS_AVAILABLE and DiagnosticsV2 is not None:
            self._diag_pub = rospy.Publisher(diag_topic, DiagnosticsV2, queue_size=10)
            rospy.loginfo(f"Publishing diagnostics to: {diag_topic}")
        else:
            self._diag_pub = None
            rospy.logwarn("Cannot create diagnostics publisher: DiagnosticsV2 not available")
        
        # 状态发布器 (标准消息，始终可用)
        state_topic = self._topics.get('state', TOPICS_DEFAULTS['state'])
        self._state_pub = rospy.Publisher(state_topic, Int32, queue_size=1)
        rospy.loginfo(f"Publishing state to: {state_topic}")
        
        # 调试路径发布器
        debug_path_topic = self._topics.get('debug_path', TOPICS_DEFAULTS['debug_path'])
        self._debug_path_pub = rospy.Publisher(debug_path_topic, Path, queue_size=1)
        rospy.loginfo(f"Publishing debug_path to: {debug_path_topic}")
        
        # MPC 预测路径发布器
        mpc_path_topic = self._topics.get('mpc_path', '/controller/mpc_path')
        self._mpc_path_pub = rospy.Publisher(mpc_path_topic, Path, queue_size=1)
        rospy.loginfo(f"Publishing mpc_path to: {mpc_path_topic}")
        
        # 姿态命令发布器 (四旋翼平台)
        if self._is_quadrotor:
            attitude_topic = self._topics.get('attitude_cmd', TOPICS_DEFAULTS['attitude_cmd'])
            if CUSTOM_MSGS_AVAILABLE and AttitudeCmd is not None:
                self._attitude_pub = rospy.Publisher(attitude_topic, AttitudeCmd, queue_size=1)
                rospy.loginfo(f"Publishing attitude to: {attitude_topic}")
            else:
                self._attitude_pub = None
                rospy.logwarn("AttitudeCmd message not available")
        else:
            self._attitude_pub = None
    
    def _vis_worker_loop(self):
        """可视化工作线程循环"""
        while self._vis_running and not rospy.is_shutdown():
            try:
                # 使用 timeout 允许定期检查停止标志
                task = self._vis_queue.get(timeout=0.5)
                if task is None:
                    break
                
                func, args = task
                try:
                    func(*args)
                except Exception as e:
                    rospy.logdebug(f"Vis task failed: {e}")
                finally:
                    self._vis_queue.task_done()
            except queue.Empty:
                continue
            except Exception as e:
                rospy.logerr(f"Vis worker error: {e}")

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
        if isinstance(diag, dict):
             current_state = diag.get('state', 0)
        else:
             current_state = getattr(diag, 'state', 0)

        state_msg = Int32()
        state_msg.data = current_state
        self._state_pub.publish(state_msg)
        
        # 使用节流器判断是否发布诊断
        if not self._diag_throttler.should_publish(diag, force=force):
            return
        
        if self._diag_pub is None:
            return
        
        if self._reusable_diag_msg is None:
            return
        
        try:
            # 复用消息对象，避免每次创建新对象
            fill_diagnostics_msg(self._reusable_diag_msg, diag, get_time_func=lambda: rospy.Time.now())
            self._diag_pub.publish(self._reusable_diag_msg)
        except Exception as e:
            # 同样忽略错误以保护控制环
             pass

    def publish_attitude_cmd(self, attitude_cmd: AttitudeCommand,
                             yaw_mode: int = 0, is_hovering: bool = False):
        """发布姿态命令"""
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
        发布调试路径 (异步/非阻塞版)
        
        Fixes:
        1. Async execution to prevent blocking ROS1 control loop.
        2. Correct handling of Numpy array (Fixed Crash).
        """
        if self._debug_path_pub is None:
            return
            
        # 优化: 检查订阅数
        if self._debug_path_pub.get_num_connections() == 0:
            return
        
        # 节流
        now = self._get_time_func()
        if now - self._last_debug_path_time < self._debug_path_interval:
            return
        self._last_debug_path_time = now

        # 检查有效性
        if trajectory.points is None or len(trajectory.points) == 0:
            return
            
        # 提交到队列
        try:
             # Snapshot timestamp now
            stamp = rospy.Time.now()
            self._vis_queue.put_nowait(
                (self._do_publish_debug_path_sync, (trajectory, stamp))
            )
        except queue.Full:
            pass

    def _do_publish_debug_path_sync(self, trajectory: Trajectory, stamp):
        """同步执行路径转换和发布 (Worker 线程)"""
        if trajectory is None:
            return
            
        try:
            path = Path()
            path.header.stamp = stamp
            path.header.frame_id = trajectory.header.frame_id or 'odom'
            
            # --- CRITICAL FIX START ---
            # 处理 Numpy 数组，避免 attribute error
            if isinstance(trajectory.points, np.ndarray):
                # 优化: 先转换为 Python List of Lists
                # tolist() on [N, 3] array produces [[x,y,z], [x,y,z], ...]
                points_list = trajectory.points.tolist()
                path.poses = [
                    self._create_pose_stamped(p[0], p[1], p[2], path.header)
                    for p in points_list
                ]
            else:
                # Legacy List[Point3D]
                path.poses = [
                    self._create_pose_stamped(p.x, p.y, p.z, path.header)
                    for p in trajectory.points
                ]
            # --- CRITICAL FIX END ---
            
            self._debug_path_pub.publish(path)
        except Exception:
            pass

    def _create_pose_stamped(self, x, y, z, header):
        """Helper to create PoseStamped"""
        pose = PoseStamped()
        pose.header = header
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)
        pose.pose.orientation.w = 1.0
        return pose

    def publish_predicted_path(self, predicted_states: List, frame_id: str = 'odom'):
        """发布 MPC 预测路径 (异步)"""
        if self._mpc_path_pub is None:
            return
            
        if not predicted_states:
            return
            
        if self._mpc_path_pub.get_num_connections() == 0:
            return

        # 节流 MPC path (Limit to ~5Hz)
        now = self._get_time_func()
        if now - self._last_mpc_path_time < 0.2:
            return
        self._last_mpc_path_time = now
            
        try:
            stamp = rospy.Time.now()
            self._vis_queue.put_nowait(
                (self._do_publish_mpc_path_sync, (predicted_states, frame_id, stamp))
            )
        except queue.Full:
            pass

    def _do_publish_mpc_path_sync(self, predicted_states, frame_id, stamp):
        """同步发布 MPC 路径"""
        try:
            path = Path()
            path.header.stamp = stamp
            path.header.frame_id = frame_id
            
            if isinstance(predicted_states, np.ndarray):
                predicted_states = predicted_states.tolist()
            
            for state in predicted_states:
                # ACADOS state: [px, py, pz, vx, vy, vz, theta, omega]
                if len(state) < 3:
                    continue
                    
                pose = PoseStamped()
                pose.header = path.header
                pose.pose.position.x = float(state[0])
                pose.pose.position.y = float(state[1])
                pose.pose.position.z = float(state[2])
                
                if len(state) > 6:
                    theta = float(state[6])
                    pose.pose.orientation.z = math.sin(theta / 2.0)
                    pose.pose.orientation.w = math.cos(theta / 2.0)
                else:
                    pose.pose.orientation.w = 1.0
                    
                path.poses.append(pose)
            
            self._mpc_path_pub.publish(path)
        except Exception:
            pass
    
    @property
    def output_adapter(self) -> OutputAdapter:
        return self._output_adapter
    
    @property
    def attitude_adapter(self) -> Optional[AttitudeAdapter]:
        return self._attitude_adapter

    def shutdown(self) -> None:
        """关闭发布管理器"""
        # 1. 停止线程
        self._vis_running = False
        try:
            self._vis_queue.put(None, timeout=0.1)
        except queue.Full:
            pass
        
        if self._vis_thread is not None:
            self._vis_thread.join(timeout=1.0)
            self._vis_thread = None

        # 2. 注销发布器
        if self._cmd_pub is not None:
            self._cmd_pub.unregister()
            self._cmd_pub = None
        
        if self._diag_pub is not None:
            self._diag_pub.unregister()
            self._diag_pub = None
        
        if self._state_pub is not None:
            self._state_pub.unregister()
            self._state_pub = None
        
        if self._debug_path_pub is not None:
            self._debug_path_pub.unregister()
            self._debug_path_pub = None
        
        if self._mpc_path_pub is not None:
            self._mpc_path_pub.unregister()
            self._mpc_path_pub = None
        
        if self._attitude_pub is not None:
            self._attitude_pub.unregister()
            self._attitude_pub = None
        
        self._output_adapter = None
        self._attitude_adapter = None
        self._diag_throttler = None
