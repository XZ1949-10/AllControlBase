"""
发布管理器

管理所有 ROS2 发布器。
"""
from typing import Dict, Any, Optional
import numpy as np
import threading
import queue
import time
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32

from universal_controller.core.data_types import ControlOutput, Trajectory, AttitudeCommand
from ..adapters import OutputAdapter, AttitudeAdapter
from ..utils.ros_compat import get_time_sec
from ..utils.diagnostics_publisher import fill_diagnostics_msg, DiagnosticsThrottler
from ..utils.param_loader import TOPICS_DEFAULTS

# 延迟导入处理或尝试导入
try:
    from controller_ros.msg import UnifiedCmd, DiagnosticsV2, AttitudeCmd
except ImportError:
    UnifiedCmd = None
    DiagnosticsV2 = None
    AttitudeCmd = None



class PublisherManager:
    """
    发布管理器 (ROS2)
    
    职责:
    - 创建和管理所有发布器
    - 发布统一控制命令和诊断
    - 支持四旋翼平台姿态命令发布
    
    性能优化:
    - 可视化路径发布 (Debug Path) 采用异步线程处理，避免阻塞主控制循环
    - 诊断信息发布支持节流
    """
    
    def __init__(self, node: Node, topics: Dict[str, str],
                 default_frame_id: str = 'base_link',
                 diag_publish_rate: int = 10,
                 is_quadrotor: bool = False):
        """
        初始化发布管理器
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
        
        # 调试路径发布节流
        self._last_debug_path_time = 0.0
        self._last_mpc_path_time = 0.0  # MPC 路径发布节流
        self._debug_path_interval = 0.5  # 2Hz Limit
        
        # 异步可视化工作线程
        self._vis_queue = queue.Queue(maxsize=2)  # 小缓冲区实现自动背压
        self._vis_running = True
        self._vis_thread = threading.Thread(
            target=self._vis_worker_loop,
            daemon=True,
            name="VisWorker"
        )
        self._vis_thread.start()
    
    def _vis_worker_loop(self):
        """可视化工作线程循环"""
        while self._vis_running and rclpy.ok():
            try:
                # 使用 timeout 允许定期检查停止标志
                task = self._vis_queue.get(timeout=0.5)
                if task is None:
                    break
                
                func, args = task
                try:
                    func(*args)
                except Exception as e:
                    # 避免让单个任务崩溃线程
                    self._node.get_logger().debug(f"Vis task failed: {e}")
                finally:
                    self._vis_queue.task_done()
            except queue.Empty:
                continue
            except Exception as e:
                self._node.get_logger().error(f"Vis worker error: {e}")
    
    def _create_publishers(self):
        """创建所有发布器"""
        # 统一控制命令发布
        cmd_topic = self._topics.get('cmd_unified', TOPICS_DEFAULTS['cmd_unified'])
        if UnifiedCmd is not None:
            self._cmd_pub = self._node.create_publisher(
                UnifiedCmd, cmd_topic, 10
            )
            self._node.get_logger().info(f"Publishing cmd to: {cmd_topic}")
        else:
            self._node.get_logger().warn("UnifiedCmd message not available")
            self._cmd_pub = None
        
        # 诊断发布
        diag_topic = self._topics.get('diagnostics', TOPICS_DEFAULTS['diagnostics'])
        if DiagnosticsV2 is not None:
            self._diag_pub = self._node.create_publisher(
                DiagnosticsV2, diag_topic, 10
            )
            # 预创建可复用的诊断消息对象，减少 GC 压力 (与 ROS1 实现保持一致)
            self._reusable_diag_msg = DiagnosticsV2()
            self._node.get_logger().info(f"Publishing diagnostics to: {diag_topic}")
        else:
            self._node.get_logger().warn("DiagnosticsV2 message not available")
            self._diag_pub = None
            self._reusable_diag_msg = None
        
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
        
        # MPC 预测路径发布
        mpc_path_topic = self._topics.get('mpc_path', '/controller/mpc_path')
        self._mpc_path_pub = self._node.create_publisher(
            Path, mpc_path_topic, 1
        )
        
        # 姿态命令发布器 (四旋翼平台)
        if self._is_quadrotor:
            attitude_topic = self._topics.get('attitude_cmd', TOPICS_DEFAULTS['attitude_cmd'])
            if AttitudeCmd is not None:
                self._attitude_pub = self._node.create_publisher(
                    AttitudeCmd, attitude_topic, 1
                )
                self._node.get_logger().info(f"Publishing attitude to: {attitude_topic}")
            else:
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
        # Support both Dict and Object (DiagnosticsV2)
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
        
        if self._diag_pub is None or self._reusable_diag_msg is None:
            return
        
        try:
            # 复用消息对象，避免每次创建新对象 (与 ROS1 实现保持一致)
            fill_diagnostics_msg(
                self._reusable_diag_msg, diag,
                get_time_func=lambda: self._node.get_clock().now().to_msg()
            )
            
            self._diag_pub.publish(self._reusable_diag_msg)
        except Exception:
            # 忽略所有发布错误，保持控制循环稳定
            pass
    
    def publish_debug_path(self, trajectory: Trajectory):
        """
        发布调试路径 (异步/非阻塞版)
        
        Conversion of trajectory points to PoseStamped can be slow (O(N) Python loop).
        We offload this to a separate thread to avoid blocking the critical control loop.
        """
        if self._path_pub is None:
            return
            
        # 优化: 如果没人订阅调试路径，直接跳过 (节省大量序列化开销)
        if self._path_pub.get_subscription_count() == 0:
            return
            
        # Throttling
        now = self._get_time_func()
        if now - self._last_debug_path_time < self._debug_path_interval:
            return
        self._last_debug_path_time = now
        
        # 检查轨迹点是否有效
        if trajectory.points is None or len(trajectory.points) == 0:
            return

        # 智能降采样 (Intelligent Downsampling)
        # 避免在 RVIZ 可视化大量点时造成不必要的序列化开销和带宽浪费
        # 目标: 限制可视化的点数在 100-200 之间，足以展示路径形状
        MAX_VIS_POINTS = 100
        num_points = len(trajectory.points)
        step = 1
        if num_points > MAX_VIS_POINTS:
            step = max(1, num_points // MAX_VIS_POINTS)
        
        # 将任务放入队列
        try:
            # put_nowait 实现非阻塞背压：如果队列满了，直接丢弃当前帧 (Drop Frame)
            # 这比堆积过时数据要好得多
            self._vis_queue.put_nowait(
                (self._do_publish_debug_path_sync, (trajectory, self._node.get_clock().now().to_msg(), step))
            )
        except queue.Full:
            self._node.get_logger().warn(
                "Debug path visualization queue full, dropping frame (visualization/network too slow).",
                throttle_duration_sec=30.0
            )

    def _do_publish_debug_path_sync(self, trajectory: Trajectory, stamp_msg, step: int = 1):
        """同步执行路径转换和发布 (由 Worker 线程调用)"""
        if trajectory is None:
            return

        try:
            path = Path()
            path.header.stamp = stamp_msg
            path.header.frame_id = trajectory.header.frame_id or 'odom'
            
            # Handle both numpy array and Point3D list
            # 耗时操作在这里执行
            # Optimization: Use list comprehension with inline object creation to minimize python loop overhead
            # Performance Note: Cache method reference to avoid 'self.' lookup in tight loop
            create_pose = self._create_pose_inline
            header = path.header
            
            if isinstance(trajectory.points, np.ndarray):
                # 优化: 先转换为 Python List，避免 numpy scalar 访问开销
                # 使用 tolist() 后，p 是 [x, y, z] 列表
                # 降采样: 使用 slice [::step]
                points_list = trajectory.points[::step].tolist()
                poses = [
                    create_pose(p[0], p[1], p[2], header) 
                    for p in points_list
                ]
            else:
                poses = [
                    create_pose(p.x, p.y, p.z, header) 
                    for p in trajectory.points[::step]
                ]
            
            path.poses = poses
            
            self._path_pub.publish(path)
        except Exception:
            pass

    def _create_pose_inline(self, x, y, z, header):
        """Helper for fast list comprehension"""
        pose = PoseStamped()
        pose.header = header
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)
        pose.pose.orientation.w = 1.0
        return pose

    def _create_pose_stamped(self, x, y, z, header):
        return self._create_pose_inline(x, y, z, header)
    
    def publish_predicted_path(self, predicted_states: list, frame_id: str = 'odom'):
        """
        发布 MPC 预测路径 (异步/非阻塞版)
        """
        if self._mpc_path_pub is None:
            return
            
        # 优化: 如果没人订阅 MPC 路径，直接跳过
        if self._mpc_path_pub.get_subscription_count() == 0:
            return
            
        if not predicted_states:
            return

        # 节流 (Limit to ~5Hz -> 0.2s)
        now = self._get_time_func()
        if now - self._last_mpc_path_time < 0.2:
            return
        self._last_mpc_path_time = now
            
        # 启动后台任务
        # MPC 轨迹通常较短，但为了代码健壮性也加上降采样逻辑
        MAX_MPC_VIS = 50
        num_points = len(predicted_states)
        step = 1
        if num_points > MAX_MPC_VIS:
             step = max(1, num_points // MAX_MPC_VIS)

        try:
            self._vis_queue.put_nowait(
                (self._do_publish_mpc_path_sync, (predicted_states, frame_id, self._node.get_clock().now().to_msg(), step))
            )
        except queue.Full:
            self._node.get_logger().warn(
                "MPC path visualization queue full, dropping frame.",
                throttle_duration_sec=30.0
            )

    def _do_publish_mpc_path_sync(self, predicted_states, frame_id, stamp_msg, step: int = 1):
        """同步发布 MPC 预测路径 (由 Worker 线程调用)"""
        try:
            path = Path()
            path.header.stamp = stamp_msg
            path.header.frame_id = frame_id
            
            # 优化: Handle numpy array overhead
            if isinstance(predicted_states, np.ndarray):
                predicted_states = predicted_states.tolist()
            
            for state in predicted_states[::step]:
                # ACADOS state: [px, py, pz, vx, vy, vz, theta, omega]
                if len(state) < 3:
                    continue
                    
                pose = PoseStamped()
                pose.header = path.header
                pose.pose.position.x = float(state[0])
                pose.pose.position.y = float(state[1])
                pose.pose.position.z = float(state[2])
                
                # Simple orientation from theta if available (state[6])
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
    
    def publish_attitude_cmd(self, attitude_cmd: AttitudeCommand,
                             yaw_mode: int = 0, is_hovering: bool = False):
        """发布姿态命令 (四旋翼平台)"""
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
        return self._output_adapter
    
    @property
    def attitude_adapter(self) -> Optional[AttitudeAdapter]:
        return self._attitude_adapter

    def shutdown(self) -> None:
        """关闭发布管理器"""
        # 1. 停止 Worker 线程
        self._vis_running = False
        try:
             # 发送 poison pill 以尽快唤醒
             self._vis_queue.put(None, timeout=0.1)
        except queue.Full:
             pass 
        if self._vis_thread is not None:
             self._vis_thread.join(timeout=1.0)
             self._vis_thread = None
        
        # 2. 清空所有发布器引用 (按创建顺序)
        # 注意: ROS2 发布器会在节点销毁时自动清理，
        # 但显式清空可以更早释放资源并防止悬挂引用
        self._cmd_pub = None
        self._diag_pub = None
        self._state_pub = None
        self._path_pub = None
        self._mpc_path_pub = None
        self._attitude_pub = None
        
        # 3. 清空适配器和节流器 (与 ROS1 版本保持一致)
        self._output_adapter = None
        self._attitude_adapter = None
        self._diag_throttler = None
