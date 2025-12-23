"""
发布管理器

管理所有 ROS 发布器。
"""
from typing import Dict, Any, Optional
import time

from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from universal_controller.core.data_types import ControlOutput, Trajectory
from ..adapters import OutputAdapter


class PublisherManager:
    """
    发布管理器
    
    职责:
    - 创建和管理所有发布器
    - 发布统一控制命令和诊断
    
    注意: 不做平台特定转换，只发布 UnifiedCmd
    """
    
    def __init__(self, node: Node, topics: Dict[str, str],
                 default_frame_id: str = 'base_link'):
        """
        初始化发布管理器
        
        Args:
            node: ROS2 节点
            topics: 话题配置字典
            default_frame_id: 默认输出坐标系
        """
        self._node = node
        self._topics = topics
        self._output_adapter = OutputAdapter(default_frame_id)
        
        # 创建发布器
        self._create_publishers()
        
        # 诊断降频计数器
        self._diag_counter = 0
        self._diag_publish_rate = 5  # 每 5 次控制循环发布一次诊断
    
    def _create_publishers(self):
        """创建所有发布器"""
        # 统一控制命令发布
        cmd_topic = self._topics.get('cmd_unified', '/cmd_unified')
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
        diag_topic = self._topics.get('diagnostics', '/controller/diagnostics')
        try:
            from controller_ros.msg import DiagnosticsV2
            self._diag_pub = self._node.create_publisher(
                DiagnosticsV2, diag_topic, 10
            )
            self._node.get_logger().info(f"Publishing diagnostics to: {diag_topic}")
        except ImportError:
            self._node.get_logger().warn("DiagnosticsV2 message not available")
            self._diag_pub = None
        
        # 调试路径发布
        self._path_pub = self._node.create_publisher(
            Path, '/controller/debug_path', 1
        )
    
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
        if self._diag_pub is None:
            return
        
        # 降频发布
        self._diag_counter += 1
        if not force and self._diag_counter < self._diag_publish_rate:
            return
        self._diag_counter = 0
        
        try:
            from controller_ros.msg import DiagnosticsV2
            
            msg = DiagnosticsV2()
            msg.header.stamp = self._node.get_clock().now().to_msg()
            msg.header.frame_id = 'controller'
            
            # 填充诊断数据
            msg.state = diag.get('state', 0)
            msg.mpc_success = diag.get('mpc_success', False)
            msg.mpc_solve_time_ms = float(diag.get('mpc_solve_time_ms', 0.0))
            msg.backup_active = diag.get('backup_active', False)
            
            # MPC 健康状态
            mpc_health = diag.get('mpc_health', {})
            msg.mpc_health_kkt_residual = float(mpc_health.get('kkt_residual', 0.0))
            msg.mpc_health_condition_number = float(mpc_health.get('condition_number', 1.0))
            msg.mpc_health_consecutive_near_timeout = int(mpc_health.get('consecutive_near_timeout', 0))
            msg.mpc_health_degradation_warning = mpc_health.get('degradation_warning', False)
            msg.mpc_health_can_recover = mpc_health.get('can_recover', True)
            
            # 一致性指标
            consistency = diag.get('consistency', {})
            msg.consistency_curvature = float(consistency.get('curvature', 0.0))
            msg.consistency_velocity_dir = float(consistency.get('velocity_dir', 1.0))
            msg.consistency_temporal = float(consistency.get('temporal', 1.0))
            msg.consistency_alpha_soft = float(consistency.get('alpha_soft', 0.0))
            msg.consistency_data_valid = consistency.get('data_valid', True)
            
            # 超时状态
            timeout = diag.get('timeout', {})
            msg.timeout_odom = timeout.get('odom_timeout', False)
            msg.timeout_traj = timeout.get('traj_timeout', False)
            msg.timeout_traj_grace_exceeded = timeout.get('traj_grace_exceeded', False)
            msg.timeout_imu = timeout.get('imu_timeout', False)
            msg.timeout_last_odom_age_ms = float(timeout.get('last_odom_age_ms', 0.0))
            msg.timeout_last_traj_age_ms = float(timeout.get('last_traj_age_ms', 0.0))
            msg.timeout_last_imu_age_ms = float(timeout.get('last_imu_age_ms', 0.0))
            msg.timeout_in_startup_grace = timeout.get('in_startup_grace', False)
            
            # 控制命令
            cmd = diag.get('cmd', {})
            msg.cmd_vx = float(cmd.get('vx', 0.0))
            msg.cmd_vy = float(cmd.get('vy', 0.0))
            msg.cmd_vz = float(cmd.get('vz', 0.0))
            msg.cmd_omega = float(cmd.get('omega', 0.0))
            msg.cmd_frame_id = cmd.get('frame_id', '')
            
            msg.transition_progress = float(diag.get('transition_progress', 0.0))
            
            self._diag_pub.publish(msg)
        except Exception as e:
            self._node.get_logger().warn(f"Failed to publish diagnostics: {e}")
    
    def publish_debug_path(self, trajectory: Trajectory):
        """发布调试路径"""
        if self._path_pub is None:
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
