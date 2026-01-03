"""
控制器 ROS2 节点

主节点实现，组装所有组件，管理控制循环。
支持 TF2 坐标变换集成和四旋翼平台。

继承 ControllerNodeBase，实现 ROS2 特定的接口。
"""
from typing import Dict, Any, Optional

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

from universal_controller.core.data_types import ControlOutput, AttitudeCommand

from .base_node import ControllerNodeBase
from ..bridge import TFBridge
from ..io import PublisherManager, ServiceManager
from ..utils import ParamLoader
from ..utils.param_loader import TOPICS_DEFAULTS


class ControllerNode(ControllerNodeBase, Node):
    """
    控制器主节点 (ROS2)
    
    继承 ControllerNodeBase 获取共享逻辑，
    实现 ROS2 特定的接口。
    
    输出:
    - /cmd_unified (UnifiedCmd)
    - /controller/diagnostics (DiagnosticsV2)
    - /controller/state (Int32)
    - /controller/attitude_cmd (AttitudeCmd, 仅四旋翼)
    """
    
    def __init__(self):
        # 先初始化 ROS2 Node
        Node.__init__(self, 'universal_controller_node')
        # 再初始化基类
        ControllerNodeBase.__init__(self)
        
        # 1. 加载参数
        self._params = ParamLoader.load(self)
        self._topics = ParamLoader.get_topics(self)
        
        # 2. 创建回调组（用于并发控制）
        self._sensor_cb_group = ReentrantCallbackGroup()
        self._control_cb_group = MutuallyExclusiveCallbackGroup()
        # 紧急停止专用回调组 (高优先级，互斥)
        # 使用单独的组确保即使传感器组积压，E-Stop 也能被调度
        self._estop_cb_group = MutuallyExclusiveCallbackGroup()
        
        # 3. 初始化核心组件（基类方法）
        self._initialize()
        
        # 4. 创建 TF 桥接
        self._tf_bridge = TFBridge(self)
        
        # 5. 注入 TF2 到坐标变换器（基类方法）
        self._inject_tf2_to_controller()
        
        # 6. 创建 ROS2 接口
        self._create_ros_interfaces()
        
        # 7. 创建控制定时器
        # 从 system.ctrl_freq 读取控制频率
        control_rate = self._params.get('system', {}).get('ctrl_freq', 50)
        control_period = 1.0 / control_rate
        self._control_timer = self.create_timer(
            control_period,
            self._control_callback,
            callback_group=self._control_cb_group
        )
        
        self.get_logger().info(
            f'Controller node initialized (platform={self._platform_type}, '
            f'rate={control_rate}Hz, tf2={self._tf_bridge.is_initialized}, '
            f'quadrotor={self._is_quadrotor})'
        )
    
    def _create_ros_interfaces(self):
        """创建 ROS2 特定的接口"""
        # 创建订阅
        self._create_subscriptions()
        
        # 创建发布管理器
        diag_publish_rate = self._params.get('diagnostics', {}).get('publish_rate', 10)
        self._publishers = PublisherManager(
            self, self._topics, self._default_frame_id,
            diag_publish_rate=diag_publish_rate,
            is_quadrotor=self._is_quadrotor
        )
        
        # 创建服务管理器
        self._services = ServiceManager(
            self,
            reset_callback=self._handle_reset,
            get_diagnostics_callback=self._handle_get_diagnostics,
            set_state_callback=self._handle_set_state,
            set_hover_yaw_callback=self._handle_set_hover_yaw if self._is_quadrotor else None,
            get_attitude_rate_limits_callback=self._handle_get_attitude_rate_limits if self._is_quadrotor else None,
            is_quadrotor=self._is_quadrotor
        )
    
    def _create_subscriptions(self):
        """创建所有订阅"""
        from nav_msgs.msg import Odometry as RosOdometry
        from sensor_msgs.msg import Imu as RosImu
        from std_msgs.msg import Empty
        
        # 里程计订阅
        odom_topic = self._topics.get('odom', TOPICS_DEFAULTS['odom'])
        self._odom_sub = self.create_subscription(
            RosOdometry,
            odom_topic,
            self._odom_callback,
            10,
            callback_group=self._sensor_cb_group
        )
        self.get_logger().info(f"Subscribed to odom: {odom_topic}")
        
        # IMU 订阅 - 仅在配置了 IMU topic 时创建
        imu_topic = self._topics.get('imu', '')
        if imu_topic:
            self._imu_sub = self.create_subscription(
                RosImu,
                imu_topic,
                self._imu_callback,
                10,
                callback_group=self._sensor_cb_group
            )
            self.get_logger().info(f"Subscribed to imu: {imu_topic}")
        else:
            self._imu_sub = None
            self.get_logger().info("IMU topic not configured, skipping IMU subscription")
        
        # 轨迹订阅
        traj_topic = self._topics.get('trajectory', TOPICS_DEFAULTS['trajectory'])
        try:
            from controller_ros.msg import LocalTrajectoryV4
            self._traj_sub = self.create_subscription(
                LocalTrajectoryV4,
                traj_topic,
                self._traj_callback,
                10,
                callback_group=self._sensor_cb_group
            )
            self._traj_msg_available = True
            self.get_logger().info(f"Subscribed to trajectory: {traj_topic}")
        except ImportError:
            self.get_logger().error(
                "=" * 60 + "\n"
                "CRITICAL: LocalTrajectoryV4 message type not available!\n"
                "Controller cannot work without trajectory data.\n"
                "\n"
                "To fix this issue:\n"
                "1. Ensure controller_ros package is built with colcon:\n"
                "   cd ~/your_ws && colcon build --packages-select controller_ros\n"
                "2. Source the workspace:\n"
                "   source ~/your_ws/install/setup.bash\n"
                "3. Restart the controller node\n"
                "\n"
                "If the problem persists, check:\n"
                "- CMakeLists.txt has rosidl_generate_interfaces() for msg files\n"
                "- package.xml has rosidl_default_generators dependency\n"
                "=" * 60
            )
            self._traj_sub = None
            self._traj_msg_available = False
        
        # 紧急停止话题订阅
        emergency_stop_topic = self._topics.get('emergency_stop', TOPICS_DEFAULTS['emergency_stop'])
        self._emergency_stop_sub = self.create_subscription(
            Empty,
            emergency_stop_topic,
            self._emergency_stop_callback,
            1,  # 低队列深度，确保及时响应
            callback_group=self._estop_cb_group
        )
        self.get_logger().info(f"Subscribed to emergency_stop: {emergency_stop_topic}")
    
    # ==================== 订阅回调 ====================
    
    def _odom_callback(self, msg):
        """里程计回调"""
        self._data_manager.update_odom(msg)
    
    def _imu_callback(self, msg):
        """IMU 回调"""
        self._data_manager.update_imu(msg)
    
    def _traj_callback(self, msg):
        """轨迹回调"""
        self._data_manager.update_trajectory(msg)
    
    def _emergency_stop_callback(self, msg):
        """紧急停止回调"""
        self._handle_emergency_stop()
    
    # ==================== 控制循环 ====================
    
    def _control_callback(self):
        """控制循环回调"""
        # 关闭检查已在基类 _control_loop_core() 中统一处理
        cmd = self._control_loop_core()
        
        if cmd is not None:
            # 发布控制命令
            self._publish_cmd(cmd)
    
    # ==================== 基类抽象方法实现 ====================
    
    def _get_time(self) -> float:
        """获取当前 ROS 时间（秒）"""
        clock_time = self.get_clock().now().nanoseconds * 1e-9
        # 仿真时间模式下可能为 0
        if clock_time > 0:
            return clock_time
        import time
        return time.time()
    
    def _log_info(self, msg: str):
        """记录信息日志"""
        self.get_logger().info(msg)
    
    def _log_warn(self, msg: str):
        """记录警告日志"""
        self.get_logger().warn(msg)
    
    def _log_warn_throttle(self, period: float, msg: str):
        """记录节流警告日志"""
        self.get_logger().warn(msg, throttle_duration_sec=period)
    
    def _log_error(self, msg: str):
        """记录错误日志"""
        self.get_logger().error(msg)
    
    def _publish_cmd(self, cmd: ControlOutput):
        """发布控制命令"""
        self._publishers.publish_cmd(cmd)
    
    def _publish_stop_cmd(self):
        """发布停止命令"""
        self._publishers.publish_stop_cmd()
    
    def _publish_diagnostics(self, diag: Dict[str, Any], force: bool = False):
        """发布诊断信息"""
        self._publishers.publish_diagnostics(diag, force=force)
    

    
    def _publish_debug_path(self, trajectory):
        """发布调试路径 (用于 RViz 可视化)"""
        self._publishers.publish_debug_path(trajectory)
    
    # ==================== 生命周期 ====================
    
    def shutdown(self):
        """
        清理资源
        
        完整的资源清理流程:
        1. 设置关闭标志，阻止控制循环继续执行
        2. 取消控制定时器
        3. 发送最终停止命令
        4. 销毁订阅器
        5. 关闭发布器和服务
        6. 调用基类清理
        """
        # 1. 设置关闭标志 (基类会设置，但这里提前设置以阻止回调)
        # 使用 Event.set() 确保线程安全
        self._shutting_down.set()
        
        # 2. 取消控制定时器
        if self._control_timer is not None:
            self._control_timer.cancel()
            self._control_timer = None
        
        # 3. 发送最终停止命令
        try:
            if self._publishers is not None:
                self._publishers.publish_stop_cmd()
                self.get_logger().info("Final stop command sent")
        except Exception as e:
            self.get_logger().warn(f"Failed to send final stop command: {e}")
        
        # 4. 销毁订阅器
        if self._odom_sub is not None:
            self.destroy_subscription(self._odom_sub)
            self._odom_sub = None
        
        if self._imu_sub is not None:
            self.destroy_subscription(self._imu_sub)
            self._imu_sub = None
        
        if self._traj_sub is not None:
            self.destroy_subscription(self._traj_sub)
            self._traj_sub = None
        
        if self._emergency_stop_sub is not None:
            self.destroy_subscription(self._emergency_stop_sub)
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
        
        self.get_logger().info("Controller node shutdown complete")


def main(args=None):
    """主入口"""
    rclpy.init(args=args)
    
    node = ControllerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
