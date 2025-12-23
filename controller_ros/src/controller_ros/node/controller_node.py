"""
控制器 ROS2 节点

主节点实现，组装所有组件，管理控制循环。
支持 TF2 坐标变换集成。

继承 ControllerNodeBase，实现 ROS2 特定的接口。
"""
from typing import Dict, Any, Optional

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

from universal_controller.core.data_types import ControlOutput

from .base_node import ControllerNodeBase
from ..bridge import TFBridge
from ..io import PublisherManager, ServiceManager
from ..utils import ParamLoader


class ControllerNode(ControllerNodeBase, Node):
    """
    控制器主节点 (ROS2)
    
    继承 ControllerNodeBase 获取共享逻辑，
    实现 ROS2 特定的接口。
    
    输出:
    - /cmd_unified (UnifiedCmd)
    - /controller/diagnostics (DiagnosticsV2)
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
        
        # 3. 初始化核心组件（基类方法）
        self._initialize()
        
        # 4. 创建 TF 桥接
        self._tf_bridge = TFBridge(self)
        
        # 5. 注入 TF2 到坐标变换器（基类方法）
        self._inject_tf2_to_controller()
        
        # 6. 创建 ROS2 接口
        self._create_ros_interfaces()
        
        # 7. 创建控制定时器
        # 统一从 system.ctrl_freq 读取控制频率 (ParamLoader._merge_params 已将 node.control_rate 合并到此)
        control_rate = self._params.get('system', {}).get('ctrl_freq', 50)
        control_period = 1.0 / control_rate
        self._control_timer = self.create_timer(
            control_period,
            self._control_callback,
            callback_group=self._control_cb_group
        )
        
        self.get_logger().info(
            f'Controller node initialized (platform={self._platform_type}, '
            f'rate={control_rate}Hz, tf2={self._tf_bridge.is_initialized})'
        )
    
    def _create_ros_interfaces(self):
        """创建 ROS2 特定的接口"""
        # 创建订阅
        self._create_subscriptions()
        
        # 创建发布管理器
        diag_publish_rate = self._params.get('diagnostics', {}).get('publish_rate', 5)
        self._publishers = PublisherManager(
            self, self._topics, self._default_frame_id,
            diag_publish_rate=diag_publish_rate
        )
        
        # 创建服务管理器
        self._services = ServiceManager(
            self,
            reset_callback=self._handle_reset,
            get_diagnostics_callback=self._handle_get_diagnostics,
            set_state_callback=self._handle_set_state
        )
    
    def _create_subscriptions(self):
        """创建所有订阅"""
        from nav_msgs.msg import Odometry as RosOdometry
        from sensor_msgs.msg import Imu as RosImu
        
        # 里程计订阅
        odom_topic = self._topics.get('odom', '/odom')
        self._odom_sub = self.create_subscription(
            RosOdometry,
            odom_topic,
            self._odom_callback,
            10,
            callback_group=self._sensor_cb_group
        )
        self.get_logger().info(f"Subscribed to odom: {odom_topic}")
        
        # IMU 订阅
        imu_topic = self._topics.get('imu', '/imu')
        self._imu_sub = self.create_subscription(
            RosImu,
            imu_topic,
            self._imu_callback,
            10,
            callback_group=self._sensor_cb_group
        )
        self.get_logger().info(f"Subscribed to imu: {imu_topic}")
        
        # 轨迹订阅
        traj_topic = self._topics.get('trajectory', '/nn/local_trajectory')
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
                f"LocalTrajectoryV4 message not available! "
                f"Controller will not work without trajectory data. "
                f"Please build the controller_ros package with message generation."
            )
            self._traj_sub = None
            self._traj_msg_available = False
    
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
    
    # ==================== 控制循环 ====================
    
    def _control_callback(self):
        """控制循环回调"""
        # 使用基类的核心控制逻辑
        cmd = self._control_loop_core()
        
        if cmd is not None:
            # 发布控制命令
            self._publish_cmd(cmd)
            
            # 发布调试路径
            trajectory = self._data_manager.get_latest_trajectory()
            if trajectory is not None:
                self._publishers.publish_debug_path(trajectory)
    
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
    
    # ==================== 生命周期 ====================
    
    def shutdown(self):
        """清理资源"""
        ControllerNodeBase.shutdown(self)


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
