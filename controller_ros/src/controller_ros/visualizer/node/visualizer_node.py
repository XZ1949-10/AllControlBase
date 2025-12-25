"""
可视化器 ROS 节点

管理 ROS 订阅/发布和 GUI 的交互。
支持 ROS1 (rospy) 和 ROS2 (rclpy)。

生命周期说明：
- initialize(): 创建订阅和发布（在构造时自动调用）
- shutdown(): 停止 ROS 线程，关闭 GUI
- reset(): 重置数据聚合器状态
"""
from typing import Dict, Any, Optional
import threading
import sys
import logging

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy, Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, Bool

# 尝试导入 controller_ros 消息（在 sys.path 被修改之前可能已经导入）
# 这些消息需要从 devel/lib/python3/dist-packages 导入
try:
    from controller_ros.msg import LocalTrajectoryV4
    _LocalTrajectoryV4 = LocalTrajectoryV4
except ImportError:
    _LocalTrajectoryV4 = None

try:
    from controller_ros.msg import UnifiedCmd
    _UnifiedCmd = UnifiedCmd
except ImportError:
    _UnifiedCmd = None

try:
    from controller_ros.msg import DiagnosticsV2
    _DiagnosticsV2 = DiagnosticsV2
except ImportError:
    _DiagnosticsV2 = None

from ..models import (
    VelocityData, JoystickState, ControllerStatus, 
    ControlMode, TrajectoryMode
)
from ..adapters import JoyAdapter, ImageAdapter, VelocityAdapter
from ..handlers import JoystickHandler, DataAggregator, JoystickConfig
from ..main_window import VisualizerMainWindow
from .ros_bridge import create_ros_bridge, ROS_VERSION

logger = logging.getLogger(__name__)


class VisualizerNode:
    """
    可视化器 ROS 节点
    
    职责:
    - 订阅传感器和控制数据
    - 发布手柄控制命令
    - 管理 GUI 生命周期
    - 桥接 ROS 回调和 GUI 更新
    
    生命周期:
    - __init__(): 初始化节点，创建订阅和发布
    - start_gui(): 启动 GUI（阻塞直到关闭）
    - shutdown(): 停止 ROS 线程，关闭 GUI
    - reset(): 重置数据聚合器状态
    
    支持 ROS1 和 ROS2。
    """
    
    def __init__(self, config: Dict[str, Any] = None):
        # ROS 桥接
        self._ros = create_ros_bridge()
        # 只在未初始化时初始化 (可能已在 load_config_from_ros 中初始化)
        if ROS_VERSION == 1:
            import rospy
            if not rospy.core.is_initialized():
                self._ros.init('turtlebot_visualizer')
        else:
            self._ros.init('turtlebot_visualizer')
        
        self._config = config or self._load_default_config()
        
        # 话题配置
        topics = self._config.get('topics', {})
        self._odom_topic = topics.get('odom', '/odom')
        self._traj_topic = topics.get('trajectory', '/nn/local_trajectory')
        self._cmd_topic = topics.get('cmd_unified', '/cmd_unified')
        self._diag_topic = topics.get('diagnostics', '/controller/diagnostics')
        self._camera_topic = topics.get('camera_image', '')
        self._joy_topic = topics.get('joy', '/joy')
        self._cmd_vel_topic = topics.get('cmd_vel_output', '/cmd_vel')
        self._mode_topic = topics.get('control_mode', '/visualizer/control_mode')
        self._estop_topic = topics.get('emergency_stop', '/controller/emergency_stop')
        
        # 相机标定配置
        camera_config = self._config.get('camera', {})
        self._homography_file = camera_config.get('calibration_file', '')

        # 适配器
        joy_config = self._config.get('joystick', {})
        self._joy_adapter = JoyAdapter(joy_config)
        self._image_adapter = ImageAdapter()
        self._velocity_adapter = VelocityAdapter()
        
        # 处理器
        self._data_aggregator = DataAggregator(
            velocity_history_sec=self._config.get('display', {}).get('velocity_history_sec', 10)
        )
        self._joystick_handler = JoystickHandler(
            JoystickConfig.from_dict(joy_config)
        )
        
        # 设置手柄回调
        self._joystick_handler.set_mode_change_callback(self._on_mode_change)
        self._joystick_handler.set_cmd_callback(self._on_joystick_cmd)
        self._joystick_handler.set_estop_callback(self._publish_emergency_stop)
        self._joystick_handler.set_resume_callback(self._publish_resume)
        
        # GUI (延迟初始化)
        self._window: Optional[VisualizerMainWindow] = None
        self._ros_thread: Optional[threading.Thread] = None
        self._running = False
        self._shutting_down = False
        
        # 发布器
        self._cmd_vel_pub = None
        self._mode_pub = None
        self._estop_pub = None
        
        # 创建订阅和发布
        self._create_subscriptions()
        self._create_publishers()
        
        self._ros.log_info(f"Visualizer node initialized (ROS{ROS_VERSION})")
        self._ros.log_info(f"  Odom topic: {self._odom_topic}")
        self._ros.log_info(f"  Trajectory topic: {self._traj_topic}")
        self._ros.log_info(f"  Joy topic: {self._joy_topic}")
    
    def shutdown(self):
        """关闭可视化器节点"""
        if self._shutting_down:
            return
        self._shutting_down = True
        
        self._ros.log_info("Visualizer node shutting down...")
        
        # 停止 ROS 线程
        self._running = False
        if self._ros_thread is not None and self._ros_thread.is_alive():
            self._ros_thread.join(timeout=1.0)
        
        # 关闭 ROS
        self._ros.shutdown()
        
        self._ros.log_info("Visualizer node shutdown complete")
    
    def reset(self):
        """重置可视化器状态"""
        self._data_aggregator.reset()
        self._joystick_handler.reset()
        self._ros.log_info("Visualizer node reset")
    
    def _load_default_config(self) -> Dict[str, Any]:
        """加载默认配置"""
        return {
            'topics': {
                'odom': '/odom',
                'trajectory': '/nn/local_trajectory',
                'cmd_unified': '/cmd_unified',
                'diagnostics': '/controller/diagnostics',
                'camera_image': '',
                'joy': '/joy',
                'cmd_vel_output': '/joy_cmd_vel',  # 发到单独话题，由 cmd_vel_adapter 选择
                'control_mode': '/visualizer/control_mode',
                'emergency_stop': '/controller/emergency_stop',
            },
            'display': {
                'update_rate': 30,
                'velocity_history_sec': 10,
            },
            'joystick': {
                'enable_button': 4,
                'estop_button': 5,
                'resume_button': 7,
                'linear_axis': 1,
                'angular_axis': 3,
                'max_linear': 0.5,
                'max_angular': 1.0,
                'deadzone': 0.1,
            },
            'constraints': {
                'v_max': 0.5,
                'omega_max': 1.0,
            },
            'camera': {
                'use_camera': False,
                'calibration_file': '',  # 单应性标定文件路径
            },
        }

    def _create_subscriptions(self):
        """创建订阅"""
        # 里程计
        self._ros.create_subscriber(
            Odometry, self._odom_topic,
            self._odom_callback, 10
        )
        
        # 轨迹
        if _LocalTrajectoryV4 is not None:
            self._ros.create_subscriber(
                _LocalTrajectoryV4, self._traj_topic,
                self._traj_callback, 10
            )
            self._traj_available = True
        else:
            self._ros.log_warn("LocalTrajectoryV4 not available")
            self._traj_available = False
        
        # 控制命令
        if _UnifiedCmd is not None:
            self._ros.create_subscriber(
                _UnifiedCmd, self._cmd_topic,
                self._cmd_callback, 10
            )
        else:
            self._ros.log_warn("UnifiedCmd not available")
        
        # 诊断
        if _DiagnosticsV2 is not None:
            self._ros.create_subscriber(
                _DiagnosticsV2, self._diag_topic,
                self._diag_callback, 10
            )
        else:
            self._ros.log_warn("DiagnosticsV2 not available")
        
        # 手柄
        self._ros.create_subscriber(
            Joy, self._joy_topic,
            self._joy_callback, 10
        )
        
        # 相机 (可选)
        if self._camera_topic:
            self._ros.create_subscriber(
                Image, self._camera_topic,
                self._image_callback, 1
            )
            self._ros.log_info(f"  Camera topic: {self._camera_topic}")
            self._ros.log_info(f"  Homography file: {self._homography_file}")
        else:
            self._ros.log_info("  Camera mode: disabled (no camera_image topic)")
    
    def _create_publishers(self):
        """创建发布器"""
        self._cmd_vel_pub = self._ros.create_publisher(Twist, self._cmd_vel_topic, 10)
        self._mode_pub = self._ros.create_publisher(Bool, self._mode_topic, 10)
        self._estop_pub = self._ros.create_publisher(Empty, self._estop_topic, 1)

    # ==================== 订阅回调 ====================
    
    def _odom_callback(self, msg: Odometry):
        """里程计回调"""
        timestamp = self._ros.get_time()
        
        position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        )
        orientation = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )
        velocity = self._velocity_adapter.from_odom(msg, timestamp)
        self._data_aggregator.update_odom(position, orientation, velocity, timestamp)
    
    def _traj_callback(self, msg):
        """轨迹回调"""
        timestamp = self._ros.get_time()
        points = [(p.x, p.y, p.z) for p in msg.points]
        
        self._data_aggregator.update_trajectory(
            points=points,
            mode=msg.mode,
            confidence=msg.confidence,
            dt_sec=msg.dt_sec,
            frame_id=msg.header.frame_id or 'base_link',
            timestamp=timestamp,
        )
    
    def _cmd_callback(self, msg):
        """控制命令回调"""
        timestamp = self._ros.get_time()
        velocity = self._velocity_adapter.from_unified_cmd(msg, timestamp)
        self._data_aggregator.update_target_velocity(velocity)
    
    def _diag_callback(self, msg):
        """诊断回调"""
        status = ControllerStatus(
            state=msg.state,
            state_name=self._get_state_name(msg.state),
            mpc_success=msg.mpc_success,
            mpc_solve_time_ms=msg.mpc_solve_time_ms,
            backup_active=msg.backup_active,
            odom_timeout=msg.timeout_odom,
            traj_timeout=msg.timeout_traj,
            emergency_stop=msg.emergency_stop,
            error_message=msg.error_message,
        )
        self._data_aggregator.update_controller_status(status)
    
    def _joy_callback(self, msg: Joy):
        """手柄回调"""
        state = self._joy_adapter.to_model(msg)
        self._data_aggregator.update_joystick(state)
        
        # 处理手柄输入
        cmd = self._joystick_handler.update(state)
        
        # 如果在手柄模式，发布命令
        if cmd is not None:
            self._publish_cmd_vel(cmd)
    
    def _image_callback(self, msg: Image):
        """相机图像回调"""
        image = self._image_adapter.to_model(msg)
        if image is not None:
            self._data_aggregator.update_camera_image(image)
            # 只在第一次收到图像时记录
            if not hasattr(self, '_image_logged'):
                self._ros.log_info(f"Camera image received: {msg.width}x{msg.height}, encoding={msg.encoding}")
                self._image_logged = True
        else:
            if not hasattr(self, '_image_error_logged'):
                self._ros.log_warn(f"Failed to convert camera image: encoding={msg.encoding}")
                self._image_error_logged = True

    # ==================== 处理器回调 ====================
    
    def _on_mode_change(self, mode: ControlMode):
        """控制模式切换回调"""
        self._data_aggregator.update_control_mode(mode)
        
        # 发布模式
        msg = Bool()
        msg.data = (mode == ControlMode.JOYSTICK)
        self._ros.publish(self._mode_pub, msg)
        
        self._ros.log_info(f"Control mode: {mode.name}")
    
    def _on_joystick_cmd(self, velocity: VelocityData):
        """手柄命令生成回调"""
        self._publish_cmd_vel(velocity)
    
    # ==================== 发布方法 ====================
    
    def _publish_cmd_vel(self, velocity: VelocityData):
        """发布速度命令"""
        msg = Twist()
        msg.linear.x = velocity.linear_x
        msg.linear.y = velocity.linear_y
        msg.angular.z = velocity.angular_z
        self._ros.publish(self._cmd_vel_pub, msg)
    
    def _publish_emergency_stop(self):
        """发布紧急停止"""
        self._ros.publish(self._estop_pub, Empty())
        self._ros.log_warn("Emergency stop published!")
    
    def _publish_resume(self):
        """
        发布恢复控制请求
        
        通过调用 /controller/set_state 服务，将状态设置为 NORMAL (1)
        """
        try:
            from controller_ros.srv import SetState
            
            # 创建服务客户端
            service_name = '/controller/set_state'
            
            if ROS_VERSION == 1:
                import rospy
                rospy.wait_for_service(service_name, timeout=1.0)
                set_state = rospy.ServiceProxy(service_name, SetState)
                response = set_state(1)  # NORMAL = 1
                if response.success:
                    self._ros.log_info("Resume control: success")
                else:
                    self._ros.log_warn(f"Resume control failed: {response.message}")
            else:
                # ROS2 服务调用
                self._ros.log_warn("Resume via service not implemented for ROS2 yet")
                
        except Exception as e:
            self._ros.log_error(f"Failed to call set_state service: {e}")
    
    # ==================== GUI 管理 ====================
    
    def start_gui(self):
        """启动 GUI (在主线程中调用)"""
        from PyQt5.QtWidgets import QApplication
        from PyQt5.QtCore import QTimer
        
        # 创建 Qt 应用
        app = QApplication(sys.argv)
        
        # 创建主窗口
        self._window = VisualizerMainWindow(
            self._data_aggregator,
            self._config
        )
        self._window.emergency_stop_requested.connect(self._publish_emergency_stop)
        self._window.resume_requested.connect(self._publish_resume)
        
        # 加载单应性标定 (如果配置了)
        if self._homography_file:
            if self._window.load_homography_calibration(self._homography_file):
                self._ros.log_info(f"Homography calibration loaded: {self._homography_file}")
            else:
                self._ros.log_warn(f"Failed to load homography calibration: {self._homography_file}")
        
        self._window.show()
        
        # 启动 ROS 线程
        self._running = True
        self._ros_thread = threading.Thread(target=self._ros_spin, daemon=True)
        self._ros_thread.start()
        
        # 创建定时器检查 ROS 关闭状态，确保 Qt 能响应 Ctrl+C
        def check_ros_shutdown():
            if self._ros.is_shutdown():
                self._ros.log_info("ROS shutdown detected, closing GUI...")
                app.quit()
        
        shutdown_timer = QTimer()
        shutdown_timer.timeout.connect(check_ros_shutdown)
        shutdown_timer.start(100)  # 每 100ms 检查一次
        
        # 运行 Qt 事件循环
        exit_code = app.exec_()
        
        # 清理
        self._running = False
        shutdown_timer.stop()
        self._ros.shutdown()
        
        return exit_code
    
    def _ros_spin(self):
        """ROS 事件循环 (在单独线程中运行)"""
        try:
            while self._running and not self._ros.is_shutdown():
                self._ros.spin_once(timeout=0.1)
        except Exception as e:
            logger.error(f"ROS spin error: {e}")
    
    # ==================== 工具方法 ====================
    
    @staticmethod
    def _get_state_name(state: int) -> str:
        """获取状态名称"""
        names = {
            0: 'INIT',
            1: 'NORMAL',
            2: 'SOFT_DISABLED',
            3: 'MPC_DEGRADED',
            4: 'BACKUP_ACTIVE',
            5: 'STOPPING',
            6: 'STOPPED',
        }
        return names.get(state, f'UNKNOWN({state})')
