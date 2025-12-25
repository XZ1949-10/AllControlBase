#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
控制器 ROS1 节点

使用与 ROS2 对齐的架构：
- ROS1PublisherManager 管理发布
- ROS1ServiceManager 管理服务
- ControllerNodeBase 提供共享逻辑
"""

# 注意：不要在这里修改 sys.path！
# PYTHONPATH 已经由 source devel/setup.bash 正确设置

import rospy
from typing import Dict, Any, Optional

from nav_msgs.msg import Odometry as RosOdometry
from sensor_msgs.msg import Imu as RosImu
from std_msgs.msg import Empty

# 尝试导入自定义消息，记录可用性
_CUSTOM_MSGS_AVAILABLE = True
_CUSTOM_MSGS_ERROR = None
try:
    from controller_ros.msg import LocalTrajectoryV4
except ImportError as e:
    _CUSTOM_MSGS_AVAILABLE = False
    _CUSTOM_MSGS_ERROR = str(e)
    LocalTrajectoryV4 = None

# 导入 universal_controller
from universal_controller.core.data_types import ControlOutput, AttitudeCommand

# 导入共享模块
from controller_ros.node.base_node import ControllerNodeBase
from controller_ros.io import ROS1PublisherManager, ROS1ServiceManager
from controller_ros.utils.ros_compat import get_time_sec
from controller_ros.utils.param_loader import ParamLoader
from controller_ros.bridge import TFBridge

# 默认话题名常量
DEFAULT_EMERGENCY_STOP_TOPIC = '/controller/emergency_stop'


class ControllerNodeROS1(ControllerNodeBase):
    """
    控制器主节点 (ROS1 Noetic)
    
    使用与 ROS2 对齐的架构，通过 Manager 类管理发布和服务。
    
    输入话题:
    - /odom (nav_msgs/Odometry)
    - /imu (sensor_msgs/Imu)
    - /nn/local_trajectory (LocalTrajectoryV4)
    - /controller/emergency_stop (std_msgs/Empty)
    
    输出话题:
    - /cmd_unified (UnifiedCmd)
    - /controller/diagnostics (DiagnosticsV2)
    - /controller/state (std_msgs/Int32)
    - /controller/attitude_cmd (AttitudeCmd, 仅四旋翼)
    
    服务:
    - /controller/reset (std_srvs/Trigger)
    - /controller/set_state (SetControllerState)
    - /controller/get_diagnostics (GetDiagnostics)
    - /controller/set_hover_yaw (SetHoverYaw, 仅四旋翼)
    - /controller/get_attitude_rate_limits (GetAttitudeRateLimits, 仅四旋翼)
    """
    
    def __init__(self):
        # 先初始化 ROS1 节点
        rospy.init_node('universal_controller_node')
        
        # 再初始化基类
        super().__init__()
        
        # 0. 检查自定义消息是否可用
        if not _CUSTOM_MSGS_AVAILABLE:
            rospy.logfatal(
                f"Custom messages not available! Error: {_CUSTOM_MSGS_ERROR}. "
                f"Please build the package with 'catkin_make' or 'catkin build'."
            )
            self._traj_msg_available = False
        else:
            self._traj_msg_available = True
        
        # 1. 加载参数
        self._params = ParamLoader.load(None)
        self._topics = ParamLoader.get_topics(None)
        
        # 2. 初始化核心组件 (基类方法)
        self._initialize()
        
        # 3. 创建 TF2 桥接
        self._tf_bridge = TFBridge(node=None)
        
        # 4. 注入 TF2 到坐标变换器 (基类方法)
        self._inject_tf2_to_controller()
        
        # 5. 创建 ROS1 接口 (使用 Manager 类)
        self._create_ros_interfaces()
        
        # 6. 创建控制定时器
        control_rate = self._params.get('system', {}).get('ctrl_freq', 50)
        control_period = 1.0 / control_rate
        self._timer = rospy.Timer(rospy.Duration(control_period), self._control_callback)
        
        rospy.loginfo(
            f"Controller node initialized (platform={self._platform_type}, "
            f"rate={control_rate}Hz, tf2={self._tf_bridge.is_initialized}, "
            f"quadrotor={self._is_quadrotor}, msgs_available={self._traj_msg_available})"
        )
    
    def _create_ros_interfaces(self):
        """创建 ROS1 特定的接口"""
        # 创建订阅
        self._create_subscriptions()
        
        # 创建发布管理器
        diag_publish_rate = self._params.get('diagnostics', {}).get('publish_rate', 10)
        self._publishers = ROS1PublisherManager(
            topics=self._topics,
            default_frame_id=self._default_frame_id,
            diag_publish_rate=diag_publish_rate,
            get_time_func=self._get_time,
            is_quadrotor=self._is_quadrotor
        )
        
        # 创建服务管理器
        self._services = ROS1ServiceManager(
            reset_callback=self._handle_reset,
            get_diagnostics_callback=self._handle_get_diagnostics,
            set_state_callback=self._handle_set_state,
            set_hover_yaw_callback=self._handle_set_hover_yaw if self._is_quadrotor else None,
            get_attitude_rate_limits_callback=self._handle_get_attitude_rate_limits if self._is_quadrotor else None,
            is_quadrotor=self._is_quadrotor
        )
    
    def _create_subscriptions(self):
        """创建所有订阅"""
        odom_topic = self._topics.get('odom', '/odom')
        imu_topic = self._topics.get('imu', '')
        traj_topic = self._topics.get('trajectory', '/nn/local_trajectory')
        emergency_stop_topic = self._topics.get('emergency_stop', DEFAULT_EMERGENCY_STOP_TOPIC)
        
        # 里程计订阅
        self._odom_sub = rospy.Subscriber(
            odom_topic, RosOdometry,
            self._odom_callback, queue_size=10
        )
        rospy.loginfo(f"Subscribed to odom: {odom_topic}")
        
        # IMU 订阅 - 仅在配置了 IMU topic 时创建
        if imu_topic:
            self._imu_sub = rospy.Subscriber(
                imu_topic, RosImu,
                self._imu_callback, queue_size=10
            )
            rospy.loginfo(f"Subscribed to imu: {imu_topic}")
        else:
            self._imu_sub = None
            rospy.loginfo("IMU topic not configured, skipping IMU subscription")
        
        # 轨迹订阅 - 仅在消息类型可用时创建
        if self._traj_msg_available and LocalTrajectoryV4 is not None:
            self._traj_sub = rospy.Subscriber(
                traj_topic, LocalTrajectoryV4,
                self._traj_callback, queue_size=10
            )
            rospy.loginfo(f"Subscribed to trajectory: {traj_topic}")
        else:
            self._traj_sub = None
            rospy.logerr(
                f"Cannot subscribe to trajectory topic '{traj_topic}': "
                f"LocalTrajectoryV4 message type not available"
            )
        
        # 紧急停止话题订阅
        self._emergency_stop_sub = rospy.Subscriber(
            emergency_stop_topic, Empty,
            self._emergency_stop_callback, queue_size=1
        )
        rospy.loginfo(f"Subscribed to emergency_stop: {emergency_stop_topic}")
    
    # ==================== 订阅回调 ====================
    
    def _odom_callback(self, msg: RosOdometry):
        """里程计回调"""
        self._data_manager.update_odom(msg)
        self._notify_odom_received()
    
    def _imu_callback(self, msg: RosImu):
        """IMU 回调"""
        self._data_manager.update_imu(msg)
        self._notify_imu_received()
    
    def _traj_callback(self, msg):
        """轨迹回调"""
        self._data_manager.update_trajectory(msg)
        self._notify_trajectory_received()
    
    def _emergency_stop_callback(self, msg: Empty):
        """紧急停止回调"""
        self._handle_emergency_stop()
    
    # ==================== 控制循环 ====================
    
    def _control_callback(self, event):
        """控制循环回调"""
        cmd = self._control_loop_core()
        
        if cmd is not None:
            self._publish_cmd(cmd)
    
    # ==================== 基类抽象方法实现 ====================
    
    def _get_time(self) -> float:
        """获取当前 ROS 时间（秒）"""
        return get_time_sec(None)
    
    def _log_info(self, msg: str):
        """记录信息日志"""
        rospy.loginfo(msg)
    
    def _log_warn(self, msg: str):
        """记录警告日志"""
        rospy.logwarn(msg)
    
    def _log_warn_throttle(self, period: float, msg: str):
        """记录节流警告日志"""
        rospy.logwarn_throttle(period, msg)
    
    def _log_error(self, msg: str):
        """记录错误日志"""
        rospy.logerr(msg)
    
    def _publish_cmd(self, cmd: ControlOutput):
        """发布控制命令"""
        self._publishers.publish_cmd(cmd)
    
    def _publish_stop_cmd(self):
        """发布停止命令"""
        self._publishers.publish_stop_cmd()
    
    def _publish_diagnostics(self, diag: Dict[str, Any], force: bool = False):
        """发布诊断信息"""
        self._publishers.publish_diagnostics(diag, force=force)
    
    def _publish_attitude_cmd(self, attitude_cmd: AttitudeCommand):
        """发布姿态命令 (四旋翼平台)"""
        if not self._is_quadrotor:
            return
        
        # 获取悬停状态
        is_hovering = False
        if self._controller_bridge and self._controller_bridge.manager:
            attitude_controller = self._controller_bridge.manager.attitude_controller
            if attitude_controller and hasattr(attitude_controller, '_is_hovering'):
                is_hovering = attitude_controller._is_hovering
        
        self._publishers.publish_attitude_cmd(
            attitude_cmd,
            yaw_mode=self._attitude_yaw_mode,
            is_hovering=is_hovering
        )
    
    # ==================== 生命周期 ====================
    
    def shutdown(self):
        """关闭节点"""
        if hasattr(self, '_timer') and self._timer is not None:
            self._timer.shutdown()
            self._timer = None
        
        super().shutdown()


# 保持向后兼容的类名
ControllerNode = ControllerNodeROS1


def main():
    """主入口"""
    try:
        node = ControllerNodeROS1()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        if 'node' in locals():
            node.shutdown()


if __name__ == '__main__':
    main()
