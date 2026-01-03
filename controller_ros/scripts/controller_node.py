#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
控制器 ROS1 节点

使用与 ROS2 对齐的架构：
- ROS1PublisherManager 管理发布
- ROS1ServiceManager 管理服务
- ControllerNodeBase 提供共享逻辑
"""

import rospy
from typing import Dict, Any, Optional

from nav_msgs.msg import Odometry as RosOdometry
from sensor_msgs.msg import Imu as RosImu
from std_msgs.msg import Empty

# 统一的消息可用性检查
from controller_ros.utils.msg_availability import (
    CUSTOM_MSGS_AVAILABLE, 
    get_msg_import_error,
    LocalTrajectoryV4,
)

# 导入 universal_controller
from universal_controller.core.data_types import ControlOutput, AttitudeCommand

# 导入共享模块
from controller_ros.node.base_node import ControllerNodeBase
from controller_ros.io import ROS1PublisherManager, ROS1ServiceManager
from controller_ros.utils.ros_compat import get_time_sec
from controller_ros.utils.param_loader import ParamLoader, TOPICS_DEFAULTS
from controller_ros.bridge import TFBridge


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
        
        # 再初始化基类（基类会初始化 _shutting_down 标志）
        super().__init__()
        
        # 0. 检查自定义消息是否可用
        if not CUSTOM_MSGS_AVAILABLE:
            rospy.logfatal(
                f"Custom messages not available! Error: {get_msg_import_error()}. "
                f"Please build the package with 'catkin_make' or 'catkin build'. "
                f"Node will exit."
            )
            rospy.sleep(2.0)
            raise RuntimeError(
                f"Cannot start controller: custom messages not available. "
                f"Build the package first."
            )
        
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
        odom_topic = self._topics.get('odom', TOPICS_DEFAULTS['odom'])
        imu_topic = self._topics.get('imu', '')
        traj_topic = self._topics.get('trajectory', TOPICS_DEFAULTS['trajectory'])
        emergency_stop_topic = self._topics.get('emergency_stop', TOPICS_DEFAULTS['emergency_stop'])
        
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
    
    def _imu_callback(self, msg: RosImu):
        """IMU 回调"""
        self._data_manager.update_imu(msg)
    
    def _traj_callback(self, msg):
        """轨迹回调"""
        self._data_manager.update_trajectory(msg)
    
    def _emergency_stop_callback(self, msg: Empty):
        """紧急停止回调"""
        self._handle_emergency_stop()
    
    # ==================== 控制循环 ====================
    
    def _control_callback(self, event):
        """控制循环回调"""
        # 关闭检查已在基类 _control_loop_core() 中统一处理
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
    

    
    def _publish_debug_path(self, trajectory):
        """发布调试路径 (用于 RViz 可视化)"""
        self._publishers.publish_debug_path(trajectory)
    
    # ==================== 生命周期 ====================
    
    def shutdown(self):
        """
        关闭节点
        
        完整的资源清理流程:
        1. 设置关闭标志，阻止控制循环继续执行
        2. 停止定时器
        3. 发送最终停止命令
        4. 关闭订阅器
        5. 关闭发布器和服务
        6. 调用基类关闭
        """
        # 1. 设置关闭标志 (基类会设置，但这里提前设置以阻止回调)
        # 使用 Event.set() 确保线程安全
        self._shutting_down.set()
        
        # 2. 停止定时器
        if self._timer is not None:
            self._timer.shutdown()
            self._timer = None
        
        # 3. 发送最终停止命令
        try:
            if self._publishers is not None:
                self._publishers.publish_stop_cmd()
                rospy.loginfo("Final stop command sent")
        except Exception as e:
            rospy.logwarn(f"Failed to send final stop command: {e}")
        
        # 4. 关闭订阅器
        if self._odom_sub is not None:
            self._odom_sub.unregister()
            self._odom_sub = None
        
        if self._imu_sub is not None:
            self._imu_sub.unregister()
            self._imu_sub = None
        
        if self._traj_sub is not None:
            self._traj_sub.unregister()
            self._traj_sub = None
        
        if self._emergency_stop_sub is not None:
            self._emergency_stop_sub.unregister()
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
        
        # 7. 调用基类关闭
        super().shutdown()
        
        rospy.loginfo("Controller node shutdown complete")


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
