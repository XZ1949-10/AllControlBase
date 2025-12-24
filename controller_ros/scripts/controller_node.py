#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
控制器 ROS1 节点
"""

# 注意：不要在这里修改 sys.path！
# PYTHONPATH 已经由 source devel/setup.bash 正确设置

import rospy
from typing import Dict, Any, Optional

from nav_msgs.msg import Odometry as RosOdometry
from sensor_msgs.msg import Imu as RosImu
from std_msgs.msg import Int32, Empty
from std_srvs.srv import Trigger, TriggerResponse

# 尝试导入自定义消息，记录可用性
_CUSTOM_MSGS_AVAILABLE = True
_CUSTOM_MSGS_ERROR = None
try:
    from controller_ros.msg import LocalTrajectoryV4, UnifiedCmd, DiagnosticsV2
except ImportError as e:
    _CUSTOM_MSGS_AVAILABLE = False
    _CUSTOM_MSGS_ERROR = str(e)
    # 创建占位符，避免后续代码报错
    LocalTrajectoryV4 = None
    UnifiedCmd = None
    DiagnosticsV2 = None

# 导入 universal_controller
from universal_controller.core.data_types import ControlOutput, AttitudeCommand
from universal_controller.core.enums import ControllerState

# 导入共享模块
from controller_ros.node.base_node import ControllerNodeBase
from controller_ros.io import DataManager
from controller_ros.adapters import OutputAdapter, AttitudeAdapter
from controller_ros.utils.diagnostics_publisher import fill_diagnostics_msg, DiagnosticsThrottler
from controller_ros.utils.ros_compat import get_time_sec
from controller_ros.utils.param_loader import ParamLoader
from controller_ros.bridge import TFBridge

# 默认话题名常量
DEFAULT_STATE_TOPIC = '/controller/state'
DEFAULT_EMERGENCY_STOP_TOPIC = '/controller/emergency_stop'
DEFAULT_ATTITUDE_CMD_TOPIC = '/controller/attitude_cmd'


class ControllerNodeROS1(ControllerNodeBase):
    """
    控制器主节点 (ROS1 Noetic)
    
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
        
        # 2. 初始化核心组件
        self._initialize()
        
        # 3. 创建输出适配器
        self._output_adapter = OutputAdapter(
            self._default_frame_id, 
            get_time_func=self._get_time
        )
        
        # 4. 创建姿态适配器 (四旋翼平台)
        if self._is_quadrotor:
            self._attitude_adapter = AttitudeAdapter(get_time_func=self._get_time)
        else:
            self._attitude_adapter = None
        
        # 5. 创建 TF2 桥接 (统一使用 TFBridge，ROS1 传 None)
        self._tf_bridge = TFBridge(node=None)
        
        # 6. 注入 TF2 到坐标变换器
        self._inject_tf2_to_controller()
        
        # 7. 创建 ROS1 接口
        self._create_ros_interfaces()
        
        # 8. 初始化诊断发布辅助器
        diag_publish_rate = self._params.get('diagnostics', {}).get('publish_rate', 10)
        self._diag_throttler = DiagnosticsThrottler(publish_rate=diag_publish_rate)
        
        # 9. 创建控制定时器
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
        self._create_subscriptions()
        self._create_publishers()
        self._create_services()
    
    def _create_subscriptions(self):
        """创建所有订阅"""
        odom_topic = self._topics.get('odom', '/odom')
        imu_topic = self._topics.get('imu', '')
        traj_topic = self._topics.get('trajectory', '/nn/local_trajectory')
        emergency_stop_topic = self._topics.get('emergency_stop', DEFAULT_EMERGENCY_STOP_TOPIC)
        
        self._odom_sub = rospy.Subscriber(
            odom_topic, RosOdometry, 
            self._odom_callback, queue_size=10
        )
        
        # IMU 订阅 - 仅在配置了 IMU topic 时创建
        if imu_topic:
            self._imu_sub = rospy.Subscriber(
                imu_topic, RosImu, 
                self._imu_callback, queue_size=10
            )
        else:
            self._imu_sub = None
            rospy.loginfo("IMU topic not configured, skipping IMU subscription")
        
        # 轨迹订阅 - 仅在消息类型可用时创建
        if self._traj_msg_available and LocalTrajectoryV4 is not None:
            self._traj_sub = rospy.Subscriber(
                traj_topic, LocalTrajectoryV4, 
                self._traj_callback, queue_size=10
            )
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
        
        rospy.loginfo(
            f"Subscribed to: odom={odom_topic}, imu={imu_topic}, "
            f"trajectory={traj_topic if self._traj_msg_available else 'N/A'}, "
            f"emergency_stop={emergency_stop_topic}"
        )
    
    def _create_publishers(self):
        """创建所有发布器"""
        cmd_topic = self._topics.get('cmd_unified', '/cmd_unified')
        diag_topic = self._topics.get('diagnostics', '/controller/diagnostics')
        state_topic = self._topics.get('state', DEFAULT_STATE_TOPIC)
        
        # 控制命令发布器 - 仅在消息类型可用时创建
        if self._traj_msg_available and UnifiedCmd is not None:
            self._cmd_pub = rospy.Publisher(cmd_topic, UnifiedCmd, queue_size=1)
        else:
            self._cmd_pub = None
            rospy.logerr(
                f"Cannot create command publisher '{cmd_topic}': "
                f"UnifiedCmd message type not available"
            )
        
        # 诊断发布器 - 仅在消息类型可用时创建
        if self._traj_msg_available and DiagnosticsV2 is not None:
            self._diag_pub = rospy.Publisher(diag_topic, DiagnosticsV2, queue_size=10)
        else:
            self._diag_pub = None
            rospy.logwarn(
                f"Cannot create diagnostics publisher '{diag_topic}': "
                f"DiagnosticsV2 message type not available"
            )
        
        # 状态发布器 (使用标准消息，始终可用)
        self._state_pub = rospy.Publisher(state_topic, Int32, queue_size=1)
        
        # 姿态命令发布器 (四旋翼平台)
        if self._is_quadrotor:
            attitude_topic = self._topics.get('attitude_cmd', DEFAULT_ATTITUDE_CMD_TOPIC)
            try:
                from controller_ros.msg import AttitudeCmd
                self._attitude_pub = rospy.Publisher(attitude_topic, AttitudeCmd, queue_size=1)
                rospy.loginfo(f"Attitude command publisher created: {attitude_topic}")
            except ImportError:
                self._attitude_pub = None
                rospy.logwarn("AttitudeCmd message not available, attitude publishing disabled")
        else:
            self._attitude_pub = None
        
        rospy.loginfo(
            f"Publishing to: cmd={cmd_topic if self._cmd_pub else 'N/A'}, "
            f"diagnostics={diag_topic if self._diag_pub else 'N/A'}, "
            f"state={state_topic}"
        )
    
    def _create_services(self):
        """创建所有服务"""
        # 重置服务
        self._reset_srv = rospy.Service(
            '/controller/reset', Trigger, self._handle_reset_service
        )
        rospy.loginfo("Created reset service: /controller/reset")
        
        # 获取诊断服务
        try:
            from controller_ros.srv import GetDiagnostics
            self._get_diag_srv = rospy.Service(
                '/controller/get_diagnostics', GetDiagnostics, 
                self._handle_get_diagnostics_service
            )
            rospy.loginfo("Created get_diagnostics service: /controller/get_diagnostics")
        except ImportError:
            self._get_diag_srv = None
            rospy.logwarn("GetDiagnostics service not available (srv not built)")
        
        # 设置状态服务
        try:
            from controller_ros.srv import SetControllerState
            self._set_state_srv = rospy.Service(
                '/controller/set_state', SetControllerState,
                self._handle_set_state_service
            )
            rospy.loginfo("Created set_state service: /controller/set_state")
        except ImportError:
            self._set_state_srv = None
            rospy.logwarn("SetControllerState service not available (srv not built)")
        
        # 四旋翼平台专用服务
        if self._is_quadrotor:
            self._create_quadrotor_services()
    
    def _create_quadrotor_services(self):
        """创建四旋翼平台专用服务"""
        # 设置悬停航向服务
        try:
            from controller_ros.srv import SetHoverYaw, SetHoverYawResponse
            self._set_hover_yaw_srv = rospy.Service(
                '/controller/set_hover_yaw', SetHoverYaw,
                self._handle_set_hover_yaw_service
            )
            rospy.loginfo("Created set_hover_yaw service: /controller/set_hover_yaw")
        except ImportError:
            self._set_hover_yaw_srv = None
            rospy.logwarn("SetHoverYaw service not available (srv not built)")
        
        # 获取姿态角速度限制服务
        try:
            from controller_ros.srv import GetAttitudeRateLimits, GetAttitudeRateLimitsResponse
            self._get_attitude_limits_srv = rospy.Service(
                '/controller/get_attitude_rate_limits', GetAttitudeRateLimits,
                self._handle_get_attitude_rate_limits_service
            )
            rospy.loginfo("Created get_attitude_rate_limits service: /controller/get_attitude_rate_limits")
        except ImportError:
            self._get_attitude_limits_srv = None
            rospy.logwarn("GetAttitudeRateLimits service not available (srv not built)")
    
    # ==================== 订阅回调 ====================
    
    def _odom_callback(self, msg: RosOdometry):
        """里程计回调"""
        self._data_manager.update_odom(msg)
    
    def _imu_callback(self, msg: RosImu):
        """IMU 回调"""
        self._data_manager.update_imu(msg)
    
    def _traj_callback(self, msg: LocalTrajectoryV4):
        """轨迹回调"""
        self._data_manager.update_trajectory(msg)
    
    def _emergency_stop_callback(self, msg: Empty):
        """紧急停止回调"""
        self._handle_emergency_stop()
    
    # ==================== 控制循环 ====================
    
    def _control_callback(self, event):
        """控制循环回调"""
        cmd = self._control_loop_core()
        
        if cmd is not None:
            self._publish_cmd(cmd)
    
    # ==================== 服务处理 ====================
    
    def _handle_reset_service(self, req):
        """处理重置服务请求"""
        try:
            self._handle_reset()
            return TriggerResponse(success=True, message="Controller reset successfully")
        except Exception as e:
            return TriggerResponse(success=False, message=f"Reset failed: {e}")
    
    def _handle_get_diagnostics_service(self, req):
        """处理获取诊断服务请求"""
        from controller_ros.srv import GetDiagnosticsResponse
        
        response = GetDiagnosticsResponse()
        try:
            diag = self._handle_get_diagnostics()
            if diag:
                response.success = True
                response.message = "Diagnostics retrieved successfully"
                fill_diagnostics_msg(
                    response.diagnostics, diag,
                    get_time_func=lambda: rospy.Time.now()
                )
            else:
                response.success = False
                response.message = "Diagnostics not available"
        except Exception as e:
            response.success = False
            response.message = f"Get diagnostics failed: {e}"
            rospy.logerr(f"Get diagnostics failed: {e}")
        return response
    
    def _handle_set_state_service(self, req):
        """处理设置状态服务请求"""
        from controller_ros.srv import SetControllerStateResponse
        
        response = SetControllerStateResponse()
        try:
            success = self._handle_set_state(req.target_state)
            response.success = success
            if success:
                response.message = f"State change to {req.target_state} requested"
            else:
                response.message = f"Failed to request state change to {req.target_state}"
        except Exception as e:
            response.success = False
            response.message = f"Set state failed: {e}"
            rospy.logerr(f"Set state failed: {e}")
        return response
    
    def _handle_set_hover_yaw_service(self, req):
        """处理设置悬停航向服务请求"""
        from controller_ros.srv import SetHoverYawResponse
        
        response = SetHoverYawResponse()
        try:
            success = self._handle_set_hover_yaw(req.yaw)
            response.success = success
            if success:
                response.message = f"Hover yaw set to {req.yaw:.3f} rad"
            else:
                response.message = "Failed to set hover yaw"
        except Exception as e:
            response.success = False
            response.message = f"Set hover yaw failed: {e}"
            rospy.logerr(f"Set hover yaw failed: {e}")
        return response
    
    def _handle_get_attitude_rate_limits_service(self, req):
        """处理获取姿态角速度限制服务请求"""
        from controller_ros.srv import GetAttitudeRateLimitsResponse
        
        response = GetAttitudeRateLimitsResponse()
        try:
            limits = self._handle_get_attitude_rate_limits()
            if limits:
                response.success = True
                response.message = "Attitude rate limits retrieved"
                response.roll_rate_max = limits.get('roll_rate_max', 0.0)
                response.pitch_rate_max = limits.get('pitch_rate_max', 0.0)
                response.yaw_rate_max = limits.get('yaw_rate_max', 0.0)
            else:
                response.success = False
                response.message = "Attitude rate limits not available"
        except Exception as e:
            response.success = False
            response.message = f"Get attitude rate limits failed: {e}"
            rospy.logerr(f"Get attitude rate limits failed: {e}")
        return response
    
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
        if self._cmd_pub is None:
            return
        ros_msg = self._output_adapter.to_ros(cmd)
        self._cmd_pub.publish(ros_msg)
    
    def _publish_stop_cmd(self):
        """发布停止命令"""
        if self._cmd_pub is None:
            return
        ros_msg = self._output_adapter.create_stop_cmd()
        self._cmd_pub.publish(ros_msg)
    
    def _publish_diagnostics(self, diag: Dict[str, Any], force: bool = False):
        """发布诊断信息"""
        # 始终发布状态话题
        current_state = diag.get('state', 0)
        state_msg = Int32()
        state_msg.data = current_state
        self._state_pub.publish(state_msg)
        
        # 使用节流器判断是否发布诊断
        if not self._diag_throttler.should_publish(diag, force=force):
            return
        
        # 仅在诊断发布器可用且消息类型可用时发布
        if self._diag_pub is None:
            return
        
        # 检查 DiagnosticsV2 消息类型是否可用
        if DiagnosticsV2 is None:
            return
        
        try:
            msg = DiagnosticsV2()
            fill_diagnostics_msg(msg, diag, get_time_func=lambda: rospy.Time.now())
            self._diag_pub.publish(msg)
        except Exception as e:
            # 只在首次失败时记录错误，避免日志泛滥
            rospy.logwarn_throttle(10.0, f"Failed to publish diagnostics: {e}")
    
    def _publish_attitude_cmd(self, attitude_cmd: AttitudeCommand):
        """发布姿态命令 (四旋翼平台)"""
        if self._attitude_pub is None or self._attitude_adapter is None:
            return
        
        # 获取悬停状态
        is_hovering = False
        if self._controller_bridge and self._controller_bridge.manager:
            attitude_controller = self._controller_bridge.manager.attitude_controller
            if attitude_controller and hasattr(attitude_controller, '_is_hovering'):
                is_hovering = attitude_controller._is_hovering
        
        ros_msg = self._attitude_adapter.to_ros(
            attitude_cmd, 
            yaw_mode=self._attitude_yaw_mode,
            is_hovering=is_hovering
        )
        self._attitude_pub.publish(ros_msg)
    
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
