#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
控制器 ROS1 节点

主节点实现，组装所有组件，管理控制循环。
支持 TF2 坐标变换集成。

继承 ControllerNodeBase，实现 ROS1 特定的接口。
复用共享模块，避免代码重复：
- 使用 ControllerNodeBase 的控制循环核心逻辑
- 使用 DataManager 管理数据缓存
- 使用 ParamLoader 加载参数
- 使用 adapters 模块进行数据转换
- 使用 diagnostics_publisher 进行诊断消息填充
- 使用 ros_compat 的 TF2Compat 类
"""
import sys
import os

# 添加 src 目录到路径（开发模式和 catkin 安装模式兼容）
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

import rospy
from typing import Dict, Any, Optional

from nav_msgs.msg import Odometry as RosOdometry
from sensor_msgs.msg import Imu as RosImu
from std_msgs.msg import Int32
from std_srvs.srv import Trigger, TriggerResponse

# 导入自定义消息
from controller_ros.msg import LocalTrajectoryV4, UnifiedCmd, DiagnosticsV2

# 导入 universal_controller
from universal_controller.core.data_types import ControlOutput
from universal_controller.core.enums import ControllerState

# 导入共享模块
from controller_ros.node.base_node import ControllerNodeBase
from controller_ros.io import DataManager
from controller_ros.adapters import OutputAdapter
from controller_ros.utils.diagnostics_publisher import fill_diagnostics_msg, DiagnosticsPublishHelper
from controller_ros.utils.ros_compat import TF2Compat, get_time_sec
from controller_ros.utils.param_loader import ParamLoader


class ControllerNodeROS1(ControllerNodeBase):
    """
    控制器主节点 (ROS1 Noetic)
    
    继承 ControllerNodeBase 获取共享逻辑，
    实现 ROS1 特定的接口。
    
    输出:
    - /cmd_unified (UnifiedCmd)
    - /controller/diagnostics (DiagnosticsV2)
    - /controller/state (Int32)
    """
    
    def __init__(self):
        # 先初始化 ROS1 节点
        rospy.init_node('universal_controller_node')
        
        # 再初始化基类
        super().__init__()
        
        # 1. 加载参数（使用统一的 ParamLoader）
        self._params = ParamLoader.load(None)  # ROS1 传 None
        self._topics = ParamLoader.get_topics(None)
        
        # 2. 初始化核心组件（基类方法）
        self._initialize()
        
        # 3. 创建输出适配器
        self._output_adapter = OutputAdapter(
            self._default_frame_id, 
            get_time_func=self._get_time
        )
        
        # 4. 创建 TF2 桥接
        self._tf_bridge = TF2Compat(node=None)
        
        # 5. 注入 TF2 到坐标变换器（基类方法）
        self._inject_tf2_to_controller()
        
        # 6. 创建 ROS1 接口
        self._create_ros_interfaces()
        
        # 7. 初始化诊断发布辅助器
        diag_publish_rate = self._params.get('diagnostics', {}).get('publish_rate', 5)
        self._diag_helper = DiagnosticsPublishHelper(publish_rate=diag_publish_rate)
        
        # 8. 创建控制定时器
        # 统一从 system.ctrl_freq 读取控制频率
        control_rate = self._params.get('system', {}).get('ctrl_freq', 50)
        control_period = 1.0 / control_rate
        self._timer = rospy.Timer(rospy.Duration(control_period), self._control_callback)
        
        rospy.loginfo(
            f"Controller node initialized (platform={self._platform_type}, "
            f"rate={control_rate}Hz, tf2={self._tf_bridge.is_initialized})"
        )
    
    def _create_ros_interfaces(self):
        """创建 ROS1 特定的接口"""
        # 创建订阅
        self._create_subscriptions()
        
        # 创建发布器
        self._create_publishers()
        
        # 创建服务
        self._create_services()
    
    def _create_subscriptions(self):
        """创建所有订阅"""
        odom_topic = self._topics.get('odom', '/odom')
        imu_topic = self._topics.get('imu', '/imu')
        traj_topic = self._topics.get('trajectory', '/nn/local_trajectory')
        
        self._odom_sub = rospy.Subscriber(
            odom_topic, RosOdometry, 
            self._odom_callback, queue_size=10
        )
        self._imu_sub = rospy.Subscriber(
            imu_topic, RosImu, 
            self._imu_callback, queue_size=10
        )
        self._traj_sub = rospy.Subscriber(
            traj_topic, LocalTrajectoryV4, 
            self._traj_callback, queue_size=10
        )
        
        rospy.loginfo(
            f"Subscribed to: odom={odom_topic}, imu={imu_topic}, "
            f"trajectory={traj_topic}"
        )
    
    def _create_publishers(self):
        """创建所有发布器"""
        cmd_topic = self._topics.get('cmd_unified', '/cmd_unified')
        diag_topic = self._topics.get('diagnostics', '/controller/diagnostics')
        
        self._cmd_pub = rospy.Publisher(cmd_topic, UnifiedCmd, queue_size=1)
        self._diag_pub = rospy.Publisher(diag_topic, DiagnosticsV2, queue_size=10)
        
        # 状态发布器 (需求文档要求)
        self._state_pub = rospy.Publisher('/controller/state', Int32, queue_size=1)
        
        rospy.loginfo(
            f"Publishing to: cmd={cmd_topic}, diagnostics={diag_topic}, "
            f"state=/controller/state"
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
    
    # ==================== 控制循环 ====================
    
    def _control_callback(self, event):
        """控制循环回调"""
        # 使用基类的核心控制逻辑
        cmd = self._control_loop_core()
        
        if cmd is not None:
            # 发布控制命令
            self._publish_cmd(cmd)
    
    # ==================== 服务处理 ====================
    
    def _handle_reset_service(self, req):
        """处理重置服务请求"""
        try:
            self._handle_reset()  # 调用基类方法
            return TriggerResponse(success=True, message="Controller reset successfully")
        except Exception as e:
            return TriggerResponse(success=False, message=f"Reset failed: {e}")
    
    def _handle_get_diagnostics_service(self, req):
        """处理获取诊断服务请求"""
        from controller_ros.srv import GetDiagnosticsResponse
        
        response = GetDiagnosticsResponse()
        try:
            diag = self._handle_get_diagnostics()  # 调用基类方法
            if diag:
                response.success = True
                response.message = "Diagnostics retrieved successfully"
                # 使用共享工具填充诊断数据
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
            success = self._handle_set_state(req.target_state)  # 调用基类方法
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
        ros_msg = self._output_adapter.to_ros(cmd)
        self._cmd_pub.publish(ros_msg)
    
    def _publish_stop_cmd(self):
        """发布停止命令"""
        ros_msg = self._output_adapter.create_stop_cmd()
        self._cmd_pub.publish(ros_msg)
    
    def _publish_diagnostics(self, diag: Dict[str, Any], force: bool = False):
        """
        发布诊断信息
        
        Args:
            diag: 诊断数据字典
            force: 是否强制发布 (忽略降频)
        """
        # 始终发布状态话题 (不降频)
        current_state = diag.get('state', 0)
        state_msg = Int32()
        state_msg.data = current_state
        self._state_pub.publish(state_msg)
        
        # 使用辅助器判断是否发布诊断
        if not self._diag_helper.should_publish(diag, force=force):
            return
        
        # 发布诊断消息
        msg = DiagnosticsV2()
        fill_diagnostics_msg(msg, diag, get_time_func=lambda: rospy.Time.now())
        self._diag_pub.publish(msg)
    
    # ==================== 生命周期 ====================
    
    def shutdown(self):
        """关闭节点"""
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
