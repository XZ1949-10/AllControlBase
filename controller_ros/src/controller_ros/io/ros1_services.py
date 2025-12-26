"""
ROS1 服务管理器

管理所有 ROS1 服务，与 ROS2 的 ServiceManager 接口对齐。
"""
from typing import Dict, Any, Optional, Callable
import logging

try:
    import rospy
    from std_srvs.srv import Trigger, TriggerResponse
    ROS1_AVAILABLE = True
except ImportError:
    ROS1_AVAILABLE = False

from ..utils.diagnostics_publisher import fill_diagnostics_msg

logger = logging.getLogger(__name__)


class ROS1ServiceManager:
    """
    ROS1 服务管理器
    
    职责:
    - 创建和管理所有 ROS1 服务
    - 与 ROS2 的 ServiceManager 接口对齐
    """
    
    def __init__(self,
                 reset_callback: Callable[[], None],
                 get_diagnostics_callback: Callable[[], Optional[Dict[str, Any]]],
                 set_state_callback: Callable[[int], bool],
                 set_hover_yaw_callback: Optional[Callable[[float], bool]] = None,
                 get_attitude_rate_limits_callback: Optional[Callable[[], Optional[Dict[str, float]]]] = None,
                 is_quadrotor: bool = False):
        """
        初始化服务管理器
        
        Args:
            reset_callback: 重置回调
            get_diagnostics_callback: 获取诊断回调
            set_state_callback: 设置状态回调
            set_hover_yaw_callback: 设置悬停航向回调 (四旋翼)
            get_attitude_rate_limits_callback: 获取姿态角速度限制回调 (四旋翼)
            is_quadrotor: 是否为四旋翼平台
        """
        if not ROS1_AVAILABLE:
            raise RuntimeError("ROS1 (rospy) not available")
        
        self._reset_callback = reset_callback
        self._get_diagnostics_callback = get_diagnostics_callback
        self._set_state_callback = set_state_callback
        self._set_hover_yaw_callback = set_hover_yaw_callback
        self._get_attitude_rate_limits_callback = get_attitude_rate_limits_callback
        self._is_quadrotor = is_quadrotor
        
        self._create_services()
    
    def _create_services(self):
        """创建所有服务"""
        # 重置服务
        self._reset_srv = rospy.Service(
            '/controller/reset', Trigger, self._handle_reset
        )
        rospy.loginfo("Created service: /controller/reset")
        
        # 获取诊断服务
        try:
            from controller_ros.srv import GetDiagnostics
            self._get_diag_srv = rospy.Service(
                '/controller/get_diagnostics', GetDiagnostics,
                self._handle_get_diagnostics
            )
            rospy.loginfo("Created service: /controller/get_diagnostics")
        except ImportError:
            self._get_diag_srv = None
            rospy.logwarn("GetDiagnostics service not available")
        
        # 设置状态服务
        try:
            from controller_ros.srv import SetControllerState
            self._set_state_srv = rospy.Service(
                '/controller/set_state', SetControllerState,
                self._handle_set_state
            )
            rospy.loginfo("Created service: /controller/set_state")
        except ImportError:
            self._set_state_srv = None
            rospy.logwarn("SetControllerState service not available")
        
        # 四旋翼平台专用服务
        if self._is_quadrotor:
            self._create_quadrotor_services()
    
    def _create_quadrotor_services(self):
        """创建四旋翼平台专用服务"""
        # 设置悬停航向服务
        try:
            from controller_ros.srv import SetHoverYaw
            self._set_hover_yaw_srv = rospy.Service(
                '/controller/set_hover_yaw', SetHoverYaw,
                self._handle_set_hover_yaw
            )
            rospy.loginfo("Created service: /controller/set_hover_yaw")
        except ImportError:
            self._set_hover_yaw_srv = None
            rospy.logwarn("SetHoverYaw service not available")
        
        # 获取姿态角速度限制服务
        try:
            from controller_ros.srv import GetAttitudeRateLimits
            self._get_attitude_limits_srv = rospy.Service(
                '/controller/get_attitude_rate_limits', GetAttitudeRateLimits,
                self._handle_get_attitude_rate_limits
            )
            rospy.loginfo("Created service: /controller/get_attitude_rate_limits")
        except ImportError:
            self._get_attitude_limits_srv = None
            rospy.logwarn("GetAttitudeRateLimits service not available")
    
    def _handle_reset(self, req):
        """处理重置服务请求"""
        try:
            self._reset_callback()
            return TriggerResponse(success=True, message="Controller reset successfully")
        except Exception as e:
            return TriggerResponse(success=False, message=f"Reset failed: {e}")
    
    def _handle_get_diagnostics(self, req):
        """处理获取诊断服务请求"""
        from controller_ros.srv import GetDiagnosticsResponse
        
        response = GetDiagnosticsResponse()
        try:
            diag = self._get_diagnostics_callback()
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
    
    def _handle_set_state(self, req):
        """处理设置状态服务请求"""
        from controller_ros.srv import SetControllerStateResponse
        
        response = SetControllerStateResponse()
        try:
            success = self._set_state_callback(req.target_state)
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
    
    def _handle_set_hover_yaw(self, req):
        """处理设置悬停航向服务请求"""
        from controller_ros.srv import SetHoverYawResponse
        
        response = SetHoverYawResponse()
        try:
            if self._set_hover_yaw_callback is None:
                response.success = False
                response.message = "Set hover yaw not supported"
            else:
                success = self._set_hover_yaw_callback(req.yaw)
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
    
    def _handle_get_attitude_rate_limits(self, req):
        """处理获取姿态角速度限制服务请求"""
        from controller_ros.srv import GetAttitudeRateLimitsResponse
        
        response = GetAttitudeRateLimitsResponse()
        try:
            if self._get_attitude_rate_limits_callback is None:
                response.success = False
                response.message = "Get attitude rate limits not supported"
            else:
                limits = self._get_attitude_rate_limits_callback()
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

    def shutdown(self) -> None:
        """
        关闭服务管理器，释放资源
        
        在 ROS1 中关闭服务并清理回调引用。
        """
        # 关闭服务
        if hasattr(self, '_reset_srv') and self._reset_srv is not None:
            self._reset_srv.shutdown()
            self._reset_srv = None
        
        if hasattr(self, '_get_diag_srv') and self._get_diag_srv is not None:
            self._get_diag_srv.shutdown()
            self._get_diag_srv = None
        
        if hasattr(self, '_set_state_srv') and self._set_state_srv is not None:
            self._set_state_srv.shutdown()
            self._set_state_srv = None
        
        # 关闭四旋翼服务
        if hasattr(self, '_set_hover_yaw_srv') and self._set_hover_yaw_srv is not None:
            self._set_hover_yaw_srv.shutdown()
            self._set_hover_yaw_srv = None
        
        if hasattr(self, '_get_attitude_limits_srv') and self._get_attitude_limits_srv is not None:
            self._get_attitude_limits_srv.shutdown()
            self._get_attitude_limits_srv = None
        
        # 清理回调引用
        self._reset_callback = None
        self._get_diagnostics_callback = None
        self._set_state_callback = None
        self._set_hover_yaw_callback = None
        self._get_attitude_rate_limits_callback = None
        
        rospy.logdebug("ROS1ServiceManager shutdown complete")
