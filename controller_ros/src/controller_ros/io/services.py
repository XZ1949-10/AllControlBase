"""
服务管理器

管理 ROS 服务。
"""
from typing import Optional, Callable
import logging

from rclpy.node import Node

logger = logging.getLogger(__name__)


class ServiceManager:
    """
    服务管理器
    
    职责:
    - 创建和管理服务
    - 处理服务请求
    """
    
    def __init__(self, node: Node,
                 reset_callback: Optional[Callable] = None,
                 get_diagnostics_callback: Optional[Callable] = None,
                 set_state_callback: Optional[Callable[[int], bool]] = None):
        """
        初始化服务管理器
        
        Args:
            node: ROS2 节点
            reset_callback: 重置回调函数
            get_diagnostics_callback: 获取诊断回调函数
            set_state_callback: 设置状态回调函数，接收目标状态，返回是否成功
        """
        self._node = node
        self._reset_callback = reset_callback
        self._get_diagnostics_callback = get_diagnostics_callback
        self._set_state_callback = set_state_callback
        
        self._create_services()
    
    def _create_services(self):
        """创建所有服务"""
        from std_srvs.srv import Trigger
        
        # 重置服务
        self._reset_srv = self._node.create_service(
            Trigger,
            '/controller/reset',
            self._handle_reset
        )
        logger.info("Created reset service: /controller/reset")
        
        # 获取诊断服务 (使用自定义消息时启用)
        try:
            from controller_ros.srv import GetDiagnostics
            self._get_diag_srv = self._node.create_service(
                GetDiagnostics,
                '/controller/get_diagnostics',
                self._handle_get_diagnostics
            )
            logger.info("Created get_diagnostics service")
        except ImportError:
            self._get_diag_srv = None
            logger.warning("GetDiagnostics service not available")
        
        # 设置状态服务
        try:
            from controller_ros.srv import SetControllerState
            self._set_state_srv = self._node.create_service(
                SetControllerState,
                '/controller/set_state',
                self._handle_set_state
            )
            logger.info("Created set_state service: /controller/set_state")
        except ImportError:
            self._set_state_srv = None
            logger.warning("SetControllerState service not available")
    
    def _handle_reset(self, request, response):
        """处理重置请求"""
        try:
            if self._reset_callback:
                self._reset_callback()
            response.success = True
            response.message = "Controller reset successfully"
        except Exception as e:
            response.success = False
            response.message = f"Reset failed: {e}"
        return response
    
    def _handle_get_diagnostics(self, request, response):
        """处理获取诊断请求"""
        try:
            if self._get_diagnostics_callback:
                diag = self._get_diagnostics_callback()
                if diag:
                    response.success = True
                    response.message = "Diagnostics retrieved successfully"
                    # 使用共享工具填充诊断数据
                    from ..utils.diagnostics_publisher import fill_diagnostics_msg
                    fill_diagnostics_msg(
                        response.diagnostics, diag,
                        get_time_func=lambda: self._node.get_clock().now().to_msg()
                    )
                else:
                    response.success = False
                    response.message = "Diagnostics callback returned None"
                    logger.warning("Get diagnostics callback returned None")
            else:
                response.success = False
                response.message = "Diagnostics callback not configured"
                logger.warning("Get diagnostics callback not set")
        except Exception as e:
            response.success = False
            logger.error(f"Get diagnostics failed: {e}")
        return response
    
    def set_reset_callback(self, callback: Callable):
        """设置重置回调"""
        self._reset_callback = callback
    
    def set_get_diagnostics_callback(self, callback: Callable):
        """设置获取诊断回调"""
        self._get_diagnostics_callback = callback
    
    def set_state_callback(self, callback: Callable[[int], bool]):
        """设置状态切换回调"""
        self._set_state_callback = callback
    
    def _handle_set_state(self, request, response):
        """处理设置状态请求"""
        try:
            if self._set_state_callback:
                success = self._set_state_callback(request.target_state)
                response.success = success
                if success:
                    response.message = f"State changed to {request.target_state}"
                else:
                    response.message = f"Failed to change state to {request.target_state}"
            else:
                response.success = False
                response.message = "Set state callback not configured"
        except Exception as e:
            response.success = False
            response.message = f"Set state failed: {e}"
            logger.error(f"Set state failed: {e}")
        return response
