"""
控制器节点基类

封装 ROS1 和 ROS2 节点的共享逻辑，避免代码重复。
"""
from abc import ABC, abstractmethod
from typing import Dict, Any, Optional, Callable
import logging

from universal_controller.config.default_config import DEFAULT_CONFIG, PLATFORM_CONFIG
from universal_controller.core.data_types import Odometry, Imu, Trajectory, ControlOutput
from universal_controller.core.enums import ControllerState

from ..bridge import ControllerBridge
from ..io.data_manager import DataManager
from ..utils import TimeSync

logger = logging.getLogger(__name__)


class ControllerNodeBase(ABC):
    """
    控制器节点基类
    
    封装 ROS1 和 ROS2 节点的共享逻辑：
    - 控制器初始化
    - 数据管理
    - 控制循环核心逻辑
    - TF2 注入
    - 诊断处理
    
    子类需要实现：
    - _create_ros_interfaces(): 创建 ROS 特定的接口（订阅、发布、服务）
    - _get_time(): 获取当前 ROS 时间
    - _log_info/warn/error(): 日志方法
    - _publish_cmd(): 发布控制命令
    - _publish_stop_cmd(): 发布停止命令
    - _publish_diagnostics(): 发布诊断信息
    """
    
    def __init__(self):
        """初始化基类（子类应在调用 super().__init__() 前完成 ROS 节点初始化）"""
        # 这些属性由子类在调用 _initialize() 前设置
        self._params: Dict[str, Any] = {}
        self._topics: Dict[str, str] = {}
        self._default_frame_id: str = 'base_link'
        self._platform_type: str = 'differential'
        
        # 核心组件（由 _initialize() 创建）
        self._controller_bridge: Optional[ControllerBridge] = None
        self._data_manager: Optional[DataManager] = None
        self._time_sync: Optional[TimeSync] = None
        self._tf_bridge: Any = None  # TFBridge 或 TF2Compat
        
        # 状态
        self._waiting_for_data = True
        self._consecutive_errors = 0
        self._max_consecutive_errors = 10
    
    def _initialize(self):
        """
        初始化控制器组件
        
        子类应在加载参数后调用此方法。
        """
        # 1. 获取平台配置
        self._platform_type = self._params.get('system', {}).get('platform', 'differential')
        platform_config = PLATFORM_CONFIG.get(self._platform_type, PLATFORM_CONFIG['differential'])
        self._default_frame_id = platform_config.get('output_frame', 'base_link')
        
        # 2. 创建数据管理器
        self._data_manager = DataManager(get_time_func=self._get_time)
        
        # 3. 创建控制器桥接
        self._controller_bridge = ControllerBridge(self._params)
        
        # 4. 创建时间同步
        watchdog_config = self._params.get('watchdog', {})
        self._time_sync = TimeSync(
            max_odom_age_ms=watchdog_config.get('odom_timeout_ms', 100),
            max_traj_age_ms=watchdog_config.get('traj_timeout_ms', 200),
            max_imu_age_ms=watchdog_config.get('imu_timeout_ms', 50)
        )
        
        # 5. 设置诊断回调
        self._controller_bridge.set_diagnostics_callback(self._on_diagnostics)
    
    def _inject_tf2_to_controller(self):
        """
        将 TF2 注入到坐标变换器
        
        子类应在创建 TF 桥接后调用此方法。
        """
        if self._tf_bridge is None:
            return
        
        if not getattr(self._tf_bridge, 'is_initialized', False):
            self._log_info("TF2 not available, using fallback coordinate transform")
            return
        
        # 获取 universal_controller 的坐标变换器
        manager = self._controller_bridge.manager
        if manager is None:
            return
        
        coord_transformer = getattr(manager, 'coord_transformer', None)
        if coord_transformer is None:
            self._log_info("ControllerManager has no coord_transformer, TF2 injection skipped")
            return
        
        # 尝试注入
        if hasattr(coord_transformer, 'set_tf2_lookup_callback'):
            # 获取 lookup 函数
            lookup_func = getattr(self._tf_bridge, 'lookup_transform', None)
            if lookup_func is not None:
                coord_transformer.set_tf2_lookup_callback(lookup_func)
                self._log_info("TF2 successfully injected to coordinate transformer")
            else:
                self._log_warn("TF bridge has no lookup_transform method")
        else:
            self._log_warn("Coordinate transformer does not support TF2 injection")
    
    def _control_loop_core(self) -> Optional[ControlOutput]:
        """
        控制循环核心逻辑
        
        Returns:
            控制输出，如果无法执行控制则返回 None
        """
        # 1. 获取最新数据
        odom = self._data_manager.get_latest_odom()
        imu = self._data_manager.get_latest_imu()
        trajectory = self._data_manager.get_latest_trajectory()
        
        # 2. 检查数据有效性
        if odom is None or trajectory is None:
            if self._waiting_for_data:
                self._log_warn_throttle(5.0, "Waiting for odom and trajectory data...")
            return None
        
        if self._waiting_for_data:
            self._log_info("Data received, starting control")
            self._waiting_for_data = False
        
        # 3. 检查数据新鲜度
        ages = self._data_manager.get_data_ages()
        timeouts = self._time_sync.check_freshness(ages)
        
        if timeouts.get('odom_timeout', False):
            self._log_warn_throttle(
                1.0, 
                f"Odom timeout: age={ages.get('odom', 0)*1000:.1f}ms"
            )
        
        # 4. 执行控制更新
        try:
            cmd = self._controller_bridge.update(odom, trajectory, imu)
            # 成功时重置错误计数
            self._consecutive_errors = 0
            return cmd
        except Exception as e:
            self._consecutive_errors += 1
            self._handle_control_error(e, timeouts)
            return None
    
    def _handle_control_error(self, error: Exception, timeouts: Dict[str, bool]):
        """
        处理控制更新错误
        
        Args:
            error: 异常对象
            timeouts: 超时状态字典
        """
        # 根据连续错误次数调整日志频率
        if self._consecutive_errors <= self._max_consecutive_errors:
            self._log_error(f'Controller update failed ({self._consecutive_errors}): {error}')
        elif self._consecutive_errors % 50 == 0:
            self._log_error(
                f'Controller update still failing ({self._consecutive_errors} consecutive errors): {error}'
            )
        
        # 发布停止命令
        self._publish_stop_cmd()
        
        # 发布错误诊断信息
        error_diag = self._create_error_diagnostics(error, timeouts)
        self._publish_diagnostics(error_diag, force=True)
    
    def _create_error_diagnostics(self, error: Exception, 
                                   timeouts: Dict[str, bool]) -> Dict[str, Any]:
        """
        创建错误诊断信息
        
        Args:
            error: 异常对象
            timeouts: 超时状态字典
        
        Returns:
            诊断信息字典
        """
        ages = self._data_manager.get_data_ages()
        
        return {
            'state': 0,  # INIT/ERROR state
            'mpc_success': False,
            'backup_active': False,
            'error_message': str(error),
            'consecutive_errors': self._consecutive_errors,
            'timeout': {
                'odom_timeout': timeouts.get('odom_timeout', False),
                'traj_timeout': timeouts.get('traj_timeout', False),
                'imu_timeout': timeouts.get('imu_timeout', False),
                'last_odom_age_ms': ages.get('odom', float('inf')) * 1000,
                'last_traj_age_ms': ages.get('trajectory', float('inf')) * 1000,
                'last_imu_age_ms': ages.get('imu', float('inf')) * 1000,
            },
            'cmd': {'vx': 0.0, 'vy': 0.0, 'vz': 0.0, 'omega': 0.0},
            'transform': {
                'tf2_available': getattr(self._tf_bridge, 'is_initialized', False) 
                                 if self._tf_bridge else False
            },
        }
    
    def _on_diagnostics(self, diag: Dict[str, Any]):
        """
        诊断回调
        
        Args:
            diag: 诊断信息字典
        """
        # 添加 TF2 状态
        if 'transform' not in diag:
            diag['transform'] = {}
        diag['transform']['tf2_available'] = (
            getattr(self._tf_bridge, 'is_initialized', False) 
            if self._tf_bridge else False
        )
        
        self._publish_diagnostics(diag)
    
    def _handle_reset(self):
        """处理重置请求"""
        if self._controller_bridge is not None:
            self._controller_bridge.reset()
        if self._data_manager is not None:
            self._data_manager.clear()
        self._waiting_for_data = True
        self._consecutive_errors = 0
        self._log_info('Controller reset')
    
    def _handle_get_diagnostics(self) -> Optional[Dict[str, Any]]:
        """处理获取诊断请求"""
        if self._controller_bridge is not None:
            return self._controller_bridge.get_diagnostics()
        return None
    
    def _handle_set_state(self, target_state: int) -> bool:
        """
        处理设置状态请求
        
        出于安全考虑，只支持请求 STOPPING 状态。
        
        Args:
            target_state: 目标状态值
        
        Returns:
            是否成功
        """
        # 只允许请求 STOPPING 状态
        if target_state == ControllerState.STOPPING.value:
            if self._controller_bridge is not None:
                success = self._controller_bridge.request_stop()
                if success:
                    self._log_info('Stop requested via service')
                return success
            return False
        else:
            self._log_warn(
                f'Set state to {target_state} not allowed. '
                f'Only STOPPING ({ControllerState.STOPPING.value}) is supported for safety reasons.'
            )
            return False
    
    def shutdown(self):
        """关闭控制器"""
        if self._controller_bridge is not None:
            self._controller_bridge.shutdown()
        self._log_info('Controller node shutdown')
    
    # ==================== 抽象方法 ====================
    
    @abstractmethod
    def _get_time(self) -> float:
        """
        获取当前时间（秒）
        
        应返回支持仿真时间的 ROS 时间。
        """
        pass
    
    @abstractmethod
    def _log_info(self, msg: str):
        """记录信息日志"""
        pass
    
    @abstractmethod
    def _log_warn(self, msg: str):
        """记录警告日志"""
        pass
    
    @abstractmethod
    def _log_warn_throttle(self, period: float, msg: str):
        """记录节流警告日志"""
        pass
    
    @abstractmethod
    def _log_error(self, msg: str):
        """记录错误日志"""
        pass
    
    @abstractmethod
    def _publish_cmd(self, cmd: ControlOutput):
        """发布控制命令"""
        pass
    
    @abstractmethod
    def _publish_stop_cmd(self):
        """发布停止命令"""
        pass
    
    @abstractmethod
    def _publish_diagnostics(self, diag: Dict[str, Any], force: bool = False):
        """发布诊断信息"""
        pass
