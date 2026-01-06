"""
控制器桥接层

封装 ControllerManager 调用，隔离 ROS 层与算法库。

生命周期说明：
- 实现 ILifecycleComponent 接口（通过 LifecycleMixin）
- initialize(): 创建并初始化 ControllerManager
- reset(): 重置控制器状态
- shutdown(): 关闭控制器，释放资源

使用方式：
- 推荐: 使用 create() 工厂方法，失败时返回 None
- 备选: 直接构造后调用 initialize()，需检查返回值
"""
from typing import Dict, Any, Optional
import logging

from universal_controller.manager.controller_manager import ControllerManager
from universal_controller.core.data_types import (
    Odometry, Imu, Trajectory, ControlOutput, DiagnosticsV2
)
from universal_controller.core.enums import ControllerState
from ..lifecycle import LifecycleMixin, LifecycleState

logger = logging.getLogger(__name__)


class ControllerBridge(LifecycleMixin):
    """
    控制器桥接层
    
    职责:
    - 封装 ControllerManager 调用
    - 管理控制器生命周期
    - 提供统一的更新接口
    - 隔离 ROS 层与算法库
    
    生命周期:
    - 实现 ILifecycleComponent 接口（通过 LifecycleMixin）
    - initialize(): 创建并初始化 ControllerManager
    - reset(): 重置控制器状态
    - shutdown(): 关闭控制器，释放资源
    
    使用示例:
        # 方式 1: 工厂方法 (推荐)
        bridge = ControllerBridge.create(config)
        if bridge is None:
            handle_error()
        
        # 方式 2: 显式初始化
        bridge = ControllerBridge(config)
        if not bridge.initialize():
            handle_error(bridge.last_error)
    """
    
    @classmethod
    def create(cls, config: Dict[str, Any]) -> Optional['ControllerBridge']:
        """
        工厂方法：创建并初始化 ControllerBridge
        
        Args:
            config: 控制器配置字典
        
        Returns:
            初始化成功的 ControllerBridge 实例，失败返回 None
        
        Example:
            bridge = ControllerBridge.create(config)
            if bridge is None:
                logger.error("Failed to create controller bridge")
                return
        """
        bridge = cls(config)
        if bridge.initialize():
            return bridge
        logger.error(f"ControllerBridge initialization failed: {bridge.last_error}")
        return None
    
    def __init__(self, config: Dict[str, Any]):
        """
        初始化控制器桥接（不执行实际初始化）
        
        Args:
            config: 控制器配置字典，将传递给 ControllerManager
        
        Note:
            构造后需调用 initialize() 或使用 create() 工厂方法
        """
        super().__init__()
        self._config = config
        self._manager: Optional[ControllerManager] = None
        
        # 频率监控
        self._last_update_time = 0.0
        ctrl_freq = config.get('system', {}).get('ctrl_freq', 50.0)
        self._expected_period = 1.0 / ctrl_freq if ctrl_freq > 0 else 0.02
        self._freq_warn_threshold = 1.5  # 允许 50% 的抖动
        
        # 预热阈值: 至少 1 秒或 50 帧，避免启动初期的瞬态抖动误报
        self._warmup_threshold = max(50, int(1.0 / self._expected_period))

    # ==================== LifecycleMixin 实现 ====================
    
    def _do_initialize(self) -> bool:
        """
        初始化控制器
        
        Note:
            不捕获异常，让异常传播到 LifecycleMixin.initialize()，
            由其统一处理并保存完整错误信息到 _last_error。
            如果初始化失败，_manager 会在 _do_shutdown() 中被清理。
        """
        self._last_ros_time = 0.0
        self._last_monotonic_time = 0.0
        
        self._manager = ControllerManager(self._config)
        self._manager.initialize_default_components()
        self._warmup_counter = 0
        logger.info(f"Controller bridge initialized (expected rate: {1.0/self._expected_period:.1f}Hz)")
        return True
    
    def _do_shutdown(self) -> None:
        """关闭控制器"""
        if self._manager is not None:
            try:
                self._manager.shutdown()
            except Exception as e:
                logger.warning(f"Error during controller shutdown: {e}")
            finally:
                self._manager = None
        logger.info("Controller bridge shutdown")
    
    def _do_reset(self) -> None:
        """重置控制器状态"""
        self._last_ros_time = 0.0
        self._warmup_counter = 0
        if self._manager is not None:
            self._manager.reset()
            logger.info("Controller bridge reset")
    
    def _get_health_details(self) -> Dict[str, Any]:
        """获取详细健康信息"""
        if self._manager is None:
            return {'manager_available': False}
        
        return {
            'manager_available': True,
            'state': self._manager.get_state().name,
            'timeout_status': {
                'odom_timeout': self._manager.timeout_monitor.odom_timeout_enabled,
                'traj_timeout': self._manager.timeout_monitor.traj_timeout_enabled,
            },
        }
    
    def update(self, current_time: float, odom: Odometry, trajectory: Trajectory, 
               data_ages: Dict[str, float], imu: Optional[Imu] = None) -> ControlOutput:
        """
        执行一次控制更新
        
        Args:
            current_time: 当前时间 (ROS/Sim time, seconds)
            odom: 里程计数据 (UC 格式)
            trajectory: 轨迹数据 (UC 格式)
            data_ages: 数据年龄字典 {'odom': ms, 'trajectory': ms, 'imu': ms}
            imu: IMU 数据 (UC 格式，可选)
        
        Returns:
            控制输出 (UC 格式)
        
        Raises:
            RuntimeError: 控制器未初始化或已关闭
        """
        if not self.is_running or self._manager is None:
            raise RuntimeError("Controller not initialized or already shutdown")
        
        # 频率监控 (修正: 使用 ROS Time 监控控制频率，适应仿真变速)
        # current_time 是从 Node 传入的 ROS 时间 (Sim Time or System Time)
        if self._last_ros_time > 0:
            dt_ros = current_time - self._last_ros_time
            
            # 1. 处理时钟回退 (如仿真重置)
            if dt_ros < 0:
                # 此时不触发警告，直接重置基准时间
                self._last_ros_time = current_time
            
            # 2. 正常的频率检查
            # 忽略过长的间隔（可能是首次运行、暂停后恢复、时钟跳变）
            # 注意: 如果仿真暂停，dt_ros 为 0，不应触发警告
            # 如果仿真变慢 (e.g. 0.1x)，dt_ros 仍然是 0.02s (Sim Time)，不会触发警告 -> 正确行为
            # 如果计算过慢导致丢帧 (Overrun)，dt_ros 会变成 0.04s, 0.06s -> 触发警告 -> 正确行为
            # 增加 warmup 检查，跳过启动初期的不稳定阶段
            elif (self._warmup_counter > self._warmup_threshold and 
                  dt_ros > self._expected_period * self._freq_warn_threshold and 
                  dt_ros < 2.0):
                 logger.warning(
                    f"Control loop jitter detected (ROS Time): {1.0/dt_ros:.1f}Hz "
                    f"(expected {1.0/self._expected_period:.1f}Hz, dt={dt_ros*1000:.1f}ms)"
                )
        
        # 不要使用 time.monotonic()，它无法感知仿真速率
        self._last_ros_time = current_time
        self._warmup_counter += 1
        
        return self._manager.update(current_time, odom, trajectory, data_ages, imu)
    
    def get_state(self) -> ControllerState:
        """获取控制器状态"""
        if self._manager is None:
            return ControllerState.INIT
        return self._manager.get_state()
    
    def get_diagnostics(self) -> Optional[DiagnosticsV2]:
        """获取诊断信息"""
        if self._manager is None:
            return None
        return self._manager.get_last_published_diagnostics()
    
    def set_diagnostics_callback(self, callback: callable) -> None:
        """设置诊断回调函数"""
        if self._manager is not None:
            self._manager.set_diagnostics_callback(callback)
    
    def request_stop(self) -> bool:
        """
        请求控制器进入停止状态
        
        这是唯一允许的外部状态干预，用于紧急停止场景。
        
        Returns:
            是否成功请求停止
        """
        if not self.is_running or self._manager is None:
            return False
        
        # 使用经理层的代理方法，不再直接访问内部状态机
        return self._manager.request_stop()
    
    @property
    def manager(self) -> Optional[ControllerManager]:
        """获取底层 ControllerManager (用于高级操作)"""
        return self._manager
