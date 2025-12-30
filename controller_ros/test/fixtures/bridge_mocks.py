"""
桥接层 Mock 类

模拟 TF Bridge 和 Controller Bridge，用于单元测试。
"""
from typing import Dict, Any, Optional, Tuple


class MockTFBridge:
    """
    模拟 TF Bridge
    
    用于测试 TF2 注入和坐标变换功能。
    """
    
    def __init__(self, 
                 initialized: bool = True, 
                 can_transform_result: bool = True,
                 default_transform: Optional[Dict[str, Tuple]] = None):
        """
        初始化 Mock TF Bridge
        
        Args:
            initialized: TF2 是否已初始化
            can_transform_result: can_transform() 的返回值
            default_transform: 默认变换结果
        """
        self.is_initialized = initialized
        self._can_transform_result = can_transform_result
        self._default_transform = default_transform or {
            'translation': (0.0, 0.0, 0.0),
            'rotation': (0.0, 0.0, 0.0, 1.0)
        }
        
        # 调用计数器（用于测试验证）
        self._can_transform_call_count = 0
        self._lookup_transform_call_count = 0
        
        # 记录调用参数
        self._can_transform_calls = []
        self._lookup_transform_calls = []
        
        # 自定义变换结果
        self._custom_transforms: Dict[Tuple[str, str], Dict] = {}
    
    def can_transform(self, target_frame: str, source_frame: str, 
                     timeout_sec: float = 0.01) -> bool:
        """
        检查是否可以进行坐标变换
        
        Args:
            target_frame: 目标坐标系
            source_frame: 源坐标系
            timeout_sec: 超时时间
        
        Returns:
            是否可以变换
        """
        self._can_transform_call_count += 1
        self._can_transform_calls.append({
            'target_frame': target_frame,
            'source_frame': source_frame,
            'timeout_sec': timeout_sec
        })
        return self._can_transform_result
    
    def lookup_transform(self, target_frame: str, source_frame: str, 
                        time=None, timeout_sec: float = 0.01) -> Dict[str, Tuple]:
        """
        查询坐标变换
        
        Args:
            target_frame: 目标坐标系
            source_frame: 源坐标系
            time: 时间戳
            timeout_sec: 超时时间
        
        Returns:
            变换字典 {'translation': (x, y, z), 'rotation': (x, y, z, w)}
        """
        self._lookup_transform_call_count += 1
        self._lookup_transform_calls.append({
            'target_frame': target_frame,
            'source_frame': source_frame,
            'time': time,
            'timeout_sec': timeout_sec
        })
        
        # 检查是否有自定义变换
        key = (target_frame, source_frame)
        if key in self._custom_transforms:
            return self._custom_transforms[key]
        
        return self._default_transform
    
    def set_transform_result(self, target_frame: str, source_frame: str,
                            translation: Tuple[float, float, float],
                            rotation: Tuple[float, float, float, float]) -> None:
        """
        设置自定义变换结果
        
        Args:
            target_frame: 目标坐标系
            source_frame: 源坐标系
            translation: 平移 (x, y, z)
            rotation: 旋转四元数 (x, y, z, w)
        """
        key = (target_frame, source_frame)
        self._custom_transforms[key] = {
            'translation': translation,
            'rotation': rotation
        }
    
    def set_can_transform(self, result: bool) -> None:
        """设置 can_transform 的返回值"""
        self._can_transform_result = result
    
    def set_initialized(self, initialized: bool) -> None:
        """设置初始化状态"""
        self.is_initialized = initialized
    
    def reset_counters(self) -> None:
        """重置调用计数器"""
        self._can_transform_call_count = 0
        self._lookup_transform_call_count = 0
        self._can_transform_calls.clear()
        self._lookup_transform_calls.clear()


class MockControllerBridge:
    """
    模拟 Controller Bridge
    
    用于测试控制器节点的集成。
    """
    
    def __init__(self, 
                 initialized: bool = True,
                 update_success: bool = True):
        """
        初始化 Mock Controller Bridge
        
        Args:
            initialized: 是否已初始化
            update_success: update() 是否成功
        """
        self._initialized = initialized
        self._update_success = update_success
        
        # 模拟状态
        self._state = 0  # INIT
        self._diagnostics: Dict[str, Any] = {}
        self._stop_requested = False
        
        # 调用计数器
        self._update_call_count = 0
        self._reset_call_count = 0
        
        # 模拟 manager
        self.manager = MockControllerManager()
    
    @property
    def is_initialized(self) -> bool:
        return self._initialized
    
    def update(self, odom, trajectory, imu=None):
        """模拟控制更新"""
        from universal_controller.core.data_types import ControlOutput
        
        self._update_call_count += 1
        
        if not self._update_success:
            raise RuntimeError("Mock update failure")
        
        return ControlOutput(
            vx=0.5, vy=0.0, vz=0.0, omega=0.1,
            frame_id='base_link', success=True, solve_time_ms=5.0
        )
    
    def get_state(self):
        """获取控制器状态"""
        from universal_controller.core.enums import ControllerState
        return ControllerState(self._state)
    
    def get_diagnostics(self) -> Optional[Dict[str, Any]]:
        """获取诊断信息"""
        return self._diagnostics or {
            'state': self._state,
            'mpc_success': True,
            'backup_active': False,
        }
    
    def set_diagnostics_callback(self, callback) -> None:
        """设置诊断回调"""
        self._diagnostics_callback = callback
    
    def reset(self) -> None:
        """重置控制器"""
        self._reset_call_count += 1
        self._state = 0
        self._stop_requested = False
    
    def request_stop(self) -> bool:
        """请求停止"""
        self._stop_requested = True
        self._state = 5  # STOPPING
        return True
    
    def shutdown(self) -> None:
        """关闭控制器"""
        self._initialized = False


class MockControllerManager:
    """模拟 ControllerManager"""
    
    def __init__(self):
        self.coord_transformer = MockCoordTransformer()
        self.state_estimator = None
        self.attitude_controller = None


class MockCoordTransformer:
    """模拟坐标变换器
    
    模拟 RobustCoordinateTransformer 的接口，用于测试。
    get_status() 返回的字段与实际实现保持一致。
    """
    
    def __init__(self):
        self._tf2_callback = None
        self._tf2_available = True
        self._fallback_duration_ms = 0.0
        self._accumulated_drift = 0.0
    
    def set_tf2_lookup_callback(self, callback) -> None:
        """设置 TF2 查找回调"""
        self._tf2_callback = callback
    
    def get_status(self) -> Dict[str, Any]:
        """获取状态
        
        返回与 RobustCoordinateTransformer.get_status() 一致的字段。
        """
        return {
            'tf2_available': self._tf2_available,
            'tf2_injected': self._tf2_callback is not None,
            'fallback_duration_ms': self._fallback_duration_ms,
            'accumulated_drift': self._accumulated_drift,
            'drift_estimate_reliable': True,
            'is_critical': False,
            'status': 'TF2_OK',
            'tf2_initialized': True,
            'source_frame': 'base_link',
            'target_frame': 'odom',
            'error_message': ''
        }
