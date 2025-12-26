"""接口定义"""
from abc import ABC, abstractmethod
from typing import Dict, Any, Tuple, Optional, TYPE_CHECKING
import numpy as np

from .data_types import (
    Trajectory, EstimatorOutput, ControlOutput, ConsistencyResult,
    SafetyDecision, Odometry, Imu, AttitudeCommand
)
from .enums import TransformStatus

# 类型检查时导入，避免循环导入
if TYPE_CHECKING:
    from .diagnostics_input import DiagnosticsInput


# 生命周期状态枚举 - 统一定义，供所有层使用
from enum import Enum, auto


class LifecycleState(Enum):
    """
    生命周期状态枚举
    
    状态说明：
    - UNINITIALIZED: 组件已创建但未初始化
    - RUNNING: 组件正在运行
    - SHUTDOWN: 组件已关闭，资源已释放
    - ERROR: 组件处于错误状态
    """
    UNINITIALIZED = auto()
    RUNNING = auto()
    SHUTDOWN = auto()
    ERROR = auto()


class ILifecycleComponent(ABC):
    """
    统一生命周期组件接口
    
    所有需要生命周期管理的组件都应实现此接口。
    
    核心方法 (必须实现):
    - reset(): 重置内部状态，保留资源，可继续使用
    
    可选方法 (有默认实现):
    - shutdown(): 释放所有资源，对象不应再使用
    - initialize(): 初始化组件，分配资源
    - get_health_status(): 获取组件健康状态
    
    设计原则:
    - reset() 是必须实现的，用于状态重置
    - 其他方法有默认实现，按需覆盖
    - 算法层组件只需实现 reset()
    - ROS 层组件可覆盖 initialize() 和 get_health_status() 获得完整监控能力
    
    使用示例:
        # 简单算法组件 - 只实现 reset()
        class SimpleEstimator(ILifecycleComponent):
            def reset(self) -> None:
                self._state = np.zeros(6)
        
        # 完整监控组件 - 覆盖所有方法
        class MonitoredController(ILifecycleComponent):
            def reset(self) -> None: ...
            def shutdown(self) -> None: ...
            def initialize(self) -> bool: ...
            def get_health_status(self) -> Dict[str, Any]: ...
    """
    
    @abstractmethod
    def reset(self) -> None:
        """
        重置组件内部状态
        
        调用后组件应恢复到初始状态，但保留已分配的资源。
        可以继续调用其他方法。
        
        Note:
            - 这是唯一必须实现的方法
            - 应该是幂等的：多次调用应该安全
        """
        pass
    
    def shutdown(self) -> None:
        """
        关闭组件并释放所有资源
        
        调用后组件不应再使用。默认实现为空，
        只有持有外部资源（如 C 库、文件句柄、网络连接）的组件需要覆盖。
        
        子类覆盖时应该:
        1. 释放外部资源
        2. 调用 reset() 清理内部状态
        
        Note:
            - 应该是幂等的：多次调用应该安全
            - 不应抛出异常
        """
        pass
    
    def initialize(self) -> bool:
        """
        初始化组件
        
        分配必要的资源，建立连接，准备运行。
        默认实现返回 True，表示组件在构造时已完成初始化。
        
        Returns:
            bool: 初始化是否成功
        
        Note:
            - 应该是幂等的：多次调用应该安全
            - 初始化失败后，组件应处于可以重试的状态
            - 不应抛出异常，失败时返回 False
        """
        return True
    
    def get_health_status(self) -> Optional[Dict[str, Any]]:
        """
        获取组件健康状态
        
        默认实现返回 None，表示组件不支持健康检查。
        覆盖此方法以提供详细的健康状态信息。
        
        Returns:
            Optional[Dict[str, Any]]: 健康状态字典，或 None 表示不支持
                如果返回字典，应至少包含：
                - 'healthy' (bool): 组件是否健康
                - 'state' (str): 当前状态名称
                - 'message' (str): 状态描述信息
                
                可选字段：
                - 'details' (Dict): 详细状态信息
                - 'last_error' (str): 最后一次错误信息
                - 'uptime_sec' (float): 运行时间（秒）
        
        Note:
            - 应该快速返回，不应阻塞
            - 不应抛出异常
        """
        return None


class IStateEstimator(ILifecycleComponent):
    """状态估计器接口"""
    
    @abstractmethod
    def predict(self, dt: float) -> None:
        pass
    
    @abstractmethod
    def update_odom(self, odom: Odometry) -> None:
        pass
    
    @abstractmethod
    def update_imu(self, imu: Imu) -> None:
        pass
    
    @abstractmethod
    def get_state(self) -> EstimatorOutput:
        pass
    
    @abstractmethod
    def set_imu_available(self, available: bool) -> None:
        pass
    
    @abstractmethod
    def apply_drift_correction(self, dx: float, dy: float, dtheta: float) -> None:
        """应用外部漂移校正"""
        pass


class ITrajectoryTracker(ILifecycleComponent):
    """轨迹跟踪器接口"""
    
    @abstractmethod
    def compute(self, state: np.ndarray, trajectory: Trajectory, 
                consistency: ConsistencyResult) -> ControlOutput:
        pass
    
    @abstractmethod
    def get_health_metrics(self) -> Dict[str, Any]:
        pass
    
    @abstractmethod
    def set_horizon(self, horizon: int) -> bool:
        """
        设置预测时域长度
        
        Args:
            horizon: 新的预测时域长度
        
        Returns:
            bool: True 表示成功更新，False 表示被节流或无需更新
        """
        pass


class IConsistencyChecker(ILifecycleComponent):
    """一致性检查器接口"""
    
    @abstractmethod
    def compute(self, trajectory: Trajectory) -> ConsistencyResult:
        pass


class ISafetyMonitor(ILifecycleComponent):
    """安全监控器接口"""
    
    @abstractmethod
    def check(self, state: np.ndarray, cmd: ControlOutput, 
              diagnostics: 'DiagnosticsInput') -> SafetyDecision:
        pass


class ISmoothTransition(ILifecycleComponent):
    """平滑过渡接口"""
    
    @abstractmethod
    def start_transition(self, from_cmd: ControlOutput) -> None:
        pass
    
    @abstractmethod
    def get_blended_output(self, new_cmd: ControlOutput, 
                          current_time: float) -> ControlOutput:
        pass
    
    @abstractmethod
    def is_complete(self) -> bool:
        pass
    
    @abstractmethod
    def get_progress(self) -> float:
        pass


class ICoordinateTransformer(ILifecycleComponent):
    """坐标变换器接口"""
    
    @abstractmethod
    def transform_trajectory(self, traj: Trajectory, target_frame: str, 
                            target_time: float) -> Tuple[Trajectory, TransformStatus]:
        pass
    
    @abstractmethod
    def get_status(self) -> Dict[str, Any]:
        pass
    
    @abstractmethod
    def set_state_estimator(self, estimator: IStateEstimator) -> None:
        pass


class IAttitudeController(ILifecycleComponent):
    """
    姿态内环控制器接口 (F14.1)
    
    用于无人机的姿态控制，将速度命令转换为姿态命令
    """
    
    @abstractmethod
    def compute_attitude(self, velocity_cmd: ControlOutput, 
                        current_state: np.ndarray,
                        yaw_mode: str = 'velocity') -> 'AttitudeCommand':
        """
        计算姿态命令
        
        Args:
            velocity_cmd: 速度命令 (vx, vy, vz, omega in world frame)
            current_state: 当前状态 [px, py, pz, vx, vy, vz, theta, omega]
            yaw_mode: 航向模式 ('velocity', 'fixed', 'manual')
        
        Returns:
            AttitudeCommand: 姿态命令 (roll, pitch, yaw, thrust)
        """
        pass
    
    @abstractmethod
    def set_hover_yaw(self, yaw: float) -> None:
        """设置悬停时的目标航向"""
        pass
    
    @abstractmethod
    def get_attitude_rate_limits(self) -> Dict[str, float]:
        """获取姿态角速度限制"""
        pass
