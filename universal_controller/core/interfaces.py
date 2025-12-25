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


class ILifecycleComponent(ABC):
    """
    生命周期组件基础接口
    
    所有需要生命周期管理的组件都应实现此接口。
    提供统一的 reset() 和 shutdown() 方法。
    
    生命周期说明:
    - reset(): 重置内部状态，保留资源，可继续使用
    - shutdown(): 释放所有资源，对象不应再使用
    
    设计原则:
    - reset() 是必须实现的，用于状态重置
    - shutdown() 有默认空实现，只有持有外部资源的组件需要覆盖
    """
    
    @abstractmethod
    def reset(self) -> None:
        """
        重置组件内部状态
        
        调用后组件应恢复到初始状态，但保留已分配的资源。
        可以继续调用其他方法。
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
        """
        pass


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
