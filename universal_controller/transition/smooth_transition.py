"""平滑过渡控制器"""
from typing import Dict, Any, Optional
import numpy as np

from ..core.interfaces import ISmoothTransition
from ..core.data_types import ControlOutput
from ..core.ros_compat import get_monotonic_time


class ExponentialSmoothTransition(ISmoothTransition):
    """指数平滑过渡"""
    
    def __init__(self, config: Dict[str, Any]):
        transition_config = config.get('transition', config)
        
        self.tau = transition_config.get('tau', 0.1)
        self.max_duration = transition_config.get('max_duration', 0.5)
        self.completion_threshold = transition_config.get('completion_threshold', 0.95)
        self.start_time: Optional[float] = None
        self.in_transition = False
        self.from_cmd: Optional[ControlOutput] = None
        self.progress = 0.0
    
    def start_transition(self, from_cmd: ControlOutput) -> None:
        self.start_time = get_monotonic_time()
        self.in_transition = True
        self.from_cmd = from_cmd.copy()
        self.progress = 0.0
    
    def get_blended_output(self, new_cmd: ControlOutput, 
                          current_time: float) -> ControlOutput:
        """
        获取混合输出
        
        Args:
            new_cmd: 新的控制命令
            current_time: 当前时间（忽略，使用内部单调时钟）
        """
        if not self.in_transition or self.from_cmd is None:
            return new_cmd
        
        # 使用单调时钟计算经过时间
        monotonic_now = get_monotonic_time()
        elapsed = monotonic_now - self.start_time
        alpha = 1.0 - np.exp(-elapsed / self.tau)
        self.progress = alpha
        
        blended_vx = self.from_cmd.vx * (1 - alpha) + new_cmd.vx * alpha
        blended_vy = self.from_cmd.vy * (1 - alpha) + new_cmd.vy * alpha
        blended_vz = self.from_cmd.vz * (1 - alpha) + new_cmd.vz * alpha
        blended_omega = self.from_cmd.omega * (1 - alpha) + new_cmd.omega * alpha
        
        if alpha >= self.completion_threshold or elapsed > self.max_duration:
            self.in_transition = False
            self.from_cmd = None
            self.start_time = None  # 重置 start_time
            self.progress = 1.0  # 设置为完成状态
            return new_cmd
        
        return ControlOutput(
            vx=blended_vx, vy=blended_vy, vz=blended_vz, omega=blended_omega,
            frame_id=new_cmd.frame_id, success=new_cmd.success,
            solve_time_ms=new_cmd.solve_time_ms,
            health_metrics={'transition_progress': alpha}
        )
    
    def is_complete(self) -> bool:
        return not self.in_transition
    
    def get_progress(self) -> float:
        return self.progress
    
    def reset(self) -> None:
        """重置过渡状态"""
        self.start_time = None
        self.in_transition = False
        self.from_cmd = None
        self.progress = 0.0


class LinearSmoothTransition(ISmoothTransition):
    """线性平滑过渡"""
    
    def __init__(self, config: Dict[str, Any]):
        transition_config = config.get('transition', config)
        
        self.duration = transition_config.get('duration', 0.2)
        self.start_time: Optional[float] = None
        self.in_transition = False
        self.from_cmd: Optional[ControlOutput] = None
        self.progress = 0.0
    
    def start_transition(self, from_cmd: ControlOutput) -> None:
        self.start_time = get_monotonic_time()
        self.in_transition = True
        self.from_cmd = from_cmd.copy()
        self.progress = 0.0
    
    def get_blended_output(self, new_cmd: ControlOutput, 
                          current_time: float) -> ControlOutput:
        """
        获取混合输出
        
        Args:
            new_cmd: 新的控制命令
            current_time: 当前时间（忽略，使用内部单调时钟）
        """
        if not self.in_transition or self.from_cmd is None or self.start_time is None:
            return new_cmd
        
        # 使用单调时钟计算经过时间
        monotonic_now = get_monotonic_time()
        elapsed = monotonic_now - self.start_time
        alpha = min(elapsed / self.duration, 1.0)
        self.progress = alpha
        
        blended_vx = self.from_cmd.vx * (1 - alpha) + new_cmd.vx * alpha
        blended_vy = self.from_cmd.vy * (1 - alpha) + new_cmd.vy * alpha
        blended_vz = self.from_cmd.vz * (1 - alpha) + new_cmd.vz * alpha
        blended_omega = self.from_cmd.omega * (1 - alpha) + new_cmd.omega * alpha
        
        if alpha >= 1.0:
            self.in_transition = False
            self.from_cmd = None
            self.start_time = None  # 重置 start_time
            self.progress = 1.0  # 设置为完成状态
            return new_cmd
        
        return ControlOutput(
            vx=blended_vx, vy=blended_vy, vz=blended_vz, omega=blended_omega,
            frame_id=new_cmd.frame_id, success=new_cmd.success,
            solve_time_ms=new_cmd.solve_time_ms,
            health_metrics={'transition_progress': alpha}
        )
    
    def is_complete(self) -> bool:
        return not self.in_transition
    
    def get_progress(self) -> float:
        return self.progress
    
    def reset(self) -> None:
        """重置过渡状态"""
        self.start_time = None
        self.in_transition = False
        self.from_cmd = None
        self.progress = 0.0
