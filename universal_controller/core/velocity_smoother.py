"""
速度平滑工具

提供统一的速度平滑和限制功能，避免代码重复。
"""
from typing import Optional
import numpy as np

from .data_types import ControlOutput


class VelocitySmoother:
    """
    速度平滑器
    
    用于限制控制命令的加速度，确保平滑的速度变化。
    
    使用示例:
        smoother = VelocitySmoother(a_max=1.5, az_max=1.0, alpha_max=3.0, dt=0.02)
        smoothed_cmd = smoother.smooth(new_cmd, last_cmd)
    """
    
    def __init__(self, a_max: float, az_max: float, alpha_max: float, dt: float):
        """
        初始化速度平滑器
        
        Args:
            a_max: 最大水平加速度 (m/s²)
            az_max: 最大垂直加速度 (m/s²)
            alpha_max: 最大角加速度 (rad/s²)
            dt: 时间步长 (s)
        """
        self.a_max = a_max
        self.az_max = az_max
        self.alpha_max = alpha_max
        self.dt = dt
    
    @property
    def max_dv(self) -> float:
        """最大水平速度变化量"""
        return self.a_max * self.dt
    
    @property
    def max_dvz(self) -> float:
        """最大垂直速度变化量"""
        return self.az_max * self.dt
    
    @property
    def max_domega(self) -> float:
        """最大角速度变化量"""
        return self.alpha_max * self.dt
    
    def smooth(self, cmd: ControlOutput, last_cmd: Optional[ControlOutput]) -> ControlOutput:
        """
        平滑控制命令
        
        对水平速度使用向量限制，确保合成加速度不超过 a_max。
        
        Args:
            cmd: 新的控制命令
            last_cmd: 上一次的控制命令，如果为 None 则不进行平滑
        
        Returns:
            平滑后的控制命令
        """
        if last_cmd is None:
            # 防御性检查: 即使无历史命令，也要拦截 NaN/Inf
            if not (np.isfinite(cmd.vx) and np.isfinite(cmd.vy) and 
                    np.isfinite(cmd.vz) and np.isfinite(cmd.omega)):
                return ControlOutput(vx=0.0, vy=0.0, vz=0.0, omega=0.0,
                                   frame_id=cmd.frame_id, success=False,
                                   health_metrics={'error_type': 'nan_in_smoother'})
            return cmd
        
        # NaN/Inf 源头拦截 - 使用短路求值，正常情况下几乎零开销
        # 这比依赖下游 SafetyMonitor 更早拦截问题，便于定位 NaN 来源
        if not (np.isfinite(cmd.vx) and np.isfinite(cmd.vy) and 
                np.isfinite(cmd.vz) and np.isfinite(cmd.omega)):
            # 返回上一次的安全命令，避免 NaN 传播
            return last_cmd
        
        # 水平速度变化向量限制
        # 确保合成加速度不超过 a_max
        dvx = cmd.vx - last_cmd.vx
        dvy = cmd.vy - last_cmd.vy
        dv_magnitude = np.sqrt(dvx**2 + dvy**2)
        
        if dv_magnitude > self.max_dv:
            # 按比例缩放，保持方向
            scale = self.max_dv / dv_magnitude
            dvx *= scale
            dvy *= scale
        
        smoothed_vx = last_cmd.vx + dvx
        smoothed_vy = last_cmd.vy + dvy
        
        # 垂直速度和角速度仍然独立限制
        smoothed_vz = np.clip(cmd.vz, last_cmd.vz - self.max_dvz, last_cmd.vz + self.max_dvz)
        smoothed_omega = np.clip(cmd.omega, last_cmd.omega - self.max_domega, 
                                 last_cmd.omega + self.max_domega)
        
        return ControlOutput(
            vx=smoothed_vx,
            vy=smoothed_vy,
            vz=smoothed_vz,
            omega=smoothed_omega,
            frame_id=cmd.frame_id,
            success=cmd.success,
            solve_time_ms=cmd.solve_time_ms,
            health_metrics=cmd.health_metrics.copy() if cmd.health_metrics else {},
            extras=cmd.extras.copy() if cmd.extras else {}
        )
    
    def smooth_to_stop(self, last_cmd: Optional[ControlOutput], 
                       frame_id: str = "") -> ControlOutput:
        """
        平滑停止
        
        使用向量限制确保合成加速度不超过 a_max，与 smooth() 方法保持一致。
        
        Args:
            last_cmd: 上一次的控制命令
            frame_id: 输出坐标系
        
        Returns:
            平滑减速的控制命令
        """
        if last_cmd is None:
            return ControlOutput(vx=0.0, vy=0.0, vz=0.0, omega=0.0, 
                               frame_id=frame_id, success=True)
        
        # NaN/Inf 检测 - 与 smooth() 方法保持一致
        # 如果历史命令包含 NaN，直接返回零速度（已经在停止了）
        if not (np.isfinite(last_cmd.vx) and np.isfinite(last_cmd.vy) and 
                np.isfinite(last_cmd.vz) and np.isfinite(last_cmd.omega)):
            return ControlOutput(vx=0.0, vy=0.0, vz=0.0, omega=0.0, 
                               frame_id=frame_id, success=True)
        
        # 水平速度使用向量限制，确保合成加速度不超过 a_max
        v_horizontal = np.sqrt(last_cmd.vx**2 + last_cmd.vy**2)
        if v_horizontal <= self.max_dv:
            # 可以在一步内停止
            new_vx, new_vy = 0.0, 0.0
        elif v_horizontal > 1e-9:
            # 按比例缩减，保持速度方向
            scale = (v_horizontal - self.max_dv) / v_horizontal
            new_vx = last_cmd.vx * scale
            new_vy = last_cmd.vy * scale
        else:
            new_vx, new_vy = 0.0, 0.0
        
        # 垂直速度和角速度独立处理
        def smooth_scalar_to_zero(value: float, max_change: float) -> float:
            if abs(value) <= max_change:
                return 0.0
            return value - max_change if value > 0 else value + max_change
        
        return ControlOutput(
            vx=new_vx,
            vy=new_vy,
            vz=smooth_scalar_to_zero(last_cmd.vz, self.max_dvz),
            omega=smooth_scalar_to_zero(last_cmd.omega, self.max_domega),
            frame_id=frame_id,
            success=True
        )


def clip_velocity(vx: float, vy: float, last_vx: float, last_vy: float,
                  max_dv: float) -> tuple:
    """
    限制水平速度变化（分量独立限制）
    
    .. deprecated::
        此函数使用分量独立限制，可能允许合成加速度超过 √2 * max_dv。
        推荐使用 `VelocitySmoother.smooth()` 方法，它使用向量限制确保合成加速度不超过限制。
    
    注意:
        - 此函数分别限制 vx 和 vy 的变化量
        - 对角方向运动时，合成加速度可能达到 √2 * a_max
        - 对于安全关键场景，请使用 VelocitySmoother 类
    
    Args:
        vx, vy: 新的速度
        last_vx, last_vy: 上一次的速度
        max_dv: 最大速度变化量（每个分量）
    
    Returns:
        (clipped_vx, clipped_vy): 限制后的速度
    """
    import warnings
    warnings.warn(
        "clip_velocity() is deprecated. Use VelocitySmoother.smooth() for proper vector limiting.",
        DeprecationWarning,
        stacklevel=2
    )
    clipped_vx = np.clip(vx, last_vx - max_dv, last_vx + max_dv)
    clipped_vy = np.clip(vy, last_vy - max_dv, last_vy + max_dv)
    return clipped_vx, clipped_vy
