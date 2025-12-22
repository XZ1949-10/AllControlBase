"""Pure Pursuit 备用控制器"""
from typing import Dict, Any, Optional, List, Tuple
import numpy as np

from ..core.interfaces import ITrajectoryTracker
from ..core.data_types import Trajectory, ControlOutput, ConsistencyResult, Point3D
from ..core.enums import PlatformType, HeadingMode
from ..core.ros_compat import normalize_angle, angle_difference
from ..core.velocity_smoother import VelocitySmoother
from ..config.default_config import PLATFORM_CONFIG


class PurePursuitController(ITrajectoryTracker):
    """Pure Pursuit 备用控制器"""
    
    def __init__(self, config: Dict[str, Any], platform_config: Dict[str, Any]):
        backup_config = config.get('backup', config)
        
        self.lookahead_dist = backup_config.get('lookahead_dist', 1.0)
        self.min_lookahead = backup_config.get('min_lookahead', 0.5)
        self.max_lookahead = backup_config.get('max_lookahead', 3.0)
        self.lookahead_ratio = backup_config.get('lookahead_ratio', 0.5)
        self.kp_z = backup_config.get('kp_z', 1.0)
        self.dt = backup_config.get('dt', 0.02)
        
        constraints = config.get('constraints', platform_config.get('constraints', {}))
        self.v_max = constraints.get('v_max', 2.0)
        self.v_min = constraints.get('v_min', 0.0)
        self.omega_max = constraints.get('omega_max', 2.0)
        self.omega_max_low = constraints.get('omega_max_low', 1.0)
        self.v_low_thresh = constraints.get('v_low_thresh', 0.1)
        self.a_max = constraints.get('a_max', 1.5)
        self.az_max = constraints.get('az_max', 1.0)
        self.alpha_max = constraints.get('alpha_max', 3.0)
        self.vz_max = constraints.get('vz_max', 2.0)
        
        self.vx_max = constraints.get('vx_max', self.v_max)
        self.vx_min = constraints.get('vx_min', -self.v_max)
        self.vy_max = constraints.get('vy_max', self.v_max)
        self.vy_min = constraints.get('vy_min', -self.v_max)
        
        self.output_type = platform_config.get('output_type', 'differential')
        self.output_frame = platform_config.get('output_frame', 'base_link')
        self.platform_type = platform_config.get('type', PlatformType.DIFFERENTIAL)
        self.is_3d = self.platform_type == PlatformType.QUADROTOR
        self.is_omni = self.platform_type == PlatformType.OMNI
        
        self.heading_mode = self._parse_heading_mode(backup_config.get('heading_mode', 'follow_velocity'))
        self.kp_heading = backup_config.get('kp_heading', 1.5)
        self.fixed_heading: Optional[float] = backup_config.get('fixed_heading')
        
        # Pure Pursuit 控制参数 - 从配置读取
        self.heading_error_thresh = backup_config.get('heading_error_thresh', np.pi / 3)  # ~60°
        self.pure_pursuit_angle_thresh = backup_config.get('pure_pursuit_angle_thresh', np.pi / 3)  # ~60°
        self.heading_control_angle_thresh = backup_config.get('heading_control_angle_thresh', np.pi / 2)  # ~90°
        self.max_curvature = backup_config.get('max_curvature', 5.0)
        self.min_turn_speed = backup_config.get('min_turn_speed', 0.1)
        self.default_speed_ratio = backup_config.get('default_speed_ratio', 0.5)  # 默认速度比例
        
        # 低速过渡和距离阈值参数 - 从配置读取
        self.low_speed_transition_factor = backup_config.get('low_speed_transition_factor', 0.5)
        self.curvature_speed_limit_thresh = backup_config.get('curvature_speed_limit_thresh', 0.1)
        self.min_distance_thresh = backup_config.get('min_distance_thresh', 0.1)
        
        # 角速度变化率限制 (用于防止目标点在正后方时的跳变)
        # 默认使用 alpha_max * dt，但可以单独配置
        omega_rate_config = backup_config.get('omega_rate_limit')
        if omega_rate_config is not None:
            self.omega_rate_limit = omega_rate_config
        else:
            self.omega_rate_limit = self.alpha_max * self.dt
        
        self.last_cmd: Optional[ControlOutput] = None
        self._horizon: int = 20
        self._current_position: Optional[np.ndarray] = None
        self._manual_heading: Optional[float] = None
        self._is_shutdown = False
        
        # 速度平滑器
        self._velocity_smoother = VelocitySmoother(
            a_max=self.a_max, az_max=self.az_max, 
            alpha_max=self.alpha_max, dt=self.dt
        )
    
    def _parse_heading_mode(self, mode_str: str) -> HeadingMode:
        mode_map = {
            'follow_velocity': HeadingMode.FOLLOW_VELOCITY,
            'fixed': HeadingMode.FIXED,
            'target_point': HeadingMode.TARGET_POINT,
            'manual': HeadingMode.MANUAL
        }
        return mode_map.get(mode_str.lower(), HeadingMode.FOLLOW_VELOCITY)
    
    def set_manual_heading(self, heading: float) -> None:
        self._manual_heading = heading
    
    def shutdown(self) -> None:
        self._is_shutdown = True
    
    def compute(self, state: np.ndarray, trajectory: Trajectory, 
                consistency: ConsistencyResult) -> ControlOutput:
        if len(trajectory.points) < 2:
            return self._smooth_stop_command()
        
        px, py, pz = state[0], state[1], state[2]
        current_v = np.sqrt(state[3]**2 + state[4]**2)
        theta = state[6]
        
        self._current_position = np.array([px, py, pz])
        
        lookahead = self._compute_lookahead(current_v)
        lookahead_point, target_idx = self._find_lookahead_point(
            trajectory.points, px, py, lookahead)
        
        if lookahead_point is None:
            return self._smooth_stop_command()
        
        dx = lookahead_point.x - px
        dy = lookahead_point.y - py
        dist_to_target = np.sqrt(dx**2 + dy**2)
        target_v = self._compute_target_velocity(trajectory, target_idx)
        
        vz = 0.0
        if self.is_3d and target_idx < len(trajectory.points):
            target_z = trajectory.points[target_idx].z
            vz = self.kp_z * (target_z - pz)
            vz = np.clip(vz, -self.vz_max, self.vz_max)
        
        omega_limit = self._get_omega_limit(current_v)
        
        if self.output_type == 'differential':
            cmd = self._compute_differential_output(dx, dy, theta, target_v, dist_to_target, omega_limit)
        elif self.output_type == 'omni':
            cmd = self._compute_omni_output(dx, dy, theta, target_v, dist_to_target, trajectory, target_idx, omega_limit)
        else:
            cmd = self._compute_3d_output(dx, dy, theta, target_v, dist_to_target, vz, trajectory, target_idx, omega_limit)
        
        cmd = self._apply_velocity_smoothing(cmd)
        
        if self.is_omni:
            cmd = self._apply_omni_constraints(cmd)
        
        self.last_cmd = cmd
        return cmd
    
    def get_health_metrics(self) -> Dict[str, Any]:
        return {'type': 'pure_pursuit', 'active': True}
    
    def set_horizon(self, horizon: int) -> None:
        self._horizon = horizon
    
    def _get_omega_limit(self, current_v: float) -> float:
        """获取角速度限制，使用线性插值避免阶跃切换"""
        # 使用配置的低速过渡因子
        low_speed_boundary = self.v_low_thresh * self.low_speed_transition_factor
        if current_v < low_speed_boundary:
            # 非常低速时使用低速限制
            return self.omega_max_low
        elif current_v < self.v_low_thresh:
            # 过渡区域：线性插值
            ratio = (current_v - low_speed_boundary) / (self.v_low_thresh - low_speed_boundary)
            return self.omega_max_low + ratio * (self.omega_max - self.omega_max_low)
        return self.omega_max
    
    def _compute_differential_output(self, dx: float, dy: float, theta: float,
                                     target_v: float, dist_to_target: float,
                                     omega_limit: float) -> ControlOutput:
        """
        计算差速车控制输出
        
        使用 Pure Pursuit 算法计算曲率和角速度。
        当目标点在车辆后方时，使用航向误差控制而非曲率控制。
        
        Args:
            dx, dy: 目标点相对于当前位置的世界坐标系位移
            theta: 当前航向角
            target_v: 目标速度
            dist_to_target: 到目标点的距离
            omega_limit: 角速度限制
        
        Returns:
            ControlOutput: 控制命令 (机体坐标系)
        """
        # 转换到机体坐标系
        local_x = dx * np.cos(theta) + dy * np.sin(theta)
        local_y = -dx * np.sin(theta) + dy * np.cos(theta)
        
        L_sq = local_x**2 + local_y**2
        
        if L_sq > 1e-6:
            # 计算目标点相对于车辆的角度
            # 当目标点在正前方时，angle = 0
            # 当目标点在正侧方时，angle = ±90°
            # 当目标点在正后方时，angle = ±180°
            target_angle = np.arctan2(local_y, local_x)
            abs_target_angle = abs(target_angle)
            
            if abs_target_angle > self.heading_control_angle_thresh:
                # 目标点在车辆后方，使用航向误差控制
                # 差速车可以原地转向，所以先转向再前进
                target_heading = np.arctan2(dy, dx)
                heading_error = angle_difference(target_heading, theta)
                
                # 计算期望角速度
                omega_desired = self.kp_heading * heading_error
                omega_desired = np.clip(omega_desired, -omega_limit, omega_limit)
                
                # 应用角速度变化率限制，防止跳变
                # 特别是当目标点在正后方时（heading_error ≈ ±π）
                if self.last_cmd is not None:
                    omega_change = omega_desired - self.last_cmd.omega
                    omega_change = np.clip(omega_change, -self.omega_rate_limit, self.omega_rate_limit)
                    omega = self.last_cmd.omega + omega_change
                else:
                    omega = omega_desired
                
                omega = np.clip(omega, -omega_limit, omega_limit)
                
                # 当航向误差较大时，减速或停止前进
                # 使用平滑的速度衰减函数，避免突变
                abs_heading_error = abs(heading_error)
                if abs_heading_error > self.heading_error_thresh:
                    # 航向误差大于阈值，完全停止前进，专注于旋转
                    vx = 0.0
                else:
                    # 航向误差小于阈值，使用余弦函数平滑过渡
                    # 当 heading_error = 0 时，heading_factor = 1
                    # 当 heading_error = heading_error_thresh 时，heading_factor ≈ 0.5
                    heading_factor = np.cos(abs_heading_error)
                    # 额外的线性衰减，确保在阈值处速度较低
                    linear_factor = 1.0 - abs_heading_error / self.heading_error_thresh * 0.5
                    vx = target_v * heading_factor * linear_factor
                
                return ControlOutput(vx=vx, vy=0.0, vz=0.0, omega=omega, 
                                    frame_id=self.output_frame, success=True)
            
            elif abs_target_angle > self.pure_pursuit_angle_thresh:
                # 过渡区域：目标点在侧前方
                # 混合 Pure Pursuit 和航向误差控制
                # 使用线性插值
                blend_factor = (abs_target_angle - self.pure_pursuit_angle_thresh) / (self.heading_control_angle_thresh - self.pure_pursuit_angle_thresh)
                
                # Pure Pursuit 部分
                curvature = 2.0 * local_y / L_sq
                curvature = np.clip(curvature, -self.max_curvature, self.max_curvature)
                
                if abs(curvature) > self.curvature_speed_limit_thresh:
                    v_curvature = min(omega_limit / abs(curvature), self.v_max)
                    pp_target_v = min(target_v, v_curvature)
                else:
                    pp_target_v = target_v
                
                pp_omega = pp_target_v * curvature
                pp_omega = np.clip(pp_omega, -omega_limit, omega_limit)
                
                # 航向误差控制部分
                target_heading = np.arctan2(dy, dx)
                heading_error = angle_difference(target_heading, theta)
                hc_omega = self.kp_heading * heading_error
                hc_omega = np.clip(hc_omega, -omega_limit, omega_limit)
                
                # 混合
                omega = (1 - blend_factor) * pp_omega + blend_factor * hc_omega
                omega = np.clip(omega, -omega_limit, omega_limit)
                
                # 速度也需要混合：Pure Pursuit 使用 pp_target_v，航向控制使用降低的速度
                abs_heading_error = abs(heading_error)
                if abs_heading_error > self.heading_error_thresh:
                    hc_vx = 0.0
                else:
                    heading_factor = np.cos(abs_heading_error)
                    linear_factor = 1.0 - abs_heading_error / self.heading_error_thresh * 0.5
                    hc_vx = target_v * heading_factor * linear_factor
                
                vx = (1 - blend_factor) * pp_target_v + blend_factor * hc_vx
                
                return ControlOutput(vx=vx, vy=0.0, vz=0.0, omega=omega, 
                                    frame_id=self.output_frame, success=True)
            else:
                # 目标点在车辆前方，使用标准 Pure Pursuit 曲率控制
                curvature = 2.0 * local_y / L_sq
                # 限制曲率范围，避免数值爆炸
                curvature = np.clip(curvature, -self.max_curvature, self.max_curvature)
        else:
            curvature = 0.0
        
        # 根据曲率限制速度
        if abs(curvature) > self.curvature_speed_limit_thresh:
            v_curvature = min(omega_limit / abs(curvature), self.v_max)
            target_v = min(target_v, v_curvature)
        
        omega = target_v * curvature
        omega = np.clip(omega, -omega_limit, omega_limit)
        
        return ControlOutput(vx=target_v, vy=0.0, vz=0.0, omega=omega, 
                            frame_id=self.output_frame, success=True)
    
    def _compute_omni_output(self, dx: float, dy: float, theta: float,
                            target_v: float, dist_to_target: float,
                            trajectory: Trajectory, target_idx: int,
                            omega_limit: float) -> ControlOutput:
        if dist_to_target > 1e-6:
            vx_world = target_v * dx / dist_to_target
            vy_world = target_v * dy / dist_to_target
        else:
            vx_world = 0.0
            vy_world = 0.0
        
        omega = self._compute_heading_control(dx, dy, theta, dist_to_target, trajectory, target_idx, omega_limit)
        
        return ControlOutput(vx=vx_world, vy=vy_world, vz=0.0, omega=omega,
                            frame_id=self.output_frame, success=True)
    
    def _compute_3d_output(self, dx: float, dy: float, theta: float,
                          target_v: float, dist_to_target: float, vz: float,
                          trajectory: Trajectory, target_idx: int,
                          omega_limit: float) -> ControlOutput:
        if dist_to_target > 1e-6:
            vx_world = target_v * dx / dist_to_target
            vy_world = target_v * dy / dist_to_target
        else:
            vx_world = 0.0
            vy_world = 0.0
        
        omega = self._compute_heading_control(dx, dy, theta, dist_to_target, trajectory, target_idx, omega_limit)
        
        return ControlOutput(vx=vx_world, vy=vy_world, vz=vz, omega=omega,
                            frame_id=self.output_frame, success=True)
    
    def _compute_heading_control(self, dx: float, dy: float, theta: float,
                                 dist_to_target: float, trajectory: Trajectory,
                                 target_idx: int, omega_limit: float) -> float:
        if self.heading_mode == HeadingMode.FOLLOW_VELOCITY:
            if dist_to_target > self.min_distance_thresh:
                target_heading = np.arctan2(dy, dx)
                heading_error = angle_difference(target_heading, theta)
                omega = self.kp_heading * heading_error
            else:
                omega = 0.0
        elif self.heading_mode == HeadingMode.FIXED:
            if self.fixed_heading is not None:
                heading_error = angle_difference(self.fixed_heading, theta)
                omega = self.kp_heading * heading_error
            else:
                omega = 0.0
        elif self.heading_mode == HeadingMode.TARGET_POINT:
            if len(trajectory.points) > 0 and self._current_position is not None:
                end_point = trajectory.points[-1]
                px, py = self._current_position[0], self._current_position[1]
                target_heading = np.arctan2(end_point.y - py, end_point.x - px)
                heading_error = angle_difference(target_heading, theta)
                omega = self.kp_heading * heading_error
            else:
                omega = 0.0
        elif self.heading_mode == HeadingMode.MANUAL:
            if self._manual_heading is not None:
                heading_error = angle_difference(self._manual_heading, theta)
                omega = self.kp_heading * heading_error
            else:
                omega = 0.0
        else:
            omega = 0.0
        
        return np.clip(omega, -omega_limit, omega_limit)

    
    def _apply_velocity_smoothing(self, cmd: ControlOutput) -> ControlOutput:
        """使用 VelocitySmoother 平滑控制命令"""
        return self._velocity_smoother.smooth(cmd, self.last_cmd)
    
    def _smooth_stop_command(self) -> ControlOutput:
        """使用 VelocitySmoother 平滑停止"""
        cmd = self._velocity_smoother.smooth_to_stop(self.last_cmd, self.output_frame)
        self.last_cmd = cmd
        return cmd
    
    def _compute_lookahead(self, current_v: float) -> float:
        lookahead = self.lookahead_dist + self.lookahead_ratio * current_v
        return np.clip(lookahead, self.min_lookahead, self.max_lookahead)
    
    def _find_lookahead_point(self, points: List[Point3D], px: float, py: float, 
                              lookahead: float) -> Tuple[Optional[Point3D], int]:
        min_dist = float('inf')
        closest_idx = 0
        for i, p in enumerate(points):
            dist = np.sqrt((p.x - px)**2 + (p.y - py)**2)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        
        for i in range(closest_idx, len(points)):
            p = points[i]
            dist = np.sqrt((p.x - px)**2 + (p.y - py)**2)
            if dist >= lookahead:
                return p, i
        
        if len(points) > 0:
            return points[-1], len(points) - 1
        return None, 0
    
    def _compute_target_velocity(self, trajectory: Trajectory, target_idx: int) -> float:
        if trajectory.soft_enabled and trajectory.velocities is not None:
            try:
                if target_idx < len(trajectory.velocities):
                    vel = trajectory.velocities[target_idx]
                    # 处理不同的数组形状
                    if isinstance(vel, np.ndarray):
                        if vel.ndim == 0:
                            # 0维数组 (标量)
                            v_soft = abs(float(vel))
                        elif len(vel) >= 2:
                            # 多维速度向量
                            v_soft = np.sqrt(vel[0]**2 + vel[1]**2)
                        elif len(vel) == 1:
                            # 单元素数组
                            v_soft = abs(float(vel[0]))
                        else:
                            v_soft = self.v_max * self.default_speed_ratio
                    elif hasattr(vel, '__len__') and len(vel) >= 2:
                        # 列表或元组
                        v_soft = np.sqrt(vel[0]**2 + vel[1]**2)
                    elif np.isscalar(vel):
                        # 纯标量
                        v_soft = abs(float(vel))
                    else:
                        v_soft = self.v_max * self.default_speed_ratio
                    target_v = min(v_soft, self.v_max)
                else:
                    target_v = self.v_max * self.default_speed_ratio
            except (IndexError, TypeError, ValueError):
                # 数组形状不正确，使用默认值
                target_v = self.v_max * self.default_speed_ratio
        else:
            target_v = self.v_max * self.default_speed_ratio
        
        v_min_forward = max(0.0, self.v_min)
        return np.clip(target_v, v_min_forward, self.v_max)
    
    def _apply_omni_constraints(self, cmd: ControlOutput) -> ControlOutput:
        constrained_vx = np.clip(cmd.vx, self.vx_min, self.vx_max)
        constrained_vy = np.clip(cmd.vy, self.vy_min, self.vy_max)
        
        return ControlOutput(
            vx=constrained_vx, vy=constrained_vy, vz=cmd.vz, omega=cmd.omega,
            frame_id=cmd.frame_id, success=cmd.success,
            health_metrics=cmd.health_metrics.copy() if cmd.health_metrics else {}
        )
