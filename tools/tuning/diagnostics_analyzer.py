#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
诊断数据分析器 v3.0

分析控制器诊断数据，识别性能问题并生成调优建议。

设计原则：
1. 基于统计数据（均值、最大值、95%分位数）做决策
2. 使用置信度机制过滤不确定的建议
3. 按严重程度分类（critical > warning > info）
4. 明确区分三类参数：
   - 可调优参数：基于运行数据可以安全调整
   - 设计参数：需要系统辨识或专业知识，不应自动调整
   - 安全参数：涉及安全，不应自动放宽

v3.0 重构内容：
- 移除对设计参数的自动调优（EKF噪声、一致性阈值等）
- 移除对安全参数的自动放宽（速度/加速度限制等）
- 移除基于命令变化率的控制平滑度分析（数据不可靠）
- 统一参数分类，明确标注不建议调优的参数
- 简化置信度计算，使用更保守的策略
"""

import numpy as np
from dataclasses import dataclass, field
from typing import Dict, List, Any, Optional
from enum import Enum

# 直接从核心模块导入枚举，确保与控制器定义一致
from universal_controller.core.enums import ControllerState


class TuningCategory(Enum):
    """参数调优分类"""
    TUNABLE = "tunable"           # 可调优：基于运行数据可安全调整
    DESIGN = "design"             # 设计参数：需要系统辨识，不应自动调整
    SAFETY = "safety"             # 安全参数：不应自动放宽
    DIAGNOSTIC = "diagnostic"     # 诊断参数：仅用于显示，不影响控制


@dataclass
class AnalysisResult:
    """分析结果"""
    category: str           # 分类: mpc, tracking, timeout, safety, consistency, etc.
    severity: str           # 严重程度: info, warning, critical
    parameter: str          # 相关参数路径 (如 "mpc.weights.position")
    current_value: Any      # 当前值
    suggested_value: Any    # 建议值
    reason: str             # 原因说明
    confidence: float       # 置信度 [0, 1]
    tuning_category: TuningCategory = TuningCategory.TUNABLE  # 调优分类


# =============================================================================
# 参数分类定义
# =============================================================================
# 明确哪些参数可以自动调优，哪些不应该
# 
# 参数来源说明:
# - turtlebot1.yaml 只覆盖与默认值不同的配置项
# - 未在 turtlebot1.yaml 中定义的参数使用 universal_controller/config/*.py 的默认值
# - 调优工具可以分析所有参数，如果需要调整会在生成的配置中添加
#
# 参数标记说明:
# - [turtlebot1.yaml] 表示该参数在 turtlebot1.yaml 中有定义
# - [controller_params.yaml] 表示该参数在 controller_params.yaml 中定义
# - [internal_params.yaml] 表示该参数在 internal_params.yaml 中定义
# - [默认值] 表示该参数使用 universal_controller 的默认值

# 可调优参数：基于运行数据可以安全调整
TUNABLE_PARAMS = {
    # =========================================================================
    # 超时配置 - 基于实际延迟统计
    # 默认值来源: universal_controller/config/system_config.py WATCHDOG_CONFIG
    # =========================================================================
    'watchdog.odom_timeout_ms',      # [turtlebot1.yaml] 500
    'watchdog.traj_timeout_ms',      # [turtlebot1.yaml] 1000
    'watchdog.traj_grace_ms',        # [turtlebot1.yaml] 600
    'watchdog.imu_timeout_ms',       # [turtlebot1.yaml] -1 (禁用)
    'watchdog.startup_grace_ms',     # [turtlebot1.yaml] 5000
    'watchdog.absolute_startup_timeout_ms',  # [turtlebot1.yaml] -1 (禁用)
    
    # =========================================================================
    # MPC 健康监控阈值 - 基于求解时间统计
    # 默认值来源: universal_controller/config/mpc_config.py MPC_CONFIG['health_monitor']
    # =========================================================================
    'mpc.health_monitor.time_warning_thresh_ms',   # [turtlebot1.yaml] 20
    'mpc.health_monitor.time_critical_thresh_ms',  # [turtlebot1.yaml] 40
    'mpc.health_monitor.time_recovery_thresh_ms',  # [turtlebot1.yaml] 15
    'mpc.health_monitor.consecutive_warning_limit', # [turtlebot1.yaml] 10
    'mpc.health_monitor.consecutive_recovery_limit', # [turtlebot1.yaml] 5
    
    # =========================================================================
    # MPC 预测时域 - 基于求解时间与控制周期关系
    # 默认值来源: universal_controller/config/mpc_config.py MPC_CONFIG
    # =========================================================================
    'mpc.horizon',           # [turtlebot1.yaml] 7
    'mpc.horizon_degraded',  # [turtlebot1.yaml] 4
    'mpc.dt',                # [turtlebot1.yaml] 0.1
    
    # =========================================================================
    # MPC 权重 - 基于跟踪误差统计
    # 默认值来源: universal_controller/config/mpc_config.py MPC_CONFIG['weights']
    # =========================================================================
    'mpc.weights.position',  # [turtlebot1.yaml] 15.0
    'mpc.weights.velocity',  # [turtlebot1.yaml] 6.0
    'mpc.weights.heading',   # [turtlebot1.yaml] 8.0
    
    # =========================================================================
    # MPC Fallback 配置
    # =========================================================================
    'mpc.fallback.lookahead_steps',  # [turtlebot1.yaml] 3
    
    # =========================================================================
    # 状态机参数 - 基于状态转换统计
    # 默认值来源: universal_controller/config/safety_config.py SAFETY_CONFIG['state_machine']
    # =========================================================================
    'safety.state_machine.mpc_fail_thresh',          # [turtlebot1.yaml] 3
    'safety.state_machine.mpc_fail_window_size',     # [turtlebot1.yaml] 10
    'safety.state_machine.mpc_recovery_thresh',      # [turtlebot1.yaml] 5
    'safety.state_machine.mpc_recovery_tolerance',   # [turtlebot1.yaml] 1
    
    # =========================================================================
    # 跟踪质量阈值 - 仅影响 Dashboard 显示，不影响控制
    # 默认值来源: universal_controller/config/system_config.py TRACKING_CONFIG
    # =========================================================================
    'tracking.lateral_thresh',       # [turtlebot1.yaml] 0.25
    'tracking.longitudinal_thresh',  # [turtlebot1.yaml] 0.6
    'tracking.heading_thresh',       # [turtlebot1.yaml] 0.5
    
    # =========================================================================
    # 坐标变换配置
    # 默认值来源: universal_controller/config/transform_config.py
    # =========================================================================
    'transform.timeout_ms',          # [turtlebot1.yaml] 50
    
    # =========================================================================
    # 备份控制器参数 - 基于备份激活时的表现
    # 默认值来源: universal_controller/config/backup_config.py
    # =========================================================================
    'backup.lookahead_dist',         # [turtlebot1.yaml] 0.5
    'backup.min_lookahead',          # [turtlebot1.yaml] 0.3
    'backup.max_lookahead',          # [turtlebot1.yaml] 1.5
    'backup.lookahead_ratio',        # [turtlebot1.yaml] 0.5
    'backup.kp_heading',             # [turtlebot1.yaml] 2.0
    'backup.heading_error_thresh',   # [turtlebot1.yaml] 1.047
    'backup.max_curvature',          # [turtlebot1.yaml] 5.0
    'backup.default_speed_ratio',    # [turtlebot1.yaml] 0.5
    'backup.min_distance_thresh',    # [turtlebot1.yaml] 0.1
    
    # =========================================================================
    # 轨迹配置
    # 默认值来源: universal_controller/config/trajectory_config.py
    # =========================================================================
    'trajectory.low_speed_thresh',   # [turtlebot1.yaml] 0.05
    'trajectory.min_points',         # [turtlebot1.yaml] 2
    'trajectory.max_points',         # [turtlebot1.yaml] 100
    'trajectory.max_point_distance', # [turtlebot1.yaml] 10.0
    'trajectory.default_dt_sec',     # [turtlebot1.yaml] 0.1
    
    # =========================================================================
    # 诊断配置
    # 默认值来源: universal_controller/config/system_config.py DIAGNOSTICS_CONFIG
    # =========================================================================
    'diagnostics.publish_rate',      # [turtlebot1.yaml] 5
    
    # =========================================================================
    # 一致性检查参数 - 基于轨迹频率和响应需求调优
    # 默认值来源: universal_controller/config/consistency_config.py
    # v4.3 更新: 从 DESIGN_PARAMS 移至 TUNABLE_PARAMS
    # =========================================================================
    'consistency.alpha_min',             # [turtlebot1.yaml] 0.1 - 基于 alpha 分布调优
    'consistency.kappa_thresh',          # [turtlebot1.yaml] 0.5 - 基于曲率一致性得分调优
    'consistency.v_dir_thresh',          # [turtlebot1.yaml] 0.8 - 基于速度方向一致性得分调优
    'consistency.temporal_smooth_thresh', # [turtlebot1.yaml] 0.5 - 基于时序平滑度得分调优
    'consistency.max_curvature',         # [turtlebot1.yaml] 10.0 - 基于实际曲率范围调优
    'consistency.temporal_window_size',  # [turtlebot1.yaml] 10 - 基于轨迹频率调优
    'consistency.weights.kappa',         # [turtlebot1.yaml] 0.3 - 基于各维度得分分布调优
    'consistency.weights.velocity',      # [turtlebot1.yaml] 0.3 - 基于各维度得分分布调优
    'consistency.weights.temporal',      # [turtlebot1.yaml] 0.4 - 基于各维度得分分布调优
    
    # =========================================================================
    # 低速保护阈值 - 影响角速度限制的过渡
    # 默认值来源: universal_controller/config/safety_config.py CONSTRAINTS_CONFIG
    # =========================================================================
    'constraints.v_low_thresh',      # [turtlebot1.yaml] 0.1
    
    # =========================================================================
    # controller_params.yaml 中的可调优参数 (ROS 层)
    # 基于 TF2 降级统计可靠调优
    # =========================================================================
    'transform.fallback_duration_limit_ms',   # [controller_params.yaml] 500
    'transform.fallback_critical_limit_ms',   # [controller_params.yaml] 1000
    
    # =========================================================================
    # internal_params.yaml 中的可调优参数 (算法层)
    # 基于运行数据可靠调优
    # =========================================================================
    # 状态机内部参数 - 基于 MPC 失败/恢复统计
    'safety.state_machine.mpc_fail_ratio_thresh',      # [internal_params.yaml] 0.5
    'safety.state_machine.mpc_recovery_success_ratio', # [internal_params.yaml] 0.8
    'safety.state_machine.degraded_state_timeout',     # [internal_params.yaml] 30.0
    'safety.state_machine.backup_state_timeout',       # [internal_params.yaml] 60.0
    
    # 跟踪质量评估 - 基于预测误差统计
    'tracking.prediction_thresh',    # [internal_params.yaml] 0.5
}

# 设计参数：需要系统辨识或专业知识，不应自动调整
DESIGN_PARAMS = {
    # =========================================================================
    # MPC 控制输入权重 - 需要控制理论知识
    # 默认值来源: universal_controller/config/mpc_config.py MPC_CONFIG['weights']
    # =========================================================================
    'mpc.weights.control_accel',         # [turtlebot1.yaml] 0.1
    'mpc.weights.control_alpha',         # [turtlebot1.yaml] 0.1
    
    # =========================================================================
    # 安全状态机设计参数
    # =========================================================================
    'safety.state_machine.alpha_disable_thresh',  # [turtlebot1.yaml] 0.0
}

# 安全参数：不应自动放宽，只检测配置错误
SAFETY_PARAMS = {
    # =========================================================================
    # 速度/加速度约束 - 安全限制
    # 默认值来源: universal_controller/config/safety_config.py CONSTRAINTS_CONFIG
    # =========================================================================
    'constraints.v_max',         # [turtlebot1.yaml] 0.5
    'constraints.v_min',         # [turtlebot1.yaml] -0.2
    'constraints.omega_max',     # [turtlebot1.yaml] 1.0
    'constraints.omega_max_low', # [turtlebot1.yaml] 0.5
    'constraints.a_max',         # [turtlebot1.yaml] 0.5
    'constraints.alpha_max',     # [turtlebot1.yaml] 1.5
    
    # =========================================================================
    # 安全配置
    # 默认值来源: universal_controller/config/safety_config.py SAFETY_CONFIG
    # =========================================================================
    'safety.emergency_decel',    # [turtlebot1.yaml] 1.0
    'safety.v_stop_thresh',      # [turtlebot1.yaml] 0.05
    'safety.stopping_timeout',   # [turtlebot1.yaml] 5.0
}

# 差速车平台不使用的参数（四旋翼/全向车专用）
NON_DIFFERENTIAL_PARAMS = {
    'constraints.vx_max', 'constraints.vx_min',
    'constraints.vy_max', 'constraints.vy_min',
    'constraints.vz_max', 'constraints.az_max',
    'backup.kp_z',
    'safety.vz_stop_thresh',
}

# cmd_vel_adapter 参数（不在调优范围内，但需要保持格式一致）
CMD_VEL_ADAPTER_PARAMS = {
    'cmd_vel_adapter.publish_rate',
    'cmd_vel_adapter.cmd_timeout',
    'cmd_vel_adapter.enable_rate_limit',
    'cmd_vel_adapter.joy_topic',
    'cmd_vel_adapter.mode_topic',
    'cmd_vel_adapter.output_topic',
}

# 话题配置参数（不在调优范围内，但需要保持格式一致）
TOPICS_PARAMS = {
    'topics.odom',
    'topics.imu',
    'topics.trajectory',
    'topics.cmd_unified',
}

# 系统配置参数（不在调优范围内）
SYSTEM_PARAMS = {
    'system.platform',
    'system.ctrl_freq',
}

# 坐标变换配置参数（部分可调优）
TRANSFORM_PARAMS = {
    'transform.source_frame',    # 不可调优
    'transform.target_frame',    # 不可调优
    'transform.timeout_ms',      # 可调优
}


@dataclass
class DiagnosticsStats:
    """诊断统计数据
    
    字段命名规范:
    - xxx_count: 事件发生次数
    - xxx_state_count: 处于某状态的样本数 (与 ControllerState 枚举对应)
    
    注意: 诊断消息中的 backup_active 布尔字段等价于 state == BACKUP_ACTIVE，
    因此统一使用 backup_active_state_count，不再单独统计 backup_active 标志。
    """
    # MPC 相关
    mpc_solve_times: List[float] = field(default_factory=list)
    mpc_success_count: int = 0
    mpc_fail_count: int = 0
    kkt_residuals: List[float] = field(default_factory=list)
    condition_numbers: List[float] = field(default_factory=list)
    mpc_degradation_warnings: int = 0
    mpc_consecutive_near_timeouts: List[int] = field(default_factory=list)
    
    # 跟踪误差
    lateral_errors: List[float] = field(default_factory=list)
    longitudinal_errors: List[float] = field(default_factory=list)
    heading_errors: List[float] = field(default_factory=list)
    prediction_errors: List[float] = field(default_factory=list)
    
    # 跟踪质量评估
    tracking_quality_scores: List[float] = field(default_factory=list)
    tracking_quality_ratings: List[str] = field(default_factory=list)
    
    # 一致性
    alpha_values: List[float] = field(default_factory=list)
    curvature_scores: List[float] = field(default_factory=list)
    velocity_dir_scores: List[float] = field(default_factory=list)
    temporal_scores: List[float] = field(default_factory=list)
    data_valid_count: int = 0
    data_invalid_count: int = 0
    
    # 超时
    odom_ages: List[float] = field(default_factory=list)
    traj_ages: List[float] = field(default_factory=list)
    imu_ages: List[float] = field(default_factory=list)
    odom_timeout_count: int = 0
    traj_timeout_count: int = 0
    traj_grace_exceeded_count: int = 0
    imu_timeout_count: int = 0
    in_startup_grace_count: int = 0
    
    # 控制命令
    cmd_vx: List[float] = field(default_factory=list)
    cmd_vy: List[float] = field(default_factory=list)
    cmd_vz: List[float] = field(default_factory=list)
    cmd_omega: List[float] = field(default_factory=list)
    
    # 状态统计 (与 ControllerState 枚举对应)
    # state_counts: 按状态值分组的计数字典
    # xxx_state_count: 各状态的独立计数，便于快速访问
    state_counts: Dict[int, int] = field(default_factory=dict)
    init_state_count: int = 0           # ControllerState.INIT = 0
    normal_state_count: int = 0         # ControllerState.NORMAL = 1
    soft_disabled_state_count: int = 0  # ControllerState.SOFT_DISABLED = 2
    mpc_degraded_state_count: int = 0   # ControllerState.MPC_DEGRADED = 3
    backup_active_state_count: int = 0  # ControllerState.BACKUP_ACTIVE = 4
    stopping_state_count: int = 0       # ControllerState.STOPPING = 5
    stopped_state_count: int = 0        # ControllerState.STOPPED = 6
    
    # EKF/状态估计
    estimator_covariance_norms: List[float] = field(default_factory=list)
    estimator_innovation_norms: List[float] = field(default_factory=list)
    slip_probabilities: List[float] = field(default_factory=list)
    imu_drift_detected_count: int = 0
    imu_available_count: int = 0
    
    # 坐标变换
    tf2_fallback_durations: List[float] = field(default_factory=list)
    tf2_fallback_count: int = 0
    tf2_available_count: int = 0
    accumulated_drifts: List[float] = field(default_factory=list)
    transform_source_frames: List[str] = field(default_factory=list)
    transform_target_frames: List[str] = field(default_factory=list)
    transform_error_messages: List[str] = field(default_factory=list)
    
    # 状态持续时间统计 (用于调优 degraded_state_timeout 和 backup_state_timeout)
    # 记录每次进入状态到离开状态的持续时间
    degraded_state_durations: List[float] = field(default_factory=list)
    backup_state_durations: List[float] = field(default_factory=list)
    
    # MPC 失败/恢复比率统计 (用于调优 mpc_fail_ratio_thresh 和 mpc_recovery_success_ratio)
    # 滑动窗口内的失败率
    mpc_fail_ratios: List[float] = field(default_factory=list)
    mpc_recovery_ratios: List[float] = field(default_factory=list)
    
    # 安全状态
    safety_check_failed_count: int = 0
    emergency_stop_count: int = 0
    consecutive_errors_max: int = 0
    
    # 过渡进度
    transition_progresses: List[float] = field(default_factory=list)
    
    # 总样本数
    total_samples: int = 0


class DiagnosticsAnalyzer:
    """诊断数据分析器
    
    分析控制器诊断数据，识别性能问题并生成调优建议。
    
    设计原则：
    1. 只对 TUNABLE_PARAMS 中的参数生成调优建议
    2. 对 DESIGN_PARAMS 只生成诊断信息，不建议调优
    3. 对 SAFETY_PARAMS 只检测配置错误（如 omega_max=0），不自动放宽
    4. 使用保守的置信度策略
    """
    
    def __init__(self, current_config: Dict[str, Any]):
        """
        初始化分析器
        
        Args:
            current_config: 当前配置字典 (从 YAML 加载)
        """
        self.config = current_config
        self.stats = DiagnosticsStats()
        self.results: List[AnalysisResult] = []
        
        # 缓存常用配置判断
        self._alpha_check_disabled: Optional[bool] = None
        self._platform: Optional[str] = None
        self._ctrl_freq: Optional[float] = None
    
    def _get_platform(self) -> str:
        """获取平台类型"""
        if self._platform is None:
            self._platform = self.config.get('system', {}).get('platform', 'differential')
        return self._platform
    
    def _get_ctrl_freq(self) -> float:
        """获取控制频率
        
        默认值 50Hz 与 universal_controller 默认配置一致
        turtlebot1.yaml 覆盖为 20Hz
        """
        if self._ctrl_freq is None:
            self._ctrl_freq = self.config.get('system', {}).get('ctrl_freq', 50)
        return self._ctrl_freq
    
    def _get_ctrl_period_ms(self) -> float:
        """获取控制周期（毫秒）"""
        return 1000.0 / self._get_ctrl_freq()
    
    def _is_alpha_check_disabled(self) -> bool:
        """检查 alpha 检查是否被禁用"""
        if self._alpha_check_disabled is None:
            state_machine = self.config.get('safety', {}).get('state_machine', {})
            alpha_disable_thresh = state_machine.get('alpha_disable_thresh', 0.0)
            self._alpha_check_disabled = alpha_disable_thresh <= 0.0
        return self._alpha_check_disabled
    
    def _is_differential_platform(self) -> bool:
        """检查是否为差速车平台"""
        return self._get_platform() == 'differential'
    
    def _should_skip_param(self, param: str) -> bool:
        """检查是否应跳过该参数（非当前平台使用）"""
        if self._is_differential_platform() and param in NON_DIFFERENTIAL_PARAMS:
            return True
        return False
    
    def _get_tuning_category(self, param: str) -> TuningCategory:
        """获取参数的调优分类"""
        if param in TUNABLE_PARAMS:
            return TuningCategory.TUNABLE
        elif param in DESIGN_PARAMS:
            return TuningCategory.DESIGN
        elif param in SAFETY_PARAMS:
            return TuningCategory.SAFETY
        else:
            return TuningCategory.DIAGNOSTIC


    def add_sample(self, diagnostics: Dict[str, Any]):
        """添加一个诊断样本
        
        数据格式严格遵循 DiagnosticsV2.to_ros_msg() 的输出格式
        """
        self.stats.total_samples += 1
        
        # MPC 数据
        if 'mpc_solve_time_ms' in diagnostics:
            self.stats.mpc_solve_times.append(diagnostics['mpc_solve_time_ms'])
        
        if diagnostics.get('mpc_success', False):
            self.stats.mpc_success_count += 1
        else:
            self.stats.mpc_fail_count += 1
            
        mpc_health = diagnostics.get('mpc_health', {})
        if isinstance(mpc_health, dict):
            if 'kkt_residual' in mpc_health:
                self.stats.kkt_residuals.append(mpc_health['kkt_residual'])
            if 'condition_number' in mpc_health:
                self.stats.condition_numbers.append(mpc_health['condition_number'])
            if 'consecutive_near_timeout' in mpc_health:
                self.stats.mpc_consecutive_near_timeouts.append(mpc_health['consecutive_near_timeout'])
            if mpc_health.get('degradation_warning', False):
                self.stats.mpc_degradation_warnings += 1
        
        # 跟踪误差
        tracking = diagnostics.get('tracking', {})
        if isinstance(tracking, dict):
            for key, lst in [
                ('lateral_error', self.stats.lateral_errors),
                ('longitudinal_error', self.stats.longitudinal_errors),
                ('heading_error', self.stats.heading_errors),
                ('prediction_error', self.stats.prediction_errors),
            ]:
                if key in tracking:
                    val = tracking[key]
                    if np.isfinite(val):
                        lst.append(val)
            
            # 跟踪质量评估
            if 'quality_score' in tracking:
                val = tracking['quality_score']
                if np.isfinite(val):
                    self.stats.tracking_quality_scores.append(val)
            if 'quality_rating' in tracking:
                self.stats.tracking_quality_ratings.append(tracking['quality_rating'])
        
        # 一致性
        consistency = diagnostics.get('consistency', {})
        if isinstance(consistency, dict):
            if 'alpha_soft' in consistency:
                self.stats.alpha_values.append(consistency['alpha_soft'])
            if 'curvature' in consistency:
                self.stats.curvature_scores.append(consistency['curvature'])
            if 'velocity_dir' in consistency:
                self.stats.velocity_dir_scores.append(consistency['velocity_dir'])
            if 'temporal' in consistency:
                self.stats.temporal_scores.append(consistency['temporal'])
            if consistency.get('data_valid', True):
                self.stats.data_valid_count += 1
            else:
                self.stats.data_invalid_count += 1
        
        # 超时状态
        timeout = diagnostics.get('timeout', {})
        if isinstance(timeout, dict):
            if 'last_odom_age_ms' in timeout:
                self.stats.odom_ages.append(timeout['last_odom_age_ms'])
            if 'last_traj_age_ms' in timeout:
                self.stats.traj_ages.append(timeout['last_traj_age_ms'])
            if 'last_imu_age_ms' in timeout:
                self.stats.imu_ages.append(timeout['last_imu_age_ms'])
            if timeout.get('odom_timeout', False):
                self.stats.odom_timeout_count += 1
            if timeout.get('traj_timeout', False):
                self.stats.traj_timeout_count += 1
            if timeout.get('traj_grace_exceeded', False):
                self.stats.traj_grace_exceeded_count += 1
            if timeout.get('imu_timeout', False):
                self.stats.imu_timeout_count += 1
            if timeout.get('in_startup_grace', False):
                self.stats.in_startup_grace_count += 1
        
        # 控制命令
        cmd = diagnostics.get('cmd', {})
        if isinstance(cmd, dict):
            if 'vx' in cmd:
                self.stats.cmd_vx.append(cmd['vx'])
            if 'vy' in cmd:
                self.stats.cmd_vy.append(cmd['vy'])
            if 'vz' in cmd:
                self.stats.cmd_vz.append(cmd['vz'])
            if 'omega' in cmd:
                self.stats.cmd_omega.append(cmd['omega'])
        
        # 状态统计
        # 将 state 转换为 ControllerState 枚举，确保类型一致性
        state_value = diagnostics.get('state', 0)
        try:
            state = ControllerState(state_value)
        except ValueError:
            # 未知状态值，使用 INIT 作为默认
            state = ControllerState.INIT
        
        self.stats.state_counts[int(state)] = self.stats.state_counts.get(int(state), 0) + 1
        
        # 按状态类型分别统计
        # 使用字典映射替代 if-elif 链，更简洁且易于维护
        state_counter_map = {
            ControllerState.INIT: 'init_state_count',
            ControllerState.NORMAL: 'normal_state_count',
            ControllerState.SOFT_DISABLED: 'soft_disabled_state_count',
            ControllerState.MPC_DEGRADED: 'mpc_degraded_state_count',
            ControllerState.BACKUP_ACTIVE: 'backup_active_state_count',
            ControllerState.STOPPING: 'stopping_state_count',
            ControllerState.STOPPED: 'stopped_state_count',
        }
        
        counter_name = state_counter_map.get(state)
        if counter_name:
            setattr(self.stats, counter_name, getattr(self.stats, counter_name) + 1)
        
        # EKF/状态估计
        estimator = diagnostics.get('estimator_health', {})
        if isinstance(estimator, dict):
            if 'covariance_norm' in estimator:
                self.stats.estimator_covariance_norms.append(estimator['covariance_norm'])
            if 'innovation_norm' in estimator:
                self.stats.estimator_innovation_norms.append(estimator['innovation_norm'])
            if 'slip_probability' in estimator:
                self.stats.slip_probabilities.append(estimator['slip_probability'])
            if estimator.get('imu_drift_detected', False):
                self.stats.imu_drift_detected_count += 1
            if estimator.get('imu_available', False):
                self.stats.imu_available_count += 1
        
        # 坐标变换
        transform = diagnostics.get('transform', {})
        if isinstance(transform, dict):
            fallback_ms = transform.get('fallback_duration_ms', 0)
            if fallback_ms > 0:
                self.stats.tf2_fallback_durations.append(fallback_ms)
                self.stats.tf2_fallback_count += 1
            if transform.get('tf2_available', True):
                self.stats.tf2_available_count += 1
            if 'accumulated_drift' in transform:
                self.stats.accumulated_drifts.append(transform['accumulated_drift'])
            # 收集坐标系信息
            source_frame = transform.get('source_frame', '')
            target_frame = transform.get('target_frame', '')
            error_message = transform.get('error_message', '')
            if source_frame:
                self.stats.transform_source_frames.append(source_frame)
            if target_frame:
                self.stats.transform_target_frames.append(target_frame)
            if error_message:
                self.stats.transform_error_messages.append(error_message)
        
        # 状态持续时间统计 (从诊断消息中提取)
        state_duration = diagnostics.get('state_duration', {})
        if isinstance(state_duration, dict):
            if 'degraded_duration_sec' in state_duration:
                dur = state_duration['degraded_duration_sec']
                if dur > 0:
                    self.stats.degraded_state_durations.append(dur)
            if 'backup_duration_sec' in state_duration:
                dur = state_duration['backup_duration_sec']
                if dur > 0:
                    self.stats.backup_state_durations.append(dur)
        
        # MPC 失败/恢复比率统计 (从诊断消息中提取)
        mpc_stats = diagnostics.get('mpc_stats', {})
        if isinstance(mpc_stats, dict):
            if 'fail_ratio' in mpc_stats:
                self.stats.mpc_fail_ratios.append(mpc_stats['fail_ratio'])
            if 'recovery_ratio' in mpc_stats:
                self.stats.mpc_recovery_ratios.append(mpc_stats['recovery_ratio'])
        
        # 安全状态
        if not diagnostics.get('safety_check_passed', True):
            self.stats.safety_check_failed_count += 1
        if diagnostics.get('emergency_stop', False):
            self.stats.emergency_stop_count += 1
        
        # 过渡进度
        if 'transition_progress' in diagnostics:
            self.stats.transition_progresses.append(diagnostics['transition_progress'])
        
        # 连续错误计数
        consecutive_errors = diagnostics.get('consecutive_errors', 0)
        if consecutive_errors > self.stats.consecutive_errors_max:
            self.stats.consecutive_errors_max = consecutive_errors


    def analyze(self) -> List[AnalysisResult]:
        """执行完整分析
        
        Returns:
            分析结果列表
        """
        self.results = []
        
        if self.stats.total_samples < 10:
            self.results.append(AnalysisResult(
                category="general",
                severity="warning",
                parameter="sample_count",
                current_value=self.stats.total_samples,
                suggested_value=100,
                reason="样本数量不足，建议收集更多数据以获得准确分析",
                confidence=0.5,
                tuning_category=TuningCategory.DIAGNOSTIC
            ))
            return self.results
        
        # 执行各项分析（按优先级排序）
        # 1. 配置错误检测（最重要，必须修复）
        self._analyze_config_errors()
        
        # 2. 可调优参数分析
        self._analyze_timeout_config()          # 超时配置
        self._analyze_mpc_performance()         # MPC 性能
        self._analyze_mpc_health_monitor()      # MPC 健康监控
        self._analyze_tracking_errors()         # 跟踪误差
        self._analyze_state_machine()           # 状态机
        self._analyze_transform()               # 坐标变换
        self._analyze_backup_controller()       # 备份控制器
        self._analyze_consistency_params()      # 一致性参数 (v4.3 新增)
        
        # 3. 诊断信息（不生成调优建议，仅报告状态）
        self._report_design_params_status()     # 设计参数状态报告
        self._report_safety_status()            # 安全状态报告
        
        return self.results


    def _analyze_config_errors(self):
        """分析配置错误 - 检测必须修复的硬性错误
        
        这些是配置错误，不是调优建议，必须修复。
        注意：此方法只检测静态配置错误，不依赖运行数据的分析在其他方法中进行。
        """
        mpc_config = self.config.get('mpc', {})
        trajectory_config = self.config.get('trajectory', {})
        constraints_config = self.config.get('constraints', {})
        
        ctrl_period_ms = self._get_ctrl_period_ms()
        
        # 1. 检查 omega_max 是否为 0（关键错误）
        omega_max = constraints_config.get('omega_max', 1.0)
        if omega_max <= 0:
            self.results.append(AnalysisResult(
                category="config_error",
                severity="critical",
                parameter="constraints.omega_max",
                current_value=omega_max,
                suggested_value=1.0,
                reason="omega_max 为 0 或负数，机器人无法转向！这是配置错误，必须修复。",
                confidence=1.0,
                tuning_category=TuningCategory.SAFETY  # 虽然是安全参数，但这是修复错误
            ))
        
        # 2. 检查 MPC dt 与轨迹 dt 是否一致（关键错误）
        mpc_dt = mpc_config.get('dt', 0.1)
        traj_dt = trajectory_config.get('default_dt_sec', 0.1)
        
        if abs(mpc_dt - traj_dt) > 0.001:
            self.results.append(AnalysisResult(
                category="config_error",
                severity="critical",
                parameter="mpc.dt",
                current_value=mpc_dt,
                suggested_value=traj_dt,
                reason=f"MPC时间步长({mpc_dt}s)与轨迹时间步长({traj_dt}s)不一致，会导致跟踪误差。",
                confidence=1.0,
                tuning_category=TuningCategory.TUNABLE
            ))
        
        # 3. 检查 horizon_degraded 与 horizon 的关系
        horizon = mpc_config.get('horizon', 7)
        horizon_degraded = mpc_config.get('horizon_degraded', 4)
        if horizon_degraded >= horizon:
            self.results.append(AnalysisResult(
                category="config_error",
                severity="warning",
                parameter="mpc.horizon_degraded",
                current_value=horizon_degraded,
                suggested_value=max(horizon - 3, 2),
                reason=f"降级时域({horizon_degraded})应小于正常时域({horizon})。",
                confidence=1.0,
                tuning_category=TuningCategory.TUNABLE
            ))
        
        # 4. 检查控制频率与超时配置的关系
        watchdog_config = self.config.get('watchdog', {})
        odom_timeout = watchdog_config.get('odom_timeout_ms', 500)
        if odom_timeout > 0 and odom_timeout < ctrl_period_ms * 2:
            self.results.append(AnalysisResult(
                category="config_error",
                severity="warning",
                parameter="watchdog.odom_timeout_ms",
                current_value=odom_timeout,
                suggested_value=int(ctrl_period_ms * 3),
                reason=f"里程计超时({odom_timeout}ms)小于2个控制周期({ctrl_period_ms*2:.0f}ms)，可能误报。",
                confidence=0.9,
                tuning_category=TuningCategory.TUNABLE
            ))
        
        # 5. 检查跟踪权重总和（静态配置检查）
        tracking_config = self.config.get('tracking', {})
        weights = tracking_config.get('weights', {})
        lateral_weight = weights.get('lateral', 0.4)
        longitudinal_weight = weights.get('longitudinal', 0.4)
        heading_weight = weights.get('heading', 0.2)
        total_weight = lateral_weight + longitudinal_weight + heading_weight
        
        if abs(total_weight - 1.0) > 0.01:
            self.results.append(AnalysisResult(
                category="config_error",
                severity="warning",
                parameter="tracking.weights",
                current_value={'lateral': lateral_weight, 'longitudinal': longitudinal_weight, 'heading': heading_weight},
                suggested_value={'lateral': 0.4, 'longitudinal': 0.4, 'heading': 0.2},
                reason=f"跟踪权重总和({total_weight:.2f})不等于1.0，会影响质量评分准确性。",
                confidence=1.0,
                tuning_category=TuningCategory.DIAGNOSTIC
            ))
        
        # 注意：MPC 求解时间与控制周期的关系检查移至 _analyze_mpc_performance，
        # 因为它依赖运行数据，属于性能分析而非静态配置错误检测。


    def _analyze_timeout_config(self):
        """分析超时配置 - 基于实际延迟统计"""
        watchdog_config = self.config.get('watchdog', {})
        
        # 里程计超时分析
        if self.stats.odom_ages:
            odom_arr = np.array(self.stats.odom_ages)
            p95_odom = np.percentile(odom_arr, 95)
            max_odom = np.max(odom_arr)
            
            current_timeout = watchdog_config.get('odom_timeout_ms', 500)
            
            if self.stats.odom_timeout_count > self.stats.total_samples * 0.05:
                timeout_rate = self.stats.odom_timeout_count / self.stats.total_samples * 100
                # 使用 95% 分位数 + 50% 裕度
                suggested_timeout = int(p95_odom * 1.5)
                self.results.append(AnalysisResult(
                    category="timeout",
                    severity="critical",
                    parameter="watchdog.odom_timeout_ms",
                    current_value=current_timeout,
                    suggested_value=suggested_timeout,
                    reason=f"里程计超时率({timeout_rate:.1f}%)过高，95%分位延迟{p95_odom:.0f}ms。",
                    confidence=0.9,
                    tuning_category=TuningCategory.TUNABLE
                ))
            elif p95_odom > current_timeout * 0.8:
                suggested_timeout = int(p95_odom * 1.3)
                self.results.append(AnalysisResult(
                    category="timeout",
                    severity="warning",
                    parameter="watchdog.odom_timeout_ms",
                    current_value=current_timeout,
                    suggested_value=suggested_timeout,
                    reason=f"95%分位里程计延迟({p95_odom:.0f}ms)接近超时阈值({current_timeout}ms)。",
                    confidence=0.8,
                    tuning_category=TuningCategory.TUNABLE
                ))
        
        # 轨迹超时分析
        if self.stats.traj_ages:
            traj_arr = np.array(self.stats.traj_ages)
            p95_traj = np.percentile(traj_arr, 95)
            
            current_timeout = watchdog_config.get('traj_timeout_ms', 1000)
            current_grace = watchdog_config.get('traj_grace_ms', 600)
            
            if p95_traj > current_timeout * 0.8:
                suggested_timeout = int(p95_traj * 1.3)
                self.results.append(AnalysisResult(
                    category="timeout",
                    severity="warning",
                    parameter="watchdog.traj_timeout_ms",
                    current_value=current_timeout,
                    suggested_value=suggested_timeout,
                    reason=f"95%分位轨迹延迟({p95_traj:.0f}ms)接近超时阈值({current_timeout}ms)。",
                    confidence=0.8,
                    tuning_category=TuningCategory.TUNABLE
                ))
            
            # 宽限期超时
            if self.stats.traj_grace_exceeded_count > 0:
                grace_rate = self.stats.traj_grace_exceeded_count / self.stats.total_samples * 100
                suggested_grace = int(current_grace * 1.5)
                self.results.append(AnalysisResult(
                    category="timeout",
                    severity="critical",
                    parameter="watchdog.traj_grace_ms",
                    current_value=current_grace,
                    suggested_value=suggested_grace,
                    reason=f"轨迹宽限期超时({grace_rate:.1f}%)，可能导致安全停止。",
                    confidence=0.9,
                    tuning_category=TuningCategory.TUNABLE
                ))
        
        # IMU 超时分析（仅当 IMU 可用时）
        if self.stats.imu_available_count > self.stats.total_samples * 0.5:
            current_imu_timeout = watchdog_config.get('imu_timeout_ms', -1)
            if current_imu_timeout < 0 and self.stats.imu_ages:
                imu_arr = np.array(self.stats.imu_ages)
                p95_imu = np.percentile(imu_arr, 95)
                self.results.append(AnalysisResult(
                    category="timeout",
                    severity="info",
                    parameter="watchdog.imu_timeout_ms",
                    current_value=current_imu_timeout,
                    suggested_value=int(p95_imu * 2),
                    reason=f"IMU可用但超时检测被禁用，建议启用。",
                    confidence=0.5,
                    tuning_category=TuningCategory.TUNABLE
                ))


    def _analyze_mpc_performance(self):
        """分析 MPC 性能
        
        分析 MPC 成功率和求解时间，生成 horizon 调优建议。
        注意：为避免重复建议，只在最严重的问题上生成一个 horizon 建议。
        """
        if not self.stats.mpc_solve_times:
            return
        
        solve_times = np.array(self.stats.mpc_solve_times)
        p95_time = np.percentile(solve_times, 95)
        
        mpc_config = self.config.get('mpc', {})
        ctrl_period_ms = self._get_ctrl_period_ms()
        current_horizon = mpc_config.get('horizon', 7)
        
        # 分析 MPC 成功率
        total = self.stats.mpc_success_count + self.stats.mpc_fail_count
        success_rate = self.stats.mpc_success_count / total if total > 0 else 1.0
        
        # 分析求解时间与控制周期的关系
        time_ratio = p95_time / ctrl_period_ms if ctrl_period_ms > 0 else 0
        
        # 综合判断是否需要减小 horizon（只生成一个建议，避免重复）
        horizon_issue_severity = None
        horizon_issue_reason = None
        horizon_confidence = 0.0
        
        # 优先级：求解时间超过控制周期 > 成功率低 > 求解时间占用过高
        if time_ratio > 1.0:
            # 求解时间超过控制周期，严重问题
            horizon_issue_severity = "critical"
            horizon_issue_reason = f"MPC求解时间({p95_time:.1f}ms)超过控制周期({ctrl_period_ms:.1f}ms)，必须减小预测时域。"
            horizon_confidence = 0.95
        elif success_rate < 0.85:
            # 成功率过低
            horizon_issue_severity = "warning"
            horizon_issue_reason = f"MPC成功率({success_rate*100:.1f}%)较低，建议减小预测时域。"
            horizon_confidence = 0.8
        elif time_ratio > 0.5:
            # 求解时间占用控制周期过高
            horizon_issue_severity = "warning"
            horizon_issue_reason = f"MPC求解时间({p95_time:.1f}ms)占用控制周期({ctrl_period_ms:.1f}ms)>{time_ratio*100:.0f}%，建议减小预测时域。"
            horizon_confidence = 0.7
        
        if horizon_issue_severity:
            self.results.append(AnalysisResult(
                category="mpc",
                severity=horizon_issue_severity,
                parameter="mpc.horizon",
                current_value=current_horizon,
                suggested_value=max(current_horizon - 1, 3),
                reason=horizon_issue_reason,
                confidence=horizon_confidence,
                tuning_category=TuningCategory.TUNABLE
            ))

    def _analyze_mpc_health_monitor(self):
        """分析 MPC 健康监控参数
        
        只分析 turtlebot1.yaml 中实际存在的参数:
        - time_warning_thresh_ms
        - time_critical_thresh_ms
        - time_recovery_thresh_ms
        - consecutive_warning_limit
        """
        if not self.stats.mpc_solve_times:
            return
        
        solve_times = np.array(self.stats.mpc_solve_times)
        p95_time = np.percentile(solve_times, 95)
        p10_time = np.percentile(solve_times, 10)
        
        mpc_config = self.config.get('mpc', {})
        health_config = mpc_config.get('health_monitor', {})
        
        current_warning = health_config.get('time_warning_thresh_ms', 20)
        current_critical = health_config.get('time_critical_thresh_ms', 40)
        current_recovery = health_config.get('time_recovery_thresh_ms', 15)
        
        # 分析警告阈值
        if p95_time > current_warning:
            suggested_warning = round(p95_time * 1.2, 1)
            self.results.append(AnalysisResult(
                category="mpc",
                severity="warning",
                parameter="mpc.health_monitor.time_warning_thresh_ms",
                current_value=current_warning,
                suggested_value=suggested_warning,
                reason=f"95%分位求解时间({p95_time:.1f}ms)超过警告阈值({current_warning}ms)。",
                confidence=0.8,
                tuning_category=TuningCategory.TUNABLE
            ))
        
        # 分析临界阈值（使用 99% 分位数而非最大值，避免异常值影响）
        p99_time = np.percentile(solve_times, 99)
        if p99_time > current_critical:
            suggested_critical = round(p99_time * 1.2, 1)
            self.results.append(AnalysisResult(
                category="mpc",
                severity="critical",
                parameter="mpc.health_monitor.time_critical_thresh_ms",
                current_value=current_critical,
                suggested_value=suggested_critical,
                reason=f"99%分位求解时间({p99_time:.1f}ms)超过临界阈值({current_critical}ms)。",
                confidence=0.85,
                tuning_category=TuningCategory.TUNABLE
            ))
        
        # 分析恢复阈值
        if p10_time > current_recovery:
            suggested_recovery = round(p10_time * 1.2, 1)
            self.results.append(AnalysisResult(
                category="mpc",
                severity="info",
                parameter="mpc.health_monitor.time_recovery_thresh_ms",
                current_value=current_recovery,
                suggested_value=suggested_recovery,
                reason=f"10%分位求解时间({p10_time:.1f}ms)高于恢复阈值({current_recovery}ms)。",
                confidence=0.6,
                tuning_category=TuningCategory.TUNABLE
            ))
        
        # 分析连续警告限制
        if self.stats.mpc_consecutive_near_timeouts:
            max_consecutive = max(self.stats.mpc_consecutive_near_timeouts)
            current_limit = health_config.get('consecutive_warning_limit', 10)
            
            if max_consecutive > current_limit * 0.8:
                self.results.append(AnalysisResult(
                    category="mpc",
                    severity="warning",
                    parameter="mpc.health_monitor.consecutive_warning_limit",
                    current_value=current_limit,
                    suggested_value=max(max_consecutive + 3, current_limit),
                    reason=f"连续超时次数({max_consecutive})接近限制({current_limit})。",
                    confidence=0.7,
                    tuning_category=TuningCategory.TUNABLE
                ))


    def _analyze_tracking_errors(self):
        """分析轨迹跟踪误差
        
        MPC 权重调优策略:
        - 使用保守的调整策略，避免过度调整
        - 只在误差明显超过阈值时才建议调整
        - 保持权重之间的相对比例
        
        turtlebot1.yaml 中定义的参数:
        - tracking.lateral_thresh, tracking.longitudinal_thresh, tracking.heading_thresh
        - mpc.weights.position, mpc.weights.velocity, mpc.weights.heading
        
        注意: tracking.prediction_thresh 在 universal_controller 中使用，
        但 turtlebot1.yaml 中未定义，使用 TRACKING_CONFIG 默认值。
        """
        tracking_config = self.config.get('tracking', {})
        mpc_config = self.config.get('mpc', {})
        weights = mpc_config.get('weights', {})
        
        # 获取当前权重
        current_pos_weight = weights.get('position', 15.0)
        current_vel_weight = weights.get('velocity', 6.0)
        current_head_weight = weights.get('heading', 8.0)
        
        # 横向误差分析
        if self.stats.lateral_errors:
            lat_arr = np.array(self.stats.lateral_errors)
            avg_lat = np.mean(lat_arr)
            p95_lat = np.percentile(lat_arr, 95)
            
            current_thresh = tracking_config.get('lateral_thresh', 0.25)
            
            # 只在平均误差超过阈值 60% 时才建议调整权重
            if avg_lat > current_thresh * 0.6:
                # 保守调整：最多增加 30%
                weight_increase = min(1.0 + (avg_lat / current_thresh - 0.6) * 0.5, 1.3)
                suggested_weight = min(current_pos_weight * weight_increase, 25.0)
                
                self.results.append(AnalysisResult(
                    category="tracking",
                    severity="warning",
                    parameter="mpc.weights.position",
                    current_value=current_pos_weight,
                    suggested_value=round(suggested_weight, 1),
                    reason=f"平均横向误差({avg_lat*100:.1f}cm)较大，建议增加位置权重。",
                    confidence=0.7,
                    tuning_category=TuningCategory.TUNABLE
                ))
            
            # 跟踪阈值调整（仅影响显示）
            if p95_lat > current_thresh:
                suggested_thresh = round(p95_lat * 1.2, 2)
                self.results.append(AnalysisResult(
                    category="tracking",
                    severity="info",
                    parameter="tracking.lateral_thresh",
                    current_value=current_thresh,
                    suggested_value=suggested_thresh,
                    reason=f"95%分位横向误差({p95_lat*100:.1f}cm)超过阈值({current_thresh*100:.0f}cm)。",
                    confidence=0.7,
                    tuning_category=TuningCategory.TUNABLE
                ))
        
        # 纵向误差分析
        if self.stats.longitudinal_errors:
            lon_arr = np.array(self.stats.longitudinal_errors)
            avg_lon = np.mean(lon_arr)
            p95_lon = np.percentile(lon_arr, 95)
            
            current_thresh = tracking_config.get('longitudinal_thresh', 0.6)
            
            if avg_lon > current_thresh * 0.6:
                weight_increase = min(1.0 + (avg_lon / current_thresh - 0.6) * 0.5, 1.3)
                suggested_weight = min(current_vel_weight * weight_increase, 12.0)
                
                self.results.append(AnalysisResult(
                    category="tracking",
                    severity="warning",
                    parameter="mpc.weights.velocity",
                    current_value=current_vel_weight,
                    suggested_value=round(suggested_weight, 1),
                    reason=f"平均纵向误差({avg_lon*100:.1f}cm)较大，建议增加速度权重。",
                    confidence=0.7,
                    tuning_category=TuningCategory.TUNABLE
                ))
            
            if p95_lon > current_thresh:
                suggested_thresh = round(p95_lon * 1.2, 2)
                self.results.append(AnalysisResult(
                    category="tracking",
                    severity="info",
                    parameter="tracking.longitudinal_thresh",
                    current_value=current_thresh,
                    suggested_value=suggested_thresh,
                    reason=f"95%分位纵向误差({p95_lon*100:.1f}cm)超过阈值({current_thresh*100:.0f}cm)。",
                    confidence=0.7,
                    tuning_category=TuningCategory.TUNABLE
                ))
        
        # 航向误差分析
        # 默认值: universal_controller/config/system_config.py TRACKING_CONFIG['heading_thresh'] = 0.5
        if self.stats.heading_errors:
            head_arr = np.array(self.stats.heading_errors)
            avg_head = np.mean(head_arr)
            p95_head = np.percentile(head_arr, 95)
            
            # 从配置读取阈值，默认 0.5 rad (约 28.6 度)
            current_thresh = tracking_config.get('heading_thresh', 0.5)
            
            if avg_head > current_thresh * 0.6:
                # 航向权重调整更保守
                weight_increase = min(1.0 + (avg_head / current_thresh - 0.6) * 0.3, 1.2)
                suggested_weight = min(current_head_weight * weight_increase, 12.0)
                
                self.results.append(AnalysisResult(
                    category="tracking",
                    severity="warning",
                    parameter="mpc.weights.heading",
                    current_value=current_head_weight,
                    suggested_value=round(suggested_weight, 1),
                    reason=f"平均航向误差({np.degrees(avg_head):.1f}deg)较大，建议增加航向权重。",
                    confidence=0.6,
                    tuning_category=TuningCategory.TUNABLE
                ))
            
            # 航向阈值调整（仅影响 Dashboard 显示）
            if p95_head > current_thresh:
                suggested_thresh = round(p95_head * 1.2, 2)
                self.results.append(AnalysisResult(
                    category="tracking",
                    severity="info",
                    parameter="tracking.heading_thresh",
                    current_value=current_thresh,
                    suggested_value=suggested_thresh,
                    reason=f"95%分位航向误差({np.degrees(p95_head):.1f}deg)超过阈值({np.degrees(current_thresh):.0f}deg)。",
                    confidence=0.7,
                    tuning_category=TuningCategory.TUNABLE
                ))
        
        # 预测误差分析
        # 默认值: universal_controller/config/system_config.py TRACKING_CONFIG['prediction_thresh'] = 0.5
        if self.stats.prediction_errors:
            # 过滤掉 NaN 值（fallback 求解器不提供预测状态）
            valid_pred_errors = [e for e in self.stats.prediction_errors if np.isfinite(e)]
            if valid_pred_errors:
                pred_arr = np.array(valid_pred_errors)
                avg_pred = np.mean(pred_arr)
                p95_pred = np.percentile(pred_arr, 95)
                
                # 从配置读取阈值
                current_thresh = tracking_config.get('prediction_thresh', 0.5)
                
                # 预测误差较大可能表明：
                # 1. MPC 时间步长与实际控制周期不匹配
                # 2. 模型参数（如加速度限制）与实际不符
                # 3. 状态估计器延迟较大
                if avg_pred > 0.3:  # 30cm 作为参考阈值
                    mpc_dt = mpc_config.get('dt', 0.1)
                    ctrl_freq = self._get_ctrl_freq()
                    expected_dt = 1.0 / ctrl_freq
                    
                    # 检查 MPC dt 与控制周期是否匹配
                    if abs(mpc_dt - expected_dt) > 0.01:
                        self.results.append(AnalysisResult(
                            category="tracking",
                            severity="warning",
                            parameter="mpc.dt",
                            current_value=mpc_dt,
                            suggested_value=round(expected_dt, 3),
                            reason=f"预测误差({avg_pred*100:.1f}cm)较大，MPC时间步长({mpc_dt}s)与控制周期({expected_dt:.3f}s)不匹配。",
                            confidence=0.8,
                            tuning_category=TuningCategory.TUNABLE
                        ))
                    else:
                        # dt 匹配但预测误差仍然较大，仅报告诊断信息
                        self.results.append(AnalysisResult(
                            category="tracking",
                            severity="info",
                            parameter="mpc.model",
                            current_value=f"预测误差 {avg_pred*100:.1f}cm",
                            suggested_value=None,
                            reason=f"[诊断信息] 预测误差({avg_pred*100:.1f}cm)较大，可能需要检查MPC模型参数或状态估计器延迟。",
                            confidence=0.0,
                            tuning_category=TuningCategory.DIAGNOSTIC
                        ))
                
                # 预测阈值调整（仅影响 Dashboard 显示）
                if p95_pred > current_thresh:
                    suggested_thresh = round(p95_pred * 1.2, 2)
                    self.results.append(AnalysisResult(
                        category="tracking",
                        severity="info",
                        parameter="tracking.prediction_thresh",
                        current_value=current_thresh,
                        suggested_value=suggested_thresh,
                        reason=f"95%分位预测误差({p95_pred*100:.1f}cm)超过阈值({current_thresh*100:.0f}cm)。",
                        confidence=0.7,
                        tuning_category=TuningCategory.TUNABLE
                    ))


    def _analyze_state_machine(self):
        """分析状态机配置
        
        可调优参数 (turtlebot1.yaml):
        - safety.state_machine.mpc_recovery_tolerance
        - safety.state_machine.mpc_fail_thresh
        - safety.state_machine.mpc_recovery_thresh
        
        可调优参数 (internal_params.yaml):
        - safety.state_machine.mpc_fail_ratio_thresh
        - safety.state_machine.mpc_recovery_success_ratio
        - safety.state_machine.degraded_state_timeout
        - safety.state_machine.backup_state_timeout
        """
        safety_config = self.config.get('safety', {})
        state_machine = safety_config.get('state_machine', {})
        
        if self.stats.total_samples == 0:
            return
        
        # 计算各种比率（统一使用基于状态枚举的统计）
        degraded_rate = self.stats.mpc_degraded_state_count / self.stats.total_samples
        backup_rate = self.stats.backup_active_state_count / self.stats.total_samples
        
        total_mpc = self.stats.mpc_success_count + self.stats.mpc_fail_count
        mpc_success_rate = self.stats.mpc_success_count / total_mpc if total_mpc > 0 else 0
        
        # 1. 如果 MPC 成功率高但降级频繁，说明恢复容忍度可能需要调整
        if degraded_rate > 0.1 and mpc_success_rate > 0.8:
            current_tolerance = state_machine.get('mpc_recovery_tolerance', 1)
            self.results.append(AnalysisResult(
                category="state_machine",
                severity="warning",
                parameter="safety.state_machine.mpc_recovery_tolerance",
                current_value=current_tolerance,
                suggested_value=min(current_tolerance + 1, 3),
                reason=f"MPC成功率({mpc_success_rate*100:.1f}%)高但降级频繁({degraded_rate*100:.1f}%)，可增加恢复容忍度。",
                confidence=0.8,
                tuning_category=TuningCategory.TUNABLE
            ))
        
        # 2. 如果备用控制器激活率过高，报告诊断信息
        if backup_rate > 0.15:
            self.results.append(AnalysisResult(
                category="state_machine",
                severity="warning",
                parameter="safety.state_machine",
                current_value=f"备用控制器激活率 {backup_rate*100:.1f}%",
                suggested_value=None,
                reason=f"[诊断信息] 备用控制器激活率({backup_rate*100:.1f}%)过高，建议检查 MPC 配置或轨迹质量。",
                confidence=0.0,
                tuning_category=TuningCategory.DIAGNOSTIC
            ))
        
        # 3. 分析 MPC 失败/恢复比率阈值 (internal_params.yaml 参数)
        self._analyze_mpc_ratios()
        
        # 4. 分析状态超时参数 (internal_params.yaml 参数)
        self._analyze_state_timeouts()
    
    def _analyze_mpc_ratios(self):
        """分析 MPC 失败/恢复比率阈值
        
        基于实际 MPC 失败/恢复统计，调优以下参数:
        - safety.state_machine.mpc_fail_ratio_thresh: 失败率阈值
        - safety.state_machine.mpc_recovery_success_ratio: 恢复成功率要求
        
        这些参数在 internal_params.yaml 中定义。
        """
        safety_config = self.config.get('safety', {})
        state_machine = safety_config.get('state_machine', {})
        
        # 分析 MPC 失败率阈值
        if self.stats.mpc_fail_ratios:
            fail_ratio_arr = np.array(self.stats.mpc_fail_ratios)
            avg_fail_ratio = np.mean(fail_ratio_arr)
            p95_fail_ratio = np.percentile(fail_ratio_arr, 95)
            
            current_thresh = state_machine.get('mpc_fail_ratio_thresh', 0.5)
            
            # 如果实际失败率经常超过阈值，可能需要放宽
            if p95_fail_ratio > current_thresh and self.stats.mpc_degraded_state_count > self.stats.total_samples * 0.2:
                # 降级过于频繁，考虑放宽阈值
                suggested_thresh = min(p95_fail_ratio * 1.1, 0.7)  # 最大 0.7
                self.results.append(AnalysisResult(
                    category="state_machine",
                    severity="warning",
                    parameter="safety.state_machine.mpc_fail_ratio_thresh",
                    current_value=current_thresh,
                    suggested_value=round(suggested_thresh, 2),
                    reason=f"MPC失败率({p95_fail_ratio*100:.1f}%)经常超过阈值({current_thresh*100:.0f}%)，降级过于频繁。",
                    confidence=0.7,
                    tuning_category=TuningCategory.TUNABLE
                ))
            elif avg_fail_ratio < current_thresh * 0.3 and self.stats.mpc_degraded_state_count < self.stats.total_samples * 0.01:
                # 失败率很低，可以收紧阈值以更早检测问题
                suggested_thresh = max(avg_fail_ratio * 2.0, 0.3)  # 最小 0.3
                self.results.append(AnalysisResult(
                    category="state_machine",
                    severity="info",
                    parameter="safety.state_machine.mpc_fail_ratio_thresh",
                    current_value=current_thresh,
                    suggested_value=round(suggested_thresh, 2),
                    reason=f"MPC失败率({avg_fail_ratio*100:.1f}%)很低，可收紧阈值以更早检测问题。",
                    confidence=0.5,
                    tuning_category=TuningCategory.TUNABLE
                ))
        
        # 分析 MPC 恢复成功率要求
        if self.stats.mpc_recovery_ratios:
            recovery_ratio_arr = np.array(self.stats.mpc_recovery_ratios)
            avg_recovery_ratio = np.mean(recovery_ratio_arr)
            p10_recovery_ratio = np.percentile(recovery_ratio_arr, 10)
            
            current_ratio = state_machine.get('mpc_recovery_success_ratio', 0.8)
            
            # 如果恢复成功率经常低于要求，可能需要放宽
            if p10_recovery_ratio < current_ratio and self.stats.backup_active_state_count > self.stats.total_samples * 0.1:
                # 恢复困难，考虑放宽要求
                suggested_ratio = max(p10_recovery_ratio * 0.9, 0.6)  # 最小 0.6
                self.results.append(AnalysisResult(
                    category="state_machine",
                    severity="warning",
                    parameter="safety.state_machine.mpc_recovery_success_ratio",
                    current_value=current_ratio,
                    suggested_value=round(suggested_ratio, 2),
                    reason=f"MPC恢复成功率({p10_recovery_ratio*100:.1f}%)经常低于要求({current_ratio*100:.0f}%)，恢复困难。",
                    confidence=0.7,
                    tuning_category=TuningCategory.TUNABLE
                ))
            elif avg_recovery_ratio > current_ratio * 1.1:
                # 恢复成功率很高，可以收紧要求
                suggested_ratio = min(avg_recovery_ratio * 0.95, 0.95)  # 最大 0.95
                self.results.append(AnalysisResult(
                    category="state_machine",
                    severity="info",
                    parameter="safety.state_machine.mpc_recovery_success_ratio",
                    current_value=current_ratio,
                    suggested_value=round(suggested_ratio, 2),
                    reason=f"MPC恢复成功率({avg_recovery_ratio*100:.1f}%)很高，可收紧要求以提高稳定性。",
                    confidence=0.5,
                    tuning_category=TuningCategory.TUNABLE
                ))
    
    def _analyze_state_timeouts(self):
        """分析状态超时参数
        
        基于实际状态持续时间统计，调优以下参数:
        - safety.state_machine.degraded_state_timeout: MPC_DEGRADED 状态超时
        - safety.state_machine.backup_state_timeout: BACKUP_ACTIVE 状态超时
        
        这些参数在 internal_params.yaml 中定义。
        """
        safety_config = self.config.get('safety', {})
        state_machine = safety_config.get('state_machine', {})
        
        # 分析 MPC_DEGRADED 状态超时
        if self.stats.degraded_state_durations:
            degraded_arr = np.array(self.stats.degraded_state_durations)
            avg_degraded = np.mean(degraded_arr)
            p95_degraded = np.percentile(degraded_arr, 95)
            max_degraded = np.max(degraded_arr)
            
            current_timeout = state_machine.get('degraded_state_timeout', 30.0)
            
            if max_degraded > current_timeout * 0.8:
                # 最大持续时间接近超时，可能需要放宽
                suggested_timeout = max_degraded * 1.3
                self.results.append(AnalysisResult(
                    category="state_machine",
                    severity="warning",
                    parameter="safety.state_machine.degraded_state_timeout",
                    current_value=current_timeout,
                    suggested_value=round(suggested_timeout, 1),
                    reason=f"MPC_DEGRADED最大持续时间({max_degraded:.1f}s)接近超时({current_timeout}s)。",
                    confidence=0.7,
                    tuning_category=TuningCategory.TUNABLE
                ))
            elif p95_degraded < current_timeout * 0.2:
                # 95% 分位数远低于超时，可以收紧
                suggested_timeout = max(p95_degraded * 3.0, 10.0)  # 最小 10s
                self.results.append(AnalysisResult(
                    category="state_machine",
                    severity="info",
                    parameter="safety.state_machine.degraded_state_timeout",
                    current_value=current_timeout,
                    suggested_value=round(suggested_timeout, 1),
                    reason=f"MPC_DEGRADED 95%分位持续时间({p95_degraded:.1f}s)远低于超时({current_timeout}s)，可收紧。",
                    confidence=0.5,
                    tuning_category=TuningCategory.TUNABLE
                ))
        
        # 分析 BACKUP_ACTIVE 状态超时
        if self.stats.backup_state_durations:
            backup_arr = np.array(self.stats.backup_state_durations)
            avg_backup = np.mean(backup_arr)
            p95_backup = np.percentile(backup_arr, 95)
            max_backup = np.max(backup_arr)
            
            current_timeout = state_machine.get('backup_state_timeout', 60.0)
            
            if max_backup > current_timeout * 0.8:
                # 最大持续时间接近超时
                suggested_timeout = max_backup * 1.3
                self.results.append(AnalysisResult(
                    category="state_machine",
                    severity="warning",
                    parameter="safety.state_machine.backup_state_timeout",
                    current_value=current_timeout,
                    suggested_value=round(suggested_timeout, 1),
                    reason=f"BACKUP_ACTIVE最大持续时间({max_backup:.1f}s)接近超时({current_timeout}s)。",
                    confidence=0.7,
                    tuning_category=TuningCategory.TUNABLE
                ))
            elif p95_backup < current_timeout * 0.2:
                # 95% 分位数远低于超时，可以收紧
                suggested_timeout = max(p95_backup * 3.0, 20.0)  # 最小 20s
                self.results.append(AnalysisResult(
                    category="state_machine",
                    severity="info",
                    parameter="safety.state_machine.backup_state_timeout",
                    current_value=current_timeout,
                    suggested_value=round(suggested_timeout, 1),
                    reason=f"BACKUP_ACTIVE 95%分位持续时间({p95_backup:.1f}s)远低于超时({current_timeout}s)，可收紧。",
                    confidence=0.5,
                    tuning_category=TuningCategory.TUNABLE
                ))


    def _analyze_transform(self):
        """分析坐标变换配置
        
        turtlebot1.yaml 中定义的参数:
        - transform.timeout_ms (可调优)
        - transform.source_frame (不可调优)
        - transform.target_frame (不可调优)
        
        controller_params.yaml 中定义的参数:
        - transform.fallback_duration_limit_ms (可调优)
        - transform.fallback_critical_limit_ms (可调优)
        
        注意: buffer_warmup_* 参数在 controller_ros 的 ROS 层使用，
        但 turtlebot1.yaml 中未定义，使用 TRANSFORM_ROS_DEFAULTS 默认值。
        """
        transform_config = self.config.get('transform', {})
        
        if self.stats.total_samples == 0:
            return
        
        # 分析 TF2 可用率
        tf2_available_rate = self.stats.tf2_available_count / self.stats.total_samples if self.stats.total_samples > 0 else 1.0
        if tf2_available_rate < 0.95:
            current_timeout = transform_config.get('timeout_ms', 50)
            self.results.append(AnalysisResult(
                category="transform",
                severity="warning",
                parameter="transform.timeout_ms",
                current_value=current_timeout,
                suggested_value=int(current_timeout * 1.5),
                reason=f"TF2可用率({tf2_available_rate*100:.1f}%)较低，可能超时设置过短。",
                confidence=0.6,
                tuning_category=TuningCategory.TUNABLE
            ))
        
        # 分析 TF2 降级情况 - 仅报告诊断信息
        if self.stats.tf2_fallback_count > 0:
            fallback_rate = self.stats.tf2_fallback_count / self.stats.total_samples
            
            if fallback_rate > 0.05:
                self.results.append(AnalysisResult(
                    category="transform",
                    severity="warning" if fallback_rate > 0.1 else "info",
                    parameter="transform.fallback",
                    current_value=f"降级率 {fallback_rate*100:.1f}%",
                    suggested_value=None,
                    reason=f"[诊断信息] TF2降级频繁({fallback_rate*100:.1f}%)，建议检查 TF2 发布频率和网络延迟。",
                    confidence=0.0,
                    tuning_category=TuningCategory.DIAGNOSTIC
                ))
        
        # 分析 TF2 降级持续时间限制 (controller_params.yaml 参数)
        self._analyze_tf2_fallback_limits()
        
        # 分析坐标系配置一致性
        self._analyze_transform_frames()
        
        # 分析变换错误信息
        self._analyze_transform_errors()
    
    def _analyze_tf2_fallback_limits(self):
        """分析 TF2 降级持续时间限制
        
        基于实际降级持续时间统计，调优以下参数:
        - transform.fallback_duration_limit_ms: 降级警告阈值
        - transform.fallback_critical_limit_ms: 降级临界阈值
        
        这些参数在 controller_params.yaml 中定义。
        """
        if not self.stats.tf2_fallback_durations:
            return
        
        transform_config = self.config.get('transform', {})
        fallback_arr = np.array(self.stats.tf2_fallback_durations)
        
        p95_fallback = np.percentile(fallback_arr, 95)
        p99_fallback = np.percentile(fallback_arr, 99)
        max_fallback = np.max(fallback_arr)
        
        # 分析警告阈值
        current_limit = transform_config.get('fallback_duration_limit_ms', 500)
        if p95_fallback > current_limit:
            # 95% 分位数超过当前限制，建议放宽
            suggested_limit = int(p95_fallback * 1.3)
            self.results.append(AnalysisResult(
                category="transform",
                severity="warning",
                parameter="transform.fallback_duration_limit_ms",
                current_value=current_limit,
                suggested_value=suggested_limit,
                reason=f"95%分位降级持续时间({p95_fallback:.0f}ms)超过警告阈值({current_limit}ms)。",
                confidence=0.8,
                tuning_category=TuningCategory.TUNABLE
            ))
        elif p95_fallback < current_limit * 0.3:
            # 95% 分位数远低于当前限制，可以收紧以更早发现问题
            suggested_limit = int(p95_fallback * 2.0)
            self.results.append(AnalysisResult(
                category="transform",
                severity="info",
                parameter="transform.fallback_duration_limit_ms",
                current_value=current_limit,
                suggested_value=max(suggested_limit, 100),  # 最小 100ms
                reason=f"95%分位降级持续时间({p95_fallback:.0f}ms)远低于警告阈值({current_limit}ms)，可收紧以更早发现问题。",
                confidence=0.6,
                tuning_category=TuningCategory.TUNABLE
            ))
        
        # 分析临界阈值
        current_critical = transform_config.get('fallback_critical_limit_ms', 1000)
        if p99_fallback > current_critical:
            # 99% 分位数超过临界限制
            suggested_critical = int(p99_fallback * 1.2)
            self.results.append(AnalysisResult(
                category="transform",
                severity="critical",
                parameter="transform.fallback_critical_limit_ms",
                current_value=current_critical,
                suggested_value=suggested_critical,
                reason=f"99%分位降级持续时间({p99_fallback:.0f}ms)超过临界阈值({current_critical}ms)，可能触发安全停止。",
                confidence=0.85,
                tuning_category=TuningCategory.TUNABLE
            ))
        elif max_fallback > current_critical * 0.8:
            # 最大值接近临界限制
            self.results.append(AnalysisResult(
                category="transform",
                severity="warning",
                parameter="transform.fallback_critical_limit_ms",
                current_value=current_critical,
                suggested_value=int(max_fallback * 1.3),
                reason=f"最大降级持续时间({max_fallback:.0f}ms)接近临界阈值({current_critical}ms)。",
                confidence=0.7,
                tuning_category=TuningCategory.TUNABLE
            ))
    
    def _analyze_transform_frames(self):
        """分析坐标系配置
        
        检测坐标系配置问题：
        1. 源坐标系不一致（可能是配置错误）
        2. 目标坐标系不一致（可能是配置错误）
        3. 坐标系与配置文件不匹配
        """
        transform_config = self.config.get('transform', {})
        expected_source = transform_config.get('source_frame', 'base_link')
        expected_target = transform_config.get('target_frame', 'odom')
        
        # 分析源坐标系
        if self.stats.transform_source_frames:
            unique_sources = set(self.stats.transform_source_frames)
            if len(unique_sources) > 1:
                # 多个不同的源坐标系，可能是配置问题
                self.results.append(AnalysisResult(
                    category="transform",
                    severity="warning",
                    parameter="transform.source_frame",
                    current_value=expected_source,
                    suggested_value=list(unique_sources)[0],
                    reason=f"检测到多个源坐标系: {unique_sources}，可能存在配置不一致。",
                    confidence=0.8,
                    tuning_category=TuningCategory.DIAGNOSTIC
                ))
            elif len(unique_sources) == 1:
                actual_source = list(unique_sources)[0]
                if actual_source != expected_source:
                    self.results.append(AnalysisResult(
                        category="transform",
                        severity="info",
                        parameter="transform.source_frame",
                        current_value=expected_source,
                        suggested_value=actual_source,
                        reason=f"实际源坐标系({actual_source})与配置({expected_source})不一致。",
                        confidence=0.9,
                        tuning_category=TuningCategory.DIAGNOSTIC
                    ))
        
        # 分析目标坐标系
        if self.stats.transform_target_frames:
            unique_targets = set(self.stats.transform_target_frames)
            if len(unique_targets) > 1:
                self.results.append(AnalysisResult(
                    category="transform",
                    severity="warning",
                    parameter="transform.target_frame",
                    current_value=expected_target,
                    suggested_value=list(unique_targets)[0],
                    reason=f"检测到多个目标坐标系: {unique_targets}，可能存在配置不一致。",
                    confidence=0.8,
                    tuning_category=TuningCategory.DIAGNOSTIC
                ))
            elif len(unique_targets) == 1:
                actual_target = list(unique_targets)[0]
                if actual_target != expected_target:
                    self.results.append(AnalysisResult(
                        category="transform",
                        severity="info",
                        parameter="transform.target_frame",
                        current_value=expected_target,
                        suggested_value=actual_target,
                        reason=f"实际目标坐标系({actual_target})与配置({expected_target})不一致。",
                        confidence=0.9,
                        tuning_category=TuningCategory.DIAGNOSTIC
                    ))
    
    def _analyze_transform_errors(self):
        """分析坐标变换错误信息
        
        统计和分析变换过程中的错误，帮助诊断 TF2 问题。
        """
        if not self.stats.transform_error_messages:
            return
        
        # 统计错误类型
        error_counts = {}
        for msg in self.stats.transform_error_messages:
            if msg:
                # 简化错误信息用于分类
                key = msg[:50] if len(msg) > 50 else msg
                error_counts[key] = error_counts.get(key, 0) + 1
        
        if error_counts:
            total_errors = sum(error_counts.values())
            error_rate = total_errors / self.stats.total_samples
            
            # 找出最常见的错误
            most_common = max(error_counts.items(), key=lambda x: x[1])
            
            if error_rate > 0.01:  # 超过 1% 的错误率
                self.results.append(AnalysisResult(
                    category="transform",
                    severity="warning" if error_rate > 0.05 else "info",
                    parameter="transform.error_analysis",
                    current_value=f"{error_rate*100:.1f}% 错误率",
                    suggested_value="检查 TF2 配置",
                    reason=f"坐标变换错误率 {error_rate*100:.1f}%，最常见错误: {most_common[0]}",
                    confidence=0.9,
                    tuning_category=TuningCategory.DIAGNOSTIC
                ))

    def _analyze_backup_controller(self):
        """分析备份控制器配置
        
        只分析 turtlebot1.yaml 中实际存在的参数:
        - backup.lookahead_dist
        - backup.min_lookahead
        - backup.max_lookahead
        - backup.kp_heading
        """
        backup_config = self.config.get('backup', {})
        constraints_config = self.config.get('constraints', {})
        
        if self.stats.total_samples == 0:
            return
        
        backup_rate = self.stats.backup_active_state_count / self.stats.total_samples
        
        # 只有当备用控制器激活频繁时才分析
        if backup_rate > 0.05:
            v_max = constraints_config.get('v_max', 0.5)
            
            # 检查前瞻距离
            lookahead = backup_config.get('lookahead_dist', 0.5)
            min_lookahead = backup_config.get('min_lookahead', 0.3)
            max_lookahead = backup_config.get('max_lookahead', 1.5)
            suggested_lookahead = v_max * 1.0  # 约1秒的前瞻
            
            if lookahead < suggested_lookahead * 0.5:
                self.results.append(AnalysisResult(
                    category="backup",
                    severity="info",
                    parameter="backup.lookahead_dist",
                    current_value=lookahead,
                    suggested_value=round(min(suggested_lookahead, max_lookahead), 2),
                    reason=f"备用控制器激活率({backup_rate*100:.1f}%)较高，前瞻距离可能过小。",
                    confidence=0.5,
                    tuning_category=TuningCategory.TUNABLE
                ))
            
            # 检查航向增益
            if self.stats.heading_errors:
                avg_heading_error = np.mean(self.stats.heading_errors)
                kp_heading = backup_config.get('kp_heading', 2.0)
                
                if avg_heading_error > 0.3:  # > 17度
                    suggested_kp = min(kp_heading * 1.2, 3.0)
                    self.results.append(AnalysisResult(
                        category="backup",
                        severity="info",
                        parameter="backup.kp_heading",
                        current_value=kp_heading,
                        suggested_value=round(suggested_kp, 2),
                        reason=f"航向误差({np.degrees(avg_heading_error):.1f}deg)较大，可增加航向增益。",
                        confidence=0.5,
                        tuning_category=TuningCategory.TUNABLE
                    ))


    def _analyze_consistency_params(self):
        """分析一致性检查参数
        
        基于运行数据分析并调优以下参数:
        - consistency.temporal_window_size: 基于轨迹频率
        - consistency.alpha_min: 基于 alpha 分布
        - consistency.kappa_thresh: 基于曲率一致性得分
        - consistency.v_dir_thresh: 基于速度方向一致性得分
        - consistency.temporal_smooth_thresh: 基于时序平滑度得分
        - consistency.weights.*: 基于各维度得分分布
        
        v4.3 新增: 一致性参数从设计参数改为可调优参数
        """
        consistency_config = self.config.get('consistency', {})
        
        if self.stats.total_samples < 50:
            return
        
        # 1. 分析 alpha 分布，调优 alpha_min
        if self.stats.alpha_values:
            alpha_arr = np.array(self.stats.alpha_values)
            alpha_p5 = np.percentile(alpha_arr, 5)
            alpha_p10 = np.percentile(alpha_arr, 10)
            avg_alpha = np.mean(alpha_arr)
            
            current_alpha_min = consistency_config.get('alpha_min', 0.1)
            
            # 如果 5% 分位数的 alpha 值低于当前阈值，说明阈值可能过高
            # 导致过多的 soft 模式被禁用
            if alpha_p5 < current_alpha_min and avg_alpha > 0.3:
                # 建议将 alpha_min 设为 p5 的一半，但不低于 0.05
                suggested_alpha_min = max(round(alpha_p5 * 0.5, 2), 0.05)
                if suggested_alpha_min < current_alpha_min * 0.8:
                    self.results.append(AnalysisResult(
                        category="consistency",
                        severity="info",
                        parameter="consistency.alpha_min",
                        current_value=current_alpha_min,
                        suggested_value=suggested_alpha_min,
                        reason=f"alpha p5={alpha_p5:.2f} 低于阈值，降低 alpha_min 避免过早禁用 soft 模式",
                        confidence=0.6,
                        tuning_category=TuningCategory.TUNABLE
                    ))
            
            # 如果平均 alpha 很低，可能需要放宽一致性阈值
            if avg_alpha < 0.4:
                self._suggest_relax_consistency_thresholds(consistency_config, avg_alpha)
        
        # 2. 分析各维度得分，调优阈值
        self._analyze_consistency_scores(consistency_config)
        
        # 3. 分析权重分配
        self._analyze_consistency_weights(consistency_config)
    
    def _suggest_relax_consistency_thresholds(self, consistency_config: Dict, avg_alpha: float):
        """建议放宽一致性阈值
        
        当平均 alpha 较低时，可能是阈值过严导致的
        """
        # 曲率阈值
        if self.stats.curvature_scores:
            kappa_arr = np.array(self.stats.curvature_scores)
            avg_kappa = np.mean(kappa_arr)
            
            if avg_kappa < 0.6:  # 曲率一致性得分较低
                current_thresh = consistency_config.get('kappa_thresh', 0.5)
                suggested_thresh = round(current_thresh * 1.3, 2)
                suggested_thresh = min(suggested_thresh, 1.0)
                
                if suggested_thresh > current_thresh:
                    self.results.append(AnalysisResult(
                        category="consistency",
                        severity="info",
                        parameter="consistency.kappa_thresh",
                        current_value=current_thresh,
                        suggested_value=suggested_thresh,
                        reason=f"曲率一致性得分较低(avg={avg_kappa:.2f})，放宽阈值提高 alpha",
                        confidence=0.5,
                        tuning_category=TuningCategory.TUNABLE
                    ))
        
        # 速度方向阈值
        if self.stats.velocity_dir_scores:
            v_dir_arr = np.array(self.stats.velocity_dir_scores)
            avg_v_dir = np.mean(v_dir_arr)
            
            if avg_v_dir < 0.6:
                current_thresh = consistency_config.get('v_dir_thresh', 0.8)
                # 降低阈值（v_dir_thresh 是 cos 相似度阈值，越低越宽松）
                suggested_thresh = round(current_thresh * 0.8, 2)
                suggested_thresh = max(suggested_thresh, 0.5)
                
                if suggested_thresh < current_thresh:
                    self.results.append(AnalysisResult(
                        category="consistency",
                        severity="info",
                        parameter="consistency.v_dir_thresh",
                        current_value=current_thresh,
                        suggested_value=suggested_thresh,
                        reason=f"速度方向一致性得分较低(avg={avg_v_dir:.2f})，降低阈值提高 alpha",
                        confidence=0.5,
                        tuning_category=TuningCategory.TUNABLE
                    ))
        
        # 时序平滑度阈值
        if self.stats.temporal_scores:
            temporal_arr = np.array(self.stats.temporal_scores)
            avg_temporal = np.mean(temporal_arr)
            
            if avg_temporal < 0.6:
                current_thresh = consistency_config.get('temporal_smooth_thresh', 0.5)
                suggested_thresh = round(current_thresh * 1.3, 2)
                suggested_thresh = min(suggested_thresh, 1.0)
                
                if suggested_thresh > current_thresh:
                    self.results.append(AnalysisResult(
                        category="consistency",
                        severity="info",
                        parameter="consistency.temporal_smooth_thresh",
                        current_value=current_thresh,
                        suggested_value=suggested_thresh,
                        reason=f"时序平滑度得分较低(avg={avg_temporal:.2f})，放宽阈值提高 alpha",
                        confidence=0.5,
                        tuning_category=TuningCategory.TUNABLE
                    ))
    
    def _analyze_consistency_scores(self, consistency_config: Dict):
        """分析各维度一致性得分"""
        # 检查是否有某个维度得分特别低，可能需要调整权重
        scores = {}
        
        if self.stats.curvature_scores:
            scores['kappa'] = np.mean(self.stats.curvature_scores)
        if self.stats.velocity_dir_scores:
            scores['velocity'] = np.mean(self.stats.velocity_dir_scores)
        if self.stats.temporal_scores:
            scores['temporal'] = np.mean(self.stats.temporal_scores)
        
        if len(scores) < 2:
            return
        
        # 找出得分最低的维度
        min_dim = min(scores, key=scores.get)
        min_score = scores[min_dim]
        avg_score = np.mean(list(scores.values()))
        
        # 如果某个维度得分显著低于平均，可能需要降低其权重
        if min_score < avg_score * 0.7 and min_score < 0.5:
            weights_config = consistency_config.get('weights', {})
            current_weight = weights_config.get(min_dim, 0.33)
            suggested_weight = round(current_weight * 0.7, 2)
            suggested_weight = max(suggested_weight, 0.1)
            
            if suggested_weight < current_weight * 0.9:
                self.results.append(AnalysisResult(
                    category="consistency",
                    severity="info",
                    parameter=f"consistency.weights.{min_dim}",
                    current_value=current_weight,
                    suggested_value=suggested_weight,
                    reason=f"{min_dim} 维度得分({min_score:.2f})显著低于平均({avg_score:.2f})，降低权重减少影响",
                    confidence=0.5,
                    tuning_category=TuningCategory.TUNABLE
                ))
    
    def _analyze_consistency_weights(self, consistency_config: Dict):
        """分析一致性权重配置"""
        weights_config = consistency_config.get('weights', {})
        
        w_kappa = weights_config.get('kappa', 0.3)
        w_velocity = weights_config.get('velocity', 0.3)
        w_temporal = weights_config.get('temporal', 0.4)
        
        total_weight = w_kappa + w_velocity + w_temporal
        
        # 检查权重总和是否为 1.0
        if abs(total_weight - 1.0) > 0.01:
            # 归一化权重
            self.results.append(AnalysisResult(
                category="consistency",
                severity="warning",
                parameter="consistency.weights",
                current_value={'kappa': w_kappa, 'velocity': w_velocity, 'temporal': w_temporal},
                suggested_value={
                    'kappa': round(w_kappa / total_weight, 2),
                    'velocity': round(w_velocity / total_weight, 2),
                    'temporal': round(w_temporal / total_weight, 2)
                },
                reason=f"一致性权重总和({total_weight:.2f})不等于1.0，建议归一化",
                confidence=0.9,
                tuning_category=TuningCategory.TUNABLE
            ))


    def _report_design_params_status(self):
        """报告设计参数状态 - 仅诊断，不建议调优
        
        这些参数需要系统辨识或专业知识，不应自动调整。
        这里只报告当前状态，供用户参考。
        
        注意: 一致性参数已在 v4.3 中移至可调优参数，由 _analyze_consistency_params() 处理
        """
        # 打滑检测状态报告
        if self.stats.slip_probabilities:
            slip_arr = np.array(self.stats.slip_probabilities)
            high_slip_count = np.sum(slip_arr > 0.5)
            high_slip_rate = high_slip_count / len(slip_arr)
            
            if high_slip_rate > 0.1:
                self.results.append(AnalysisResult(
                    category="estimator_status",
                    severity="info",
                    parameter="estimator.slip_detection",
                    current_value=f"高打滑率 {high_slip_rate*100:.1f}%",
                    suggested_value=None,
                    reason=f"[诊断信息] 打滑检测频繁({high_slip_rate*100:.1f}%)。打滑阈值与底盘特性相关，不建议自动调整。",
                    confidence=0.0,
                    tuning_category=TuningCategory.DESIGN
                ))

    def _report_safety_status(self):
        """报告安全状态 - 仅诊断，不自动放宽安全参数
        
        安全参数不应自动放宽，这里只报告当前状态。
        只报告 turtlebot1.yaml 中实际存在的参数相关信息。
        """
        constraints_config = self.config.get('constraints', {})
        
        # 速度约束使用情况
        if self.stats.cmd_vx:
            vx_arr = np.array(self.stats.cmd_vx)
            current_v_max = constraints_config.get('v_max', 0.5)
            
            at_limit_count = np.sum(np.abs(vx_arr) > current_v_max * 0.95)
            at_limit_rate = at_limit_count / len(vx_arr)
            
            if at_limit_rate > 0.1:
                self.results.append(AnalysisResult(
                    category="safety_status",
                    severity="info",
                    parameter="constraints.v_max",
                    current_value=current_v_max,
                    suggested_value=None,  # 不建议调优
                    reason=f"[诊断信息] 速度经常({at_limit_rate*100:.1f}%)达到上限。速度限制是安全参数，不建议自动放宽。",
                    confidence=0.0,
                    tuning_category=TuningCategory.SAFETY
                ))
        
        # 角速度约束使用情况
        if self.stats.cmd_omega:
            omega_arr = np.array(self.stats.cmd_omega)
            current_omega_max = constraints_config.get('omega_max', 1.0)
            
            if current_omega_max > 0:
                at_limit_count = np.sum(np.abs(omega_arr) > current_omega_max * 0.95)
                at_limit_rate = at_limit_count / len(omega_arr)
                
                if at_limit_rate > 0.1:
                    self.results.append(AnalysisResult(
                        category="safety_status",
                        severity="info",
                        parameter="constraints.omega_max",
                        current_value=current_omega_max,
                        suggested_value=None,
                        reason=f"[诊断信息] 角速度经常({at_limit_rate*100:.1f}%)达到上限。角速度限制是安全参数，不建议自动放宽。",
                        confidence=0.0,
                        tuning_category=TuningCategory.SAFETY
                    ))
        
        # 紧急停止统计
        if self.stats.emergency_stop_count > 0:
            emergency_rate = self.stats.emergency_stop_count / self.stats.total_samples * 100
            self.results.append(AnalysisResult(
                category="safety_status",
                severity="warning" if emergency_rate > 1 else "info",
                parameter="safety.emergency_decel",
                current_value=self.config.get('safety', {}).get('emergency_decel', 1.0),
                suggested_value=None,
                reason=f"[诊断信息] 紧急停止发生{self.stats.emergency_stop_count}次({emergency_rate:.1f}%)。紧急减速度是安全参数，不建议自动调整。",
                confidence=0.0,
                tuning_category=TuningCategory.SAFETY
            ))
        
        # 安全检查失败统计
        if self.stats.safety_check_failed_count > 0:
            fail_rate = self.stats.safety_check_failed_count / self.stats.total_samples * 100
            self.results.append(AnalysisResult(
                category="safety_status",
                severity="warning" if fail_rate > 5 else "info",
                parameter="safety.check",
                current_value=f"失败率 {fail_rate:.1f}%",
                suggested_value=None,
                reason=f"[诊断信息] 安全检查失败{self.stats.safety_check_failed_count}次({fail_rate:.1f}%)。建议检查约束配置是否合理。",
                confidence=0.0,
                tuning_category=TuningCategory.SAFETY
            ))


    def get_summary(self) -> Dict[str, Any]:
        """获取分析摘要
        
        Returns:
            包含各项统计数据的摘要字典
        """
        summary = {
            'total_samples': self.stats.total_samples,
            'platform': self._get_platform(),
            'ctrl_freq': self._get_ctrl_freq(),
            'mpc': {},
            'tracking': {},
            'timeout': {},
            'control': {},
            'state_machine': {},
            'consistency': {},
            'estimator': {},
            'transform': {},
            'safety': {},
        }
        
        # MPC 摘要
        if self.stats.mpc_solve_times:
            solve_times = np.array(self.stats.mpc_solve_times)
            total = self.stats.mpc_success_count + self.stats.mpc_fail_count
            summary['mpc'] = {
                'avg_solve_time_ms': float(round(np.mean(solve_times), 2)),
                'max_solve_time_ms': float(round(np.max(solve_times), 2)),
                'p95_solve_time_ms': float(round(np.percentile(solve_times, 95), 2)),
                'success_rate': float(round(self.stats.mpc_success_count / total * 100, 1)) if total > 0 else 0,
                'backup_rate': float(round(self.stats.backup_active_state_count / self.stats.total_samples * 100, 1)) if self.stats.total_samples > 0 else 0,
                'degradation_warning_count': int(self.stats.mpc_degradation_warnings)
            }
            if self.stats.kkt_residuals:
                summary['mpc']['avg_kkt_residual'] = float(np.mean(self.stats.kkt_residuals))
                summary['mpc']['max_kkt_residual'] = float(np.max(self.stats.kkt_residuals))
            if self.stats.condition_numbers:
                summary['mpc']['avg_condition_number'] = float(np.mean(self.stats.condition_numbers))
                summary['mpc']['max_condition_number'] = float(np.max(self.stats.condition_numbers))
        
        # 跟踪摘要
        if self.stats.lateral_errors:
            summary['tracking']['lateral'] = {
                'avg_cm': float(round(np.mean(self.stats.lateral_errors) * 100, 2)),
                'max_cm': float(round(np.max(self.stats.lateral_errors) * 100, 2)),
                'p95_cm': float(round(np.percentile(self.stats.lateral_errors, 95) * 100, 2))
            }
        if self.stats.longitudinal_errors:
            summary['tracking']['longitudinal'] = {
                'avg_cm': float(round(np.mean(self.stats.longitudinal_errors) * 100, 2)),
                'max_cm': float(round(np.max(self.stats.longitudinal_errors) * 100, 2)),
                'p95_cm': float(round(np.percentile(self.stats.longitudinal_errors, 95) * 100, 2))
            }
        if self.stats.heading_errors:
            summary['tracking']['heading'] = {
                'avg_deg': float(round(np.degrees(np.mean(self.stats.heading_errors)), 2)),
                'max_deg': float(round(np.degrees(np.max(self.stats.heading_errors)), 2))
            }
        if self.stats.prediction_errors:
            summary['tracking']['prediction'] = {
                'avg_cm': float(round(np.mean(self.stats.prediction_errors) * 100, 2)),
                'max_cm': float(round(np.max(self.stats.prediction_errors) * 100, 2))
            }
        # 跟踪质量评估
        if self.stats.tracking_quality_scores:
            summary['tracking']['quality'] = {
                'avg_score': float(round(np.mean(self.stats.tracking_quality_scores), 1)),
                'min_score': float(round(np.min(self.stats.tracking_quality_scores), 1))
            }
            if self.stats.tracking_quality_ratings:
                # 统计各等级占比
                rating_counts = {}
                for rating in self.stats.tracking_quality_ratings:
                    rating_counts[rating] = rating_counts.get(rating, 0) + 1
                total = len(self.stats.tracking_quality_ratings)
                summary['tracking']['quality']['rating_distribution'] = {
                    k: float(round(v / total * 100, 1)) for k, v in rating_counts.items()
                }
        
        # 超时摘要
        summary['timeout'] = {
            'odom_timeout_count': int(self.stats.odom_timeout_count),
            'traj_timeout_count': int(self.stats.traj_timeout_count),
            'traj_grace_exceeded_count': int(self.stats.traj_grace_exceeded_count),
            'imu_timeout_count': int(self.stats.imu_timeout_count),
            'startup_grace_samples': int(self.stats.in_startup_grace_count)
        }
        if self.stats.odom_ages:
            summary['timeout']['avg_odom_age_ms'] = float(round(np.mean(self.stats.odom_ages), 1))
            summary['timeout']['max_odom_age_ms'] = float(round(np.max(self.stats.odom_ages), 1))
            summary['timeout']['p95_odom_age_ms'] = float(round(np.percentile(self.stats.odom_ages, 95), 1))
        if self.stats.traj_ages:
            summary['timeout']['avg_traj_age_ms'] = float(round(np.mean(self.stats.traj_ages), 1))
            summary['timeout']['max_traj_age_ms'] = float(round(np.max(self.stats.traj_ages), 1))
            summary['timeout']['p95_traj_age_ms'] = float(round(np.percentile(self.stats.traj_ages, 95), 1))
        
        # 控制摘要
        if self.stats.cmd_vx:
            summary['control']['avg_vx'] = float(round(np.mean(np.abs(self.stats.cmd_vx)), 3))
            summary['control']['max_vx'] = float(round(np.max(np.abs(self.stats.cmd_vx)), 3))
        if self.stats.cmd_omega:
            summary['control']['avg_omega'] = float(round(np.mean(np.abs(self.stats.cmd_omega)), 3))
            summary['control']['max_omega'] = float(round(np.max(np.abs(self.stats.cmd_omega)), 3))
        
        # 状态机摘要（统一使用基于状态枚举的统计）
        state_dist = {int(k): int(v) for k, v in self.stats.state_counts.items()}
        
        summary['state_machine'] = {
            'state_distribution': state_dist,
            'normal_rate': float(round(self.stats.normal_state_count / self.stats.total_samples * 100, 2)) if self.stats.total_samples > 0 else 0,
            'backup_active_rate': float(round(self.stats.backup_active_state_count / self.stats.total_samples * 100, 2)) if self.stats.total_samples > 0 else 0,
            'soft_disabled_rate': float(round(self.stats.soft_disabled_state_count / self.stats.total_samples * 100, 2)) if self.stats.total_samples > 0 else 0,
            'mpc_degraded_rate': float(round(self.stats.mpc_degraded_state_count / self.stats.total_samples * 100, 2)) if self.stats.total_samples > 0 else 0,
            'stopping_rate': float(round(self.stats.stopping_state_count / self.stats.total_samples * 100, 2)) if self.stats.total_samples > 0 else 0,
            'stopped_rate': float(round(self.stats.stopped_state_count / self.stats.total_samples * 100, 2)) if self.stats.total_samples > 0 else 0,
        }
        
        # 一致性摘要
        if self.stats.alpha_values:
            total_valid = self.stats.data_valid_count + self.stats.data_invalid_count
            summary['consistency'] = {
                'avg_alpha': float(round(np.mean(self.stats.alpha_values), 3)),
                'min_alpha': float(round(np.min(self.stats.alpha_values), 3)),
                'data_valid_rate': float(round(self.stats.data_valid_count / total_valid * 100, 1)) if total_valid > 0 else 100
            }
            if self.stats.curvature_scores:
                summary['consistency']['avg_curvature_score'] = float(round(np.mean(self.stats.curvature_scores), 3))
            if self.stats.velocity_dir_scores:
                summary['consistency']['avg_velocity_dir_score'] = float(round(np.mean(self.stats.velocity_dir_scores), 3))
            if self.stats.temporal_scores:
                summary['consistency']['avg_temporal_score'] = float(round(np.mean(self.stats.temporal_scores), 3))
        
        # 状态估计摘要
        if self.stats.estimator_covariance_norms:
            summary['estimator']['avg_covariance_norm'] = float(round(np.mean(self.stats.estimator_covariance_norms), 2))
            summary['estimator']['max_covariance_norm'] = float(round(np.max(self.stats.estimator_covariance_norms), 2))
        if self.stats.estimator_innovation_norms:
            summary['estimator']['avg_innovation_norm'] = float(round(np.mean(self.stats.estimator_innovation_norms), 2))
            summary['estimator']['max_innovation_norm'] = float(round(np.max(self.stats.estimator_innovation_norms), 2))
        if self.stats.slip_probabilities:
            summary['estimator']['avg_slip_probability'] = float(round(np.mean(self.stats.slip_probabilities), 3))
            summary['estimator']['high_slip_rate'] = float(round(
                np.sum(np.array(self.stats.slip_probabilities) > 0.5) / len(self.stats.slip_probabilities) * 100, 1))
        summary['estimator']['imu_drift_detected_count'] = int(self.stats.imu_drift_detected_count)
        summary['estimator']['imu_available_rate'] = float(round(
            self.stats.imu_available_count / self.stats.total_samples * 100, 1)) if self.stats.total_samples > 0 else 0
        
        # 坐标变换摘要
        summary['transform'] = {
            'tf2_fallback_count': int(self.stats.tf2_fallback_count),
            'tf2_fallback_rate': float(round(self.stats.tf2_fallback_count / self.stats.total_samples * 100, 2)) if self.stats.total_samples > 0 else 0,
            'tf2_available_rate': float(round(self.stats.tf2_available_count / self.stats.total_samples * 100, 2)) if self.stats.total_samples > 0 else 100
        }
        if self.stats.tf2_fallback_durations:
            summary['transform']['avg_fallback_duration_ms'] = float(round(np.mean(self.stats.tf2_fallback_durations), 1))
            summary['transform']['max_fallback_duration_ms'] = float(round(np.max(self.stats.tf2_fallback_durations), 1))
            summary['transform']['p95_fallback_duration_ms'] = float(round(np.percentile(self.stats.tf2_fallback_durations, 95), 1))
        if self.stats.accumulated_drifts:
            summary['transform']['max_accumulated_drift_m'] = float(round(np.max(self.stats.accumulated_drifts), 4))
        # 坐标系信息
        if self.stats.transform_source_frames:
            unique_sources = list(set(self.stats.transform_source_frames))
            summary['transform']['source_frames'] = unique_sources
        if self.stats.transform_target_frames:
            unique_targets = list(set(self.stats.transform_target_frames))
            summary['transform']['target_frames'] = unique_targets
        if self.stats.transform_error_messages:
            error_count = len([m for m in self.stats.transform_error_messages if m])
            summary['transform']['error_count'] = error_count
            summary['transform']['error_rate'] = float(round(error_count / self.stats.total_samples * 100, 2)) if self.stats.total_samples > 0 else 0
        
        # 状态持续时间摘要
        if self.stats.degraded_state_durations:
            summary['state_machine']['degraded_duration'] = {
                'avg_sec': float(round(np.mean(self.stats.degraded_state_durations), 2)),
                'max_sec': float(round(np.max(self.stats.degraded_state_durations), 2)),
                'p95_sec': float(round(np.percentile(self.stats.degraded_state_durations, 95), 2)),
                'count': len(self.stats.degraded_state_durations)
            }
        if self.stats.backup_state_durations:
            summary['state_machine']['backup_duration'] = {
                'avg_sec': float(round(np.mean(self.stats.backup_state_durations), 2)),
                'max_sec': float(round(np.max(self.stats.backup_state_durations), 2)),
                'p95_sec': float(round(np.percentile(self.stats.backup_state_durations, 95), 2)),
                'count': len(self.stats.backup_state_durations)
            }
        
        # MPC 失败/恢复比率摘要
        if self.stats.mpc_fail_ratios:
            summary['state_machine']['mpc_fail_ratio'] = {
                'avg': float(round(np.mean(self.stats.mpc_fail_ratios), 3)),
                'max': float(round(np.max(self.stats.mpc_fail_ratios), 3)),
                'p95': float(round(np.percentile(self.stats.mpc_fail_ratios, 95), 3))
            }
        if self.stats.mpc_recovery_ratios:
            summary['state_machine']['mpc_recovery_ratio'] = {
                'avg': float(round(np.mean(self.stats.mpc_recovery_ratios), 3)),
                'min': float(round(np.min(self.stats.mpc_recovery_ratios), 3)),
                'p10': float(round(np.percentile(self.stats.mpc_recovery_ratios, 10), 3))
            }
        
        # 安全状态摘要
        summary['safety'] = {
            'safety_check_failed_count': int(self.stats.safety_check_failed_count),
            'emergency_stop_count': int(self.stats.emergency_stop_count),
            'consecutive_errors_max': int(self.stats.consecutive_errors_max)
        }
        if self.stats.total_samples > 0:
            summary['safety']['safety_check_failed_rate'] = float(round(
                self.stats.safety_check_failed_count / self.stats.total_samples * 100, 2))
            summary['safety']['emergency_stop_rate'] = float(round(
                self.stats.emergency_stop_count / self.stats.total_samples * 100, 2))
        
        return summary
