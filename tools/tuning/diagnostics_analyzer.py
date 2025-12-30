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

# 可调优参数：基于运行数据可以安全调整
TUNABLE_PARAMS = {
    # 超时配置 - 基于实际延迟统计
    'watchdog.odom_timeout_ms',
    'watchdog.traj_timeout_ms',
    'watchdog.traj_grace_ms',
    'watchdog.imu_timeout_ms',
    'watchdog.startup_grace_ms',
    
    # MPC 健康监控阈值 - 基于求解时间统计
    'mpc.health_monitor.time_warning_thresh_ms',
    'mpc.health_monitor.time_critical_thresh_ms',
    'mpc.health_monitor.time_recovery_thresh_ms',
    'mpc.health_monitor.consecutive_warning_limit',
    'mpc.health_monitor.kkt_residual_thresh',
    'mpc.health_monitor.condition_number_thresh',
    'mpc.health_monitor.condition_number_recovery',
    
    # MPC 预测时域 - 基于求解时间与控制周期关系
    'mpc.horizon',
    'mpc.horizon_degraded',
    
    # MPC 权重 - 基于跟踪误差统计（需验证）
    'mpc.weights.position',
    'mpc.weights.velocity',
    'mpc.weights.heading',
    
    # 状态机参数 - 基于状态转换统计
    'safety.state_machine.mpc_fail_thresh',
    'safety.state_machine.mpc_fail_ratio_thresh',
    'safety.state_machine.mpc_recovery_thresh',
    'safety.state_machine.mpc_recovery_success_ratio',
    
    # 跟踪质量阈值 - 仅影响显示，不影响控制
    'tracking.lateral_thresh',
    'tracking.longitudinal_thresh',
    'tracking.heading_thresh',
    'tracking.prediction_thresh',
    
    # 坐标变换降级阈值 - 基于 TF2 可用性统计
    'transform.fallback_duration_limit_ms',
    'transform.fallback_critical_limit_ms',
    'transform.timeout_ms',
    
    # 备份控制器参数 - 基于备份激活时的表现
    'backup.lookahead_dist',
    'backup.kp_heading',
}

# 设计参数：需要系统辨识或专业知识，不应自动调整
DESIGN_PARAMS = {
    # EKF 噪声参数 - 需要传感器规格或系统辨识
    'ekf.process_noise.position',
    'ekf.process_noise.velocity',
    'ekf.process_noise.orientation',
    'ekf.process_noise.angular_velocity',
    'ekf.process_noise.imu_bias',
    'ekf.measurement_noise.odom_position',
    'ekf.measurement_noise.odom_velocity',
    'ekf.measurement_noise.odom_orientation',
    'ekf.measurement_noise.odom_angular_velocity',
    'ekf.measurement_noise.imu_accel',
    'ekf.measurement_noise.imu_gyro',
    
    # EKF 自适应参数 - 与底盘特性相关
    'ekf.adaptive.base_slip_thresh',
    'ekf.adaptive.slip_velocity_factor',
    'ekf.adaptive.slip_covariance_scale',
    
    # 一致性检查阈值 - 是设计参数，不应根据运行数据调整
    'consistency.kappa_thresh',
    'consistency.v_dir_thresh',
    'consistency.temporal_smooth_thresh',
    'consistency.alpha_min',
    'consistency.invalid_data_confidence',
    
    # 过渡配置 - 是设计参数
    'transition.tau',
    'transition.max_duration',
    'transition.completion_threshold',
    
    # MPC 控制输入权重 - 需要控制理论知识
    'mpc.weights.control_accel',
    'mpc.weights.control_alpha',
}

# 安全参数：不应自动放宽
SAFETY_PARAMS = {
    # 速度/加速度约束 - 安全限制
    'constraints.v_max',
    'constraints.v_min',
    'constraints.omega_max',
    'constraints.omega_max_low',
    'constraints.a_max',
    'constraints.alpha_max',
    'constraints.vx_max',
    'constraints.vy_max',
    'constraints.vz_max',
    
    # 安全配置
    'safety.emergency_decel',
    'safety.velocity_margin',
    'safety.accel_margin',
    'safety.v_stop_thresh',
    
    # EKF 异常检测阈值 - 安全相关
    'ekf.anomaly_detection.covariance_explosion_thresh',
    'ekf.anomaly_detection.innovation_anomaly_thresh',
    'ekf.anomaly_detection.drift_thresh',
    'ekf.anomaly_detection.jump_thresh',
}

# 差速车平台不使用的参数
NON_DIFFERENTIAL_PARAMS = {
    'constraints.vy_max', 'constraints.vy_min',
    'constraints.vz_max', 'constraints.az_max',
    'backup.kp_z', 'safety.vz_stop_thresh',
}


@dataclass
class DiagnosticsStats:
    """诊断统计数据"""
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
    state_counts: Dict[int, int] = field(default_factory=dict)
    backup_active_count: int = 0
    soft_disabled_count: int = 0
    mpc_degraded_count: int = 0
    stopping_count: int = 0
    stopped_count: int = 0
    normal_count: int = 0
    
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
    transform_source_frames: List[str] = field(default_factory=list)  # 源坐标系记录
    transform_target_frames: List[str] = field(default_factory=list)  # 目标坐标系记录
    transform_error_messages: List[str] = field(default_factory=list)  # 错误信息记录
    
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
        """获取控制频率"""
        if self._ctrl_freq is None:
            self._ctrl_freq = self.config.get('system', {}).get('ctrl_freq', 20)
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
        state = diagnostics.get('state', 0)
        self.stats.state_counts[state] = self.stats.state_counts.get(state, 0) + 1
        
        if diagnostics.get('backup_active', False):
            self.stats.backup_active_count += 1
        
        # 按状态类型分别统计
        if state == ControllerState.NORMAL:
            self.stats.normal_count += 1
        elif state == ControllerState.SOFT_DISABLED:
            self.stats.soft_disabled_count += 1
        elif state == ControllerState.MPC_DEGRADED:
            self.stats.mpc_degraded_count += 1
        elif state == ControllerState.STOPPING:
            self.stats.stopping_count += 1
        elif state == ControllerState.STOPPED:
            self.stats.stopped_count += 1
        
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
        
        # 3. 诊断信息（不生成调优建议，仅报告状态）
        self._report_design_params_status()     # 设计参数状态报告
        self._report_safety_status()            # 安全状态报告
        
        return self.results


    def _analyze_config_errors(self):
        """分析配置错误 - 检测必须修复的硬性错误
        
        这些是配置错误，不是调优建议，必须修复。
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
        
        # 5. 检查 MPC 求解时间与控制周期的关系
        if self.stats.mpc_solve_times:
            solve_arr = np.array(self.stats.mpc_solve_times)
            max_solve = np.max(solve_arr)
            
            if max_solve > ctrl_period_ms:
                self.results.append(AnalysisResult(
                    category="config_error",
                    severity="critical",
                    parameter="mpc.horizon",
                    current_value=horizon,
                    suggested_value=max(horizon - 2, 3),
                    reason=f"MPC最大求解时间({max_solve:.1f}ms)超过控制周期({ctrl_period_ms:.1f}ms)，需要减小预测时域。",
                    confidence=0.95,
                    tuning_category=TuningCategory.TUNABLE
                ))
        
        # 6. 检查跟踪权重总和
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
        """分析 MPC 性能"""
        if not self.stats.mpc_solve_times:
            return
        
        solve_times = np.array(self.stats.mpc_solve_times)
        avg_time = np.mean(solve_times)
        max_time = np.max(solve_times)
        p95_time = np.percentile(solve_times, 95)
        
        mpc_config = self.config.get('mpc', {})
        ctrl_period_ms = self._get_ctrl_period_ms()
        
        # 分析 MPC 成功率
        total = self.stats.mpc_success_count + self.stats.mpc_fail_count
        if total > 0:
            success_rate = self.stats.mpc_success_count / total
            if success_rate < 0.9:
                current_horizon = mpc_config.get('horizon', 7)
                self.results.append(AnalysisResult(
                    category="mpc",
                    severity="warning",
                    parameter="mpc.horizon",
                    current_value=current_horizon,
                    suggested_value=max(current_horizon - 1, 3),
                    reason=f"MPC成功率({success_rate*100:.1f}%)较低，建议减小预测时域。",
                    confidence=0.7,
                    tuning_category=TuningCategory.TUNABLE
                ))
        
        # 分析求解时间与控制周期的关系
        if p95_time > ctrl_period_ms * 0.5:
            current_horizon = mpc_config.get('horizon', 7)
            self.results.append(AnalysisResult(
                category="mpc",
                severity="warning",
                parameter="mpc.horizon",
                current_value=current_horizon,
                suggested_value=max(current_horizon - 1, 3),
                reason=f"MPC求解时间({p95_time:.1f}ms)占用控制周期({ctrl_period_ms:.1f}ms)>50%，建议减小预测时域。",
                confidence=0.7,
                tuning_category=TuningCategory.TUNABLE
            ))

    def _analyze_mpc_health_monitor(self):
        """分析 MPC 健康监控参数"""
        if not self.stats.mpc_solve_times:
            return
        
        solve_times = np.array(self.stats.mpc_solve_times)
        p95_time = np.percentile(solve_times, 95)
        max_time = np.max(solve_times)
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
        
        # 分析 KKT 残差阈值
        if self.stats.kkt_residuals:
            kkt_arr = np.array(self.stats.kkt_residuals)
            p99_kkt = np.percentile(kkt_arr, 99)
            current_kkt_thresh = health_config.get('kkt_residual_thresh', 0.001)
            
            if p99_kkt > current_kkt_thresh:
                self.results.append(AnalysisResult(
                    category="mpc",
                    severity="warning",
                    parameter="mpc.health_monitor.kkt_residual_thresh",
                    current_value=current_kkt_thresh,
                    suggested_value=round(p99_kkt * 1.5, 6),
                    reason=f"99%分位KKT残差({p99_kkt:.6f})超过阈值({current_kkt_thresh})。",
                    confidence=0.7,
                    tuning_category=TuningCategory.TUNABLE
                ))
        
        # 分析条件数阈值
        if self.stats.condition_numbers:
            cond_arr = np.array(self.stats.condition_numbers)
            p99_cond = np.percentile(cond_arr, 99)
            current_cond_thresh = health_config.get('condition_number_thresh', 1e8)
            
            if p99_cond > current_cond_thresh * 0.5:
                self.results.append(AnalysisResult(
                    category="mpc",
                    severity="warning",
                    parameter="mpc.health_monitor.condition_number_thresh",
                    current_value=current_cond_thresh,
                    suggested_value=p99_cond * 2,
                    reason=f"99%分位条件数({p99_cond:.2e})接近阈值({current_cond_thresh:.2e})。",
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
        if self.stats.heading_errors:
            head_arr = np.array(self.stats.heading_errors)
            avg_head = np.mean(head_arr)
            p95_head = np.percentile(head_arr, 95)
            
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
            
            if p95_head > current_thresh:
                suggested_thresh = round(p95_head * 1.2, 2)
                self.results.append(AnalysisResult(
                    category="tracking",
                    severity="info",
                    parameter="tracking.heading_thresh",
                    current_value=current_thresh,
                    suggested_value=suggested_thresh,
                    reason=f"95%分位航向误差({np.degrees(p95_head):.1f}deg)超过阈值。",
                    confidence=0.7,
                    tuning_category=TuningCategory.TUNABLE
                ))


    def _analyze_state_machine(self):
        """分析状态机配置"""
        safety_config = self.config.get('safety', {})
        state_machine = safety_config.get('state_machine', {})
        
        if self.stats.total_samples == 0:
            return
        
        # 计算各种比率
        degraded_rate = self.stats.mpc_degraded_count / self.stats.total_samples
        backup_rate = self.stats.backup_active_count / self.stats.total_samples
        
        total_mpc = self.stats.mpc_success_count + self.stats.mpc_fail_count
        mpc_success_rate = self.stats.mpc_success_count / total_mpc if total_mpc > 0 else 0
        
        # 1. 如果 MPC 成功率高但降级频繁，说明阈值太敏感
        if degraded_rate > 0.1 and mpc_success_rate > 0.8:
            current_fail_thresh = state_machine.get('mpc_fail_thresh', 3)
            self.results.append(AnalysisResult(
                category="state_machine",
                severity="warning",
                parameter="safety.state_machine.mpc_fail_thresh",
                current_value=current_fail_thresh,
                suggested_value=min(current_fail_thresh + 2, 10),
                reason=f"MPC成功率({mpc_success_rate*100:.1f}%)高但降级频繁({degraded_rate*100:.1f}%)，阈值过敏感。",
                confidence=0.8,
                tuning_category=TuningCategory.TUNABLE
            ))
        
        # 2. 如果备用控制器激活率过高，可能恢复条件太严格
        if backup_rate > 0.15:
            current_success_ratio = state_machine.get('mpc_recovery_success_ratio', 0.8)
            self.results.append(AnalysisResult(
                category="state_machine",
                severity="warning",
                parameter="safety.state_machine.mpc_recovery_success_ratio",
                current_value=current_success_ratio,
                suggested_value=max(current_success_ratio - 0.1, 0.6),
                reason=f"备用控制器激活率({backup_rate*100:.1f}%)过高，恢复条件可能太严格。",
                confidence=0.7,
                tuning_category=TuningCategory.TUNABLE
            ))
            
            current_recovery_thresh = state_machine.get('mpc_recovery_thresh', 5)
            self.results.append(AnalysisResult(
                category="state_machine",
                severity="info",
                parameter="safety.state_machine.mpc_recovery_thresh",
                current_value=current_recovery_thresh,
                suggested_value=max(current_recovery_thresh - 1, 3),
                reason=f"备用控制器激活率高，可降低恢复阈值。",
                confidence=0.5,
                tuning_category=TuningCategory.TUNABLE
            ))
        
        # 3. Alpha 禁用阈值分析（仅当 alpha 检查未被禁用时）
        if self.stats.alpha_values and not self._is_alpha_check_disabled():
            alpha_arr = np.array(self.stats.alpha_values)
            avg_alpha = np.mean(alpha_arr)
            alpha_disable_thresh = state_machine.get('alpha_disable_thresh', 0.0)
            soft_disabled_rate = self.stats.soft_disabled_count / self.stats.total_samples
            
            if avg_alpha < 0.3 and soft_disabled_rate > 0.1:
                self.results.append(AnalysisResult(
                    category="state_machine",
                    severity="warning",
                    parameter="safety.state_machine.alpha_disable_thresh",
                    current_value=alpha_disable_thresh,
                    suggested_value=0.0,
                    reason=f"平均alpha({avg_alpha:.2f})较低且SOFT_DISABLED频繁({soft_disabled_rate*100:.1f}%)，建议禁用alpha检查。",
                    confidence=0.7,
                    tuning_category=TuningCategory.TUNABLE
                ))


    def _analyze_transform(self):
        """分析坐标变换配置"""
        transform_config = self.config.get('transform', {})
        
        if self.stats.total_samples == 0:
            return
        
        # 分析 TF2 降级
        if self.stats.tf2_fallback_count > 0:
            fallback_rate = self.stats.tf2_fallback_count / self.stats.total_samples
            
            if fallback_rate > 0.05 and self.stats.tf2_fallback_durations:
                p95_duration = np.percentile(self.stats.tf2_fallback_durations, 95)
                
                current_limit = transform_config.get('fallback_duration_limit_ms', 500)
                current_critical = transform_config.get('fallback_critical_limit_ms', 1000)
                
                if p95_duration > current_limit * 0.8:
                    self.results.append(AnalysisResult(
                        category="transform",
                        severity="warning",
                        parameter="transform.fallback_duration_limit_ms",
                        current_value=current_limit,
                        suggested_value=int(p95_duration * 1.5),
                        reason=f"TF2降级频繁({fallback_rate*100:.1f}%)，95%分位持续{p95_duration:.0f}ms。",
                        confidence=0.7,
                        tuning_category=TuningCategory.TUNABLE
                    ))
                
                if p95_duration > current_critical * 0.8:
                    self.results.append(AnalysisResult(
                        category="transform",
                        severity="warning",
                        parameter="transform.fallback_critical_limit_ms",
                        current_value=current_critical,
                        suggested_value=int(p95_duration * 2),
                        reason=f"TF2降级持续时间({p95_duration:.0f}ms)接近临界限制。",
                        confidence=0.7,
                        tuning_category=TuningCategory.TUNABLE
                    ))
        
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
        
        # 分析坐标系配置一致性
        self._analyze_transform_frames()
        
        # 分析变换错误信息
        self._analyze_transform_errors()
    
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
        """分析备份控制器配置"""
        backup_config = self.config.get('backup', {})
        constraints_config = self.config.get('constraints', {})
        
        if self.stats.total_samples == 0:
            return
        
        backup_rate = self.stats.backup_active_count / self.stats.total_samples
        
        # 只有当备用控制器激活频繁时才分析
        if backup_rate > 0.05:
            v_max = constraints_config.get('v_max', 0.5)
            
            # 检查前瞻距离
            lookahead = backup_config.get('lookahead_dist', 0.5)
            suggested_lookahead = v_max * 1.0  # 约1秒的前瞻
            
            if lookahead < suggested_lookahead * 0.5:
                self.results.append(AnalysisResult(
                    category="backup",
                    severity="info",
                    parameter="backup.lookahead_dist",
                    current_value=lookahead,
                    suggested_value=round(suggested_lookahead, 2),
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


    def _report_design_params_status(self):
        """报告设计参数状态 - 仅诊断，不建议调优
        
        这些参数需要系统辨识或专业知识，不应自动调整。
        这里只报告当前状态，供用户参考。
        """
        # EKF 状态报告
        if self.stats.estimator_covariance_norms:
            cov_arr = np.array(self.stats.estimator_covariance_norms)
            max_cov = np.max(cov_arr)
            avg_cov = np.mean(cov_arr)
            
            ekf_config = self.config.get('ekf', {})
            anomaly_config = ekf_config.get('anomaly_detection', {})
            cov_thresh = anomaly_config.get('covariance_explosion_thresh', 1000.0)
            
            if max_cov > cov_thresh * 0.5:
                self.results.append(AnalysisResult(
                    category="ekf_status",
                    severity="info",
                    parameter="ekf.anomaly_detection.covariance_explosion_thresh",
                    current_value=cov_thresh,
                    suggested_value=None,  # 不建议调优
                    reason=f"[诊断信息] 协方差范数(max={max_cov:.1f})接近阈值。此为设计参数，不建议自动调整。",
                    confidence=0.0,  # 置信度为0表示不应采纳
                    tuning_category=TuningCategory.DESIGN
                ))
        
        # 一致性检查状态报告
        if self.stats.alpha_values and not self._is_alpha_check_disabled():
            alpha_arr = np.array(self.stats.alpha_values)
            avg_alpha = np.mean(alpha_arr)
            min_alpha = np.min(alpha_arr)
            
            if avg_alpha < 0.5:
                self.results.append(AnalysisResult(
                    category="consistency_status",
                    severity="info",
                    parameter="consistency.alpha_min",
                    current_value=self.config.get('consistency', {}).get('alpha_min', 0.1),
                    suggested_value=None,
                    reason=f"[诊断信息] 平均alpha值({avg_alpha:.2f})较低。一致性阈值是设计参数，不建议自动调整。",
                    confidence=0.0,
                    tuning_category=TuningCategory.DESIGN
                ))
        
        # 打滑检测状态报告
        if self.stats.slip_probabilities:
            slip_arr = np.array(self.stats.slip_probabilities)
            high_slip_count = np.sum(slip_arr > 0.5)
            high_slip_rate = high_slip_count / len(slip_arr)
            
            if high_slip_rate > 0.1:
                self.results.append(AnalysisResult(
                    category="ekf_status",
                    severity="info",
                    parameter="ekf.adaptive.base_slip_thresh",
                    current_value=self.config.get('ekf', {}).get('adaptive', {}).get('base_slip_thresh', 2.0),
                    suggested_value=None,
                    reason=f"[诊断信息] 打滑检测频繁({high_slip_rate*100:.1f}%)。打滑阈值与底盘特性相关，不建议自动调整。",
                    confidence=0.0,
                    tuning_category=TuningCategory.DESIGN
                ))

    def _report_safety_status(self):
        """报告安全状态 - 仅诊断，不自动放宽安全参数
        
        安全参数不应自动放宽，这里只报告当前状态。
        """
        constraints_config = self.config.get('constraints', {})
        
        # 速度约束使用情况
        if self.stats.cmd_vx:
            vx_arr = np.array(self.stats.cmd_vx)
            max_vx = np.max(np.abs(vx_arr))
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
            max_omega = np.max(np.abs(omega_arr))
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
            safety_config = self.config.get('safety', {})
            margins_info = {
                'velocity_margin': safety_config.get('velocity_margin', 1.1),
                'accel_margin': safety_config.get('accel_margin', 1.5)
            }
            self.results.append(AnalysisResult(
                category="safety_status",
                severity="warning" if fail_rate > 5 else "info",
                parameter="safety.velocity_margin",
                current_value=margins_info,
                suggested_value=None,
                reason=f"[诊断信息] 安全检查失败{self.stats.safety_check_failed_count}次({fail_rate:.1f}%)。安全裕度是安全参数，不建议自动调整。",
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
                'backup_rate': float(round(self.stats.backup_active_count / self.stats.total_samples * 100, 1)) if self.stats.total_samples > 0 else 0,
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
        
        # 状态机摘要
        state_dist = {int(k): int(v) for k, v in self.stats.state_counts.items()}
        
        summary['state_machine'] = {
            'state_distribution': state_dist,
            'normal_rate': float(round(self.stats.normal_count / self.stats.total_samples * 100, 2)) if self.stats.total_samples > 0 else 0,
            'backup_active_rate': float(round(self.stats.backup_active_count / self.stats.total_samples * 100, 2)) if self.stats.total_samples > 0 else 0,
            'soft_disabled_rate': float(round(self.stats.soft_disabled_count / self.stats.total_samples * 100, 2)) if self.stats.total_samples > 0 else 0,
            'mpc_degraded_rate': float(round(self.stats.mpc_degraded_count / self.stats.total_samples * 100, 2)) if self.stats.total_samples > 0 else 0,
            'stopping_rate': float(round(self.stats.stopping_count / self.stats.total_samples * 100, 2)) if self.stats.total_samples > 0 else 0,
            'stopped_rate': float(round(self.stats.stopped_count / self.stats.total_samples * 100, 2)) if self.stats.total_samples > 0 else 0
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
