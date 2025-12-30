#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
统一诊断工具 v2.7 (Unified Diagnostics Tool)

完整合并 diagnose_trajectory.py v3.0 和 full_diagnostics.py 的所有功能。
此脚本是控制器诊断的唯一入口，其他诊断脚本已废弃并重定向到此处。

功能模式:
  1. realtime  - 实时监控模式
                 订阅 DiagnosticsV2，显示完整控制器内部状态（10个诊断板块）
                 需要控制器运行
  
  2. tuning    - 系统调优模式
                 传感器频率/延迟/抖动分析，底盘测试，完整配置生成（15个配置模块）
                 不需要控制器运行
  
  3. full      - 完整模式
                 先运行调优分析，再进入实时监控
                 两阶段日志统一保存到同一文件

诊断内容:
  [实时监控] 轨迹输入、控制输出、MPC健康、一致性、状态估计、跟踪误差、超时、紧急停止、坐标变换、问题汇总
  [系统调优] 传感器频率/延迟/抖动、底盘特性、轨迹质量(含曲率和速度统计)、运行时调优建议、完整配置生成

配置模块 (15个):
  system, watchdog, diagnostics, mpc, constraints, safety, ekf, consistency, transform,
  transition, backup, tf, tracking, trajectory, cmd_vel_adapter

DiagnosticsV2 字段覆盖:
  - 基本状态: state, mpc_success, backup_active, solve_time_ms
  - MPC健康: kkt_residual, condition_number, consecutive_near_timeout, degradation_warning, can_recover
  - 一致性: alpha, curvature_consistency, velocity_dir_consistency, temporal_smooth, consistency_data_valid
  - 状态估计器: covariance_norm, innovation_norm, slip_probability, imu_drift_detected, imu_available, imu_bias
  - 跟踪误差: tracking_lateral_error, tracking_longitudinal_error, tracking_heading_error, tracking_prediction_error
  - 坐标变换: tf2_available, tf2_injected, fallback_duration_ms, accumulated_drift
  - 超时: timeout_odom, timeout_traj, timeout_traj_grace_exceeded, timeout_imu, last_*_age_ms, in_startup_grace
  - 控制命令: cmd_vx, cmd_vy, cmd_vz, cmd_omega, cmd_frame_id
  - 其他: transition_progress, emergency_stop, consecutive_errors, error_message

使用方法:
  # 实时监控 (需要控制器运行)
  rosrun controller_ros unified_diagnostics.py --mode realtime
  
  # 系统调优 + 完整配置生成
  rosrun controller_ros unified_diagnostics.py --mode tuning --output config.yaml
  
  # 底盘测试 (机器人会移动!)
  rosrun controller_ros unified_diagnostics.py --mode tuning --test-chassis
  
  # 运行时调优 (需要控制器运行)
  rosrun controller_ros unified_diagnostics.py --mode tuning --runtime-tuning
  
  # 指定低速阈值 (影响角速度计算)
  rosrun controller_ros unified_diagnostics.py --mode tuning --low-speed-thresh 0.05
  
  # 完整诊断
  rosrun controller_ros unified_diagnostics.py --mode full --duration 10

作者: Kiro Auto-generated
版本: 2.7 (统一配置模块数量为15个、修复参数引用链、消除硬编码)
"""
import sys
import os
import time
import threading
import argparse
import yaml
import re
import numpy as np
from collections import deque
from dataclasses import dataclass, field
from typing import Optional, List, Dict, Any, Tuple
from datetime import datetime

# 修复Windows终端编码问题
if sys.platform == 'win32':
    # 设置标准输出为UTF-8
    if hasattr(sys.stdout, 'reconfigure'):
        sys.stdout.reconfigure(encoding='utf-8')
    if hasattr(sys.stderr, 'reconfigure'):
        sys.stderr.reconfigure(encoding='utf-8')
    # 设置环境变量
    os.environ['PYTHONIOENCODING'] = 'utf-8'

# ROS 导入 - 仅支持 ROS1 (Noetic)
# 注意: 虽然保留了 ROS2 的导入结构，但实际功能仅在 ROS1 下完整实现
try:
    import rospy
    from std_msgs.msg import Header
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Twist, PoseStamped
    from nav_msgs.msg import Path
    from sensor_msgs.msg import Imu
    import tf2_ros
    ROS_VERSION = 1
except ImportError:
    # ROS2 导入 - 仅用于基本兼容性检测，功能未完整实现
    try:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import Header
        from nav_msgs.msg import Odometry
        from geometry_msgs.msg import Twist, PoseStamped
        from nav_msgs.msg import Path
        from sensor_msgs.msg import Imu
        ROS_VERSION = 2
        print("警告: 检测到 ROS2 环境，但本工具仅完整支持 ROS1 (Noetic)")
        print("      部分功能可能无法正常工作")
    except ImportError:
        print("错误: 未找到ROS环境")
        sys.exit(1)

# 增强诊断模块
try:
    from enhanced_diagnostics import EnhancedDiagnostics
    ENHANCED_DIAGNOSTICS_AVAILABLE = True
except ImportError:
    ENHANCED_DIAGNOSTICS_AVAILABLE = False
    print("警告: enhanced_diagnostics 模块不可用，部分高级诊断功能将被禁用")


# 自定义消息
try:
    from controller_ros.msg import LocalTrajectoryV4, UnifiedCmd, DiagnosticsV2
    CUSTOM_MSG_AVAILABLE = True
except ImportError:
    CUSTOM_MSG_AVAILABLE = False


# ============================================================================
# 颜色输出
# ============================================================================

class Colors:
    RED = '\033[0;31m'
    GREEN = '\033[0;32m'
    YELLOW = '\033[1;33m'
    BLUE = '\033[0;34m'
    CYAN = '\033[0;36m'
    MAGENTA = '\033[0;35m'
    NC = '\033[0m'


# ============================================================================
# 控制器状态枚举 (与 universal_controller.core.enums.ControllerState 保持一致)
# ============================================================================

class ControllerState:
    """控制器状态枚举 - 与 universal_controller 保持一致"""
    INIT = 0
    NORMAL = 1
    SOFT_DISABLED = 2
    MPC_DEGRADED = 3
    BACKUP_ACTIVE = 4
    STOPPING = 5
    STOPPED = 6
    
    # 状态名称映射
    NAMES = {
        0: 'INIT',
        1: 'NORMAL',
        2: 'SOFT_DISABLED',
        3: 'MPC_DEGRADED',
        4: 'BACKUP_ACTIVE',
        5: 'STOPPING',
        6: 'STOPPED',
    }
    
    # 状态描述
    DESCRIPTIONS = {
        0: '初始化中',
        1: '正常运行',
        2: 'Soft约束已禁用',
        3: 'MPC降级模式',
        4: '备用控制器激活',
        5: '停止中',
        6: '已停止',
    }
    
    # 状态颜色
    COLORS = {
        0: Colors.YELLOW,
        1: Colors.GREEN,
        2: Colors.YELLOW,
        3: Colors.YELLOW,
        4: Colors.RED,
        5: Colors.YELLOW,
        6: Colors.CYAN,
    }
    
    @classmethod
    def get_name(cls, state: int) -> str:
        """获取状态名称"""
        return cls.NAMES.get(state, f'UNKNOWN({state})')
    
    @classmethod
    def get_description(cls, state: int) -> str:
        """获取状态描述"""
        return cls.DESCRIPTIONS.get(state, f'未知状态({state})')
    
    @classmethod
    def get_colored_name(cls, state: int) -> str:
        """获取带颜色的状态名称"""
        color = cls.COLORS.get(state, Colors.NC)
        name = cls.get_name(state)
        return f"{color}{name}{Colors.NC}"
    
    @classmethod
    def is_degraded(cls, state: int) -> bool:
        """判断是否处于降级状态"""
        return state in [cls.SOFT_DISABLED, cls.MPC_DEGRADED, cls.BACKUP_ACTIVE]
    
    @classmethod
    def is_stopped(cls, state: int) -> bool:
        """判断是否处于停止状态"""
        return state in [cls.STOPPING, cls.STOPPED]


class TrajectoryMode:
    """轨迹模式枚举 - 与 universal_controller 保持一致"""
    MODE_TRACK = 0
    MODE_STOP = 1
    MODE_HOVER = 2
    MODE_EMERGENCY = 3
    
    NAMES = {
        0: 'TRACK',
        1: 'STOP',
        2: 'HOVER',
        3: 'EMERGENCY',
    }
    
    @classmethod
    def get_name(cls, mode: int) -> str:
        return cls.NAMES.get(mode, f'UNKNOWN({mode})')


# ============================================================================
# 诊断阈值配置 (与 universal_controller 配置保持一致)
# ============================================================================

class DiagnosticsThresholds:
    """
    诊断阈值配置 - 统一管理所有诊断判断阈值
    
    设计原则:
    1. 所有阈值与 universal_controller 配置保持一致
    2. 使用类属性而非硬编码，便于维护和追溯
    3. 警告阈值 = 错误阈值 * WARN_RATIO，保持一致的比例关系
    
    配置来源对照:
    - MPC 健康: mpc_config.py -> MPC_CONFIG['health_monitor']
    - 跟踪误差: system_config.py -> TRACKING_CONFIG
    - 状态机: safety_config.py -> SAFETY_CONFIG['state_machine']
    - 状态估计: 基于工程经验的合理默认值
    """
    
    # 警告阈值与错误阈值的比例 (警告 = 错误 * WARN_RATIO)
    WARN_RATIO = 0.67
    
    # ===== MPC 健康监控阈值 (来自 mpc_config.py) =====
    # 注意: 诊断工具应优先使用 DiagnosticsV2.degradation_warning 字段
    # 以下阈值仅用于额外的极端情况警告
    MPC_SOLVE_TIME_CRITICAL_MS = 15.0    # 求解时间临界阈值 (ms)
    MPC_SOLVE_TIME_EXTREME_MS = 20.0     # 求解时间极端阈值 (ms)，超过此值额外警告
    MPC_KKT_RESIDUAL_THRESH = 1e-3       # KKT 残差阈值
    MPC_CONDITION_NUMBER_THRESH = 1e8    # 条件数阈值
    MPC_CONSECUTIVE_TIMEOUT_WARN = 3     # 连续接近超时警告阈值
    
    # ===== 跟踪误差阈值 (来自 system_config.py -> TRACKING_CONFIG) =====
    TRACKING_LATERAL_THRESH = 0.3        # 横向误差阈值 (m)
    TRACKING_LONGITUDINAL_THRESH = 0.5   # 纵向误差阈值 (m)
    TRACKING_HEADING_THRESH = 0.5        # 航向误差阈值 (rad, ~28.6°)
    TRACKING_PREDICTION_THRESH = 0.5     # 预测误差阈值 (m)
    
    # 计算警告阈值
    TRACKING_LATERAL_WARN = TRACKING_LATERAL_THRESH * WARN_RATIO      # ~0.2m
    TRACKING_LONGITUDINAL_WARN = TRACKING_LONGITUDINAL_THRESH * WARN_RATIO  # ~0.33m
    TRACKING_HEADING_WARN_RAD = TRACKING_HEADING_THRESH * WARN_RATIO  # ~0.33rad (~19°)
    
    # ===== 状态机阈值 (来自 safety_config.py) =====
    DEGRADED_STATE_TIMEOUT = 30.0        # 降级状态超时 (秒)
    BACKUP_STATE_TIMEOUT = 60.0          # 备用控制器状态超时 (秒)
    DEGRADED_STATE_WARN = 10.0           # 降级状态警告阈值 (秒)
    
    # ===== 一致性检查阈值 =====
    ALPHA_CRITICAL = 0.3                 # Alpha 临界值 (低于此值 soft velocity 几乎不生效)
    ALPHA_WARN = 0.5                     # Alpha 警告值
    ALPHA_VERY_LOW = 0.2                 # Alpha 极低值 (用于调优建议)
    CONSISTENCY_LOW_THRESH = 0.5         # 一致性指标低阈值
    TEMPORAL_SMOOTH_LOW = 0.3            # 时序平滑度低阈值
    
    # ===== 状态估计器阈值 =====
    COVARIANCE_NORM_CRITICAL = 1.0       # 协方差范数临界值
    INNOVATION_NORM_WARN = 0.5           # 新息范数警告值
    SLIP_PROBABILITY_CRITICAL = 0.5      # 打滑概率临界值
    SLIP_PROBABILITY_WARN = 0.3          # 打滑概率警告值
    
    # ===== 超时阈值 (用于诊断显示) =====
    ODOM_AGE_WARN_MS = 100.0             # 里程计数据年龄警告 (ms)
    TRAJ_AGE_WARN_MS = 200.0             # 轨迹数据年龄警告 (ms)
    
    # ===== 坐标变换阈值 =====
    TF2_FALLBACK_WARN_MS = 100.0         # TF2 降级警告阈值 (ms)
    TF2_FALLBACK_CRITICAL_MS = 500.0     # TF2 降级临界阈值 (ms)
    ACCUMULATED_DRIFT_WARN = 0.1         # 累积漂移警告阈值 (m)
    
    # ===== 运行时调优阈值 =====
    # 这些阈值用于生成调优建议，基于跟踪误差阈值计算
    TUNING_LATERAL_ERROR_HIGH = TRACKING_LATERAL_THRESH * 0.5    # 0.15m - 触发权重调整建议
    TUNING_LATERAL_ERROR_MED = TRACKING_LATERAL_THRESH * 0.33    # 0.10m - 触发轻微调整建议
    TUNING_HEADING_ERROR_HIGH = TRACKING_HEADING_THRESH * 0.6    # 0.3rad - 触发权重调整建议
    TUNING_HEADING_ERROR_MED = TRACKING_HEADING_THRESH * 0.4     # 0.2rad - 触发轻微调整建议
    
    # MPC 成功率阈值
    MPC_SUCCESS_RATE_CRITICAL = 0.9      # MPC 成功率临界值
    MPC_SUCCESS_RATE_WARN = 0.98         # MPC 成功率警告值
    
    # 备用控制器使用率阈值
    BACKUP_ACTIVE_RATIO_WARN = 0.1       # 备用控制器使用率警告值
    
    # 一致性拒绝率阈值
    CONSISTENCY_REJECTION_HIGH = 0.1     # 一致性拒绝率高阈值
    CONSISTENCY_REJECTION_MED = 0.05     # 一致性拒绝率中阈值
    
    # 控制平滑性阈值 (用于 MPC 权重调优)
    MAX_ACCEL_SMOOTH = 3.0               # 加速度平滑阈值 (m/s²)
    MAX_ACCEL_JITTER = 8.0               # 加速度抖动阈值 (m/s²)
    MAX_ANGULAR_ACCEL_JITTER = 15.0      # 角加速度抖动阈值 (rad/s²)


# ============================================================================
# 工具函数
# ============================================================================

def normalize_angle(angle: float) -> float:
    """归一化角度到 [-pi, pi]"""
    while angle > np.pi:
        angle -= 2 * np.pi
    while angle < -np.pi:
        angle += 2 * np.pi
    return angle


def quaternion_to_yaw(q) -> float:
    """四元数转yaw角"""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return np.arctan2(siny_cosp, cosy_cosp)


def get_ros_time(msg_stamp) -> float:
    """获取ROS时间戳（秒）"""
    if ROS_VERSION == 1:
        return msg_stamp.to_sec() if hasattr(msg_stamp, 'to_sec') else 0.0
    else:
        return msg_stamp.sec + msg_stamp.nanosec * 1e-9


def format_duration(seconds: float) -> str:
    """格式化时间持续"""
    if seconds < 1:
        return f"{seconds*1000:.0f}ms"
    elif seconds < 60:
        return f"{seconds:.1f}s"
    else:
        return f"{seconds/60:.1f}min"


def safe_print(text: str):
    """
    安全打印函数，处理编码错误
    
    在Windows系统上，如果终端不支持UTF-8，会尝试使用系统默认编码
    """
    try:
        print(text)
    except UnicodeEncodeError:
        # 如果UTF-8失败，尝试使用系统默认编码
        try:
            # 移除ANSI颜色代码
            text_no_color = re.sub(r'\033\[[0-9;]+m', '', text)
            print(text_no_color.encode(sys.stdout.encoding, errors='replace').decode(sys.stdout.encoding))
        except:
            # 最后的fallback：只打印ASCII字符
            print(text.encode('ascii', errors='replace').decode('ascii'))


# ============================================================================
# 数据结构
# ============================================================================

@dataclass
class TrajectoryAnalysis:
    """轨迹分析结果"""
    timestamp: float = 0.0
    frame_id: str = ""
    num_points: int = 0
    dt_sec: float = 0.0
    soft_enabled: bool = False
    mode: int = 0
    confidence: float = 0.0
    total_distance: float = 0.0
    total_turn_deg: float = 0.0
    avg_speed: float = 0.0
    min_speed: float = 0.0
    max_speed: float = 0.0
    max_curvature: float = 0.0
    soft_wz_available: bool = False
    soft_wz_sum: float = 0.0
    soft_wz_max: float = 0.0
    hard_wz_sum: float = 0.0
    hard_wz_max: float = 0.0
    hard_wz_zero_count: int = 0
    issues: List[str] = field(default_factory=list)


@dataclass 
class ControlAnalysis:
    """控制输出分析"""
    vx: float = 0.0
    vy: float = 0.0
    omega: float = 0.0


@dataclass
class TransformAnalysis:
    """坐标变换分析"""
    tf2_available: bool = False
    source_frame: str = ""
    target_frame: str = ""
    position: Tuple[float, float, float] = (0, 0, 0)
    yaw: float = 0.0


@dataclass
class StateAnalysis:
    """状态估计分析"""
    position: Tuple[float, float, float] = (0, 0, 0)
    velocity: Tuple[float, float, float] = (0, 0, 0)
    yaw: float = 0.0
    omega: float = 0.0


# ============================================================================
# 轨迹分析函数
# ============================================================================

def compute_hard_velocities(points: List[Tuple[float, float, float]], 
                            dt_sec: float, 
                            low_speed_thresh: float = 0.1) -> Tuple[np.ndarray, List[str]]:
    """
    从轨迹点计算 hard velocities，模拟 universal_controller 中的实现
    
    此函数与 universal_controller.core.data_types.Trajectory.get_hard_velocities() 保持一致。
    low_speed_thresh 参数应该从配置中读取，默认值 0.1 与 trajectory_config.py 中的定义一致。
    
    Args:
        points: 轨迹点列表 [(x, y, z), ...]
        dt_sec: 时间步长 (秒)
        low_speed_thresh: 低速阈值 (m/s)，低于此速度时 wz 被置为 0
                         此值应与 trajectory.low_speed_thresh 配置一致
    
    Returns:
        (velocities, issues): 速度数组 [N, 4] 和问题列表
    """
    issues = []
    if len(points) < 2:
        return np.zeros((1, 4)), ["轨迹点数不足(<2)"]
    
    # 确保 dt_sec 有效
    if dt_sec <= 0:
        dt_sec = 0.1  # 使用默认值
        issues.append("dt_sec无效，使用默认值0.1s")
    
    velocities = []
    n = len(points)
    
    for i in range(n - 1):
        p0, p1 = points[i], points[i + 1]
        vx = (p1[0] - p0[0]) / dt_sec
        vy = (p1[1] - p0[1]) / dt_sec
        vz = (p1[2] - p0[2]) / dt_sec
        
        if i < n - 2:
            p2 = points[i + 2]
            vx_next = (p2[0] - p1[0]) / dt_sec
            vy_next = (p2[1] - p1[1]) / dt_sec
        elif i > 0:
            p_prev = points[i - 1]
            vx_prev = (p0[0] - p_prev[0]) / dt_sec
            vy_prev = (p0[1] - p_prev[1]) / dt_sec
            vx_next = 2 * vx - vx_prev
            vy_next = 2 * vy - vy_prev
        else:
            vx_next, vy_next = vx, vy
        
        speed = np.sqrt(vx**2 + vy**2)
        speed_next = np.sqrt(vx_next**2 + vy_next**2)
        
        if speed > low_speed_thresh and speed_next > low_speed_thresh:
            heading_curr = np.arctan2(vy, vx)
            heading_next = np.arctan2(vy_next, vx_next)
            dheading = normalize_angle(heading_next - heading_curr)
            wz = dheading / dt_sec
        else:
            wz = 0.0
            if speed <= low_speed_thresh:
                issues.append(f"点{i}: speed={speed:.4f}m/s < thresh={low_speed_thresh}, wz强制=0")
        
        velocities.append([vx, vy, vz, wz])
    
    velocities.append(velocities[-1])
    return np.array(velocities), issues


def compute_menger_curvature(points: List[Tuple[float, float]]) -> List[float]:
    """计算 Menger 曲率"""
    curvatures = []
    if len(points) < 3:
        return curvatures
    
    for i in range(1, len(points) - 1):
        p0, p1, p2 = points[i-1], points[i], points[i+1]
        # Menger curvature: κ = 4A / (|p0-p1| * |p1-p2| * |p0-p2|)
        area = abs((p1[0]-p0[0])*(p2[1]-p0[1]) - (p2[0]-p0[0])*(p1[1]-p0[1])) / 2
        d01 = np.sqrt((p1[0]-p0[0])**2 + (p1[1]-p0[1])**2)
        d12 = np.sqrt((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2)
        d02 = np.sqrt((p2[0]-p0[0])**2 + (p2[1]-p0[1])**2)
        if d01 * d12 * d02 > 1e-6:
            kappa = 4 * area / (d01 * d12 * d02)
            curvatures.append(kappa)
    return curvatures


def analyze_trajectory_msg(msg, low_speed_thresh: float = 0.1) -> TrajectoryAnalysis:
    """
    完整分析轨迹消息
    
    Args:
        msg: LocalTrajectoryV4 消息
        low_speed_thresh: 低速阈值，用于角速度计算
                         此值应与 trajectory.low_speed_thresh 配置一致
    
    Returns:
        TrajectoryAnalysis: 分析结果
    """
    result = TrajectoryAnalysis()
    result.timestamp = get_ros_time(msg.header.stamp)
    result.frame_id = msg.header.frame_id or "unknown"
    result.num_points = len(msg.points)
    result.dt_sec = msg.dt_sec
    result.soft_enabled = msg.soft_enabled
    result.mode = msg.mode
    result.confidence = msg.confidence
    
    if result.num_points < 2:
        result.issues.append("❌ 轨迹点数不足")
        return result
    
    points = [(p.x, p.y, p.z) for p in msg.points]
    points_2d = [(p.x, p.y) for p in msg.points]
    
    # 几何分析
    distances = []
    headings = []
    for i in range(len(points) - 1):
        dx = points[i+1][0] - points[i][0]
        dy = points[i+1][1] - points[i][1]
        dist = np.sqrt(dx**2 + dy**2)
        distances.append(dist)
        if dist > 1e-6:
            headings.append(np.arctan2(dy, dx))
        else:
            headings.append(headings[-1] if headings else 0)
    
    result.total_distance = sum(distances)
    
    heading_changes = []
    for i in range(len(headings) - 1):
        dh = normalize_angle(headings[i+1] - headings[i])
        heading_changes.append(dh)
    result.total_turn_deg = np.degrees(sum(heading_changes)) if heading_changes else 0
    
    # 速度分析 - 确保 dt_sec 有效
    if result.dt_sec <= 0:
        result.dt_sec = 0.1  # 使用默认值
        result.issues.append("⚠️ dt_sec无效，使用默认值0.1s")
    
    speeds = [d / result.dt_sec for d in distances]
    if speeds:
        result.avg_speed = np.mean(speeds)
        result.min_speed = min(speeds)
        result.max_speed = max(speeds)

    # 曲率分析
    curvatures = compute_menger_curvature(points_2d)
    if curvatures:
        result.max_curvature = max(curvatures)

    # Soft velocities
    if result.soft_enabled and len(msg.velocities_flat) >= 4:
        if len(msg.velocities_flat) % 4 == 0:
            soft_vels = np.array(msg.velocities_flat).reshape(-1, 4)
            result.soft_wz_available = True
            result.soft_wz_sum = np.sum(np.abs(soft_vels[:, 3]))
            result.soft_wz_max = np.max(np.abs(soft_vels[:, 3]))
            if result.soft_wz_sum < 0.001 and abs(result.total_turn_deg) > 5:
                result.issues.append(f"⚠️ soft_wz全为0，但轨迹转向{result.total_turn_deg:.1f}°")
        else:
            result.issues.append(f"❌ velocities_flat长度({len(msg.velocities_flat)})不是4的倍数")
    elif result.soft_enabled:
        result.issues.append("⚠️ soft_enabled=True但无velocity数据")
    
    # Hard velocities - 使用传入的 low_speed_thresh 参数
    hard_vels, hard_issues = compute_hard_velocities(points, result.dt_sec, low_speed_thresh)
    result.hard_wz_sum = np.sum(np.abs(hard_vels[:, 3]))
    result.hard_wz_max = np.max(np.abs(hard_vels[:, 3]))
    result.hard_wz_zero_count = np.sum(np.abs(hard_vels[:, 3]) < 1e-6)
    result.issues.extend(hard_issues)
    
    # 核心问题检测 - 使用传入的 low_speed_thresh 参数
    if abs(result.total_turn_deg) > 10 and result.hard_wz_sum < 0.1:
        result.issues.append(f"🔴 关键问题: 轨迹转向{result.total_turn_deg:.1f}°但hard_wz≈0 (可能是低速阈值问题)")
    
    if result.hard_wz_zero_count > result.num_points * 0.8:
        result.issues.append(f"🔴 {result.hard_wz_zero_count}/{result.num_points}个点的wz=0 (速度可能低于{low_speed_thresh}m/s阈值)")
    
    low_speed_count = sum(1 for s in speeds if s < low_speed_thresh)
    if low_speed_count > len(speeds) * 0.5:
        result.issues.append(f"⚠️ {low_speed_count}/{len(speeds)}个点速度<{low_speed_thresh}m/s，会导致wz被置0")
    
    return result


# ============================================================================
# 话题监控器
# ============================================================================

class TopicMonitor:
    """监控单个话题的频率、延迟和抖动"""
    
    def __init__(self, topic: str, msg_type, window_size: int = 200):
        self.topic = topic
        self.msg_type = msg_type
        self.timestamps = deque(maxlen=window_size)
        self.latencies = deque(maxlen=window_size)
        self.msg_count = 0
        self.last_msg = None
        self.lock = threading.Lock()
        self.sub = None
        
    def start(self) -> bool:
        """
        启动话题订阅
        
        Returns:
            bool: 是否成功启动
        """
        try:
            self.sub = rospy.Subscriber(self.topic, self.msg_type, self._callback, queue_size=10)
            return True
        except Exception as e:
            # 记录错误但不中断程序
            safe_print(f"警告: 订阅 {self.topic} 失败: {e}")
            return False
    
    def stop(self):
        if self.sub:
            self.sub.unregister()
            self.sub = None
    
    def _callback(self, msg):
        now = time.time()
        with self.lock:
            self.timestamps.append(now)
            self.last_msg = msg
            self.msg_count += 1
            if hasattr(msg, 'header') and msg.header.stamp.to_sec() > 0:
                msg_time = msg.header.stamp.to_sec()
                latency = now - msg_time
                if 0 < latency < 10:
                    self.latencies.append(latency * 1000)
    
    def get_stats(self) -> Dict[str, Any]:
        with self.lock:
            if len(self.timestamps) < 2:
                return {'rate': 0, 'latency_ms': 0, 'jitter_ms': 0, 'count': self.msg_count}
            dts = np.diff(list(self.timestamps))
            rate = 1.0 / np.mean(dts) if len(dts) > 0 and np.mean(dts) > 0 else 0
            jitter = np.std(dts) * 1000 if len(dts) > 1 else 0
            latency = np.mean(list(self.latencies)) if self.latencies else 0
            return {'rate': rate, 'latency_ms': latency, 'jitter_ms': jitter, 'count': self.msg_count}


class OdometryAnalyzer(TopicMonitor):
    """分析里程计获取底盘特性"""
    
    def __init__(self, topic: str):
        super().__init__(topic, Odometry)
        self.velocities = deque(maxlen=500)
        self.accelerations = deque(maxlen=500)
        self.last_vel = None
        self.last_time = None
        
    def _callback(self, msg):
        super()._callback(msg)
        now = time.time()
        with self.lock:
            vx = msg.twist.twist.linear.x
            vy = msg.twist.twist.linear.y
            wz = msg.twist.twist.angular.z
            v = np.sqrt(vx**2 + vy**2)
            self.velocities.append((now, vx, vy, wz, v))
            
            if self.last_vel is not None and self.last_time is not None:
                dt = now - self.last_time
                if dt > 0.001:
                    ax = (vx - self.last_vel[0]) / dt
                    ay = (vy - self.last_vel[1]) / dt
                    alpha = (wz - self.last_vel[2]) / dt
                    self.accelerations.append((now, ax, ay, alpha))
            
            self.last_vel = (vx, vy, wz)
            self.last_time = now
    
    def get_chassis_stats(self) -> Optional[Dict[str, float]]:
        with self.lock:
            if len(self.velocities) < 10:
                return None
            vels = np.array([(v[1], v[2], v[3], v[4]) for v in self.velocities])
            stats = {
                'max_vx': float(np.max(np.abs(vels[:, 0]))),
                'max_vy': float(np.max(np.abs(vels[:, 1]))),
                'max_wz': float(np.max(np.abs(vels[:, 2]))),
                'max_speed': float(np.max(vels[:, 3])),
                'avg_speed': float(np.mean(vels[:, 3])),
            }
            if len(self.accelerations) > 10:
                accels = np.array([(a[1], a[2], a[3]) for a in self.accelerations])
                stats['max_ax'] = float(np.max(np.abs(accels[:, 0])))
                stats['max_ay'] = float(np.max(np.abs(accels[:, 1])))
                stats['max_alpha'] = float(np.max(np.abs(accels[:, 2])))
                stats['avg_ax'] = float(np.mean(np.abs(accels[:, 0])))
            return stats


class TrajectoryMonitor(TopicMonitor):
    """分析轨迹特性（含曲率分析和速度统计）"""
    
    def __init__(self, topic: str):
        if CUSTOM_MSG_AVAILABLE:
            super().__init__(topic, LocalTrajectoryV4)
        else:
            # Fallback: 使用 AnyMsg 进行基本监控
            # 注意: 仅在 ROS1 下可用
            if ROS_VERSION == 1:
                from rospy.msg import AnyMsg
                super().__init__(topic, AnyMsg)
            else:
                # ROS2 下没有 AnyMsg，使用 Odometry 作为占位符（不会真正解析）
                super().__init__(topic, Odometry)
                safe_print(f"警告: 轨迹监控在 ROS2 下功能受限")
        self.traj_info = {
            'num_points': 0, 'dt_sec': 0.1, 'has_velocities': False,
            'confidence': 0.9, 'frame_id': '', 'total_length': 0, 'max_curvature': 0,
            'min_speed': 0.0, 'avg_speed': 0.0, 'max_speed': 0.0,  # 速度统计
        }
        self.point_counts = deque(maxlen=100)
        self.dt_values = deque(maxlen=100)
        self.speed_samples = deque(maxlen=500)  # 存储速度样本用于统计
        
    def _callback(self, msg):
        super()._callback(msg)
        try:
            if hasattr(msg, 'points'):
                self.traj_info['num_points'] = len(msg.points)
                self.point_counts.append(len(msg.points))
            if hasattr(msg, 'dt_sec'):
                self.traj_info['dt_sec'] = msg.dt_sec
                self.dt_values.append(msg.dt_sec)
            if hasattr(msg, 'velocities_flat'):
                self.traj_info['has_velocities'] = len(msg.velocities_flat) > 0
            if hasattr(msg, 'confidence'):
                self.traj_info['confidence'] = msg.confidence
            if hasattr(msg, 'header'):
                self.traj_info['frame_id'] = msg.header.frame_id
            
            # 计算轨迹长度、曲率和速度
            if hasattr(msg, 'points') and len(msg.points) >= 2:
                points = [(p.x, p.y) for p in msg.points]
                dt_sec = msg.dt_sec if hasattr(msg, 'dt_sec') and msg.dt_sec > 0 else 0.1
                
                # 计算相邻点距离和速度
                distances = []
                for i in range(len(points) - 1):
                    dx = points[i+1][0] - points[i][0]
                    dy = points[i+1][1] - points[i][1]
                    dist = np.sqrt(dx**2 + dy**2)
                    distances.append(dist)
                
                self.traj_info['total_length'] = sum(distances)
                
                # 计算速度统计 (用于 _calculate_low_speed_thresh)
                if distances:
                    speeds = [d / dt_sec for d in distances]
                    self.traj_info['min_speed'] = min(speeds)
                    self.traj_info['avg_speed'] = np.mean(speeds)
                    self.traj_info['max_speed'] = max(speeds)
                    # 存储样本用于长期统计
                    self.speed_samples.extend(speeds)
                
                # Menger 曲率
                curvatures = compute_menger_curvature(points)
                if curvatures:
                    self.traj_info['max_curvature'] = max(curvatures)
        except:
            pass
    
    def get_trajectory_stats(self) -> Dict[str, Any]:
        stats = self.traj_info.copy()
        with self.lock:
            if self.point_counts:
                stats['avg_points'] = np.mean(list(self.point_counts))
                stats['point_variance'] = np.std(list(self.point_counts))
            if self.dt_values:
                stats['avg_dt'] = np.mean(list(self.dt_values))
                stats['dt_variance'] = np.std(list(self.dt_values))
            # 长期速度统计 (基于所有收集的样本)
            if self.speed_samples:
                speed_list = list(self.speed_samples)
                stats['min_speed'] = min(speed_list)
                stats['avg_speed'] = np.mean(speed_list)
                stats['max_speed'] = max(speed_list)
                stats['speed_std'] = np.std(speed_list)
        return stats


class ControllerDiagnosticsMonitor:
    """
    监控控制器诊断信息用于运行时调优
    
    架构设计：
    - 只存储原始消息，避免数据冗余
    - 统计数据在 get_stats() 时计算，确保数据一致性
    - backup_active_count 作为累计计数器单独维护（因为需要跨越 deque 边界）
    """
    
    def __init__(self, topic: str = '/controller/diagnostics', max_samples: int = 1000):
        """
        初始化诊断监控器
        
        Args:
            topic: 诊断话题
            max_samples: 最大样本数，应根据 duration * 诊断频率 设置
                        默认 1000 支持 100 秒 @ 10Hz 或 50 秒 @ 20Hz
        """
        self.topic = topic
        self.sub = None
        self.lock = threading.Lock()
        self.msg_count = 0
        self.backup_active_count = 0  # 累计计数器，不受 deque 大小限制
        # 只存储原始消息，统计在 get_stats() 时计算
        self.diagnostics = deque(maxlen=max_samples)
        
    def start(self) -> bool:
        """
        启动诊断话题订阅
        
        Returns:
            bool: 是否成功启动
        """
        if not CUSTOM_MSG_AVAILABLE:
            safe_print("警告: controller_ros 自定义消息不可用")
            return False
        try:
            self.sub = rospy.Subscriber(self.topic, DiagnosticsV2, self._callback, queue_size=10)
            return True
        except Exception as e:
            safe_print(f"警告: 订阅 {self.topic} 失败: {e}")
            return False
    
    def stop(self):
        if self.sub:
            self.sub.unregister()
            self.sub = None
    
    def _callback(self, msg):
        with self.lock:
            self.msg_count += 1
            self.diagnostics.append(msg)
            # backup_active_count 作为累计计数器
            if msg.backup_active:
                self.backup_active_count += 1
    
    def get_stats(self) -> Optional[Dict[str, Any]]:
        """
        计算并返回统计数据
        
        统计数据基于当前 deque 中的消息计算，确保数据一致性
        """
        with self.lock:
            if self.msg_count < 10:
                return None
            
            # 从原始消息中提取数据并计算统计
            msgs = list(self.diagnostics)
            if not msgs:
                return None
            
            mpc_times = [m.mpc_solve_time_ms for m in msgs]
            mpc_successes = [m.mpc_success for m in msgs]
            kkt_residuals = [m.mpc_health_kkt_residual for m in msgs]
            lateral_errors = [abs(m.tracking_lateral_error) for m in msgs]
            longitudinal_errors = [abs(m.tracking_longitudinal_error) for m in msgs]
            heading_errors = [abs(m.tracking_heading_error) for m in msgs]
            alpha_values = [m.consistency_alpha_soft for m in msgs]
            
            return {
                'msg_count': self.msg_count,
                'mpc_solve_time_avg_ms': np.mean(mpc_times),
                'mpc_solve_time_max_ms': np.max(mpc_times),
                'mpc_solve_time_std_ms': np.std(mpc_times),
                'mpc_success_rate': np.mean(mpc_successes),
                'mpc_kkt_residual_avg': np.mean(kkt_residuals),
                'lateral_error_avg': np.mean(lateral_errors),
                'lateral_error_max': np.max(lateral_errors),
                'longitudinal_error_avg': np.mean(longitudinal_errors),
                'heading_error_avg': np.mean(heading_errors),
                'alpha_avg': np.mean(alpha_values),
                'alpha_min': np.min(alpha_values),
                'backup_active_ratio': self.backup_active_count / self.msg_count if self.msg_count > 0 else 0,
            }


# ============================================================================
# 底盘测试器
# ============================================================================

class ChassisTestRunner:
    """
    运行底盘能力测试
    
    Args:
        cmd_topic: 速度命令话题
        odom_analyzer: 里程计分析器
        log_func: 日志函数，用于统一输出 (可选，默认使用 safe_print)
    """
    
    def __init__(self, cmd_topic: str, odom_analyzer: OdometryAnalyzer, log_func=None):
        self.cmd_topic = cmd_topic
        self.odom = odom_analyzer
        self.cmd_pub = None
        self.results = {}
        # 使用传入的日志函数，或默认使用 safe_print
        self._log = log_func if log_func else safe_print
        
    def setup(self):
        self.cmd_pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=1)
        time.sleep(0.5)
        
    def test_max_velocity(self, target_v: float = 1.0, duration: float = 3.0) -> float:
        """测试最大速度"""
        self._log(f"  测试最大速度 (目标: {target_v} m/s)...")
        # 清空历史数据，只统计本次测试（线程安全）
        with self.odom.lock:
            self.odom.velocities.clear()
        time.sleep(0.5)  # 等待足够的新数据（至少10个样本@20Hz）
        
        cmd = Twist()
        cmd.linear.x = target_v
        start = time.time()
        max_v = 0
        while time.time() - start < duration:
            self.cmd_pub.publish(cmd)
            stats = self.odom.get_chassis_stats()
            if stats:
                max_v = max(max_v, stats['max_speed'])
            time.sleep(0.05)
        self.cmd_pub.publish(Twist())
        time.sleep(0.5)
        self.results['max_velocity_achieved'] = max_v
        return max_v
    
    def test_acceleration(self, target_v: float = 0.5) -> float:
        """测试加速能力"""
        self._log(f"  测试加速度 (目标: {target_v} m/s)...")
        # 停止机器人
        self.cmd_pub.publish(Twist())
        time.sleep(1.0)
        
        # 清空所有历史数据（线程安全）
        with self.odom.lock:
            self.odom.velocities.clear()
            self.odom.accelerations.clear()
            self.odom.last_vel = None
            self.odom.last_time = None
        
        # 等待足够的新数据开始收集（与其他测试方法保持一致）
        time.sleep(0.5)
        
        # 开始加速测试
        cmd = Twist()
        cmd.linear.x = target_v
        start = time.time()
        max_ax = 0
        
        while time.time() - start < 2.0:
            self.cmd_pub.publish(cmd)
            stats = self.odom.get_chassis_stats()
            if stats and 'max_ax' in stats:
                max_ax = max(max_ax, stats['max_ax'])
            time.sleep(0.02)
        
        # 停止机器人
        self.cmd_pub.publish(Twist())
        time.sleep(0.5)
        
        # 最终检查
        stats = self.odom.get_chassis_stats()
        if stats and 'max_ax' in stats:
            max_ax = max(max_ax, stats['max_ax'])
        
        self.results['max_acceleration'] = max_ax
        return max_ax
    
    def test_angular_velocity(self, target_w: float = 1.0, duration: float = 2.0) -> float:
        """测试最大角速度"""
        self._log(f"  测试角速度 (目标: {target_w} rad/s)...")
        # 清空历史数据，只统计本次测试（线程安全）
        with self.odom.lock:
            self.odom.velocities.clear()
        time.sleep(0.5)  # 等待足够的新数据
        
        cmd = Twist()
        cmd.angular.z = target_w
        start = time.time()
        max_w = 0
        while time.time() - start < duration:
            self.cmd_pub.publish(cmd)
            stats = self.odom.get_chassis_stats()
            if stats:
                max_w = max(max_w, stats['max_wz'])
            time.sleep(0.05)
        self.cmd_pub.publish(Twist())
        time.sleep(0.5)
        self.results['max_angular_velocity'] = max_w
        return max_w
    
    def test_response_time(self, step_v: float = 0.3) -> Optional[float]:
        """测试响应时间"""
        self._log(f"  测试响应时间...")
        self.cmd_pub.publish(Twist())
        time.sleep(1.0)
        
        # 清空历史数据（线程安全）
        with self.odom.lock:
            self.odom.velocities.clear()
        time.sleep(0.5)  # 等待足够的新数据
        
        start_time = time.time()
        threshold = step_v * 0.63  # 63% 时间常数
        cmd = Twist()
        cmd.linear.x = step_v
        response_time = None
        while time.time() - start_time < 3.0:
            self.cmd_pub.publish(cmd)
            stats = self.odom.get_chassis_stats()
            if stats and stats['max_speed'] >= threshold and response_time is None:
                response_time = time.time() - start_time
            time.sleep(0.01)
        self.cmd_pub.publish(Twist())
        time.sleep(0.5)
        self.results['response_time'] = response_time or 1.0
        return response_time


# ============================================================================
# 统一诊断类
# ============================================================================

class UnifiedDiagnostics:
    """
    统一诊断类 - 合并实时监控和系统调优的所有功能
    
    架构设计:
    - 单一类处理所有诊断模式，避免代码重复
    - 内部模块化：话题监控、实时诊断、配置生成
    - 通过 mode 参数控制执行流程
    
    配置生成覆盖 (15个配置模块，与 universal_controller/config/default_config.py 对应):
    核心模块:
    - system: 系统基础配置 (ctrl_freq, platform, gravity 等)
    - watchdog: 超时配置
    - mpc: MPC 控制器配置
    - constraints: 运动约束配置
    - safety: 安全监控配置
    - ekf: 状态估计配置 (含 adaptive, anomaly_detection)
    
    功能模块:
    - consistency: 一致性检查配置
    - transform: 坐标变换配置
    - transition: 平滑过渡配置
    - backup: 备份控制器配置
    - trajectory: 轨迹处理配置
    - tracking: 跟踪质量评估配置
    
    ROS 适配模块:
    - tf: TF2 配置 (ROS 专用)
    - cmd_vel_adapter: 速度适配器配置 (ROS 专用)
    - diagnostics: 诊断发布配置
    
    不调优的模块:
    - attitude: 四旋翼姿态控制 (平台专用，差速车不需要)
    - mock: 模拟数据配置 (调试用，不应自动调优)
    """
    
    # 默认 low_speed_thresh，与 universal_controller/config/trajectory_config.py 保持一致
    DEFAULT_LOW_SPEED_THRESH = 0.1
    
    def __init__(self, args):
        self.args = args
        self.mode = args.mode
        
        # 话题配置
        self.topics = {
            'odom': args.odom_topic,
            'imu': args.imu_topic,
            'trajectory': args.traj_topic,
            'cmd_vel': args.cmd_vel_topic,
            'cmd_unified': args.cmd_topic,
            'diagnostics': args.diag_topic,
        }
        
        # 监控器
        self.monitors = {}
        self.diag_monitor = None
        self.enhanced_analyzer = None  # 增强诊断分析器
        
        # 结果存储
        self.results = {}
        self.recommended = {}
        
        # 配置参数 - 从命令行或默认值
        self.low_speed_thresh = getattr(args, 'low_speed_thresh', self.DEFAULT_LOW_SPEED_THRESH)
        
        # 实时诊断状态
        self.last_traj: Optional[TrajectoryAnalysis] = None
        self.last_odom: Optional[StateAnalysis] = None
        self.last_diag: Optional[Dict] = None
        self.last_cmd: Optional[ControlAnalysis] = None
        self.cmd_history = deque(maxlen=100)
        self.omega_history = deque(maxlen=100)
        self.traj_count = 0
        self.cmd_count = 0
        self.diag_count = 0
        
        # 状态持续时间跟踪
        self._last_state: Optional[int] = None
        self._state_start_time: Optional[float] = None
        
        # TF2
        self.tf_buffer = None
        self.tf_listener = None
        
        # 锁
        self.lock = threading.Lock()
        
        # 实时报告
        self.last_report_time = 0
        self.report_interval = 3.0
        
        # 日志
        self.log_file = args.log_file
        self.log_handle = None
    
    # ==================== 初始化 ====================
    
    def _init_ros_node(self, node_name: str):
        """初始化ROS节点"""
        rospy.init_node(node_name, anonymous=True)
    
    def _init_tf2(self) -> bool:
        """初始化TF2"""
        if ROS_VERSION == 1:
            try:
                self.tf_buffer = tf2_ros.Buffer()
                self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
                return True
            except Exception as e:
                print(f"TF2初始化失败: {e}")
                return False
        return False
    
    def _init_log(self):
        """初始化日志文件"""
        if self.log_file:
            self.log_handle = open(self.log_file, 'w', encoding='utf-8')
            self.log_handle.write(f"# 统一诊断日志 - {time.strftime('%Y-%m-%d %H:%M:%S')}\n\n")
    
    def _close_log(self):
        """
        关闭日志文件
        
        幂等操作，可以安全地多次调用。
        """
        if self.log_handle:
            try:
                self.log_handle.close()
            except Exception as e:
                print(f"警告: 关闭日志文件失败: {e}")
            finally:
                self.log_handle = None
    
    def _log(self, text: str):
        """输出到控制台和日志文件（处理编码问题）"""
        safe_print(text)
        if self.log_handle:
            try:
                # 移除ANSI颜色代码后再写入文件
                text_no_color = re.sub(r'\033\[[0-9;]+m', '', text)
                self.log_handle.write(text_no_color + '\n')
                self.log_handle.flush()
            except UnicodeEncodeError:
                # 如果写入失败，尝试只写入ASCII
                text_no_color = re.sub(r'\033\[[0-9;]+m', '', text)
                self.log_handle.write(text_no_color.encode('ascii', errors='replace').decode('ascii') + '\n')
                self.log_handle.flush()
    
    # ==================== TF查询 ====================
    
    def get_transform(self, target_frame: str, source_frame: str) -> Optional[TransformAnalysis]:
        """获取坐标变换"""
        if self.tf_buffer is None:
            return None
        try:
            if ROS_VERSION == 1:
                trans = self.tf_buffer.lookup_transform(
                    target_frame, source_frame, rospy.Time(0), rospy.Duration(0.1))
                result = TransformAnalysis()
                result.tf2_available = True
                result.source_frame = source_frame
                result.target_frame = target_frame
                result.position = (trans.transform.translation.x,
                                   trans.transform.translation.y,
                                   trans.transform.translation.z)
                result.yaw = quaternion_to_yaw(trans.transform.rotation)
                return result
        except:
            pass
        return None
    
    # ==================== 实时监控回调 ====================
    
    def _traj_callback(self, msg):
        """轨迹回调"""
        with self.lock:
            self.traj_count += 1
            # 使用配置的 low_speed_thresh 进行分析
            self.last_traj = analyze_trajectory_msg(msg, self.low_speed_thresh)
            self._maybe_print_realtime_report()
    
    def _odom_callback(self, msg):
        """里程计回调"""
        with self.lock:
            state = StateAnalysis()
            state.position = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
            state.velocity = (msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z)
            state.yaw = quaternion_to_yaw(msg.pose.pose.orientation)
            state.omega = msg.twist.twist.angular.z
            self.last_odom = state
    
    def _diag_callback(self, msg):
        """诊断回调 - 收集所有DiagnosticsV2字段"""
        with self.lock:
            self.diag_count += 1
            
            # 跟踪状态持续时间
            current_state = msg.state
            if self._last_state != current_state:
                self._last_state = current_state
                self._state_start_time = time.time()
            
            self.last_diag = {
                # 基本状态
                'state': msg.state,
                'mpc_success': msg.mpc_success,
                'backup_active': msg.backup_active,
                'solve_time_ms': msg.mpc_solve_time_ms,
                # MPC 健康状态
                'kkt_residual': msg.mpc_health_kkt_residual,
                'condition_number': msg.mpc_health_condition_number,
                'consecutive_near_timeout': msg.mpc_health_consecutive_near_timeout,
                'degradation_warning': msg.mpc_health_degradation_warning,
                'can_recover': msg.mpc_health_can_recover,
                # 一致性指标
                'alpha': msg.consistency_alpha_soft,
                'curvature_consistency': msg.consistency_curvature,
                'velocity_dir_consistency': msg.consistency_velocity_dir,
                'temporal_smooth': msg.consistency_temporal,
                'consistency_data_valid': msg.consistency_data_valid,
                # 状态估计器健康
                'covariance_norm': msg.estimator_covariance_norm,
                'innovation_norm': msg.estimator_innovation_norm,
                'slip_probability': msg.estimator_slip_probability,
                'imu_drift_detected': msg.estimator_imu_drift_detected,
                'imu_available': msg.estimator_imu_available,
                'imu_bias': list(msg.estimator_imu_bias) if hasattr(msg, 'estimator_imu_bias') else [0, 0, 0],
                # 跟踪误差
                'tracking_lateral_error': msg.tracking_lateral_error,
                'tracking_longitudinal_error': msg.tracking_longitudinal_error,
                'tracking_heading_error': msg.tracking_heading_error,
                'tracking_prediction_error': msg.tracking_prediction_error,
                # 坐标变换状态
                'tf2_available': msg.transform_tf2_available,
                'tf2_injected': msg.transform_tf2_injected,
                'fallback_duration_ms': msg.transform_fallback_duration_ms,
                'accumulated_drift': msg.transform_accumulated_drift,
                # 超时状态
                'timeout_odom': msg.timeout_odom,
                'timeout_traj': msg.timeout_traj,
                'timeout_traj_grace_exceeded': msg.timeout_traj_grace_exceeded,
                'timeout_imu': msg.timeout_imu,
                'last_odom_age_ms': msg.timeout_last_odom_age_ms,
                'last_traj_age_ms': msg.timeout_last_traj_age_ms,
                'last_imu_age_ms': msg.timeout_last_imu_age_ms,
                'in_startup_grace': msg.timeout_in_startup_grace,
                # 控制命令
                'cmd_vx': msg.cmd_vx,
                'cmd_vy': msg.cmd_vy,
                'cmd_vz': msg.cmd_vz,
                'cmd_omega': msg.cmd_omega,
                'cmd_frame_id': msg.cmd_frame_id,
                # 过渡进度
                'transition_progress': msg.transition_progress,
                # 紧急停止
                'emergency_stop': msg.emergency_stop if hasattr(msg, 'emergency_stop') else False,
                'consecutive_errors': msg.consecutive_errors if hasattr(msg, 'consecutive_errors') else 0,
                'error_message': msg.error_message if hasattr(msg, 'error_message') else '',
            }
    
    def _cmd_callback(self, msg):
        """UnifiedCmd回调"""
        with self.lock:
            self.cmd_count += 1
            cmd = ControlAnalysis()
            cmd.vx = msg.vx
            cmd.vy = msg.vy if hasattr(msg, 'vy') else 0
            cmd.omega = msg.omega
            self.last_cmd = cmd
            self.cmd_history.append((msg.vx, msg.omega))
            self.omega_history.append(msg.omega)
    
    def _twist_callback(self, msg):
        """Twist回调"""
        with self.lock:
            self.cmd_count += 1
            cmd = ControlAnalysis()
            cmd.vx = msg.linear.x
            cmd.vy = msg.linear.y
            cmd.omega = msg.angular.z
            self.last_cmd = cmd
            self.cmd_history.append((msg.linear.x, msg.angular.z))
            self.omega_history.append(msg.angular.z)
    
    def _maybe_print_realtime_report(self):
        """检查是否需要打印实时报告"""
        now = time.time()
        if now - self.last_report_time >= self.report_interval:
            self.last_report_time = now
            self._print_realtime_report()

    # ==================== 实时诊断报告 ====================
    
    def _print_realtime_report(self):
        """打印完整实时诊断报告（10个板块）"""
        lines = [
            "\n" + "="*80,
            "                    统一控制器诊断报告 v2.7",
            "="*80,
            f"时间: {time.strftime('%H:%M:%S')}  |  轨迹#{self.traj_count}  |  诊断#{self.diag_count}  |  命令#{self.cmd_count}",
            "-"*80
        ]
        for line in lines:
            self._log(line)
        
        self._print_trajectory_section()
        self._print_control_section()
        self._print_mpc_section()
        self._print_consistency_section()
        self._print_estimator_section()
        self._print_tracking_section()
        self._print_timeout_section()
        self._print_emergency_section()
        self._print_transform_section()
        self._print_issues_section()
        self._log("="*80 + "\n")
    
    def _print_trajectory_section(self):
        """【1. 轨迹输入分析】"""
        self._log("\n【1. 轨迹输入分析】")
        if self.last_traj is None:
            self._log("  ❌ 未收到轨迹数据")
            return
        t = self.last_traj
        mode_name = TrajectoryMode.get_name(t.mode)
        self._log(f"  坐标系: {t.frame_id}  |  点数: {t.num_points}  |  dt: {t.dt_sec}s")
        self._log(f"  模式: {mode_name}  |  置信度: {t.confidence:.2f}  |  soft_enabled: {t.soft_enabled}")
        self._log(f"  几何: 总距离={t.total_distance:.3f}m  总转向={t.total_turn_deg:.1f}°  最大曲率={t.max_curvature:.2f}")
        self._log(f"  速度: min={t.min_speed:.3f} avg={t.avg_speed:.3f} max={t.max_speed:.3f} m/s")
        self._log(f"  角速度wz: Hard sum={t.hard_wz_sum:.4f} max={t.hard_wz_max:.4f} 零值={t.hard_wz_zero_count}/{t.num_points}")
        if t.soft_wz_available:
            self._log(f"           Soft sum={t.soft_wz_sum:.4f} max={t.soft_wz_max:.4f}")

    def _print_control_section(self):
        """【2. 控制输出分析】"""
        self._log("\n【2. 控制输出分析】")
        if self.last_cmd is None:
            self._log("  ❌ 未收到控制命令")
            return
        c = self.last_cmd
        self._log(f"  当前: vx={c.vx:.3f}m/s  vy={c.vy:.3f}m/s  omega={c.omega:.4f}rad/s")
        
        # 显示来自 DiagnosticsV2 的控制命令（如果可用）
        if self.last_diag:
            d = self.last_diag
            self._log(f"  诊断: vx={d['cmd_vx']:.3f}  vy={d['cmd_vy']:.3f}  vz={d['cmd_vz']:.3f}  omega={d['cmd_omega']:.4f}")
            if d['cmd_frame_id']:
                self._log(f"  命令坐标系: {d['cmd_frame_id']}")
        
        if len(self.omega_history) > 0:
            omegas = list(self.omega_history)
            max_omega = max(abs(o) for o in omegas)
            nonzero = sum(1 for o in omegas if abs(o) > 0.01)
            self._log(f"  历史({len(omegas)}条): avg={np.mean(omegas):.4f} max={max_omega:.4f} 非零={nonzero}")
            if self.last_traj and abs(self.last_traj.total_turn_deg) > 10 and max_omega < 0.05:
                self._log(f"  🔴 问题: 轨迹需转{self.last_traj.total_turn_deg:.1f}°但omega输出很小!")
    
    def _print_mpc_section(self):
        """【3. MPC 健康状态】"""
        self._log("\n【3. MPC 健康状态】")
        if self.last_diag is None:
            self._log("  ❌ 未收到诊断数据 (检查/controller/diagnostics话题)")
            return
        d = self.last_diag
        state = d['state']
        state_name = ControllerState.get_name(state)
        state_desc = ControllerState.get_description(state)
        state_color = ControllerState.COLORS.get(state, Colors.NC)
        
        # 状态显示（含持续时间）
        state_duration_str = ""
        if self._state_start_time is not None:
            duration = time.time() - self._state_start_time
            state_duration_str = f"  持续: {format_duration(duration)}"
        self._log(f"  控制器状态: {state_color}{state_name}{Colors.NC} ({state_desc}){state_duration_str}")
        
        # 降级状态警告（含持续时间检查）
        if ControllerState.is_degraded(state):
            self._log(f"  ⚠️ 系统处于降级状态!")
            # 检查降级状态持续时间 - 使用 DiagnosticsThresholds 统一管理
            if self._state_start_time is not None:
                duration = time.time() - self._state_start_time
                if duration > DiagnosticsThresholds.DEGRADED_STATE_TIMEOUT:
                    self._log(f"  🔴 降级状态持续过长 ({format_duration(duration)} > {DiagnosticsThresholds.DEGRADED_STATE_TIMEOUT:.0f}s)!")
                elif duration > DiagnosticsThresholds.DEGRADED_STATE_WARN:
                    self._log(f"  ⚠️ 降级状态已持续 {format_duration(duration)}")
        if ControllerState.is_stopped(state):
            self._log(f"  ⚠️ 系统已停止或正在停止")
        
        self._log(f"  MPC成功: {d['mpc_success']}  |  备用激活: {d['backup_active']}  |  求解时间: {d['solve_time_ms']:.2f}ms")
        self._log(f"  KKT残差: {d['kkt_residual']:.6f}  |  条件数: {d['condition_number']:.2e}")
        self._log(f"  连续接近超时: {d['consecutive_near_timeout']}次  |  降级警告: {d['degradation_warning']}  |  可恢复: {d['can_recover']}")
        
        # 问题检测 - 优先使用 degradation_warning 字段
        # MPC 健康监控器已经根据配置的阈值计算了 degradation_warning
        if d['degradation_warning']:
            self._log("  ⚠️ MPC降级警告 (求解时间/KKT残差/条件数超过配置阈值)")
        # 仅在极端情况下额外警告（使用 DiagnosticsThresholds 统一管理）
        if d['solve_time_ms'] > DiagnosticsThresholds.MPC_SOLVE_TIME_EXTREME_MS:
            self._log(f"  🔴 求解时间过长 ({d['solve_time_ms']:.1f}ms > {DiagnosticsThresholds.MPC_SOLVE_TIME_EXTREME_MS}ms)")
        if d['kkt_residual'] > DiagnosticsThresholds.MPC_KKT_RESIDUAL_THRESH:
            self._log(f"  ⚠️ KKT残差较高 ({d['kkt_residual']:.6f} > {DiagnosticsThresholds.MPC_KKT_RESIDUAL_THRESH})")
        if d['condition_number'] > DiagnosticsThresholds.MPC_CONDITION_NUMBER_THRESH:
            self._log(f"  🔴 条件数过高 ({d['condition_number']:.2e} > {DiagnosticsThresholds.MPC_CONDITION_NUMBER_THRESH:.0e})，数值不稳定!")
        if d['consecutive_near_timeout'] > DiagnosticsThresholds.MPC_CONSECUTIVE_TIMEOUT_WARN:
            self._log(f"  ⚠️ 连续接近超时 {d['consecutive_near_timeout']} 次")
        if not d['mpc_success']:
            self._log("  🔴 MPC求解失败，使用备用控制器")
        if d['backup_active']:
            self._log("  ⚠️ 备用控制器(Pure Pursuit)激活中")
    
    def _print_consistency_section(self):
        """【4. 一致性指标】"""
        self._log("\n【4. 一致性指标】")
        if self.last_diag is None:
            self._log("  ❌ 无诊断数据")
            return
        d = self.last_diag
        self._log(f"  Alpha (soft权重): {d['alpha']:.3f}")
        self._log(f"  曲率一致性: {d['curvature_consistency']:.3f}  |  速度方向一致性: {d['velocity_dir_consistency']:.3f}")
        self._log(f"  时序平滑度: {d['temporal_smooth']:.3f}  |  数据有效: {d['consistency_data_valid']}")
        # 使用 DiagnosticsThresholds 统一管理阈值
        if d['alpha'] < DiagnosticsThresholds.ALPHA_CRITICAL:
            self._log(f"  🔴 Alpha过低({d['alpha']:.2f})，soft velocity几乎不生效!")
        elif d['alpha'] < DiagnosticsThresholds.ALPHA_WARN:
            self._log(f"  ⚠️ Alpha较低({d['alpha']:.2f})，soft velocity权重小")
        if d['curvature_consistency'] < DiagnosticsThresholds.CONSISTENCY_LOW_THRESH:
            self._log(f"  ⚠️ 曲率一致性低 ({d['curvature_consistency']:.2f})")
        if d['velocity_dir_consistency'] < DiagnosticsThresholds.CONSISTENCY_LOW_THRESH:
            self._log(f"  ⚠️ 速度方向一致性低 ({d['velocity_dir_consistency']:.2f})")
        if d['temporal_smooth'] < DiagnosticsThresholds.TEMPORAL_SMOOTH_LOW:
            self._log(f"  ⚠️ 时序平滑度低 ({d['temporal_smooth']:.2f})，轨迹抖动")
        if not d['consistency_data_valid']:
            self._log("  🔴 一致性数据无效 (可能包含NaN/Inf)")
    
    def _print_estimator_section(self):
        """【5. 状态估计器健康】"""
        self._log("\n【5. 状态估计器健康】")
        if self.last_diag is None:
            self._log("  ❌ 无诊断数据")
            return
        d = self.last_diag
        self._log(f"  协方差范数: {d['covariance_norm']:.4f}  |  新息范数: {d['innovation_norm']:.4f}")
        self._log(f"  打滑概率: {d['slip_probability']:.2%}  |  IMU可用: {d['imu_available']}  |  IMU漂移: {d['imu_drift_detected']}")
        if d['imu_bias'] and any(abs(b) > 0.001 for b in d['imu_bias']):
            self._log(f"  IMU偏置: [{d['imu_bias'][0]:.4f}, {d['imu_bias'][1]:.4f}, {d['imu_bias'][2]:.4f}]")
        # 使用 DiagnosticsThresholds 统一管理阈值
        if d['covariance_norm'] > DiagnosticsThresholds.COVARIANCE_NORM_CRITICAL:
            self._log(f"  🔴 协方差范数过高 ({d['covariance_norm']:.2f})，估计不确定性大!")
        if d['innovation_norm'] > DiagnosticsThresholds.INNOVATION_NORM_WARN:
            self._log(f"  ⚠️ 新息范数较高 ({d['innovation_norm']:.2f})，测量与预测偏差大")
        if d['slip_probability'] > DiagnosticsThresholds.SLIP_PROBABILITY_WARN:
            self._log(f"  🔴 打滑概率高 ({d['slip_probability']:.0%})，可能打滑!")
        if d['imu_drift_detected']:
            self._log("  ⚠️ 检测到IMU漂移")
        if not d['imu_available']:
            self._log("  ⚠️ IMU不可用，使用里程计航向")
    
    def _print_tracking_section(self):
        """【6. 跟踪误差】"""
        self._log("\n【6. 跟踪误差】")
        if self.last_diag is None:
            self._log("  ❌ 无诊断数据")
            return
        d = self.last_diag
        self._log(f"  横向误差: {d['tracking_lateral_error']:.3f}m  |  纵向误差: {d['tracking_longitudinal_error']:.3f}m")
        self._log(f"  航向误差: {np.degrees(d['tracking_heading_error']):.1f}°  |  预测误差: {d['tracking_prediction_error']:.3f}m")
        
        # 使用 DiagnosticsThresholds 统一管理阈值
        # 警告阈值 = 错误阈值 * WARN_RATIO，保持一致的比例关系
        if abs(d['tracking_lateral_error']) > DiagnosticsThresholds.TRACKING_LATERAL_THRESH:
            self._log(f"  🔴 横向误差过大 ({d['tracking_lateral_error']:.2f}m > {DiagnosticsThresholds.TRACKING_LATERAL_THRESH}m)")
        elif abs(d['tracking_lateral_error']) > DiagnosticsThresholds.TRACKING_LATERAL_WARN:
            self._log(f"  ⚠️ 横向误差较大 ({d['tracking_lateral_error']:.2f}m > {DiagnosticsThresholds.TRACKING_LATERAL_WARN:.2f}m)")
        if abs(d['tracking_longitudinal_error']) > DiagnosticsThresholds.TRACKING_LONGITUDINAL_THRESH:
            self._log(f"  🔴 纵向误差过大 ({d['tracking_longitudinal_error']:.2f}m > {DiagnosticsThresholds.TRACKING_LONGITUDINAL_THRESH}m)")
        elif abs(d['tracking_longitudinal_error']) > DiagnosticsThresholds.TRACKING_LONGITUDINAL_WARN:
            self._log(f"  ⚠️ 纵向误差较大 ({d['tracking_longitudinal_error']:.2f}m > {DiagnosticsThresholds.TRACKING_LONGITUDINAL_WARN:.2f}m)")
        # 航向误差使用弧度比较，显示时转换为度
        heading_warn_deg = np.degrees(DiagnosticsThresholds.TRACKING_HEADING_WARN_RAD)
        heading_error_deg = np.degrees(DiagnosticsThresholds.TRACKING_HEADING_THRESH)
        if abs(d['tracking_heading_error']) > DiagnosticsThresholds.TRACKING_HEADING_THRESH:
            self._log(f"  🔴 航向误差过大 ({np.degrees(d['tracking_heading_error']):.1f}° > {heading_error_deg:.1f}°)")
        elif abs(d['tracking_heading_error']) > DiagnosticsThresholds.TRACKING_HEADING_WARN_RAD:
            self._log(f"  ⚠️ 航向误差较大 ({np.degrees(d['tracking_heading_error']):.1f}° > {heading_warn_deg:.1f}°)")
        if d['tracking_prediction_error'] > DiagnosticsThresholds.TRACKING_PREDICTION_THRESH:
            self._log(f"  ⚠️ 预测误差较大 ({d['tracking_prediction_error']:.2f}m > {DiagnosticsThresholds.TRACKING_PREDICTION_THRESH}m)")
    
    def _print_timeout_section(self):
        """【7. 超时状态】"""
        self._log("\n【7. 超时状态】")
        if self.last_diag is None:
            self._log("  ❌ 无诊断数据")
            return
        d = self.last_diag
        self._log(f"  里程计: 超时={d['timeout_odom']}  年龄={d['last_odom_age_ms']:.1f}ms")
        self._log(f"  轨迹: 超时={d['timeout_traj']}  宽限期超={d['timeout_traj_grace_exceeded']}  年龄={d['last_traj_age_ms']:.1f}ms")
        self._log(f"  IMU: 超时={d['timeout_imu']}  年龄={d['last_imu_age_ms']:.1f}ms  |  启动宽限期: {d['in_startup_grace']}")
        # 使用 DiagnosticsThresholds 统一管理阈值
        if d['timeout_odom']:
            self._log("  🔴 里程计超时!")
        elif d['last_odom_age_ms'] > DiagnosticsThresholds.ODOM_AGE_WARN_MS:
            self._log(f"  ⚠️ 里程计数据较旧 ({d['last_odom_age_ms']:.0f}ms > {DiagnosticsThresholds.ODOM_AGE_WARN_MS:.0f}ms)")
        if d['timeout_traj']:
            self._log("  🔴 轨迹超时!")
        elif d['last_traj_age_ms'] > DiagnosticsThresholds.TRAJ_AGE_WARN_MS:
            self._log(f"  ⚠️ 轨迹数据较旧 ({d['last_traj_age_ms']:.0f}ms > {DiagnosticsThresholds.TRAJ_AGE_WARN_MS:.0f}ms)")
        if d['timeout_traj_grace_exceeded']:
            self._log("  🔴 轨迹超时宽限期已过!")
        if d['timeout_imu'] and d['imu_available']:
            self._log("  ⚠️ IMU超时")
    
    def _print_emergency_section(self):
        """【8. 紧急停止与错误】"""
        self._log("\n【8. 紧急停止与错误】")
        if self.last_diag is None:
            self._log("  ❌ 无诊断数据")
            return
        d = self.last_diag
        self._log(f"  紧急停止: {d['emergency_stop']}  |  连续错误: {d['consecutive_errors']}次  |  过渡进度: {d['transition_progress']:.0%}")
        if d['emergency_stop']:
            self._log("  🔴 紧急停止已触发!")
        if d['consecutive_errors'] > 0:
            self._log(f"  ⚠️ 连续错误 {d['consecutive_errors']} 次")
            if d['error_message']:
                self._log(f"  错误信息: {d['error_message']}")
        if 0 < d['transition_progress'] < 1.0:
            self._log(f"  ⚠️ 正在过渡中 ({d['transition_progress']:.0%})")
    
    def _print_transform_section(self):
        """【9. 坐标变换状态】"""
        self._log("\n【9. 坐标变换状态】")
        if self.last_diag is None:
            self._log("  ❌ 无诊断数据")
            tf_result = self.get_transform('odom', 'base_link')
            if tf_result and tf_result.tf2_available:
                self._log(f"  直接查询: base_link → odom 成功")
                self._log(f"  位置: ({tf_result.position[0]:.3f}, {tf_result.position[1]:.3f})  航向: {np.degrees(tf_result.yaw):.1f}°")
            else:
                self._log("  直接查询TF失败")
            return
        d = self.last_diag
        self._log(f"  TF2可用: {d['tf2_available']}  |  已注入: {d['tf2_injected']}")
        self._log(f"  降级持续时间: {d['fallback_duration_ms']:.1f}ms  |  累积漂移: {d['accumulated_drift']:.4f}m")
        # 使用 DiagnosticsThresholds 统一管理阈值
        if not d['tf2_available']:
            self._log("  🔴 TF2不可用，使用fallback模式!")
        if not d['tf2_injected']:
            self._log("  ⚠️ TF2未注入到控制器")
        if d['fallback_duration_ms'] > DiagnosticsThresholds.TF2_FALLBACK_CRITICAL_MS:
            self._log(f"  🔴 TF2降级时间过长 ({d['fallback_duration_ms']:.0f}ms > {DiagnosticsThresholds.TF2_FALLBACK_CRITICAL_MS:.0f}ms)")
        elif d['fallback_duration_ms'] > DiagnosticsThresholds.TF2_FALLBACK_WARN_MS:
            self._log(f"  ⚠️ TF2降级中 ({d['fallback_duration_ms']:.0f}ms > {DiagnosticsThresholds.TF2_FALLBACK_WARN_MS:.0f}ms)")
        if d['accumulated_drift'] > DiagnosticsThresholds.ACCUMULATED_DRIFT_WARN:
            self._log(f"  ⚠️ 累积漂移较大 ({d['accumulated_drift']:.3f}m > {DiagnosticsThresholds.ACCUMULATED_DRIFT_WARN}m)")
        # 检查轨迹坐标系
        if self.last_traj:
            frame = self.last_traj.frame_id
            if frame in ['base_link', 'base_link_0']:
                self._log(f"  轨迹坐标系: {frame} (局部坐标系，需要TF变换)")
            elif frame in ['odom', 'map', 'world']:
                self._log(f"  轨迹坐标系: {frame} (全局坐标系，无需变换)")
            else:
                self._log(f"  ⚠️ 未知坐标系: {frame}")
    
    def _print_issues_section(self):
        """【10. 问题汇总与建议】"""
        self._log("\n【10. 问题汇总与建议】")
        issues, warnings = [], []
        
        # 轨迹问题
        if self.last_traj and self.last_traj.issues:
            for issue in self.last_traj.issues:
                (issues if '🔴' in issue else warnings).append(issue)
        
        # 控制问题
        if self.last_traj and self.last_cmd:
            if abs(self.last_traj.total_turn_deg) > 15 and len(self.omega_history) > 0:
                max_omega = max(abs(o) for o in self.omega_history)
                if max_omega < 0.1:
                    issues.append(f"🔴 核心问题: 轨迹转向{self.last_traj.total_turn_deg:.1f}°但输出omega最大仅{max_omega:.4f}rad/s")
        
        # 诊断问题 - 使用 DiagnosticsThresholds 统一管理阈值
        if self.last_diag:
            d = self.last_diag
            if d['emergency_stop']: issues.append("🔴 紧急停止已触发!")
            if not d['mpc_success'] and d['backup_active']: issues.append("🔴 MPC求解失败，备用控制器激活")
            if d['condition_number'] > DiagnosticsThresholds.MPC_CONDITION_NUMBER_THRESH: 
                issues.append(f"🔴 MPC条件数过高 ({d['condition_number']:.2e})")
            if d['timeout_odom']: issues.append("🔴 里程计超时!")
            if d['timeout_traj_grace_exceeded']: issues.append("🔴 轨迹超时宽限期已过!")
            if d['covariance_norm'] > DiagnosticsThresholds.COVARIANCE_NORM_CRITICAL: 
                issues.append(f"🔴 状态估计不确定性过高 (协方差范数={d['covariance_norm']:.2f})")
            if d['slip_probability'] > DiagnosticsThresholds.SLIP_PROBABILITY_CRITICAL: 
                issues.append(f"🔴 高打滑概率 ({d['slip_probability']:.0%})")
            # 横向误差使用更严格的阈值（1.5倍配置阈值）作为严重问题判断
            if abs(d['tracking_lateral_error']) > DiagnosticsThresholds.TRACKING_LATERAL_THRESH * 1.5: 
                issues.append(f"🔴 横向跟踪误差过大 ({d['tracking_lateral_error']:.2f}m)")
            if not d['tf2_available'] and d['fallback_duration_ms'] > DiagnosticsThresholds.TF2_FALLBACK_CRITICAL_MS * 2: 
                issues.append(f"🔴 TF2长时间不可用")
            if d['alpha'] < DiagnosticsThresholds.ALPHA_CRITICAL: 
                warnings.append(f"⚠️ Alpha过低({d['alpha']:.2f})，soft velocity几乎不生效")
            if d['solve_time_ms'] > DiagnosticsThresholds.MPC_SOLVE_TIME_CRITICAL_MS: 
                warnings.append(f"⚠️ MPC求解时间较长 ({d['solve_time_ms']:.1f}ms)")
            if d['consecutive_near_timeout'] > DiagnosticsThresholds.MPC_CONSECUTIVE_TIMEOUT_WARN: 
                warnings.append(f"⚠️ 连续接近超时 {d['consecutive_near_timeout']} 次")
            if d['imu_drift_detected']: warnings.append("⚠️ 检测到IMU漂移")
            if d['consecutive_errors'] > 0: warnings.append(f"⚠️ 连续错误 {d['consecutive_errors']} 次")
        
        if issues:
            self._log("  严重问题:")
            for i in issues: self._log(f"    {i}")
        if warnings:
            self._log("  警告:")
            for w in warnings: self._log(f"    {w}")
        if not issues and not warnings:
            self._log("  ✅ 未检测到明显问题")
        
        # 建议
        suggestions = []
        all_issues = issues + warnings
        if any('低速' in str(i) or 'wz' in str(i) or 'omega' in str(i) for i in all_issues):
            suggestions.append("检查轨迹速度是否过低 (< 0.1 m/s)")
            suggestions.append("尝试降低 trajectory.low_speed_thresh 配置 (如改为0.01)")
            suggestions.append("检查轨迹消息中 velocities_flat 是否包含有效的wz数据")
        if any('MPC' in str(i) or '条件数' in str(i) for i in all_issues):
            suggestions.append("检查 MPC 权重配置是否合理")
            suggestions.append("考虑降低 MPC horizon 或增加 dt")
        if any('超时' in str(i) for i in all_issues):
            suggestions.append("检查传感器数据发布频率")
            suggestions.append("调整 watchdog 超时配置")
        if any('TF2' in str(i) or '坐标' in str(i) for i in all_issues):
            suggestions.append("检查 TF2 树是否完整 (rosrun tf2_tools view_frames.py)")
            suggestions.append("确认 base_link → odom 变换可用")
        if any('打滑' in str(i) or '协方差' in str(i) for i in all_issues):
            suggestions.append("检查里程计数据质量")
            suggestions.append("考虑调整 EKF 噪声参数")
        if suggestions:
            self._log("\n  建议:")
            for i, s in enumerate(suggestions, 1):
                self._log(f"    {i}. {s}")

    # ==================== 系统调优功能 ====================
    
    def _wait_for_confirmation(self, stage_name: str, prerequisites: list, warnings: list = None):
        """
        等待用户确认阶段执行
        
        Args:
            stage_name: 阶段名称
            prerequisites: 前提条件列表
            warnings: 警告信息列表 (可选)
        
        Returns:
            bool: True 表示用户确认继续，False 表示用户跳过
        """
        self._log(f"\n  {Colors.CYAN}[前提条件]{Colors.NC}")
        for prereq in prerequisites:
            self._log(f"    • {prereq}")
        
        if warnings:
            self._log(f"\n  {Colors.RED}[警告]{Colors.NC}")
            for warn in warnings:
                self._log(f"    ⚠️  {warn}")
        
        self._log(f"\n  {Colors.YELLOW}按 Enter 确认并开始 {stage_name} (Ctrl+C 跳过此阶段)...{Colors.NC}")
        
        try:
            input()
            return True
        except KeyboardInterrupt:
            self._log(f"\n  {Colors.YELLOW}[跳过]{Colors.NC} 用户取消 {stage_name}")
            return False
    
    def _run_topic_monitoring(self):
        """阶段1: 话题监控"""
        self._log(f"\n{Colors.BLUE}{'─'*70}")
        self._log(f"  阶段1/6: 话题监控 ({self.args.duration}秒)")
        self._log(f"{'─'*70}{Colors.NC}")
        self._log(f"\n  {Colors.YELLOW}[提示]{Colors.NC} 此阶段仅被动监听话题，如需测试底盘能力请使用 --test-chassis")
        
        # 显示前提条件并等待确认
        prerequisites = [
            "turtlebot_bringup 已启动 (roslaunch turtlebot_bringup minimal.launch)",
            "trajectory_publisher 已启动 (神经网络轨迹发布节点)",
            f"监控时长: {self.args.duration} 秒"
        ]
        if not self._wait_for_confirmation("阶段1: 话题监控", prerequisites):
            return
        
        self.monitors['odom'] = OdometryAnalyzer(self.topics['odom'])
        self.monitors['imu'] = TopicMonitor(self.topics['imu'], Imu)
        self.monitors['trajectory'] = TrajectoryMonitor(self.topics['trajectory'])
        
        self._log(f"\n  订阅话题:")
        for name, mon in self.monitors.items():
            if mon.start():
                self._log(f"    {Colors.GREEN}[OK]{Colors.NC} {mon.topic}")
            else:
                self._log(f"    {Colors.RED}[FAIL]{Colors.NC} {mon.topic}")
        
        self._log(f"\n  {Colors.CYAN}[进度]{Colors.NC} 收集数据中...")
        
        # 显示进度
        start_time = time.time()
        while time.time() - start_time < self.args.duration:
            elapsed = time.time() - start_time
            remaining = self.args.duration - elapsed
            # 每10秒显示一次进度
            if int(elapsed) % 10 == 0 and int(elapsed) > 0:
                odom_stats = self.monitors['odom'].get_stats()
                traj_stats = self.monitors['trajectory'].get_stats()
                odom_count = odom_stats.get('count', 0) if odom_stats else 0
                traj_count = traj_stats.get('count', 0) if traj_stats else 0
                self._log(f"    [{int(elapsed)}/{int(self.args.duration)}s] odom: {odom_count} msgs, traj: {traj_count} msgs")
            time.sleep(1.0)
        
        # 收集结果
        for name, mon in self.monitors.items():
            self.results[name] = mon.get_stats()
            if isinstance(mon, OdometryAnalyzer):
                chassis = mon.get_chassis_stats()
                if chassis:
                    self.results['chassis'] = chassis
            elif isinstance(mon, TrajectoryMonitor):
                self.results['trajectory_info'] = mon.get_trajectory_stats()
        
        for mon in self.monitors.values():
            mon.stop()
        
        self._log(f"\n  {Colors.GREEN}[完成]{Colors.NC} 阶段1完成")
    
    def _run_chassis_tests(self):
        """阶段2: 底盘能力测试"""
        self._log(f"\n{Colors.BLUE}{'─'*70}")
        self._log(f"  阶段2/6: 底盘能力测试")
        self._log(f"{'─'*70}{Colors.NC}")
        
        self._log(f"\n  测试内容:")
        self._log(f"    1. 最大速度测试 (3秒)")
        self._log(f"    2. 加速度测试 (2秒)")
        self._log(f"    3. 最大角速度测试 (2秒)")
        self._log(f"    4. 响应时间测试 (3秒)")
        
        # 显示前提条件并等待确认
        prerequisites = [
            "周围空间安全，机器人可以自由移动",
            "里程计话题正常发布 (/odom)",
            "速度命令话题可用 (/mobile_base/commands/velocity)"
        ]
        warnings = [
            "机器人会移动! 确保周围空间安全。",
            "测试过程中请勿触碰机器人"
        ]
        if not self._wait_for_confirmation("阶段2: 底盘能力测试", prerequisites, warnings):
            return
        
        # 重新启动里程计监控器（阶段1已经停止）
        odom_monitor = OdometryAnalyzer(self.topics['odom'])
        
        try:
            if odom_monitor.start():
                self._log(f"\n  {Colors.GREEN}[OK]{Colors.NC} 重新订阅 {self.topics['odom']}")
            else:
                self._log(f"  {Colors.RED}[FAIL]{Colors.NC} 无法订阅里程计，跳过底盘测试")
                return
            
            # 等待里程计数据稳定
            self._log(f"  {Colors.CYAN}[进度]{Colors.NC} 等待里程计数据...")
            time.sleep(1.0)
            
            # 传入 self._log 作为日志函数，确保测试输出写入日志
            tester = ChassisTestRunner(self.topics['cmd_vel'], odom_monitor, log_func=self._log)
            tester.setup()
            
            self._log(f"\n  {Colors.CYAN}[进度]{Colors.NC} 开始子阶段测试...")
            
            # 子阶段 2.1: 最大速度测试
            self._log(f"\n  {Colors.MAGENTA}--- 子阶段 2.1/4: 最大速度测试 ---{Colors.NC}")
            sub_prereqs = ["机器人前方有足够空间 (至少2米)", "目标速度: 0.5 m/s，持续3秒"]
            if self._wait_for_confirmation("子阶段2.1: 最大速度测试", sub_prereqs, ["机器人将向前移动!"]):
                tester.test_max_velocity(target_v=0.5)
                self._log(f"    {Colors.GREEN}[完成]{Colors.NC} 最大速度测试完成")
            
            # 子阶段 2.2: 加速度测试
            self._log(f"\n  {Colors.MAGENTA}--- 子阶段 2.2/4: 加速度测试 ---{Colors.NC}")
            sub_prereqs = ["机器人前方有足够空间", "目标速度: 0.3 m/s，测试加速能力"]
            if self._wait_for_confirmation("子阶段2.2: 加速度测试", sub_prereqs, ["机器人将快速加速!"]):
                tester.test_acceleration(target_v=0.3)
                self._log(f"    {Colors.GREEN}[完成]{Colors.NC} 加速度测试完成")
            
            # 子阶段 2.3: 最大角速度测试
            self._log(f"\n  {Colors.MAGENTA}--- 子阶段 2.3/4: 最大角速度测试 ---{Colors.NC}")
            sub_prereqs = ["机器人周围有足够空间", "目标角速度: 1.0 rad/s，持续2秒"]
            if self._wait_for_confirmation("子阶段2.3: 最大角速度测试", sub_prereqs, ["机器人将原地旋转!"]):
                tester.test_angular_velocity(target_w=1.0)
                self._log(f"    {Colors.GREEN}[完成]{Colors.NC} 最大角速度测试完成")
            
            # 子阶段 2.4: 响应时间测试
            self._log(f"\n  {Colors.MAGENTA}--- 子阶段 2.4/4: 响应时间测试 ---{Colors.NC}")
            sub_prereqs = ["机器人前方有足够空间", "测试阶跃响应，目标速度: 0.3 m/s"]
            if self._wait_for_confirmation("子阶段2.4: 响应时间测试", sub_prereqs, ["机器人将突然启动!"]):
                tester.test_response_time(step_v=0.3)
                self._log(f"    {Colors.GREEN}[完成]{Colors.NC} 响应时间测试完成")
            
            self.results['chassis_tests'] = tester.results
            self._log(f"\n  {Colors.GREEN}[完成]{Colors.NC} 阶段2完成")
        finally:
            # 确保监控器被停止
            odom_monitor.stop()
    
    def _run_controller_diagnostics(self):
        """阶段3: 控制器运行时诊断"""
        self._log(f"\n{Colors.BLUE}{'─'*70}")
        self._log(f"  阶段3/6: 控制器运行时诊断 ({self.args.duration}秒)")
        self._log(f"{'─'*70}{Colors.NC}")
        
        # 显示前提条件并等待确认
        prerequisites = [
            "控制器必须正在运行 (roslaunch controller_ros controller.launch)",
            f"诊断话题可用 ({self.topics['diagnostics']})",
            f"监控时长: {self.args.duration} 秒",
            "建议: 移动机器人以生成跟踪数据"
        ]
        if not self._wait_for_confirmation("阶段3: 控制器运行时诊断", prerequisites):
            return
        
        if not CUSTOM_MSG_AVAILABLE:
            self._log(f"\n  {Colors.RED}[ERROR]{Colors.NC} controller_ros 消息不可用，跳过")
            return
        
        # 计算需要的样本数：duration * 预期诊断频率(10-20Hz) * 1.5 安全系数
        expected_diag_rate = 20  # 假设最高 20Hz
        max_samples = int(self.args.duration * expected_diag_rate * 1.5)
        max_samples = max(max_samples, 500)  # 至少 500 个样本
        
        self.diag_monitor = ControllerDiagnosticsMonitor(self.topics['diagnostics'], max_samples=max_samples)
        if self.diag_monitor.start():
            self._log(f"\n  {Colors.GREEN}[OK]{Colors.NC} 订阅 {self.topics['diagnostics']}")
        else:
            self._log(f"  {Colors.RED}[FAIL]{Colors.NC} 无法订阅诊断话题")
            self._log(f"         请确认控制器正在运行: roslaunch controller_ros controller.launch")
            return
        
        # 初始化增强诊断分析器，window_size 与 max_samples 保持一致
        if ENHANCED_DIAGNOSTICS_AVAILABLE:
            self.enhanced_analyzer = EnhancedDiagnostics(window_size=max_samples)
            self._log(f"  {Colors.GREEN}[OK]{Colors.NC} 增强诊断分析器已启用")
        
        self._log(f"\n  {Colors.CYAN}[进度]{Colors.NC} 收集控制器诊断数据...")
        self._log(f"  {Colors.YELLOW}[提示]{Colors.NC} 移动机器人以生成跟踪数据!")
        
        # 收集数据并实时分析
        start_time = time.time()
        last_processed_count = 0
        last_progress_time = 0
        
        while time.time() - start_time < self.args.duration:
            time.sleep(0.05)  # 50ms 采样间隔，支持 20Hz 诊断频率
            
            # 每10秒显示一次进度
            elapsed = time.time() - start_time
            if int(elapsed) % 10 == 0 and int(elapsed) > last_progress_time:
                last_progress_time = int(elapsed)
                with self.diag_monitor.lock:
                    msg_count = self.diag_monitor.msg_count
                self._log(f"    [{int(elapsed)}/{int(self.args.duration)}s] 已收集 {msg_count} 条诊断消息")
            
            # 如果有增强分析器，处理所有新增的样本
            if self.enhanced_analyzer:
                with self.diag_monitor.lock:
                    current_count = len(self.diag_monitor.diagnostics)
                    # 处理所有新增的诊断消息，避免丢失数据
                    for i in range(last_processed_count, current_count):
                        if i < len(self.diag_monitor.diagnostics):
                            diag_msg = self.diag_monitor.diagnostics[i]
                            # 统一时间戳处理：从 header.stamp 提取
                            timestamp = diag_msg.header.stamp.to_sec() if hasattr(diag_msg.header.stamp, 'to_sec') else 0.0
                            diag_dict = {
                                'timestamp': timestamp,
                                'cmd_vx': diag_msg.cmd_vx,
                                'cmd_vy': diag_msg.cmd_vy,
                                'cmd_omega': diag_msg.cmd_omega,
                                'tracking_lateral_error': diag_msg.tracking_lateral_error,
                                'tracking_longitudinal_error': diag_msg.tracking_longitudinal_error,
                                'tracking_heading_error': diag_msg.tracking_heading_error,
                                'alpha': diag_msg.consistency_alpha_soft,
                                'state': diag_msg.state,  # 整数状态码
                                'mpc_success': diag_msg.mpc_success
                            }
                            self.enhanced_analyzer.add_sample(diag_dict)
                    last_processed_count = current_count
        
        controller_stats = self.diag_monitor.get_stats()
        self.diag_monitor.stop()
        
        if controller_stats:
            self.results['controller'] = controller_stats
            self._log(f"\n  {Colors.GREEN}[OK]{Colors.NC} 收集 {controller_stats['msg_count']} 条诊断消息")
            
            # 运行增强分析
            if self.enhanced_analyzer:
                self._log(f"  {Colors.CYAN}[进度]{Colors.NC} 运行增强诊断分析...")
                self.results['enhanced_diagnostics'] = {
                    'mpc_weights': self.enhanced_analyzer.analyze_mpc_weights(),
                    'consistency_check': self.enhanced_analyzer.analyze_consistency_check(),
                    'state_machine': self.enhanced_analyzer.analyze_state_machine()
                }
                self._log(f"  {Colors.GREEN}[OK]{Colors.NC} 增强诊断分析完成")
            
            self._log(f"\n  {Colors.GREEN}[完成]{Colors.NC} 阶段3完成")
        else:
            self._log(f"\n  {Colors.RED}[WARN]{Colors.NC} 未收到控制器诊断数据")
            self._log(f"         请确认控制器正在运行并发布诊断消息")
    
    def _calculate_recommendations(self):
        """阶段4: 计算完整推荐配置（14个配置模块）"""
        self._log(f"\n{Colors.BLUE}{'─'*70}")
        self._log(f"  阶段4/6: 计算推荐配置")
        self._log(f"{'─'*70}{Colors.NC}")
        
        # 显示前提条件并等待确认
        data_sources = []
        if self.results.get('odom'):
            data_sources.append(f"里程计数据: {self.results['odom'].get('count', 0)} 条消息")
        if self.results.get('trajectory'):
            data_sources.append(f"轨迹数据: {self.results['trajectory'].get('count', 0)} 条消息")
        if self.results.get('chassis_tests'):
            data_sources.append("底盘测试结果: 已完成")
        if self.results.get('controller'):
            data_sources.append(f"控制器诊断: {self.results['controller'].get('msg_count', 0)} 条消息")
        
        prerequisites = [
            "基于前面阶段收集的数据计算推荐配置",
            "将生成 15 个配置模块"
        ] + data_sources
        
        if not self._wait_for_confirmation("阶段4: 计算推荐配置", prerequisites):
            return
        
        self._log(f"\n  {Colors.CYAN}[进度]{Colors.NC} 分析收集的数据...")
        
        odom = self.results.get('odom', {})
        traj = self.results.get('trajectory', {})
        traj_info = self.results.get('trajectory_info', {})
        imu = self.results.get('imu', {})
        chassis = self.results.get('chassis', {})
        chassis_tests = self.results.get('chassis_tests', {})
        
        odom_rate = odom.get('rate', 0)
        traj_rate = traj.get('rate', 0)
        imu_rate = imu.get('rate', 0)
        odom_jitter = odom.get('jitter_ms', 10)
        odom_latency = odom.get('latency_ms', 0)
        traj_jitter = traj.get('jitter_ms', 0)
        
        # 控制频率 - 必须在使用前确保有效值
        if odom_rate >= 100: ctrl_freq = 50
        elif odom_rate >= 50: ctrl_freq = 40
        elif odom_rate >= 20: ctrl_freq = 20
        elif odom_rate > 0: ctrl_freq = max(10, int(odom_rate / 2))
        else: ctrl_freq = 20  # 无里程计数据时使用默认值
        
        # 安全检查：确保 ctrl_freq 不为0（必须在计算 ctrl_period_ms 之前）
        if ctrl_freq <= 0:
            self._log(f"  {Colors.RED}[ERROR]{Colors.NC} 控制频率计算错误，使用默认值 20 Hz")
            ctrl_freq = 20
        
        ctrl_period_ms = 1000.0 / ctrl_freq
        
        # 底盘参数
        # 优先使用测试结果，其次使用阶段1的里程计数据，最后使用默认值
        max_v = chassis_tests.get('max_velocity_achieved') or chassis.get('max_speed') or 0.5
        max_a = chassis_tests.get('max_acceleration') or chassis.get('max_ax') or 0.5
        max_w = chassis_tests.get('max_angular_velocity') or chassis.get('max_wz') or 1.0
        max_alpha = chassis.get('max_alpha') or 1.0
        response_time = chassis_tests.get('response_time') or 0.2
        safety_margin = 0.8
        
        # 验证底盘参数的合理性
        if max_v <= 0:
            self._log(f"  {Colors.YELLOW}[WARN]{Colors.NC} 未检测到有效速度数据，使用默认值 0.5 m/s")
            max_v = 0.5
        if max_a <= 0:
            self._log(f"  {Colors.YELLOW}[WARN]{Colors.NC} 未检测到有效加速度数据，使用默认值 0.5 m/s^2")
            max_a = 0.5
        if max_w <= 0:
            self._log(f"  {Colors.YELLOW}[WARN]{Colors.NC} 未检测到有效角速度数据，使用默认值 1.0 rad/s")
            max_w = 1.0
        
        # 轨迹参数
        num_points = traj_info.get('num_points', 8)
        dt_sec = traj_info.get('dt_sec', 0.1)
        # MPC horizon 限制：不超过轨迹点数-1，不超过 20，且考虑控制周期
        max_horizon_by_time = max(3, int(ctrl_period_ms * 10 / (dt_sec * 1000))) if dt_sec > 0 else 20
        mpc_horizon = min(max(num_points - 1, 3), max_horizon_by_time, 20)
        traj_length = traj_info.get('total_length', 1.0)
        
        # EKF噪声因子 - 限制最大放大倍数为 2
        odom_noise_factor = 1.0 + min(odom_jitter / 50.0, 1.0)
        
        # Lookahead
        lookahead = max(0.3, min(max_v * response_time * 2, 2.0)) if max_v > 0 else 0.5
        
        # ===== 1. System =====
        # 完整的系统配置，包含所有 system_config.py 中定义的参数
        self.recommended['system'] = {
            'ctrl_freq': ctrl_freq,
            'platform': 'differential',
            'gravity': 9.81,  # 物理常数，通常不需要调整
            'long_pause_threshold': 0.5,  # 长时间暂停检测阈值
            'ekf_reset_threshold': 2.0,   # EKF 重置阈值
        }
        
        # ===== 2. Watchdog =====
        # 超时设置为 5 个周期，确保有足够的容错空间
        self.recommended['watchdog'] = {
            'odom_timeout_ms': int(1000 / max(odom_rate, 1) * 5) if odom_rate > 0 else 500,
            'traj_timeout_ms': int(1000 / max(traj_rate, 1) * 3) if traj_rate > 0 else 1000,
            'traj_grace_ms': int(1000 / max(traj_rate, 1) * 1.5) if traj_rate > 0 else 500,
            'imu_timeout_ms': int(1000 / max(imu_rate, 1) * 5) if imu_rate > 0 else -1,
            'startup_grace_ms': 5000,
        }
        
        # ===== 3. MPC =====
        self.recommended['mpc'] = {
            'horizon': mpc_horizon,
            'horizon_degraded': max(mpc_horizon // 2, 3),
            'dt': dt_sec if dt_sec > 0 else 0.1,
            'weights': {
                'position': 10.0,
                'velocity': 1.0,
                'heading': 5.0,
                'control_accel': 0.2,
                'control_alpha': 0.2,
            },
            'solver': {
                'nlp_max_iter': 50,
                'qp_solver': 'PARTIAL_CONDENSING_HPIPM',
                'integrator_type': 'ERK',
                'nlp_solver_type': 'SQP_RTI',
            },
            'health_monitor': {
                'time_warning_thresh_ms': int(ctrl_period_ms * 0.4),
                'time_critical_thresh_ms': int(ctrl_period_ms * 0.8),
                'time_recovery_thresh_ms': int(ctrl_period_ms * 0.3),
                'condition_number_thresh': 1e8,
                'condition_number_recovery': 1e5,
                'kkt_residual_thresh': 1e-3,
                'consecutive_warning_limit': 10,
                'consecutive_recovery_limit': 5,
                'recovery_multiplier': 2.0,
                'consecutive_good_for_decay': 2,
                'timeout_decay_rate': 2,
            },
            'fallback': {
                'lookahead_steps': 3,
                'heading_kp': 1.5,
                'max_curvature': 5.0,
                'min_distance_thresh': 0.1,
                'min_turn_speed': 0.1,
                'default_speed_ratio': 0.5,
            },
        }
        
        # ===== 4. Constraints =====
        # 完整的运动约束配置，包含 2D 和 3D 参数
        v_max_safe = round(max_v * safety_margin, 2) if max_v > 0 else 0.5
        omega_max_safe = round(max_w * safety_margin, 2) if max_w > 0 else 1.0
        a_max_safe = round(max_a * safety_margin, 2) if max_a > 0 else 0.5
        
        self.recommended['constraints'] = {
            # 2D 约束 (差速车/阿克曼车)
            'v_max': v_max_safe,
            'v_min': -round(v_max_safe * 0.4, 2),  # 倒车速度限制为前进的 40%
            'omega_max': omega_max_safe,
            'omega_max_low': round(omega_max_safe * 0.5, 2),  # 低速时角速度限制
            'v_low_thresh': 0.1,
            'a_max': a_max_safe,
            'alpha_max': round(max_alpha * safety_margin, 2) if max_alpha > 0 else 1.0,
            
            # 3D 约束 (全向车/四旋翼，差速车可忽略)
            'az_max': 1.0,  # 垂直加速度限制
            'vx_max': v_max_safe,
            'vx_min': -round(v_max_safe * 0.4, 2),
            'vy_max': v_max_safe if self.recommended['system']['platform'] == 'omni' else 0.0,
            'vy_min': -v_max_safe if self.recommended['system']['platform'] == 'omni' else 0.0,
            'vz_max': 2.0,  # 垂直速度限制 (四旋翼)
        }
        
        # ===== 5. Safety =====
        # 使用已计算的安全值，确保一致性
        self.recommended['safety'] = {
            'v_stop_thresh': 0.05,
            'vz_stop_thresh': 0.1,
            'stopping_timeout': 5.0,
            'emergency_decel': round(a_max_safe * 1.5, 2),  # 紧急减速度为最大加速度的 1.5 倍
            'low_speed': {
                'threshold': self.recommended['constraints']['v_low_thresh'],  # 与 constraints 保持一致
                'omega_limit': round(omega_max_safe * 0.5, 2),  # 低速角速度限制
            },
            'margins': {
                'velocity': 1.1,
                'acceleration': 1.5,
            },
            'accel_filter_window': 3,
            'accel_filter_alpha': 0.3,
            'accel_filter_warmup_alpha': 0.5,
            'accel_filter_warmup_period': 3,
            'min_dt_for_accel': 0.001,
            'max_dt_for_accel': 1.0,
            'state_machine': {
                'alpha_disable_thresh': 0.1 if traj_info.get('has_velocities') else 0.0,
                'alpha_recovery_value': 0.3 if traj_info.get('has_velocities') else 0.0,
                'alpha_recovery_thresh': 5,
                'mpc_recovery_thresh': 5,
                'mpc_fail_window_size': 10,
                'mpc_fail_thresh': 3,
                'mpc_fail_ratio_thresh': 0.5,
                'mpc_recovery_history_min': 3,
                'mpc_recovery_recent_count': 5,
                'mpc_recovery_tolerance': 0,
                'mpc_recovery_success_ratio': 0.8,
            },
        }
        
        # ===== 6. EKF =====
        # 完整的 EKF 配置，包含 adaptive 和 anomaly_detection 子模块
        self.recommended['ekf'] = {
            'use_odom_orientation_fallback': imu_rate == 0,
            'imu_motion_compensation': imu_rate > 0,
            'theta_covariance_fallback_thresh': 0.5,
            
            # 自适应参数 - 根据底盘特性调整
            'adaptive': {
                'base_slip_thresh': max(2.0, max_a * 1.5) if max_a > 0 else 2.0,  # 基于最大加速度
                'slip_velocity_factor': 0.5,
                'slip_covariance_scale': 10.0,
                'stationary_covariance_scale': 0.1,
                'stationary_thresh': 0.05,
                'slip_probability_k_factor': 5.0,
                'slip_history_window': 20,
            },
            
            # IMU 相关参数
            'max_tilt_angle': 1.047,  # ~60°
            'accel_freshness_thresh': 0.1,
            
            # Jacobian 计算参数
            'min_velocity_for_jacobian': 0.01,
            
            # 过程噪声 - 根据里程计抖动调整
            'process_noise': {
                'position': round(0.001 * odom_noise_factor, 4),
                'velocity': round(0.1 * odom_noise_factor, 3),
                'orientation': round(0.01 * odom_noise_factor, 4),
                'angular_velocity': 0.1,
                'imu_bias': 0.0001,
            },
            
            # 测量噪声 - 根据里程计抖动调整
            'measurement_noise': {
                'odom_position': round(0.01 * odom_noise_factor, 4),
                'odom_velocity': round(0.1 * odom_noise_factor, 3),
                'imu_accel': 0.5,
                'imu_gyro': 0.01,
            },
            
            # 异常检测参数
            'anomaly_detection': {
                'drift_thresh': 0.1,
                'jump_thresh': 0.5,
                'covariance_explosion_thresh': 1000.0,
                'innovation_anomaly_thresh': 10.0,
            },
            
            # 协方差参数
            'covariance': {
                'min_eigenvalue': 1e-6,
                'initial_value': 0.1,
            },
        }
        
        # ===== 7. Consistency =====
        self.recommended['consistency'] = {
            'kappa_thresh': 0.5,
            'v_dir_thresh': 0.8,
            'temporal_smooth_thresh': 0.5 if traj_jitter < 50 else 0.3,
            'alpha_min': 0.1,
            'max_curvature': 10.0,
            'temporal_window_size': 10,
            'weights': {
                'kappa': 1.0,      # 与 universal_controller 默认值一致
                'velocity': 1.5,   # 与 universal_controller 默认值一致
                'temporal': 0.8,   # 与 universal_controller 默认值一致
            },
        }
        
        # ===== 8. Backup =====
        self.recommended['backup'] = {
            'lookahead_dist': round(lookahead, 2),
            'min_lookahead': round(lookahead * 0.6, 2),
            'max_lookahead': round(lookahead * 3, 2),
            'lookahead_ratio': 0.5,
            'kp_z': 1.0,
            'kp_heading': 1.5,
            'heading_mode': 'follow_velocity',
            'dt': round(1.0 / ctrl_freq, 3),
            'heading_error_thresh': 1.047,
            'pure_pursuit_angle_thresh': 1.047,
            'heading_control_angle_thresh': 1.571,
            'max_curvature': 5.0,
            'min_turn_speed': 0.1,
            'default_speed_ratio': 0.5,
            'low_speed_transition_factor': 0.5,
            'curvature_speed_limit_thresh': 0.1,
            'min_distance_thresh': 0.1,
            'omega_rate_limit': None,
        }
        
        # ===== 9. Transform =====
        self.recommended['transform'] = {
            'target_frame': 'odom',
            'source_frame': traj_info.get('frame_id', 'base_link') or 'base_link',
            'timeout_ms': max(50, int(odom_latency * 2 + 20)),
            'fallback_duration_limit_ms': 500,
            'fallback_critical_limit_ms': 1000,
            'drift_estimation_enabled': False,
            'recovery_correction_enabled': True,
            'drift_rate': 0.01,
            'drift_velocity_factor': 0.1,
            'max_drift_dt': 0.5,
            'drift_correction_thresh': 0.01,
            'expected_source_frames': ['base_link', 'base_footprint', 'base_link_0', '', 'odom'],
            'warn_unexpected_frame': True,
        }
        
        # ===== 10. Transition =====
        self.recommended['transition'] = {
            'type': 'exponential',
            'tau': round(1.0 / ctrl_freq * 2, 3),
            'max_duration': 0.5,
            'completion_threshold': 0.95,
            'duration': 0.2,
        }
        
        # ===== 11. TF =====
        self.recommended['tf'] = {
            'source_frame': traj_info.get('frame_id', 'base_link') or 'base_link',
            'target_frame': 'odom',
            'timeout_ms': max(50, int(odom_latency * 2 + 20)),
            'buffer_warmup_timeout_sec': 5.0,
            'buffer_warmup_interval_sec': 0.2,
            'expected_source_frames': ['base_link', 'base_footprint', ''],
        }
        
        # ===== 12. Tracking =====
        self.recommended['tracking'] = {
            'lateral_thresh': min(0.3, traj_length * 0.1),
            'longitudinal_thresh': min(0.5, traj_length * 0.15),
            'heading_thresh': 0.5,
            'prediction_thresh': 0.5,
            'weights': {
                'lateral': 0.4,
                'longitudinal': 0.4,
                'heading': 0.2,
            },
            'rating': {
                'excellent': 90,
                'good': 70,
                'fair': 50,
            },
        }
        
        # ===== 13. cmd_vel_adapter =====
        self.recommended['cmd_vel_adapter'] = {
            'publish_rate': float(ctrl_freq),
            'joy_timeout': 0.5,
            'max_linear': self.recommended['constraints']['v_max'],
            'max_angular': self.recommended['constraints']['omega_max'],
            'max_linear_accel': 0.0,
            'max_angular_accel': 0.0,
            'input_topic': '/cmd_unified',
            'joy_topic': '/joy_cmd_vel',
            'mode_topic': '/visualizer/control_mode',
            'output_topic': self.topics['cmd_vel'],
        }
        
        # ===== 14. Trajectory =====
        # 此配置模块定义轨迹处理参数，特别是 low_speed_thresh
        # low_speed_thresh 是影响角速度计算的关键参数
        self.recommended['trajectory'] = {
            'default_dt_sec': dt_sec if dt_sec > 0 else 0.1,
            'min_dt_sec': 0.01,
            'max_dt_sec': 1.0,
            'min_points': 2,
            'max_points': 100,
            'default_num_points': num_points if num_points > 0 else 20,
            # low_speed_thresh: 低于此速度时，角速度 wz 被置为 0
            # 这是影响轨迹跟踪的关键参数
            # 默认值 0.1 m/s，如果轨迹速度普遍较低，应降低此值
            'low_speed_thresh': self._calculate_low_speed_thresh(traj_info),
            'max_point_distance': 10.0,
            'min_confidence': 0.0,
            'max_confidence': 1.0,
            'default_confidence': traj_info.get('confidence', 0.9),
            'default_frame_id': traj_info.get('frame_id', 'base_link') or 'base_link',
            'output_frame_id': 'odom',
        }
        
        # ===== 15. Diagnostics =====
        # 诊断发布配置，与 system_config.py 中的 DIAGNOSTICS_CONFIG 对应
        self.recommended['diagnostics'] = {
            'topic': '/controller/diagnostics',
            'cmd_topic': '/cmd_unified',
            'publish_rate': 10,  # 每 N 次控制循环发布一次诊断
        }
        
        # ===== 基于运行时数据调优 MPC 权重 =====
        self._tune_mpc_weights_from_runtime()
        
        # ===== 基于运行时数据调优一致性参数 =====
        self._tune_consistency_from_runtime()
    
    def _calculate_low_speed_thresh(self, traj_info: Dict) -> float:
        """
        根据轨迹特性计算推荐的 low_speed_thresh
        
        Args:
            traj_info: 轨迹信息字典
        
        Returns:
            推荐的 low_speed_thresh 值
        """
        # 默认值
        default_thresh = 0.1
        
        # 如果没有轨迹信息，使用默认值
        if not traj_info:
            return default_thresh
        
        # 获取轨迹的最小速度和平均速度
        min_speed = traj_info.get('min_speed', 0)
        avg_speed = traj_info.get('avg_speed', 0)
        
        # 优先使用最小速度来计算阈值，确保所有点都能正确计算角速度
        # 如果最小速度接近或低于默认阈值，需要降低阈值
        if min_speed > 0 and min_speed < default_thresh * 1.5:
            # 使用最小速度的 50% 作为阈值，确保有足够裕度
            recommended = max(0.01, min_speed * 0.5)
            self._log(f"  {Colors.YELLOW}[INFO]{Colors.NC} 轨迹最小速度 ({min_speed:.3f} m/s) 接近阈值，"
                     f"建议 low_speed_thresh = {recommended:.3f}")
            return round(recommended, 3)
        
        # 如果平均速度很低，也需要降低阈值
        if avg_speed > 0 and avg_speed < 0.15:
            # 轨迹速度很低，使用更低的阈值
            # 使用平均速度的 30% 作为阈值（比之前的 50% 更保守）
            recommended = max(0.01, avg_speed * 0.3)
            self._log(f"  {Colors.YELLOW}[INFO]{Colors.NC} 轨迹平均速度较低 ({avg_speed:.3f} m/s)，"
                     f"建议 low_speed_thresh = {recommended:.3f}")
            return round(recommended, 3)
        
        return default_thresh
    
    def _tune_mpc_weights_from_runtime(self):
        """
        基于运行时跟踪误差调整 MPC 权重
        
        此方法分析控制器运行时的跟踪误差和控制平滑性，
        自动调整 MPC 权重以优化跟踪性能。
        """
        controller = self.results.get('controller')
        if not controller:
            return
        
        # 获取跟踪误差
        lateral_error_avg = controller.get('lateral_error_avg', 0)
        lateral_error_max = controller.get('lateral_error_max', 0)
        heading_error_avg = controller.get('heading_error_avg', 0)
        
        # 获取增强诊断结果
        enhanced = self.results.get('enhanced_diagnostics', {})
        mpc_analysis = enhanced.get('mpc_weights', {})
        
        if mpc_analysis.get('status') == 'ok':
            metrics = mpc_analysis.get('metrics', {})
            suggestions = mpc_analysis.get('suggestions', [])
            
            # 根据建议调整权重 - 使用 DiagnosticsThresholds 统一管理
            for sug in suggestions:
                if sug.get('priority') in ['critical', 'high']:
                    param = sug.get('parameter', '')
                    
                    if 'position' in param:
                        # 横向误差大，增加 position 权重
                        if lateral_error_avg > DiagnosticsThresholds.TUNING_LATERAL_ERROR_HIGH:
                            self.recommended['mpc']['weights']['position'] = 15.0
                            self._log(f"  {Colors.YELLOW}[调优]{Colors.NC} 横向误差较大 ({lateral_error_avg:.3f}m)，"
                                     f"增加 position 权重到 15.0")
                        elif lateral_error_avg > DiagnosticsThresholds.TUNING_LATERAL_ERROR_MED:
                            self.recommended['mpc']['weights']['position'] = 12.0
                    
                    elif 'heading' in param:
                        # 航向误差大，增加 heading 权重
                        if heading_error_avg > DiagnosticsThresholds.TUNING_HEADING_ERROR_HIGH:
                            self.recommended['mpc']['weights']['heading'] = 8.0
                            self._log(f"  {Colors.YELLOW}[调优]{Colors.NC} 航向误差较大 ({np.degrees(heading_error_avg):.1f}°)，"
                                     f"增加 heading 权重到 8.0")
                        elif heading_error_avg > DiagnosticsThresholds.TUNING_HEADING_ERROR_MED:
                            self.recommended['mpc']['weights']['heading'] = 6.0
                    
                    elif 'control_accel' in param:
                        # 加速度抖动大，增加 control_accel 权重
                        max_accel = metrics.get('max_accel', 0)
                        if max_accel > DiagnosticsThresholds.MAX_ACCEL_JITTER:
                            self.recommended['mpc']['weights']['control_accel'] = 0.5
                            self._log(f"  {Colors.YELLOW}[调优]{Colors.NC} 加速度抖动大 ({max_accel:.2f} m/s²)，"
                                     f"增加 control_accel 权重到 0.5")
                    
                    elif 'control_alpha' in param:
                        # 角加速度抖动大，增加 control_alpha 权重
                        max_angular_accel = metrics.get('max_angular_accel', 0)
                        if max_angular_accel > DiagnosticsThresholds.MAX_ANGULAR_ACCEL_JITTER:
                            self.recommended['mpc']['weights']['control_alpha'] = 0.5
                            self._log(f"  {Colors.YELLOW}[调优]{Colors.NC} 角加速度抖动大 ({max_angular_accel:.2f} rad/s²)，"
                                     f"增加 control_alpha 权重到 0.5")
        
        # 即使没有增强诊断，也根据基本指标调整
        elif lateral_error_avg > DiagnosticsThresholds.TUNING_LATERAL_ERROR_HIGH:
            self.recommended['mpc']['weights']['position'] = 15.0
            self._log(f"  {Colors.YELLOW}[调优]{Colors.NC} 横向误差较大 ({lateral_error_avg:.3f}m)，"
                     f"增加 position 权重到 15.0")
        
        if heading_error_avg > DiagnosticsThresholds.TUNING_HEADING_ERROR_HIGH:
            self.recommended['mpc']['weights']['heading'] = 8.0
            self._log(f"  {Colors.YELLOW}[调优]{Colors.NC} 航向误差较大 ({np.degrees(heading_error_avg):.1f}°)，"
                     f"增加 heading 权重到 8.0")
    
    def _tune_consistency_from_runtime(self):
        """
        基于运行时 alpha 值调整一致性参数
        
        此方法分析一致性检查的拒绝率，自动调整阈值。
        """
        controller = self.results.get('controller')
        if not controller:
            return
        
        alpha_avg = controller.get('alpha_avg', 1.0)
        alpha_min = controller.get('alpha_min', 1.0)
        
        # 获取增强诊断结果
        enhanced = self.results.get('enhanced_diagnostics', {})
        consistency_analysis = enhanced.get('consistency_check', {})
        
        if consistency_analysis.get('status') == 'ok':
            metrics = consistency_analysis.get('metrics', {})
            rejection_rate = metrics.get('rejection_rate', 0)
            
            # 如果拒绝率过高，放宽阈值 - 使用 DiagnosticsThresholds 统一管理
            if rejection_rate > DiagnosticsThresholds.CONSISTENCY_REJECTION_HIGH:
                self.recommended['consistency']['kappa_thresh'] = 0.7
                self.recommended['consistency']['v_dir_thresh'] = 0.9
                self._log(f"  {Colors.YELLOW}[调优]{Colors.NC} 一致性拒绝率过高 ({rejection_rate*100:.1f}%)，"
                         f"放宽阈值 (kappa: 0.7, v_dir: 0.9)")
            elif rejection_rate > DiagnosticsThresholds.CONSISTENCY_REJECTION_MED:
                self.recommended['consistency']['kappa_thresh'] = 0.6
                self.recommended['consistency']['v_dir_thresh'] = 0.85
        
        # 如果 alpha 经常很低，可能需要调整 - 使用 DiagnosticsThresholds 统一管理
        if alpha_min < DiagnosticsThresholds.ALPHA_VERY_LOW:
            self._log(f"  {Colors.YELLOW}[警告]{Colors.NC} Alpha 最小值很低 ({alpha_min:.2f})，"
                     f"检查轨迹质量或放宽一致性阈值")

    def _show_tuning_results(self):
        """显示调优结果和运行时调优建议"""
        self._log(f"\n{Colors.BLUE}{'─'*70}")
        self._log(f"  阶段5/6: 诊断结果")
        self._log(f"{'─'*70}{Colors.NC}")
        
        # 显示前提条件并等待确认
        prerequisites = [
            "显示所有收集的诊断数据和推荐配置",
            "包含: 传感器状态、轨迹特性、底盘特性、推荐参数"
        ]
        if not self._wait_for_confirmation("阶段5: 显示诊断结果", prerequisites):
            return
        
        # 传感器状态
        self._log(f"{Colors.CYAN}传感器状态:{Colors.NC}")
        for name in ['odom', 'imu', 'trajectory']:
            stats = self.results.get(name, {})
            rate = stats.get('rate', 0)
            latency = stats.get('latency_ms', 0)
            jitter = stats.get('jitter_ms', 0)
            status = f"{Colors.GREEN}[OK]{Colors.NC}" if rate > 0 else f"{Colors.RED}[NO DATA]{Colors.NC}"
            self._log(f"  {status} {name}: {rate:.1f} Hz, 延迟: {latency:.1f}ms, 抖动: {jitter:.1f}ms")
        
        # 轨迹特性
        traj_info = self.results.get('trajectory_info', {})
        if traj_info:
            self._log(f"\n{Colors.CYAN}轨迹特性:{Colors.NC}")
            self._log(f"  点数: {traj_info.get('num_points', 'N/A')}")
            self._log(f"  时间步长: {traj_info.get('dt_sec', 'N/A')} sec")
            self._log(f"  包含速度: {'是' if traj_info.get('has_velocities') else '否'}")
            self._log(f"  坐标系: {traj_info.get('frame_id', 'N/A')}")
            self._log(f"  长度: {traj_info.get('total_length', 0):.2f} m")
            self._log(f"  最大曲率: {traj_info.get('max_curvature', 0):.2f} 1/m")
        
        # 底盘特性
        chassis = self.results.get('chassis', {})
        if chassis:
            self._log(f"\n{Colors.CYAN}底盘特性 (阶段1被动监听，仅供参考):{Colors.NC}")
            max_speed = chassis.get('max_speed', 0)
            if max_speed < 0.01:
                self._log(f"  {Colors.YELLOW}[提示]{Colors.NC} 监听期间机器人未移动，数据为0是正常的")
            self._log(f"  最大速度: {chassis.get('max_speed', 0):.2f} m/s")
            self._log(f"  最大vx: {chassis.get('max_vx', 0):.2f} m/s")
            self._log(f"  最大wz: {chassis.get('max_wz', 0):.2f} rad/s")
            if 'max_ax' in chassis:
                self._log(f"  最大加速度: {chassis.get('max_ax', 0):.2f} m/s^2")
                self._log(f"  最大角加速度: {chassis.get('max_alpha', 0):.2f} rad/s^2")
        
        # 底盘测试结果
        tests = self.results.get('chassis_tests', {})
        if tests:
            self._log(f"\n{Colors.CYAN}底盘测试结果 (阶段2主动测试，真实能力):{Colors.NC}")
            self._log(f"  实测最大速度: {tests.get('max_velocity_achieved', 0):.2f} m/s")
            self._log(f"  实测最大加速度: {tests.get('max_acceleration', 0):.2f} m/s^2")
            self._log(f"  实测最大角速度: {tests.get('max_angular_velocity', 0):.2f} rad/s")
            self._log(f"  响应时间: {tests.get('response_time', 0):.3f} sec")
            self._log(f"  {Colors.GREEN}[提示]{Colors.NC} 配置生成将优先使用测试结果")
        
        # 推荐参数
        self._log(f"\n{Colors.CYAN}推荐参数:{Colors.NC}")
        self._log(f"  控制频率: {self.recommended['system']['ctrl_freq']} Hz")
        self._log(f"  MPC horizon: {self.recommended['mpc']['horizon']}")
        self._log(f"  MPC dt: {self.recommended['mpc']['dt']} sec")
        self._log(f"  v_max: {self.recommended['constraints']['v_max']} m/s")
        self._log(f"  omega_max: {self.recommended['constraints']['omega_max']} rad/s")
        self._log(f"  a_max: {self.recommended['constraints']['a_max']} m/s^2")
        self._log(f"  Odom timeout: {self.recommended['watchdog']['odom_timeout_ms']} ms")
        self._log(f"  Traj timeout: {self.recommended['watchdog']['traj_timeout_ms']} ms")
        self._log(f"  Lookahead: {self.recommended['backup']['lookahead_dist']} m")
        
        # 优化建议
        self._log(f"\n{Colors.CYAN}优化建议:{Colors.NC}")
        odom_rate = self.results.get('odom', {}).get('rate', 0)
        traj_rate = self.results.get('trajectory', {}).get('rate', 0)
        
        if odom_rate > 50:
            self._log(f"  {Colors.GREEN}[OK]{Colors.NC} 里程计频率良好，支持高频控制")
        elif odom_rate > 0:
            self._log(f"  {Colors.YELLOW}[WARN]{Colors.NC} 里程计频率较低，控制频率受限")
        else:
            self._log(f"  {Colors.RED}[ERROR]{Colors.NC} 无里程计数据!")
        
        if traj_rate > 0 and traj_rate < 2:
            self._log(f"  {Colors.YELLOW}[WARN]{Colors.NC} 轨迹频率较低，增加超时值")
        
        if traj_info.get('num_points', 0) < 5:
            self._log(f"  {Colors.YELLOW}[WARN]{Colors.NC} 轨迹点数少，MPC预测受限")
        
        if not traj_info.get('has_velocities'):
            self._log(f"  {Colors.YELLOW}[WARN]{Colors.NC} 轨迹无速度信息，alpha检查禁用")
        
        odom_jitter = self.results.get('odom', {}).get('jitter_ms', 0)
        if odom_jitter > 20:
            self._log(f"  {Colors.YELLOW}[WARN]{Colors.NC} 里程计抖动较大 ({odom_jitter:.1f}ms)，已增加EKF噪声")
        
        # 控制器运行时统计和调优建议
        controller = self.results.get('controller')
        if controller:
            self._log(f"\n{Colors.CYAN}控制器运行时统计:{Colors.NC}")
            self._log(f"  MPC求解时间: {controller['mpc_solve_time_avg_ms']:.2f}ms avg, {controller['mpc_solve_time_max_ms']:.2f}ms max")
            self._log(f"  MPC成功率: {controller['mpc_success_rate']*100:.1f}%")
            self._log(f"  备用控制器使用率: {controller['backup_active_ratio']*100:.1f}%")
            self._log(f"  横向误差: {controller['lateral_error_avg']*100:.1f}cm avg, {controller['lateral_error_max']*100:.1f}cm max")
            self._log(f"  航向误差: {np.degrees(controller['heading_error_avg']):.1f}° avg")
            self._log(f"  Alpha (一致性): {controller['alpha_avg']:.2f} avg, {controller['alpha_min']:.2f} min")
            
            # 运行时调优建议
            self._log(f"\n{Colors.MAGENTA}运行时调优建议:{Colors.NC}")
            ctrl_freq = self.recommended.get('system', {}).get('ctrl_freq', 20)
            ctrl_period_ms = 1000 / ctrl_freq
            
            # MPC 时间
            if controller['mpc_solve_time_avg_ms'] > ctrl_period_ms * 0.5:
                self._log(f"  {Colors.RED}[CRITICAL]{Colors.NC} MPC求解时间过高!")
                self._log(f"    → 降低 mpc.horizon (当前推荐: {self.recommended.get('mpc', {}).get('horizon', 7)})")
                self._log(f"    → 或降低 system.ctrl_freq")
            elif controller['mpc_solve_time_avg_ms'] > ctrl_period_ms * 0.3:
                self._log(f"  {Colors.YELLOW}[WARN]{Colors.NC} MPC求解时间较高")
                self._log(f"    → 考虑降低 mpc.horizon")
            else:
                self._log(f"  {Colors.GREEN}[OK]{Colors.NC} MPC求解时间良好 ({controller['mpc_solve_time_avg_ms']:.1f}ms < {ctrl_period_ms*0.3:.1f}ms)")
            
            # MPC 成功率 - 使用 DiagnosticsThresholds 统一管理
            if controller['mpc_success_rate'] < DiagnosticsThresholds.MPC_SUCCESS_RATE_CRITICAL:
                self._log(f"  {Colors.RED}[CRITICAL]{Colors.NC} MPC成功率过低 ({controller['mpc_success_rate']*100:.0f}%)")
                self._log(f"    → 检查轨迹质量")
                self._log(f"    → 降低 mpc.horizon")
                self._log(f"    → 增加 mpc.solver.nlp_max_iter")
            elif controller['mpc_success_rate'] < DiagnosticsThresholds.MPC_SUCCESS_RATE_WARN:
                self._log(f"  {Colors.YELLOW}[WARN]{Colors.NC} MPC成功率可以更好")
            else:
                self._log(f"  {Colors.GREEN}[OK]{Colors.NC} MPC成功率良好 ({controller['mpc_success_rate']*100:.0f}%)")
            
            # 备用控制器使用 - 使用 DiagnosticsThresholds 统一管理
            if controller['backup_active_ratio'] > DiagnosticsThresholds.BACKUP_ACTIVE_RATIO_WARN:
                self._log(f"  {Colors.YELLOW}[WARN]{Colors.NC} 备用控制器使用频繁 ({controller['backup_active_ratio']*100:.0f}%)")
                self._log(f"    → 检查MPC求解器健康")
                self._log(f"    → 验证轨迹一致性")
            
            # 跟踪误差 - 使用 DiagnosticsThresholds 统一管理
            if controller['lateral_error_avg'] > DiagnosticsThresholds.TUNING_LATERAL_ERROR_MED:
                self._log(f"  {Colors.YELLOW}[WARN]{Colors.NC} 横向跟踪误差较大 ({controller['lateral_error_avg']*100:.1f}cm)")
                self._log(f"    → 增加 mpc.weights.position (尝试 15-20)")
                self._log(f"    → 减小 mpc.weights.control_accel (尝试 0.1)")
            else:
                self._log(f"  {Colors.GREEN}[OK]{Colors.NC} 横向跟踪误差可接受")
            
            if controller['heading_error_avg'] > DiagnosticsThresholds.TUNING_HEADING_ERROR_HIGH:
                self._log(f"  {Colors.YELLOW}[WARN]{Colors.NC} 航向误差较大 ({np.degrees(controller['heading_error_avg']):.1f}°)")
                self._log(f"    → 增加 mpc.weights.heading (尝试 8-10)")
            
            # Alpha (一致性) - 使用 DiagnosticsThresholds 统一管理
            if controller['alpha_min'] < DiagnosticsThresholds.ALPHA_CRITICAL:
                self._log(f"  {Colors.YELLOW}[WARN]{Colors.NC} 检测到低alpha值 (min: {controller['alpha_min']:.2f})")
                self._log(f"    → 轨迹一致性较差")
                self._log(f"    → 检查网络输出质量")
        
        # 增强诊断结果
        if 'enhanced_diagnostics' in self.results and self.enhanced_analyzer:
            self._log(f"\n{Colors.CYAN}增强诊断分析:{Colors.NC}")
            
            # 显示完整报告
            report = self.enhanced_analyzer.generate_report()
            for line in report.split('\n'):
                self._log(line)
            
            # 收集所有高优先级建议
            all_suggestions = self.enhanced_analyzer.get_all_suggestions()
            high_priority_suggestions = [s for s in all_suggestions if s['priority'] in ['critical', 'high']]
            
            if high_priority_suggestions:
                self._log(f"\n{Colors.RED}⚠️  高优先级配置建议:{Colors.NC}")
                for idx, sug in enumerate(high_priority_suggestions, 1):
                    self._log(f"  {idx}. {sug['parameter']}")
                    self._log(f"     问题: {sug['current_issue']}")
                    self._log(f"     建议: {sug['suggestion']}")
    
    def _generate_config(self, output_file: str):
        """生成完整配置文件（15个配置模块，与 universal_controller 完全对应）"""
        self._log(f"\n{Colors.BLUE}{'─'*70}")
        self._log(f"  阶段6/6: 生成配置文件")
        self._log(f"{'─'*70}{Colors.NC}")
        
        # 显示前提条件并等待确认
        prerequisites = [
            f"输出文件: {output_file}",
            "将生成包含 15 个配置模块的 YAML 文件",
            "配置基于前面阶段收集的数据"
        ]
        if not self._wait_for_confirmation("阶段6: 生成配置文件", prerequisites):
            return
        
        self._log(f"\n  {Colors.CYAN}[进度]{Colors.NC} 生成配置文件: {output_file}")
        
        # 构建完整配置 (15个配置模块，与 universal_controller/config/default_config.py 对应)
        # 注意: attitude 和 mock 模块不包含在自动调优中
        config = {
            # 核心模块
            'system': self.recommended['system'],
            'watchdog': self.recommended['watchdog'],
            'mpc': self.recommended['mpc'],
            'constraints': self.recommended['constraints'],
            'safety': self.recommended['safety'],
            'ekf': self.recommended['ekf'],
            
            # 功能模块
            'consistency': self.recommended['consistency'],
            'transform': self.recommended['transform'],
            'transition': self.recommended['transition'],
            'backup': self.recommended['backup'],
            'trajectory': self.recommended['trajectory'],
            'tracking': self.recommended['tracking'],
            
            # ROS 适配模块
            'topics': {
                'odom': self.topics['odom'],
                'imu': self.topics['imu'] if self.results.get('imu', {}).get('rate', 0) > 0 else '',
                'trajectory': self.topics['trajectory'],
                'emergency_stop': '/controller/emergency_stop',
                'cmd_unified': '/cmd_unified',
                'diagnostics': '/controller/diagnostics',
                'state': '/controller/state',
            },
            'tf': self.recommended['tf'],
            'cmd_vel_adapter': self.recommended['cmd_vel_adapter'],
            'diagnostics': self.recommended['diagnostics'],
        }
        
        # 生成文件头
        odom = self.results.get('odom', {})
        traj = self.results.get('trajectory', {})
        traj_info = self.results.get('trajectory_info', {})
        chassis = self.results.get('chassis', {})
        tests = self.results.get('chassis_tests', {})
        
        header = f"""# ============================================================================
# 自动生成的优化配置
# 生成时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}
# ============================================================================
#
# 检测结果:
#   里程计: {odom.get('rate', 0):.1f} Hz, 延迟 {odom.get('latency_ms', 0):.1f}ms, 抖动 {odom.get('jitter_ms', 0):.1f}ms
#   轨迹: {traj.get('rate', 0):.1f} Hz, {traj_info.get('num_points', 0)} 点, dt={traj_info.get('dt_sec', 0.1)}s
#   IMU: {self.results.get('imu', {}).get('rate', 0):.1f} Hz
#   包含速度信息: {traj_info.get('has_velocities', False)}
#
# 底盘特性:
#   最大速度: {chassis.get('max_speed', 0):.2f} m/s
#   最大角速度: {chassis.get('max_wz', 0):.2f} rad/s
#   最大加速度: {chassis.get('max_ax', 0):.2f} m/s^2
"""
        
        if tests:
            header += f"""#
# 底盘测试结果:
#   实测最大速度: {tests.get('max_velocity_achieved', 0):.2f} m/s
#   实测最大加速度: {tests.get('max_acceleration', 0):.2f} m/s^2
#   响应时间: {tests.get('response_time', 0):.3f} sec
"""
        
        header += """#
# ============================================================================

"""
        
        try:
            with open(output_file, 'w', encoding='utf-8') as f:
                f.write(header)
                yaml.dump(config, f, default_flow_style=False, allow_unicode=True, sort_keys=False)
            self._log(f"  {Colors.GREEN}[OK]{Colors.NC} 配置已保存到: {output_file}")
        except Exception as e:
            self._log(f"  {Colors.RED}[ERROR]{Colors.NC} 保存配置文件失败: {e}")
            return
        
        self._log(f"\n{Colors.CYAN}使用方法:{Colors.NC}")
        self._log(f"  roslaunch controller_ros controller.launch config:=$(pwd)/{output_file}")

    # ==================== 运行入口 ====================
    
    def _print_header(self, mode_name: str, sub_phase: str = None):
        """
        打印统一的标题头
        
        Args:
            mode_name: 模式名称 (实时监控/系统调优/完整模式)
            sub_phase: 子阶段名称 (可选，用于 full 模式的阶段标识)
        """
        self._log(f"\n{Colors.GREEN}{'='*80}")
        self._log(f"         统一诊断工具 v2.7 - {mode_name}")
        self._log(f"{'='*80}{Colors.NC}")
        if self.log_file:
            self._log(f"\n日志文件: {self.log_file}")
        if sub_phase:
            self._log(f"\n{Colors.MAGENTA}=== {sub_phase} ==={Colors.NC}\n")
    
    def _print_realtime_info(self):
        """打印实时监控模式的诊断内容说明"""
        self._log("\n诊断内容:")
        self._log("  1. 轨迹输入分析    5. 状态估计器健康    9. 坐标变换状态")
        self._log("  2. 控制输出分析    6. 跟踪误差         10. 问题汇总")
        self._log("  3. MPC健康状态     7. 超时状态")
        self._log("  4. 一致性指标      8. 紧急停止")
    
    def _setup_realtime_subscribers(self):
        """
        设置实时监控所需的话题订阅
        
        Returns:
            bool: 是否成功设置（自定义消息是否可用）
        """
        self._log("\n订阅话题:")
        
        custom_msg_ok = False
        if CUSTOM_MSG_AVAILABLE:
            rospy.Subscriber(self.topics['trajectory'], LocalTrajectoryV4, self._traj_callback, queue_size=10)
            self._log(f"  {Colors.GREEN}✓{Colors.NC} {self.topics['trajectory']} (LocalTrajectoryV4)")
            rospy.Subscriber(self.topics['diagnostics'], DiagnosticsV2, self._diag_callback, queue_size=10)
            self._log(f"  {Colors.GREEN}✓{Colors.NC} {self.topics['diagnostics']} (DiagnosticsV2)")
            rospy.Subscriber(self.topics['cmd_unified'], UnifiedCmd, self._cmd_callback, queue_size=10)
            self._log(f"  {Colors.GREEN}✓{Colors.NC} {self.topics['cmd_unified']} (UnifiedCmd)")
            custom_msg_ok = True
        else:
            self._log(f"  {Colors.RED}✗{Colors.NC} 自定义消息不可用")
        
        rospy.Subscriber(self.topics['odom'], Odometry, self._odom_callback, queue_size=10)
        self._log(f"  {Colors.GREEN}✓{Colors.NC} {self.topics['odom']} (Odometry)")
        rospy.Subscriber(self.topics['cmd_vel'], Twist, self._twist_callback, queue_size=10)
        self._log(f"  {Colors.GREEN}✓{Colors.NC} {self.topics['cmd_vel']} (Twist)")
        
        return custom_msg_ok
    
    def _run_realtime_loop(self):
        """
        运行实时监控主循环
        
        此方法会阻塞直到 ROS 关闭或用户中断
        """
        self._log("\n" + "-"*75)
        self._log("等待数据... 每3秒输出一次完整诊断报告")
        self._log("按 Ctrl+C 退出")
        self._log("-"*75 + "\n")
        
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rate.sleep()
    
    def _run_tuning_phases(self):
        """
        运行系统调优的所有阶段
        
        包含: 话题监控、底盘测试(可选)、运行时诊断(可选)、配置计算、结果显示、配置生成(可选)
        """
        # 阶段1: 话题监控
        self._run_topic_monitoring()
        
        # 阶段2: 底盘测试 (可选)
        if self.args.test_chassis:
            self._run_chassis_tests()
        
        # 阶段3: 控制器运行时诊断 (可选)
        if self.args.runtime_tuning:
            self._run_controller_diagnostics()
        
        # 阶段4: 计算推荐配置
        self._calculate_recommendations()
        
        # 阶段5: 显示结果
        self._show_tuning_results()
        
        # 阶段6: 生成配置文件
        if self.args.output:
            self._generate_config(self.args.output)
    
    def run_realtime(self):
        """
        运行实时监控模式
        
        订阅 DiagnosticsV2 等话题，显示完整控制器内部状态（10个诊断板块）
        需要控制器运行
        """
        self._init_ros_node('unified_diagnostics_realtime')
        self._init_tf2()
        self._init_log()
        
        try:
            self._print_header("实时监控模式")
            self._print_realtime_info()
            self._setup_realtime_subscribers()
            self._run_realtime_loop()
        finally:
            self._close_log()
            if self.log_file:
                safe_print(f"\n日志已保存到: {self.log_file}")
    
    def run_tuning(self):
        """
        运行系统调优模式
        
        传感器频率/延迟/抖动分析，底盘测试，完整配置生成（14个配置模块）
        不需要控制器运行
        """
        self._init_ros_node('unified_diagnostics_tuning')
        self._init_log()
        
        try:
            self._print_header("系统调优模式")
            self._run_tuning_phases()
        finally:
            self._close_log()
            if self.log_file:
                safe_print(f"\n日志已保存到: {self.log_file}")
    
    def run_full(self):
        """
        运行完整模式 - 先调优后实时监控
        
        第一大阶段: 系统调优诊断 (包含6个子阶段)
        第二大阶段: 实时监控 (10个诊断板块)
        
        两阶段的日志会追加到同一个文件中
        """
        self._init_ros_node('unified_diagnostics_full')
        self._init_log()
        
        try:
            # ===== 打印完整模式概览 =====
            self._print_header("完整模式")
            self._print_full_mode_overview()
            
            # ===== 第一大阶段: 系统调优 =====
            self._log(f"\n{Colors.GREEN}{'='*80}")
            self._log(f"  第一大阶段: 系统调优诊断")
            self._log(f"{'='*80}{Colors.NC}")
            self._run_tuning_phases()
            
            # 阶段分隔
            self._log(f"\n{Colors.GREEN}{'='*80}")
            self._log("  ✅ 第一大阶段完成")
            self._log(f"{'='*80}{Colors.NC}\n")
            
            # ===== 等待用户确认进入第二阶段 =====
            self._log(f"{Colors.MAGENTA}{'='*80}")
            self._log(f"  第二大阶段: 实时监控")
            self._log(f"{'='*80}{Colors.NC}")
            self._log(f"\n{Colors.CYAN}前提条件:{Colors.NC}")
            self._log(f"  • 控制器必须正在运行 (roslaunch controller_ros controller.launch)")
            self._log(f"  • 需要有轨迹输入 (/nn/local_trajectory)")
            self._log(f"\n{Colors.YELLOW}按 Enter 进入实时监控模式 (Ctrl+C 退出并保存当前结果)...{Colors.NC}")
            try:
                input()
            except KeyboardInterrupt:
                self._log("\n诊断结束 (用户取消第二阶段)")
                return
            
            # ===== 第二大阶段: 实时监控 =====
            self._init_tf2()
            self._print_realtime_info()
            self._setup_realtime_subscribers()
            self._run_realtime_loop()
        finally:
            self._close_log()
            if self.log_file:
                safe_print(f"\n日志已保存到: {self.log_file}")
    
    def _print_full_mode_overview(self):
        """打印完整模式的阶段概览和前提条件"""
        self._log(f"\n{Colors.CYAN}{'='*70}")
        self._log("  完整诊断流程概览")
        self._log(f"{'='*70}{Colors.NC}")
        
        # 当前参数
        self._log(f"\n{Colors.YELLOW}当前参数:{Colors.NC}")
        self._log(f"  --mode full")
        self._log(f"  --test-chassis: {'是' if self.args.test_chassis else '否'}")
        self._log(f"  --runtime-tuning: {'是' if self.args.runtime_tuning else '否'}")
        self._log(f"  --duration: {self.args.duration} 秒")
        self._log(f"  --output: {self.args.output or '(未指定)'}")
        
        # 阶段概览
        self._log(f"\n{Colors.CYAN}执行阶段:{Colors.NC}")
        self._log(f"\n  {Colors.GREEN}【第一大阶段: 系统调优诊断】{Colors.NC}")
        self._log(f"    阶段1: 话题监控 ({self.args.duration}秒)")
        self._log(f"           - 被动监听 odom/imu/trajectory 话题")
        self._log(f"           - 收集传感器频率、延迟、抖动数据")
        self._log(f"           {Colors.YELLOW}前提: turtlebot_bringup + trajectory_publisher{Colors.NC}")
        
        if self.args.test_chassis:
            self._log(f"\n    阶段2: 底盘能力测试 ⚠️ 机器人会移动!")
            self._log(f"           - 测试最大速度 (3秒)")
            self._log(f"           - 测试加速度 (2秒)")
            self._log(f"           - 测试最大角速度 (2秒)")
            self._log(f"           - 测试响应时间 (3秒)")
            self._log(f"           {Colors.YELLOW}前提: 周围空间安全，需要用户确认{Colors.NC}")
        else:
            self._log(f"\n    阶段2: 底盘能力测试 {Colors.YELLOW}[跳过]{Colors.NC} (未指定 --test-chassis)")
        
        if self.args.runtime_tuning:
            self._log(f"\n    阶段3: 控制器运行时诊断 ({self.args.duration}秒)")
            self._log(f"           - 收集 MPC 求解时间、成功率")
            self._log(f"           - 收集跟踪误差、alpha 值")
            self._log(f"           - 运行增强诊断分析")
            self._log(f"           {Colors.RED}前提: 控制器必须正在运行!{Colors.NC}")
        else:
            self._log(f"\n    阶段3: 控制器运行时诊断 {Colors.YELLOW}[跳过]{Colors.NC} (未指定 --runtime-tuning)")
        
        self._log(f"\n    阶段4: 计算推荐配置")
        self._log(f"           - 生成 15 个配置模块")
        self._log(f"           - 基于运行时数据调优 MPC 权重")
        
        self._log(f"\n    阶段5: 显示诊断结果")
        self._log(f"           - 传感器状态、轨迹特性、底盘特性")
        self._log(f"           - 推荐参数、优化建议")
        
        if self.args.output:
            self._log(f"\n    阶段6: 生成配置文件")
            self._log(f"           - 输出到: {self.args.output}")
        else:
            self._log(f"\n    阶段6: 生成配置文件 {Colors.YELLOW}[跳过]{Colors.NC} (未指定 --output)")
        
        self._log(f"\n  {Colors.GREEN}【第二大阶段: 实时监控】{Colors.NC}")
        self._log(f"    - 订阅 DiagnosticsV2 等话题")
        self._log(f"    - 每 3 秒输出完整诊断报告 (10个板块)")
        self._log(f"    - 持续运行直到 Ctrl+C 退出")
        self._log(f"    {Colors.RED}前提: 控制器必须正在运行!{Colors.NC}")
        
        self._log(f"\n{Colors.CYAN}{'='*70}{Colors.NC}")
        self._log(f"\n{Colors.YELLOW}按 Enter 开始诊断...{Colors.NC}")
        try:
            input()
        except KeyboardInterrupt:
            self._log("\n诊断取消")
            raise
    
    def run(self):
        """
        根据模式运行诊断
        
        使用 try-finally 确保日志文件在任何情况下都能正确关闭。
        """
        try:
            if self.mode == 'realtime':
                self.run_realtime()
            elif self.mode == 'tuning':
                self.run_tuning()
            elif self.mode == 'full':
                self.run_full()
        except KeyboardInterrupt:
            print("\n诊断结束")
        except Exception as e:
            print(f"\n错误: {e}")
            import traceback
            traceback.print_exc()
        finally:
            # 确保日志文件被关闭（如果子方法没有关闭的话）
            self._close_log()


# ============================================================================
# 主函数
# ============================================================================

def main():
    # 在Windows上设置控制台编码
    if sys.platform == 'win32':
        try:
            # 尝试设置控制台代码页为UTF-8
            import subprocess
            subprocess.call('chcp 65001', shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except:
            pass
    
    parser = argparse.ArgumentParser(
        description='统一诊断工具 v2.7 - 完整合并实时监控与系统调优',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
使用示例:
  # 实时监控模式 (需要控制器运行)
  rosrun controller_ros unified_diagnostics.py --mode realtime
  
  # 系统调优模式 (不需要控制器)
  rosrun controller_ros unified_diagnostics.py --mode tuning --output config.yaml
  
  # 底盘测试 (机器人会移动!)
  rosrun controller_ros unified_diagnostics.py --mode tuning --test-chassis --output config.yaml
  
  # 运行时调优 (需要控制器运行)
  rosrun controller_ros unified_diagnostics.py --mode tuning --runtime-tuning --duration 30
  
  # 完整诊断 (先调优后实时监控)
  rosrun controller_ros unified_diagnostics.py --mode full --duration 10
  
  # 完整诊断 + 底盘测试 + 运行时调优 (推荐)
  rosrun controller_ros unified_diagnostics.py --mode full --test-chassis --runtime-tuning --output tuned.yaml

================================================================================
完整模式 (--mode full) 执行流程:
================================================================================

【第一大阶段: 系统调优诊断】

  阶段1: 话题监控 (duration 秒)
         - 被动监听 odom/imu/trajectory 话题
         - 收集传感器频率、延迟、抖动数据
         前提: turtlebot_bringup + network trajectory

  阶段2: 底盘能力测试 (需要 --test-chassis)
         - 测试最大速度、加速度、角速度、响应时间
         - ⚠️ 机器人会移动! 需要用户确认
         前提: 周围空间安全

  阶段3: 控制器运行时诊断 (需要 --runtime-tuning)
         - 收集 MPC 求解时间、成功率、跟踪误差
         - 运行增强诊断分析
         前提: 控制器必须正在运行!

  阶段4: 计算推荐配置
         - 生成 15 个配置模块
         - 基于运行时数据调优 MPC 权重

  阶段5: 显示诊断结果
         - 传感器状态、轨迹特性、底盘特性
         - 推荐参数、优化建议

  阶段6: 生成配置文件 (需要 --output)
         - 输出优化后的配置文件

【第二大阶段: 实时监控】

  - 订阅 DiagnosticsV2 等话题
  - 每 3 秒输出完整诊断报告 (10个板块)
  - 持续运行直到 Ctrl+C 退出
  前提: 控制器必须正在运行!

================================================================================
前提条件说明:
================================================================================

  turtlebot_bringup:      roslaunch turtlebot_bringup minimal.launch
  trajectory_publisher:   神经网络轨迹发布节点 (发布 /nn/local_trajectory)
  controller_ros:         roslaunch controller_ros controller.launch

选项说明:
  --test-chassis    运行底盘能力测试 (机器人会移动!)
  --runtime-tuning  运行控制器运行时诊断 (需要控制器运行)
  --output FILE     生成优化配置文件
  --duration SEC    监控时长 (默认: 60秒)
"""
    )
    
    parser.add_argument('--mode', choices=['realtime', 'tuning', 'full'], default='full',
                        help='诊断模式: realtime/tuning/full (默认: realtime)')
    parser.add_argument('--odom-topic', default='/odom', help='里程计话题')
    parser.add_argument('--traj-topic', default='/nn/local_trajectory', help='轨迹话题')
    parser.add_argument('--imu-topic', default='/imu', help='IMU话题')
    parser.add_argument('--cmd-vel-topic', default='/mobile_base/commands/velocity', help='速度命令话题')
    parser.add_argument('--cmd-topic', default='/cmd_unified', help='UnifiedCmd话题')
    parser.add_argument('--diag-topic', default='/controller/diagnostics', help='诊断话题')
    parser.add_argument('--duration', type=float, default=60.0, help='监控时长 (秒，默认: 10)')
    parser.add_argument('--output', '-o', help='输出配置文件')
    parser.add_argument('--log-file', help='日志文件路径 (默认: /moshi/unified_diag.log)')
    parser.add_argument('--test-chassis', action='store_true', help='运行底盘测试 (机器人会移动!)')
    parser.add_argument('--runtime-tuning', action='store_true', help='运行控制器运行时诊断')
    parser.add_argument('--low-speed-thresh', type=float, default=0.1,
                        help='低速阈值 (m/s)，低于此速度时角速度wz被置为0 (默认: 0.1)')
    
    args = parser.parse_args()
    
    # 默认日志文件 - 所有模式都支持日志
    if args.log_file is None:
        # 使用跨平台的临时目录
        import tempfile
        temp_dir = tempfile.gettempdir()
        if args.mode == 'realtime':
            args.log_file = os.path.join(temp_dir, 'unified_diag_realtime.log')
        elif args.mode == 'tuning':
            args.log_file = os.path.join(temp_dir, 'unified_diag_tuning.log')
        elif args.mode == 'full':
            args.log_file = os.path.join(temp_dir, 'unified_diag_full.log')
    
    try:
        diag = UnifiedDiagnostics(args)
        diag.run()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("\n诊断结束")


if __name__ == '__main__':
    main()
