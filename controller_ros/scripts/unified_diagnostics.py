#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
统一诊断工具 v2.1 (Unified Diagnostics Tool)

完整合并 diagnose_trajectory.py v3.0 和 full_diagnostics.py 的所有功能。
此脚本是控制器诊断的唯一入口，其他诊断脚本已废弃并重定向到此处。

功能模式:
  1. realtime  - 实时监控模式
                 订阅 DiagnosticsV2，显示完整控制器内部状态（10个诊断板块）
                 需要控制器运行
  
  2. tuning    - 系统调优模式
                 传感器频率/延迟/抖动分析，底盘测试，完整配置生成（13个配置模块）
                 不需要控制器运行
  
  3. full      - 完整模式
                 先运行调优分析，再进入实时监控

诊断内容:
  [实时监控] 轨迹输入、控制输出、MPC健康、一致性、状态估计、跟踪误差、超时、紧急停止、坐标变换、问题汇总
  [系统调优] 传感器频率/延迟/抖动、底盘特性、轨迹质量(含曲率)、运行时调优建议、完整配置生成

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
  
  # 完整诊断
  rosrun controller_ros unified_diagnostics.py --mode full --duration 10

作者: Kiro Auto-generated
版本: 2.1 (统一诊断工具，替代 diagnose_trajectory.py 和 full_diagnostics.py)
"""
import sys
import os
import time
import threading
import argparse
import yaml
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

# ROS 导入
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
    try:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import Header
        from nav_msgs.msg import Odometry
        from geometry_msgs.msg import Twist, PoseStamped
        from nav_msgs.msg import Path
        from sensor_msgs.msg import Imu
        ROS_VERSION = 2
    except ImportError:
        print("错误: 未找到ROS环境")
        sys.exit(1)

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
            import re
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
    """从轨迹点计算 hard velocities，模拟 universal_controller 中的实现"""
    issues = []
    if len(points) < 2:
        return np.zeros((1, 4)), ["轨迹点数不足(<2)"]
    
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


def analyze_trajectory_msg(msg) -> TrajectoryAnalysis:
    """完整分析轨迹消息"""
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
    
    # 速度分析
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
    
    # Hard velocities
    # 注意: low_speed_thresh=0.1 与 universal_controller 中的默认配置一致
    # 这是诊断工具的独立分析，用于检测轨迹中可能导致 wz=0 的低速点
    hard_vels, hard_issues = compute_hard_velocities(points, result.dt_sec, 0.1)
    result.hard_wz_sum = np.sum(np.abs(hard_vels[:, 3]))
    result.hard_wz_max = np.max(np.abs(hard_vels[:, 3]))
    result.hard_wz_zero_count = np.sum(np.abs(hard_vels[:, 3]) < 1e-6)
    result.issues.extend(hard_issues)
    
    # 核心问题检测
    if abs(result.total_turn_deg) > 10 and result.hard_wz_sum < 0.1:
        result.issues.append(f"🔴 关键问题: 轨迹转向{result.total_turn_deg:.1f}°但hard_wz≈0 (可能是低速阈值问题)")
    
    if result.hard_wz_zero_count > result.num_points * 0.8:
        result.issues.append(f"🔴 {result.hard_wz_zero_count}/{result.num_points}个点的wz=0 (速度可能低于0.1m/s阈值)")
    
    low_speed_count = sum(1 for s in speeds if s < 0.1)
    if low_speed_count > len(speeds) * 0.5:
        result.issues.append(f"⚠️ {low_speed_count}/{len(speeds)}个点速度<0.1m/s，会导致wz被置0")
    
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
        try:
            self.sub = rospy.Subscriber(self.topic, self.msg_type, self._callback, queue_size=10)
            return True
        except:
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
    """分析轨迹特性（含曲率分析）"""
    
    def __init__(self, topic: str):
        if CUSTOM_MSG_AVAILABLE:
            super().__init__(topic, LocalTrajectoryV4)
        else:
            from rospy.msg import AnyMsg
            super().__init__(topic, AnyMsg)
        self.traj_info = {
            'num_points': 0, 'dt_sec': 0.1, 'has_velocities': False,
            'confidence': 0.9, 'frame_id': '', 'total_length': 0, 'max_curvature': 0,
        }
        self.point_counts = deque(maxlen=100)
        self.dt_values = deque(maxlen=100)
        
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
            
            # 计算轨迹长度和曲率
            if hasattr(msg, 'points') and len(msg.points) >= 2:
                points = [(p.x, p.y) for p in msg.points]
                length = sum(np.sqrt((points[i+1][0]-points[i][0])**2 + 
                                    (points[i+1][1]-points[i][1])**2) 
                            for i in range(len(points)-1))
                self.traj_info['total_length'] = length
                
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
        return stats


class ControllerDiagnosticsMonitor:
    """监控控制器诊断信息用于运行时调优"""
    
    def __init__(self, topic: str = '/controller/diagnostics'):
        self.topic = topic
        self.sub = None
        self.lock = threading.Lock()
        self.msg_count = 0
        self.mpc_solve_times = deque(maxlen=500)
        self.mpc_successes = deque(maxlen=500)
        self.mpc_kkt_residuals = deque(maxlen=500)
        self.lateral_errors = deque(maxlen=500)
        self.longitudinal_errors = deque(maxlen=500)
        self.heading_errors = deque(maxlen=500)
        self.alpha_values = deque(maxlen=500)
        self.states = deque(maxlen=500)
        self.backup_active_count = 0
        
    def start(self) -> bool:
        if not CUSTOM_MSG_AVAILABLE:
            return False
        try:
            self.sub = rospy.Subscriber(self.topic, DiagnosticsV2, self._callback, queue_size=10)
            return True
        except:
            return False
    
    def stop(self):
        if self.sub:
            self.sub.unregister()
            self.sub = None
    
    def _callback(self, msg):
        with self.lock:
            self.msg_count += 1
            self.mpc_solve_times.append(msg.mpc_solve_time_ms)
            self.mpc_successes.append(msg.mpc_success)
            self.mpc_kkt_residuals.append(msg.mpc_health_kkt_residual)
            self.lateral_errors.append(abs(msg.tracking_lateral_error))
            self.longitudinal_errors.append(abs(msg.tracking_longitudinal_error))
            self.heading_errors.append(abs(msg.tracking_heading_error))
            self.alpha_values.append(msg.consistency_alpha_soft)
            self.states.append(msg.state)
            if msg.backup_active:
                self.backup_active_count += 1
    
    def get_stats(self) -> Optional[Dict[str, Any]]:
        with self.lock:
            if self.msg_count < 10:
                return None
            mpc_times = list(self.mpc_solve_times)
            mpc_success_list = list(self.mpc_successes)
            return {
                'msg_count': self.msg_count,
                'mpc_solve_time_avg_ms': np.mean(mpc_times) if mpc_times else 0,
                'mpc_solve_time_max_ms': np.max(mpc_times) if mpc_times else 0,
                'mpc_solve_time_std_ms': np.std(mpc_times) if mpc_times else 0,
                'mpc_success_rate': np.mean(mpc_success_list) if mpc_success_list else 0,
                'mpc_kkt_residual_avg': np.mean(list(self.mpc_kkt_residuals)) if self.mpc_kkt_residuals else 0,
                'lateral_error_avg': np.mean(list(self.lateral_errors)) if self.lateral_errors else 0,
                'lateral_error_max': np.max(list(self.lateral_errors)) if self.lateral_errors else 0,
                'longitudinal_error_avg': np.mean(list(self.longitudinal_errors)) if self.longitudinal_errors else 0,
                'heading_error_avg': np.mean(list(self.heading_errors)) if self.heading_errors else 0,
                'alpha_avg': np.mean(list(self.alpha_values)) if self.alpha_values else 0,
                'alpha_min': np.min(list(self.alpha_values)) if self.alpha_values else 0,
                'backup_active_ratio': self.backup_active_count / self.msg_count if self.msg_count > 0 else 0,
            }


# ============================================================================
# 底盘测试器
# ============================================================================

class ChassisTestRunner:
    """运行底盘能力测试"""
    
    def __init__(self, cmd_topic: str, odom_analyzer: OdometryAnalyzer):
        self.cmd_topic = cmd_topic
        self.odom = odom_analyzer
        self.cmd_pub = None
        self.results = {}
        
    def setup(self):
        self.cmd_pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=1)
        time.sleep(0.5)
        
    def test_max_velocity(self, target_v: float = 1.0, duration: float = 3.0) -> float:
        """测试最大速度"""
        safe_print(f"  测试最大速度 (目标: {target_v} m/s)...")
        # 清空历史数据，只统计本次测试（线程安全）
        with self.odom.lock:
            self.odom.velocities.clear()
        time.sleep(0.2)  # 等待新数据
        
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
        safe_print(f"  测试加速度 (目标: {target_v} m/s)...")
        # 停止机器人
        self.cmd_pub.publish(Twist())
        time.sleep(1.0)
        
        # 清空所有历史数据（线程安全）
        with self.odom.lock:
            self.odom.velocities.clear()
            self.odom.accelerations.clear()
            self.odom.last_vel = None
            self.odom.last_time = None
        
        # 等待新数据开始收集
        time.sleep(0.2)
        
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
        safe_print(f"  测试角速度 (目标: {target_w} rad/s)...")
        # 清空历史数据，只统计本次测试（线程安全）
        with self.odom.lock:
            self.odom.velocities.clear()
        time.sleep(0.2)  # 等待新数据
        
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
        safe_print(f"  测试响应时间...")
        self.cmd_pub.publish(Twist())
        time.sleep(1.0)
        
        # 清空历史数据（线程安全）
        with self.odom.lock:
            self.odom.velocities.clear()
        time.sleep(0.2)  # 等待新数据
        
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
    """
    
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
        
        # 结果存储
        self.results = {}
        self.recommended = {}
        
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
                self.log_handle.write(text + '\n')
                self.log_handle.flush()
            except UnicodeEncodeError:
                # 如果写入失败，尝试只写入ASCII
                self.log_handle.write(text.encode('ascii', errors='replace').decode('ascii') + '\n')
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
            self.last_traj = analyze_trajectory_msg(msg)
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
            "                    统一控制器诊断报告 v2.1",
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
            # 检查降级状态持续时间 - 与 StateMachine 的 degraded_state_timeout (默认30s) 对应
            if self._state_start_time is not None:
                duration = time.time() - self._state_start_time
                if duration > 30:
                    self._log(f"  🔴 降级状态持续过长 ({format_duration(duration)} > 30s)!")
                elif duration > 10:
                    self._log(f"  ⚠️ 降级状态已持续 {format_duration(duration)}")
        if ControllerState.is_stopped(state):
            self._log(f"  ⚠️ 系统已停止或正在停止")
        
        self._log(f"  MPC成功: {d['mpc_success']}  |  备用激活: {d['backup_active']}  |  求解时间: {d['solve_time_ms']:.2f}ms")
        self._log(f"  KKT残差: {d['kkt_residual']:.6f}  |  条件数: {d['condition_number']:.2e}")
        self._log(f"  连续接近超时: {d['consecutive_near_timeout']}次  |  降级警告: {d['degradation_warning']}  |  可恢复: {d['can_recover']}")
        
        # 问题检测 - 使用 degradation_warning 字段而非硬编码阈值
        # MPC 健康监控器已经根据配置的阈值计算了 degradation_warning
        if d['degradation_warning']:
            self._log("  ⚠️ MPC降级警告 (求解时间/KKT残差/条件数超过配置阈值)")
        if d['solve_time_ms'] > 20:  # 仅在极端情况下额外警告
            self._log(f"  🔴 求解时间过长 ({d['solve_time_ms']:.1f}ms > 20ms)")
        if d['kkt_residual'] > 1e-3:
            self._log(f"  ⚠️ KKT残差较高 ({d['kkt_residual']:.6f} > 1e-3)")
        if d['condition_number'] > 1e8:
            self._log(f"  🔴 条件数过高 ({d['condition_number']:.2e} > 1e8)，数值不稳定!")
        if d['consecutive_near_timeout'] > 3:
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
        if d['alpha'] < 0.3:
            self._log(f"  🔴 Alpha过低({d['alpha']:.2f})，soft velocity几乎不生效!")
        elif d['alpha'] < 0.5:
            self._log(f"  ⚠️ Alpha较低({d['alpha']:.2f})，soft velocity权重小")
        if d['curvature_consistency'] < 0.5:
            self._log(f"  ⚠️ 曲率一致性低 ({d['curvature_consistency']:.2f})")
        if d['velocity_dir_consistency'] < 0.5:
            self._log(f"  ⚠️ 速度方向一致性低 ({d['velocity_dir_consistency']:.2f})")
        if d['temporal_smooth'] < 0.3:
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
        if d['covariance_norm'] > 1.0:
            self._log(f"  🔴 协方差范数过高 ({d['covariance_norm']:.2f})，估计不确定性大!")
        if d['innovation_norm'] > 0.5:
            self._log(f"  ⚠️ 新息范数较高 ({d['innovation_norm']:.2f})，测量与预测偏差大")
        if d['slip_probability'] > 0.3:
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
        
        # 跟踪误差阈值 - 与配置中的 tracking.lateral_thresh 等保持一致
        # 默认值: lateral=0.3m, longitudinal=0.5m, heading=0.5rad(~28.6°)
        lateral_warn = 0.2   # 警告阈值
        lateral_error = 0.3  # 严重阈值
        longitudinal_warn = 0.3
        heading_warn_deg = 20  # 警告阈值 (度)
        heading_error_deg = 30  # 严重阈值 (度)
        
        if abs(d['tracking_lateral_error']) > lateral_error:
            self._log(f"  🔴 横向误差过大 ({d['tracking_lateral_error']:.2f}m > {lateral_error}m)")
        elif abs(d['tracking_lateral_error']) > lateral_warn:
            self._log(f"  ⚠️ 横向误差较大 ({d['tracking_lateral_error']:.2f}m > {lateral_warn}m)")
        if abs(d['tracking_longitudinal_error']) > longitudinal_warn:
            self._log(f"  ⚠️ 纵向误差较大 ({d['tracking_longitudinal_error']:.2f}m)")
        if abs(np.degrees(d['tracking_heading_error'])) > heading_error_deg:
            self._log(f"  🔴 航向误差过大 ({np.degrees(d['tracking_heading_error']):.1f}° > {heading_error_deg}°)")
        elif abs(np.degrees(d['tracking_heading_error'])) > heading_warn_deg:
            self._log(f"  ⚠️ 航向误差较大 ({np.degrees(d['tracking_heading_error']):.1f}° > {heading_warn_deg}°)")
        if d['tracking_prediction_error'] > 0.5:
            self._log(f"  ⚠️ 预测误差较大 ({d['tracking_prediction_error']:.2f}m)")
    
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
        if d['timeout_odom']:
            self._log("  🔴 里程计超时!")
        elif d['last_odom_age_ms'] > 100:
            self._log(f"  ⚠️ 里程计数据较旧 ({d['last_odom_age_ms']:.0f}ms)")
        if d['timeout_traj']:
            self._log("  🔴 轨迹超时!")
        elif d['last_traj_age_ms'] > 200:
            self._log(f"  ⚠️ 轨迹数据较旧 ({d['last_traj_age_ms']:.0f}ms)")
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
        if not d['tf2_available']:
            self._log("  🔴 TF2不可用，使用fallback模式!")
        if not d['tf2_injected']:
            self._log("  ⚠️ TF2未注入到控制器")
        if d['fallback_duration_ms'] > 500:
            self._log(f"  🔴 TF2降级时间过长 ({d['fallback_duration_ms']:.0f}ms > 500ms)")
        elif d['fallback_duration_ms'] > 100:
            self._log(f"  ⚠️ TF2降级中 ({d['fallback_duration_ms']:.0f}ms)")
        if d['accumulated_drift'] > 0.1:
            self._log(f"  ⚠️ 累积漂移较大 ({d['accumulated_drift']:.3f}m)")
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
        
        # 诊断问题
        if self.last_diag:
            d = self.last_diag
            if d['emergency_stop']: issues.append("🔴 紧急停止已触发!")
            if not d['mpc_success'] and d['backup_active']: issues.append("🔴 MPC求解失败，备用控制器激活")
            if d['condition_number'] > 1e8: issues.append(f"🔴 MPC条件数过高 ({d['condition_number']:.2e})")
            if d['timeout_odom']: issues.append("🔴 里程计超时!")
            if d['timeout_traj_grace_exceeded']: issues.append("🔴 轨迹超时宽限期已过!")
            if d['covariance_norm'] > 1.0: issues.append(f"🔴 状态估计不确定性过高 (协方差范数={d['covariance_norm']:.2f})")
            if d['slip_probability'] > 0.5: issues.append(f"🔴 高打滑概率 ({d['slip_probability']:.0%})")
            if abs(d['tracking_lateral_error']) > 0.5: issues.append(f"🔴 横向跟踪误差过大 ({d['tracking_lateral_error']:.2f}m)")
            if not d['tf2_available'] and d['fallback_duration_ms'] > 1000: issues.append(f"🔴 TF2长时间不可用")
            if d['alpha'] < 0.3: warnings.append(f"⚠️ Alpha过低({d['alpha']:.2f})，soft velocity几乎不生效")
            if d['solve_time_ms'] > 10: warnings.append(f"⚠️ MPC求解时间较长 ({d['solve_time_ms']:.1f}ms)")
            if d['consecutive_near_timeout'] > 3: warnings.append(f"⚠️ 连续接近超时 {d['consecutive_near_timeout']} 次")
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
    
    def _run_topic_monitoring(self):
        """阶段1: 话题监控"""
        self._log(f"{Colors.BLUE}阶段1: 话题监控 ({self.args.duration}秒){Colors.NC}\n")
        
        self.monitors['odom'] = OdometryAnalyzer(self.topics['odom'])
        self.monitors['imu'] = TopicMonitor(self.topics['imu'], Imu)
        self.monitors['trajectory'] = TrajectoryMonitor(self.topics['trajectory'])
        
        for name, mon in self.monitors.items():
            if mon.start():
                self._log(f"  {Colors.GREEN}[OK]{Colors.NC} 订阅 {mon.topic}")
            else:
                self._log(f"  {Colors.RED}[FAIL]{Colors.NC} 无法订阅 {mon.topic}")
        
        self._log(f"\n  收集数据 {self.args.duration} 秒...")
        time.sleep(self.args.duration)
        
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
    
    def _run_chassis_tests(self):
        """阶段2: 底盘能力测试"""
        self._log(f"\n{Colors.BLUE}阶段2: 底盘能力测试{Colors.NC}\n")
        self._log(f"  {Colors.YELLOW}警告: 机器人会移动! 确保周围空间安全。{Colors.NC}")
        
        try:
            input("  按 Enter 开始测试 (Ctrl+C 跳过)...")
        except KeyboardInterrupt:
            self._log("\n  跳过底盘测试")
            return
        
        # 重新启动里程计监控器（阶段1已经停止）
        odom_monitor = OdometryAnalyzer(self.topics['odom'])
        
        try:
            if odom_monitor.start():
                self._log(f"  {Colors.GREEN}[OK]{Colors.NC} 重新订阅 {self.topics['odom']}")
            else:
                self._log(f"  {Colors.RED}[FAIL]{Colors.NC} 无法订阅里程计，跳过底盘测试")
                return
            
            # 等待里程计数据稳定
            self._log("  等待里程计数据...")
            time.sleep(1.0)
            
            tester = ChassisTestRunner(self.topics['cmd_vel'], odom_monitor)
            tester.setup()
            tester.test_max_velocity(target_v=0.5)
            tester.test_acceleration(target_v=0.3)
            tester.test_angular_velocity(target_w=1.0)
            tester.test_response_time(step_v=0.3)
            self.results['chassis_tests'] = tester.results
        finally:
            # 确保监控器被停止
            odom_monitor.stop()
    
    def _run_controller_diagnostics(self):
        """阶段3: 控制器运行时诊断"""
        self._log(f"\n{Colors.BLUE}阶段3: 控制器运行时诊断 ({self.args.duration}秒){Colors.NC}\n")
        
        if not CUSTOM_MSG_AVAILABLE:
            self._log(f"  {Colors.YELLOW}[WARN]{Colors.NC} controller_ros 消息不可用，跳过")
            return
        
        self.diag_monitor = ControllerDiagnosticsMonitor(self.topics['diagnostics'])
        if self.diag_monitor.start():
            self._log(f"  {Colors.GREEN}[OK]{Colors.NC} 订阅 {self.topics['diagnostics']}")
        else:
            self._log(f"  {Colors.RED}[FAIL]{Colors.NC} 无法订阅，控制器是否运行?")
            return
        
        self._log(f"\n  收集控制器诊断 {self.args.duration} 秒...")
        self._log(f"  {Colors.YELLOW}[INFO]{Colors.NC} 移动机器人以生成跟踪数据!")
        time.sleep(self.args.duration)
        
        controller_stats = self.diag_monitor.get_stats()
        self.diag_monitor.stop()
        
        if controller_stats:
            self.results['controller'] = controller_stats
            self._log(f"  {Colors.GREEN}[OK]{Colors.NC} 收集 {controller_stats['msg_count']} 条诊断消息")
        else:
            self._log(f"  {Colors.YELLOW}[WARN]{Colors.NC} 未收到控制器诊断数据")
    
    def _calculate_recommendations(self):
        """阶段4: 计算完整推荐配置（13个配置模块）"""
        self._log(f"\n{Colors.BLUE}阶段4: 计算推荐配置{Colors.NC}\n")
        
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
        
        # 控制频率
        if odom_rate >= 100: ctrl_freq = 50
        elif odom_rate >= 50: ctrl_freq = 40
        elif odom_rate >= 20: ctrl_freq = 20
        else: ctrl_freq = max(10, int(odom_rate / 2))
        
        # 安全检查：确保 ctrl_freq 不为0
        if ctrl_freq <= 0:
            self._log(f"  {Colors.RED}[ERROR]{Colors.NC} 控制频率计算错误，使用默认值 20 Hz")
            ctrl_freq = 20
        
        ctrl_period_ms = 1000 / ctrl_freq
        
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
        mpc_horizon = min(max(num_points - 1, 3), 30)
        traj_length = traj_info.get('total_length', 1.0)
        
        # EKF噪声因子
        odom_noise_factor = 1.0 + (odom_jitter / 50.0)
        
        # Lookahead
        lookahead = max(0.3, min(max_v * response_time * 2, 2.0)) if max_v > 0 else 0.5
        
        # ===== 1. System =====
        self.recommended['system'] = {
            'ctrl_freq': ctrl_freq,
            'platform': 'differential',
        }
        
        # ===== 2. Watchdog =====
        self.recommended['watchdog'] = {
            'odom_timeout_ms': int(3000 / max(odom_rate, 1)) if odom_rate > 0 else 500,
            'traj_timeout_ms': int(2000 / max(traj_rate, 1)) if traj_rate > 0 else 1000,
            'traj_grace_ms': int(1000 / max(traj_rate, 1)) if traj_rate > 0 else 500,
            'imu_timeout_ms': int(3000 / max(imu_rate, 1)) if imu_rate > 0 else -1,
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
        self.recommended['constraints'] = {
            'v_max': round(max_v * safety_margin, 2) if max_v > 0 else 0.5,
            'v_min': -0.2,
            'omega_max': round(max_w * safety_margin, 2) if max_w > 0 else 1.0,
            'omega_max_low': round(max_w * safety_margin * 0.5, 2) if max_w > 0 else 0.5,
            'a_max': round(max_a * safety_margin, 2) if max_a > 0 else 0.5,
            'alpha_max': round(max_alpha * safety_margin, 2) if max_alpha > 0 else 1.0,
            'v_low_thresh': 0.1,
        }
        
        # ===== 5. Safety =====
        self.recommended['safety'] = {
            'v_stop_thresh': 0.05,
            'vz_stop_thresh': 0.1,
            'stopping_timeout': 5.0,
            'emergency_decel': round(max_a * 1.5, 2) if max_a > 0 else 1.5,
            'low_speed': {
                'threshold': 0.1,
                'omega_limit': round(max_w * 0.5, 2) if max_w > 0 else 0.5,
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
        self.recommended['ekf'] = {
            'use_odom_orientation_fallback': imu_rate == 0,
            'imu_motion_compensation': imu_rate > 0,
            'process_noise': {
                'position': round(0.01 * odom_noise_factor, 4),
                'velocity': round(0.1 * odom_noise_factor, 3),
                'orientation': round(0.05 * odom_noise_factor, 4),
                'angular_velocity': 0.1,
            },
            'measurement_noise': {
                'odom_position': round(0.01 * odom_noise_factor, 4),
                'odom_velocity': round(0.1 * odom_noise_factor, 3),
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
                'kappa': 0.3,
                'velocity': 0.3,
                'temporal': 0.4,
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

    def _show_tuning_results(self):
        """显示调优结果和运行时调优建议"""
        self._log(f"\n{Colors.BLUE}{'='*70}")
        self._log("  诊断结果")
        self._log(f"{'='*70}{Colors.NC}\n")
        
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
            self._log(f"\n{Colors.CYAN}底盘特性 (从里程计):{Colors.NC}")
            self._log(f"  最大速度: {chassis.get('max_speed', 0):.2f} m/s")
            self._log(f"  最大vx: {chassis.get('max_vx', 0):.2f} m/s")
            self._log(f"  最大wz: {chassis.get('max_wz', 0):.2f} rad/s")
            if 'max_ax' in chassis:
                self._log(f"  最大加速度: {chassis.get('max_ax', 0):.2f} m/s^2")
                self._log(f"  最大角加速度: {chassis.get('max_alpha', 0):.2f} rad/s^2")
        
        # 底盘测试结果
        tests = self.results.get('chassis_tests', {})
        if tests:
            self._log(f"\n{Colors.CYAN}底盘测试结果:{Colors.NC}")
            self._log(f"  实测最大速度: {tests.get('max_velocity_achieved', 0):.2f} m/s")
            self._log(f"  实测最大加速度: {tests.get('max_acceleration', 0):.2f} m/s^2")
            self._log(f"  实测最大角速度: {tests.get('max_angular_velocity', 0):.2f} rad/s")
            self._log(f"  响应时间: {tests.get('response_time', 0):.3f} sec")
        
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
            
            # MPC 成功率
            if controller['mpc_success_rate'] < 0.9:
                self._log(f"  {Colors.RED}[CRITICAL]{Colors.NC} MPC成功率过低 ({controller['mpc_success_rate']*100:.0f}%)")
                self._log(f"    → 检查轨迹质量")
                self._log(f"    → 降低 mpc.horizon")
                self._log(f"    → 增加 mpc.solver.nlp_max_iter")
            elif controller['mpc_success_rate'] < 0.98:
                self._log(f"  {Colors.YELLOW}[WARN]{Colors.NC} MPC成功率可以更好")
            else:
                self._log(f"  {Colors.GREEN}[OK]{Colors.NC} MPC成功率良好 ({controller['mpc_success_rate']*100:.0f}%)")
            
            # 备用控制器使用
            if controller['backup_active_ratio'] > 0.1:
                self._log(f"  {Colors.YELLOW}[WARN]{Colors.NC} 备用控制器使用频繁 ({controller['backup_active_ratio']*100:.0f}%)")
                self._log(f"    → 检查MPC求解器健康")
                self._log(f"    → 验证轨迹一致性")
            
            # 跟踪误差
            if controller['lateral_error_avg'] > 0.1:
                self._log(f"  {Colors.YELLOW}[WARN]{Colors.NC} 横向跟踪误差较大 ({controller['lateral_error_avg']*100:.1f}cm)")
                self._log(f"    → 增加 mpc.weights.position (尝试 15-20)")
                self._log(f"    → 减小 mpc.weights.control_accel (尝试 0.1)")
            else:
                self._log(f"  {Colors.GREEN}[OK]{Colors.NC} 横向跟踪误差可接受")
            
            if controller['heading_error_avg'] > 0.3:
                self._log(f"  {Colors.YELLOW}[WARN]{Colors.NC} 航向误差较大 ({np.degrees(controller['heading_error_avg']):.1f}°)")
                self._log(f"    → 增加 mpc.weights.heading (尝试 8-10)")
            
            # Alpha (一致性)
            if controller['alpha_min'] < 0.3:
                self._log(f"  {Colors.YELLOW}[WARN]{Colors.NC} 检测到低alpha值 (min: {controller['alpha_min']:.2f})")
                self._log(f"    → 轨迹一致性较差")
                self._log(f"    → 检查网络输出质量")
    
    def _generate_config(self, output_file: str):
        """生成完整配置文件"""
        self._log(f"\n{Colors.BLUE}生成配置文件: {output_file}{Colors.NC}")
        
        # 构建完整配置
        config = {
            'system': self.recommended['system'],
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
            'watchdog': self.recommended['watchdog'],
            'mpc': self.recommended['mpc'],
            'constraints': self.recommended['constraints'],
            'safety': self.recommended['safety'],
            'consistency': self.recommended['consistency'],
            'ekf': self.recommended['ekf'],
            'transform': self.recommended['transform'],
            'transition': self.recommended['transition'],
            'backup': self.recommended['backup'],
            'tracking': self.recommended['tracking'],
            'cmd_vel_adapter': self.recommended['cmd_vel_adapter'],
            'diagnostics': {'publish_rate': 10},
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
    
    def run_realtime(self):
        """运行实时监控模式"""
        self._init_ros_node('unified_diagnostics_realtime')
        self._init_tf2()
        self._init_log()
        
        print("\n" + "="*80)
        print("         统一诊断工具 v2.1 - 实时监控模式")
        print("="*80)
        if self.log_file:
            print(f"\n日志文件: {self.log_file}")
        print("\n诊断内容:")
        print("  1. 轨迹输入分析    5. 状态估计器健康    9. 坐标变换状态")
        print("  2. 控制输出分析    6. 跟踪误差         10. 问题汇总")
        print("  3. MPC健康状态     7. 超时状态")
        print("  4. 一致性指标      8. 紧急停止")
        print("\n订阅话题:")
        
        if CUSTOM_MSG_AVAILABLE:
            rospy.Subscriber(self.topics['trajectory'], LocalTrajectoryV4, self._traj_callback, queue_size=10)
            print(f"  ✓ {self.topics['trajectory']} (LocalTrajectoryV4)")
            rospy.Subscriber(self.topics['diagnostics'], DiagnosticsV2, self._diag_callback, queue_size=10)
            print(f"  ✓ {self.topics['diagnostics']} (DiagnosticsV2)")
            rospy.Subscriber(self.topics['cmd_unified'], UnifiedCmd, self._cmd_callback, queue_size=10)
            print(f"  ✓ {self.topics['cmd_unified']} (UnifiedCmd)")
        else:
            print("  ✗ 自定义消息不可用")
        
        rospy.Subscriber(self.topics['odom'], Odometry, self._odom_callback, queue_size=10)
        print(f"  ✓ {self.topics['odom']} (Odometry)")
        rospy.Subscriber(self.topics['cmd_vel'], Twist, self._twist_callback, queue_size=10)
        print(f"  ✓ {self.topics['cmd_vel']} (Twist)")
        
        print("\n" + "-"*75)
        print("等待数据... 每3秒输出一次完整诊断报告")
        print("按 Ctrl+C 退出")
        print("-"*75 + "\n")
        
        try:
            rate = rospy.Rate(1)
            while not rospy.is_shutdown():
                rate.sleep()
        finally:
            self._close_log()
            if self.log_file:
                print(f"\n日志已保存到: {self.log_file}")
    
    def run_tuning(self):
        """运行系统调优模式"""
        self._init_ros_node('unified_diagnostics_tuning')
        self._init_log()  # 初始化日志文件
        
        try:
            self._log(f"\n{Colors.GREEN}{'='*70}")
            self._log("  统一诊断工具 v2.1 - 系统调优模式")
            self._log(f"{'='*70}{Colors.NC}\n")
            
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
        finally:
            self._close_log()  # 确保日志文件被关闭
            if self.log_file:
                print(f"\n日志已保存到: {self.log_file}")
    
    def run_full(self):
        """运行完整模式 - 先调优后实时监控"""
        self._init_ros_node('unified_diagnostics_full')
        
        print(f"\n{Colors.GREEN}{'='*70}")
        print("  统一诊断工具 v2.1 - 完整模式")
        print(f"{'='*70}{Colors.NC}\n")
        
        # ===== 第一阶段: 系统调优 =====
        print(f"{Colors.MAGENTA}=== 第一阶段: 系统调优诊断 ==={Colors.NC}\n")
        
        self._init_log()  # Initialize log for tuning phase
        
        try:
            self._run_topic_monitoring()
            
            if self.args.test_chassis:
                self._run_chassis_tests()
            
            if self.args.runtime_tuning:
                self._run_controller_diagnostics()
            
            self._calculate_recommendations()
            self._show_tuning_results()
            
            if self.args.output:
                self._generate_config(self.args.output)
        finally:
            self._close_log()  # Close log after tuning phase
            if self.log_file:
                print(f"\n第一阶段日志已保存到: {self.log_file}")
        
        # ===== 第二阶段: 实时监控 =====
        print(f"\n{Colors.MAGENTA}=== 第二阶段: 实时监控 ==={Colors.NC}")
        print("按 Enter 进入实时监控模式 (Ctrl+C 退出)...")
        try:
            input()
        except KeyboardInterrupt:
            print("\n诊断结束")
            return
        
        self._init_tf2()
        self._init_log()  # Reinitialize log for realtime phase
        
        if CUSTOM_MSG_AVAILABLE:
            rospy.Subscriber(self.topics['trajectory'], LocalTrajectoryV4, self._traj_callback, queue_size=10)
            rospy.Subscriber(self.topics['diagnostics'], DiagnosticsV2, self._diag_callback, queue_size=10)
            rospy.Subscriber(self.topics['cmd_unified'], UnifiedCmd, self._cmd_callback, queue_size=10)
        
        rospy.Subscriber(self.topics['odom'], Odometry, self._odom_callback, queue_size=10)
        rospy.Subscriber(self.topics['cmd_vel'], Twist, self._twist_callback, queue_size=10)
        
        print("\n等待数据... 每3秒输出一次完整诊断报告")
        print("按 Ctrl+C 退出\n")
        
        try:
            rate = rospy.Rate(1)
            while not rospy.is_shutdown():
                rate.sleep()
        finally:
            self._close_log()
            if self.log_file:
                print(f"\n日志已保存到: {self.log_file}")
    
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
        description='统一诊断工具 v2.1 - 完整合并实时监控与系统调优',
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
  
  # 完整诊断 + 底盘测试 + 运行时调优
  rosrun controller_ros unified_diagnostics.py --mode full --test-chassis --runtime-tuning --output tuned.yaml

模式说明:
  realtime  - 实时监控控制器内部状态 (10个诊断板块)
              需要: turtlebot_bringup + network + controller_ros
              
  tuning    - 系统调优，传感器分析，完整配置生成 (13个配置模块)
              需要: turtlebot_bringup + network trajectory
              
  full      - 完整诊断，先调优后实时监控
              需要: turtlebot_bringup + network + controller_ros

选项说明:
  --test-chassis    运行底盘能力测试 (机器人会移动!)
  --runtime-tuning  运行控制器运行时诊断 (需要控制器运行)
  --output FILE     生成优化配置文件
"""
    )
    
    parser.add_argument('--mode', choices=['realtime', 'tuning', 'full'], default='realtime',
                        help='诊断模式: realtime/tuning/full (默认: realtime)')
    parser.add_argument('--odom-topic', default='/odom', help='里程计话题')
    parser.add_argument('--traj-topic', default='/nn/local_trajectory', help='轨迹话题')
    parser.add_argument('--imu-topic', default='/imu', help='IMU话题')
    parser.add_argument('--cmd-vel-topic', default='/cmd_vel', help='速度命令话题')
    parser.add_argument('--cmd-topic', default='/cmd_unified', help='UnifiedCmd话题')
    parser.add_argument('--diag-topic', default='/controller/diagnostics', help='诊断话题')
    parser.add_argument('--duration', type=float, default=5.0, help='监控时长 (秒)')
    parser.add_argument('--output', '-o', help='输出配置文件')
    parser.add_argument('--log-file', help='日志文件路径 (默认: /tmp/unified_diag.log)')
    parser.add_argument('--test-chassis', action='store_true', help='运行底盘测试 (机器人会移动!)')
    parser.add_argument('--runtime-tuning', action='store_true', help='运行控制器运行时诊断')
    
    args = parser.parse_args()
    
    # 默认日志文件 - 所有模式都支持日志
    if args.log_file is None:
        if args.mode == 'realtime':
            args.log_file = '/tmp/unified_diag_realtime.log'
        elif args.mode == 'tuning':
            args.log_file = '/tmp/unified_diag_tuning.log'
        elif args.mode == 'full':
            args.log_file = '/tmp/unified_diag_full.log'
    
    try:
        diag = UnifiedDiagnostics(args)
        diag.run()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("\n诊断结束")


if __name__ == '__main__':
    main()
