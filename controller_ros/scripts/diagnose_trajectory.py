#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
完整轨迹跟踪诊断脚本

诊断"轨迹显示左转但实际走直线"问题的完整工具。
覆盖整个控制链路：轨迹输入 → 坐标变换 → MPC求解 → 控制输出

使用方法 (ROS1):
    rosrun controller_ros diagnose_trajectory.py
    
    # 指定话题
    rosrun controller_ros diagnose_trajectory.py _trajectory_topic:=/local_trajectory

作者: Kiro Auto-generated
"""
import sys
import time
import threading
import numpy as np
from collections import deque
from dataclasses import dataclass, field
from typing import Optional, List, Dict, Any, Tuple

# ROS 导入
try:
    import rospy
    from std_msgs.msg import Header
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Twist, PoseStamped
    from nav_msgs.msg import Path
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
    
    # 几何分析
    total_distance: float = 0.0
    total_turn_deg: float = 0.0
    avg_speed: float = 0.0
    min_speed: float = 0.0
    max_speed: float = 0.0
    
    # 角速度分析
    soft_wz_available: bool = False
    soft_wz_sum: float = 0.0
    soft_wz_max: float = 0.0
    hard_wz_sum: float = 0.0
    hard_wz_max: float = 0.0
    hard_wz_zero_count: int = 0
    
    # 问题
    issues: List[str] = field(default_factory=list)


@dataclass 
class ControlAnalysis:
    """控制输出分析"""
    vx: float = 0.0
    vy: float = 0.0
    omega: float = 0.0
    mpc_success: bool = False
    backup_active: bool = False
    solver_time_ms: float = 0.0


@dataclass
class TransformAnalysis:
    """坐标变换分析"""
    tf2_available: bool = False
    source_frame: str = ""
    target_frame: str = ""
    position: Tuple[float, float, float] = (0, 0, 0)
    yaw: float = 0.0
    tf_age_ms: float = 0.0


@dataclass
class StateAnalysis:
    """状态估计分析"""
    position: Tuple[float, float, float] = (0, 0, 0)
    velocity: Tuple[float, float, float] = (0, 0, 0)
    yaw: float = 0.0
    omega: float = 0.0


# ============================================================================
# 核心分析函数
# ============================================================================

def compute_hard_velocities(points: List[Tuple[float, float, float]], 
                            dt_sec: float, 
                            low_speed_thresh: float = 0.1) -> Tuple[np.ndarray, List[str]]:
    """
    从轨迹点计算 hard velocities
    完全模拟 universal_controller/core/data_types.py 中的 get_hard_velocities()
    """
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
        
        # 计算下一段速度（用于航向变化率）
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
        
        # 关键：低速阈值检查 - 这是问题的核心！
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


def analyze_trajectory_msg(msg) -> TrajectoryAnalysis:
    """完整分析轨迹消息"""
    result = TrajectoryAnalysis()
    
    # 基本信息
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
    
    # 提取点
    points = [(p.x, p.y, p.z) for p in msg.points]
    
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
    
    # 计算总转向角
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

    # Soft velocities 分析
    if result.soft_enabled and len(msg.velocities_flat) >= 4:
        if len(msg.velocities_flat) % 4 == 0:
            soft_vels = np.array(msg.velocities_flat).reshape(-1, 4)
            result.soft_wz_available = True
            result.soft_wz_sum = np.sum(np.abs(soft_vels[:, 3]))
            result.soft_wz_max = np.max(np.abs(soft_vels[:, 3]))
            
            # 检查soft_wz是否全为0
            if result.soft_wz_sum < 0.001 and abs(result.total_turn_deg) > 5:
                result.issues.append(f"⚠️ soft_wz全为0，但轨迹转向{result.total_turn_deg:.1f}°")
        else:
            result.issues.append(f"❌ velocities_flat长度({len(msg.velocities_flat)})不是4的倍数")
    elif result.soft_enabled:
        result.issues.append("⚠️ soft_enabled=True但无velocity数据")
    
    # Hard velocities 分析（关键！）
    hard_vels, hard_issues = compute_hard_velocities(points, result.dt_sec, 0.1)
    result.hard_wz_sum = np.sum(np.abs(hard_vels[:, 3]))
    result.hard_wz_max = np.max(np.abs(hard_vels[:, 3]))
    result.hard_wz_zero_count = np.sum(np.abs(hard_vels[:, 3]) < 1e-6)
    result.issues.extend(hard_issues)
    
    # 核心问题检测
    if abs(result.total_turn_deg) > 10 and result.hard_wz_sum < 0.1:
        result.issues.append(
            f"🔴 关键问题: 轨迹转向{result.total_turn_deg:.1f}°但hard_wz≈0 "
            f"(可能是低速阈值问题)"
        )
    
    if result.hard_wz_zero_count > result.num_points * 0.8:
        result.issues.append(
            f"🔴 {result.hard_wz_zero_count}/{result.num_points}个点的wz=0 "
            f"(速度可能低于0.1m/s阈值)"
        )
    
    # 低速警告
    low_speed_count = sum(1 for s in speeds if s < 0.1)
    if low_speed_count > len(speeds) * 0.5:
        result.issues.append(
            f"⚠️ {low_speed_count}/{len(speeds)}个点速度<0.1m/s，会导致wz被置0"
        )
    
    return result


# ============================================================================
# 诊断节点
# ============================================================================

class FullDiagnosticNode:
    """完整诊断节点"""
    
    def __init__(self):
        # 数据存储
        self.last_traj: Optional[TrajectoryAnalysis] = None
        self.last_odom: Optional[StateAnalysis] = None
        self.last_diag: Optional[Dict] = None
        self.last_cmd: Optional[ControlAnalysis] = None
        
        # 历史记录
        self.cmd_history = deque(maxlen=100)
        self.omega_history = deque(maxlen=100)
        
        # 计数器
        self.traj_count = 0
        self.cmd_count = 0
        self.diag_count = 0
        
        # TF2
        self.tf_buffer = None
        self.tf_listener = None
        
        # 锁
        self.lock = threading.Lock()
        
        # 诊断间隔
        self.last_full_report_time = 0
        self.report_interval = 3.0  # 秒
    
    def init_tf2(self):
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
    
    def get_transform(self, target_frame: str, source_frame: str) -> Optional[TransformAnalysis]:
        """获取坐标变换"""
        if self.tf_buffer is None:
            return None
        
        try:
            if ROS_VERSION == 1:
                trans = self.tf_buffer.lookup_transform(
                    target_frame, source_frame, rospy.Time(0), rospy.Duration(0.1)
                )
                result = TransformAnalysis()
                result.tf2_available = True
                result.source_frame = source_frame
                result.target_frame = target_frame
                result.position = (
                    trans.transform.translation.x,
                    trans.transform.translation.y,
                    trans.transform.translation.z
                )
                result.yaw = quaternion_to_yaw(trans.transform.rotation)
                return result
        except Exception as e:
            pass
        return None

    # ==================== 回调函数 ====================
    
    def traj_callback(self, msg):
        """轨迹回调"""
        with self.lock:
            self.traj_count += 1
            self.last_traj = analyze_trajectory_msg(msg)
            self._maybe_print_report()
    
    def odom_callback(self, msg):
        """里程计回调"""
        with self.lock:
            state = StateAnalysis()
            state.position = (
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z
            )
            state.velocity = (
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.linear.z
            )
            state.yaw = quaternion_to_yaw(msg.pose.pose.orientation)
            state.omega = msg.twist.twist.angular.z
            self.last_odom = state
    
    def diag_callback(self, msg):
        """诊断回调"""
        with self.lock:
            self.diag_count += 1
            self.last_diag = {
                'state': msg.state,
                'mpc_success': msg.mpc_success,
                'backup_active': msg.backup_active,
                'solve_time_ms': msg.solve_time_ms,
                'kkt_residual': msg.kkt_residual if hasattr(msg, 'kkt_residual') else 0,
                'alpha': msg.alpha if hasattr(msg, 'alpha') else 1.0,
                'tf2_status': msg.tf2_status if hasattr(msg, 'tf2_status') else 0,
            }
    
    def cmd_callback(self, msg):
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
    
    def twist_callback(self, msg):
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
    
    # ==================== 报告生成 ====================
    
    def _maybe_print_report(self):
        """检查是否需要打印报告"""
        now = time.time()
        if now - self.last_full_report_time >= self.report_interval:
            self.last_full_report_time = now
            self._print_full_report()
    
    def _print_full_report(self):
        """打印完整诊断报告"""
        print("\n" + "="*75)
        print("                    完整轨迹跟踪诊断报告")
        print("="*75)
        print(f"时间: {time.strftime('%H:%M:%S')}  |  轨迹#{self.traj_count}  |  命令#{self.cmd_count}")
        print("-"*75)
        
        # 1. 轨迹分析
        self._print_trajectory_section()
        
        # 2. 控制输出分析
        self._print_control_section()
        
        # 3. 诊断信息
        self._print_diagnostics_section()
        
        # 4. 坐标变换
        self._print_transform_section()
        
        # 5. 问题汇总
        self._print_issues_section()
        
        print("="*75 + "\n")
    
    def _print_trajectory_section(self):
        """打印轨迹分析"""
        print("\n【1. 轨迹输入分析】")
        
        if self.last_traj is None:
            print("  ❌ 未收到轨迹数据")
            return
        
        t = self.last_traj
        print(f"  坐标系: {t.frame_id}  |  点数: {t.num_points}  |  dt: {t.dt_sec}s")
        print(f"  模式: {t.mode}(0=TRACK,1=STOP,2=HOLD)  |  置信度: {t.confidence:.2f}")
        print(f"  soft_enabled: {t.soft_enabled}")
        print()
        print(f"  几何: 总距离={t.total_distance:.3f}m  总转向={t.total_turn_deg:.1f}°")
        print(f"  速度: min={t.min_speed:.3f} avg={t.avg_speed:.3f} max={t.max_speed:.3f} m/s")
        print()
        print(f"  角速度wz分析:")
        print(f"    Hard wz: sum={t.hard_wz_sum:.4f} max={t.hard_wz_max:.4f} 零值数={t.hard_wz_zero_count}/{t.num_points}")
        if t.soft_wz_available:
            print(f"    Soft wz: sum={t.soft_wz_sum:.4f} max={t.soft_wz_max:.4f}")
        else:
            print(f"    Soft wz: 不可用")

    def _print_control_section(self):
        """打印控制输出分析"""
        print("\n【2. 控制输出分析】")
        
        if self.last_cmd is None:
            print("  ❌ 未收到控制命令")
            return
        
        c = self.last_cmd
        print(f"  当前: vx={c.vx:.3f}m/s  vy={c.vy:.3f}m/s  omega={c.omega:.4f}rad/s")
        
        if len(self.omega_history) > 0:
            omegas = list(self.omega_history)
            avg_omega = np.mean(omegas)
            max_omega = max(abs(o) for o in omegas)
            nonzero_omega = sum(1 for o in omegas if abs(o) > 0.01)
            print(f"  历史({len(omegas)}条): avg_omega={avg_omega:.4f} max_omega={max_omega:.4f}")
            print(f"  非零omega数: {nonzero_omega}/{len(omegas)}")
            
            # 关键检测
            if self.last_traj and abs(self.last_traj.total_turn_deg) > 10 and max_omega < 0.05:
                print(f"  🔴 问题: 轨迹需转{self.last_traj.total_turn_deg:.1f}°但omega输出很小!")
    
    def _print_diagnostics_section(self):
        """打印诊断信息"""
        print("\n【3. 控制器诊断】")
        
        if self.last_diag is None:
            print("  ❌ 未收到诊断数据 (检查/controller/diagnostics话题)")
            return
        
        d = self.last_diag
        state_names = {0: 'INIT', 1: 'NORMAL', 2: 'STOPPING', 3: 'STOPPED', 4: 'ERROR'}
        state_name = state_names.get(d['state'], f"UNKNOWN({d['state']})")
        
        print(f"  状态: {state_name}")
        print(f"  MPC成功: {d['mpc_success']}  |  备用激活: {d['backup_active']}")
        print(f"  求解时间: {d['solve_time_ms']:.2f}ms  |  KKT残差: {d['kkt_residual']:.6f}")
        print(f"  Alpha(soft/hard混合): {d['alpha']:.3f}")
        
        tf_status_names = {0: 'OK', 1: 'FALLBACK', 2: 'CRITICAL'}
        tf_name = tf_status_names.get(d['tf2_status'], f"UNKNOWN({d['tf2_status']})")
        print(f"  TF2状态: {tf_name}")
        
        if not d['mpc_success']:
            print("  ⚠️ MPC求解失败，使用备用控制器")
        if d['backup_active']:
            print("  ⚠️ 备用控制器激活中")
        if d['alpha'] < 0.5:
            print(f"  ⚠️ Alpha={d['alpha']:.2f}较低，soft velocity权重小")
    
    def _print_transform_section(self):
        """打印坐标变换分析"""
        print("\n【4. 坐标变换分析】")
        
        # 尝试获取TF
        tf_result = self.get_transform('odom', 'base_link')
        if tf_result and tf_result.tf2_available:
            print(f"  TF2可用: base_link → odom")
            print(f"  位置: ({tf_result.position[0]:.3f}, {tf_result.position[1]:.3f})")
            print(f"  航向: {np.degrees(tf_result.yaw):.1f}°")
        else:
            print("  ⚠️ TF2不可用或查询失败")
        
        # 检查轨迹坐标系
        if self.last_traj:
            frame = self.last_traj.frame_id
            if frame in ['base_link', 'base_link_0']:
                print(f"  轨迹坐标系: {frame} (局部坐标系，需要TF变换)")
            elif frame in ['odom', 'map', 'world']:
                print(f"  轨迹坐标系: {frame} (全局坐标系，无需变换)")
            else:
                print(f"  ⚠️ 未知坐标系: {frame}")
    
    def _print_issues_section(self):
        """打印问题汇总"""
        print("\n【5. 问题汇总】")
        
        issues = []
        
        # 轨迹问题
        if self.last_traj and self.last_traj.issues:
            issues.extend(self.last_traj.issues)
        
        # 控制问题
        if self.last_traj and self.last_cmd:
            if abs(self.last_traj.total_turn_deg) > 15:
                if len(self.omega_history) > 0:
                    max_omega = max(abs(o) for o in self.omega_history)
                    if max_omega < 0.1:
                        issues.append(
                            f"🔴 核心问题: 轨迹转向{self.last_traj.total_turn_deg:.1f}°"
                            f"但输出omega最大仅{max_omega:.4f}rad/s"
                        )
        
        # 诊断问题
        if self.last_diag:
            if not self.last_diag['mpc_success']:
                issues.append("⚠️ MPC求解失败")
            if self.last_diag['alpha'] < 0.3:
                issues.append(f"⚠️ Alpha过低({self.last_diag['alpha']:.2f})，soft velocity几乎不生效")
        
        if issues:
            for issue in issues:
                print(f"  {issue}")
        else:
            print("  ✅ 未检测到明显问题")
        
        # 建议
        if any('低速' in str(i) or 'wz' in str(i) or 'omega' in str(i) for i in issues):
            print("\n【建议】")
            print("  1. 检查轨迹速度是否过低 (< 0.1 m/s)")
            print("  2. 尝试降低 trajectory.low_speed_thresh 配置 (如改为0.01)")
            print("  3. 检查轨迹消息中 velocities_flat 是否包含有效的wz数据")
            print("  4. 如果soft_enabled=True，检查网络输出的角速度是否正确")


# ============================================================================
# 主函数
# ============================================================================

def main_ros1():
    """ROS1主函数"""
    rospy.init_node('trajectory_full_diagnostics', anonymous=True)
    
    node = FullDiagnosticNode()
    node.init_tf2()
    
    print("\n" + "="*75)
    print("         轨迹跟踪完整诊断工具 v2.0 (ROS1)")
    print("="*75)
    print("\n订阅话题:")
    
    # 轨迹
    traj_topic = rospy.get_param('~trajectory_topic', '/local_trajectory')
    if CUSTOM_MSG_AVAILABLE:
        rospy.Subscriber(traj_topic, LocalTrajectoryV4, node.traj_callback, queue_size=10)
        print(f"  ✓ {traj_topic} (LocalTrajectoryV4)")
    else:
        print(f"  ✗ LocalTrajectoryV4 消息不可用")
    
    # 里程计
    odom_topic = rospy.get_param('~odom_topic', '/odom')
    rospy.Subscriber(odom_topic, Odometry, node.odom_callback, queue_size=10)
    print(f"  ✓ {odom_topic} (Odometry)")
    
    # 诊断
    diag_topic = rospy.get_param('~diagnostics_topic', '/controller/diagnostics')
    if CUSTOM_MSG_AVAILABLE:
        rospy.Subscriber(diag_topic, DiagnosticsV2, node.diag_callback, queue_size=10)
        print(f"  ✓ {diag_topic} (DiagnosticsV2)")
    
    # 控制命令
    cmd_topic = rospy.get_param('~cmd_topic', '/cmd_unified')
    if CUSTOM_MSG_AVAILABLE:
        rospy.Subscriber(cmd_topic, UnifiedCmd, node.cmd_callback, queue_size=10)
        print(f"  ✓ {cmd_topic} (UnifiedCmd)")
    
    # cmd_vel备选
    cmd_vel_topic = rospy.get_param('~cmd_vel_topic', '/cmd_vel')
    rospy.Subscriber(cmd_vel_topic, Twist, node.twist_callback, queue_size=10)
    print(f"  ✓ {cmd_vel_topic} (Twist)")
    
    print("\n" + "-"*75)
    print("等待数据... 每3秒输出一次完整诊断报告")
    print("按 Ctrl+C 退出")
    print("-"*75 + "\n")
    
    # 定时打印
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()


def main_ros2():
    """ROS2主函数"""
    rclpy.init()
    
    class DiagNode(Node):
        def __init__(self):
            super().__init__('trajectory_full_diagnostics')
            self.diag = FullDiagnosticNode()
            
            print("\n" + "="*75)
            print("         轨迹跟踪完整诊断工具 v2.0 (ROS2)")
            print("="*75)
            
            if CUSTOM_MSG_AVAILABLE:
                self.create_subscription(LocalTrajectoryV4, '/local_trajectory',
                                        self.diag.traj_callback, 10)
                self.create_subscription(UnifiedCmd, '/cmd_unified',
                                        self.diag.cmd_callback, 10)
                self.create_subscription(DiagnosticsV2, '/controller/diagnostics',
                                        self.diag.diag_callback, 10)
            
            self.create_subscription(Odometry, '/odom', self.diag.odom_callback, 10)
            self.create_subscription(Twist, '/cmd_vel', self.diag.twist_callback, 10)
            
            print("\n等待数据... 每3秒输出一次完整诊断报告")
            print("按 Ctrl+C 退出\n")
    
    node = DiagNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    try:
        if ROS_VERSION == 1:
            main_ros1()
        else:
            main_ros2()
    except KeyboardInterrupt:
        print("\n诊断结束")
    except Exception as e:
        print(f"\n错误: {e}")
        import traceback
        traceback.print_exc()
