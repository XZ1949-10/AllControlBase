#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
一键自动调优工具 v1.0

自动收集话题帧率和诊断数据，分析后直接生成调优配置文件。

使用方法:
  # 一键调优 (收集30秒数据后自动生成调优配置)
  python -m tools.tuning.auto_tune --duration 30
  
  # 指定输出文件
  python -m tools.tuning.auto_tune --duration 30 --output tuned_turtlebot1.yaml
  
  # 直接应用到配置文件
  python -m tools.tuning.auto_tune --duration 30 --apply

输出:
  - 调优后的配置文件 (默认: tuning_output/tuned_turtlebot1.yaml)
  - 详细的分析报告 (tuning_output/auto_tune_report.txt)
"""

import argparse
import time
import json
import yaml
import shutil
from pathlib import Path
from typing import Dict, List, Any, Optional, Tuple
from dataclasses import dataclass, field
from collections import defaultdict
import numpy as np
from datetime import datetime
import sys
import copy


# =============================================================================
# 配置常量
# =============================================================================
DEFAULT_CONFIG_PATH = "controller_ros/config/platforms/turtlebot1.yaml"
DEFAULT_OUTPUT_DIR = "tuning_output"

# 话题配置 - 基于 turtlebot1.yaml 和 controller_params.yaml
TOPICS_CONFIG = {
    'odom': {
        'topic': '/odom',
        'msg_type': 'nav_msgs/Odometry',
        'param': 'watchdog.odom_timeout_ms',
        'expected_hz': 50,
        'timeout_margin': 2.0,
    },
    'trajectory': {
        'topic': '/controller/input/trajectory',
        'msg_type': 'controller_ros/Trajectory',
        'param': 'watchdog.traj_timeout_ms',
        'expected_hz': 10,
        'timeout_margin': 2.0,
        'grace_param': 'watchdog.traj_grace_ms',
        'grace_margin': 1.5,
    },
    'imu': {
        'topic': '/mobile_base/sensors/imu_data',
        'msg_type': 'sensor_msgs/Imu',
        'param': 'watchdog.imu_timeout_ms',
        'expected_hz': 100,
        'timeout_margin': 2.0,
    },
    'diagnostics': {
        'topic': '/controller/diagnostics',
        'msg_type': 'controller_ros/DiagnosticsV2',
        'param': 'diagnostics.publish_rate',
        'expected_hz': 4,
        'is_output': True,
    },
}

TF_CONFIG = {
    'source_frame': 'base_footprint',
    'target_frame': 'odom',
    'param': 'transform.timeout_ms',
    'expected_hz': 50,
    'timeout_margin': 2.0,
}


# =============================================================================
# 数据类
# =============================================================================
@dataclass
class TopicStats:
    """话题统计"""
    topic_name: str
    message_count: int = 0
    timestamps: List[float] = field(default_factory=list)
    
    @property
    def duration(self) -> float:
        if len(self.timestamps) < 2:
            return 0.0
        return self.timestamps[-1] - self.timestamps[0]
    
    @property
    def avg_hz(self) -> float:
        if self.duration <= 0 or self.message_count < 2:
            return 0.0
        return (self.message_count - 1) / self.duration
    
    @property
    def intervals(self) -> List[float]:
        if len(self.timestamps) < 2:
            return []
        return [self.timestamps[i+1] - self.timestamps[i] 
                for i in range(len(self.timestamps)-1)]
    
    @property
    def p95_interval_ms(self) -> float:
        intervals = self.intervals
        if not intervals:
            return 0.0
        return float(np.percentile(intervals, 95)) * 1000
    
    @property
    def max_interval_ms(self) -> float:
        intervals = self.intervals
        if not intervals:
            return 0.0
        return max(intervals) * 1000


@dataclass
class DiagnosticsStats:
    """诊断数据统计"""
    samples: List[Dict[str, Any]] = field(default_factory=list)
    
    # MPC 统计
    mpc_solve_times: List[float] = field(default_factory=list)
    mpc_no_solve_count: int = 0
    mpc_success_count: int = 0
    mpc_fail_count: int = 0
    
    # MPC 健康监控
    mpc_consecutive_near_timeouts: List[int] = field(default_factory=list)
    mpc_degradation_warning_count: int = 0
    mpc_kkt_residuals: List[float] = field(default_factory=list)
    
    # 状态统计
    state_counts: Dict[int, int] = field(default_factory=lambda: defaultdict(int))
    
    # 超时统计
    odom_ages: List[float] = field(default_factory=list)
    traj_ages: List[float] = field(default_factory=list)
    imu_ages: List[float] = field(default_factory=list)
    traj_timeout_count: int = 0
    traj_grace_exceeded_count: int = 0
    odom_timeout_count: int = 0
    imu_timeout_count: int = 0
    
    # 跟踪误差
    lateral_errors: List[float] = field(default_factory=list)
    longitudinal_errors: List[float] = field(default_factory=list)
    heading_errors: List[float] = field(default_factory=list)
    prediction_errors: List[float] = field(default_factory=list)
    
    # 坐标变换
    tf_fallback_durations: List[float] = field(default_factory=list)
    tf_available_count: int = 0
    tf_unavailable_count: int = 0
    
    # 安全统计
    safety_failed_count: int = 0
    emergency_stop_count: int = 0
    v_saturated_count: int = 0
    omega_saturated_count: int = 0
    
    # 备份控制器
    backup_active_count: int = 0


@dataclass
class TuningChange:
    """调优变更"""
    param_path: str
    old_value: Any
    new_value: Any
    reason: str
    severity: str  # 'critical', 'warning', 'info'


# =============================================================================
# 自动调优器
# =============================================================================
class AutoTuner:
    """一键自动调优器"""
    
    STATE_NAMES = {
        0: 'INIT', 1: 'NORMAL', 2: 'SOFT_DISABLED',
        3: 'MPC_DEGRADED', 4: 'BACKUP_ACTIVE', 5: 'STOPPING', 6: 'STOPPED',
    }
    
    def __init__(self, config_path: str = DEFAULT_CONFIG_PATH):
        self.config_path = Path(config_path)
        self.config: Dict[str, Any] = {}
        self.topic_stats: Dict[str, TopicStats] = {}
        self.tf_stats: Optional[TopicStats] = None
        self.diag_stats = DiagnosticsStats()
        self.tuning_changes: List[TuningChange] = []
        self.report_lines: List[str] = []
        
        self._load_config()
    
    def _load_config(self):
        """加载配置文件"""
        if self.config_path.exists():
            with open(self.config_path, 'r', encoding='utf-8') as f:
                self.config = yaml.safe_load(f) or {}
            self._log(f"已加载配置: {self.config_path}")
        else:
            self._log(f"警告: 配置文件不存在 {self.config_path}")
    
    def _log(self, msg: str):
        """记录日志"""
        print(msg)
        self.report_lines.append(msg)
    
    def _get_param(self, path: str, default: Any = None) -> Any:
        """获取配置参数"""
        parts = path.split('.')
        value = self.config
        for part in parts:
            if isinstance(value, dict) and part in value:
                value = value[part]
            else:
                return default
        return value
    
    def _set_param(self, config: Dict, path: str, value: Any):
        """设置配置参数"""
        parts = path.split('.')
        for part in parts[:-1]:
            if part not in config:
                config[part] = {}
            config = config[part]
        config[parts[-1]] = value

    # =========================================================================
    # 数据收集
    # =========================================================================
    def collect_data(self, duration_sec: float) -> bool:
        """收集话题帧率和诊断数据"""
        try:
            import rospy
            from nav_msgs.msg import Odometry
            from sensor_msgs.msg import Imu
            import tf2_ros
        except ImportError:
            self._log("错误: 需要 ROS 环境，请确保已 source ROS 工作空间")
            return False
        
        # 尝试导入自定义消息
        try:
            from controller_ros.msg import LocalTrajectoryV4, DiagnosticsV2
            has_custom_msgs = True
            self._log("✓ controller_ros 消息已加载")
        except ImportError as e:
            self._log(f"警告: controller_ros 消息不可用 ({e})，部分功能受限")
            has_custom_msgs = False
        
        # 初始化 ROS 节点
        try:
            rospy.init_node('auto_tuner', anonymous=True)
        except rospy.exceptions.ROSException:
            pass
        
        # 订阅回调
        def odom_cb(msg):
            self._add_topic_sample('/odom', rospy.Time.now().to_sec())
        
        def imu_cb(msg):
            self._add_topic_sample('/mobile_base/sensors/imu_data', rospy.Time.now().to_sec())
        
        def traj_cb(msg):
            self._add_topic_sample('/controller/input/trajectory', rospy.Time.now().to_sec())
        
        def diag_cb(msg):
            self._add_topic_sample('/controller/diagnostics', rospy.Time.now().to_sec())
            self._process_diagnostics(msg)
        
        # 创建订阅者
        subs = []
        subs.append(rospy.Subscriber('/odom', Odometry, odom_cb))
        subs.append(rospy.Subscriber('/mobile_base/sensors/imu_data', Imu, imu_cb))
        
        if has_custom_msgs:
            subs.append(rospy.Subscriber('/controller/input/trajectory', LocalTrajectoryV4, traj_cb))
            subs.append(rospy.Subscriber('/controller/diagnostics', DiagnosticsV2, diag_cb))
        
        # TF2 监听
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)
        
        self._log(f"\n开始收集数据 ({duration_sec}秒)...")
        self._log("监控话题:")
        for key, cfg in TOPICS_CONFIG.items():
            self._log(f"  - {cfg['topic']}")
        self._log(f"  - TF2: {TF_CONFIG['source_frame']} → {TF_CONFIG['target_frame']}")
        self._log("-" * 50)
        
        start_time = time.time()
        rate = rospy.Rate(50)
        last_progress_time = 0
        
        while not rospy.is_shutdown():
            elapsed = time.time() - start_time
            if elapsed >= duration_sec:
                break
            
            # TF 采样
            try:
                tf_buffer.lookup_transform(
                    TF_CONFIG['target_frame'],
                    TF_CONFIG['source_frame'],
                    rospy.Time(0),
                    rospy.Duration(0.01)
                )
                self._add_tf_sample(rospy.Time.now().to_sec())
            except:
                pass
            
            # 进度显示 - 每 10 秒更新一次
            if elapsed - last_progress_time >= 10:
                progress = int(elapsed / duration_sec * 100)
                diag_count = len(self.diag_stats.samples)
                topic_counts = {k: v.message_count for k, v in self.topic_stats.items()}
                self._log(f"  进度: {progress}% ({int(elapsed)}秒) | 诊断消息: {diag_count} | 话题: {topic_counts}")
                last_progress_time = elapsed
            
            rate.sleep()
        
        # 清理
        for sub in subs:
            sub.unregister()
        
        self._log(f"\n数据收集完成!")
        return True
    
    def _add_topic_sample(self, topic: str, timestamp: float):
        """添加话题样本"""
        if topic not in self.topic_stats:
            self.topic_stats[topic] = TopicStats(topic_name=topic)
        stats = self.topic_stats[topic]
        stats.message_count += 1
        stats.timestamps.append(timestamp)
    
    def _add_tf_sample(self, timestamp: float):
        """添加 TF 样本"""
        if self.tf_stats is None:
            self.tf_stats = TopicStats(topic_name="TF2")
        self.tf_stats.message_count += 1
        self.tf_stats.timestamps.append(timestamp)

    def _process_diagnostics(self, msg):
        """处理诊断消息"""
        sample = {}
        
        # 基本信息
        sample['state'] = msg.state
        self.diag_stats.state_counts[msg.state] += 1
        
        # MPC 信息
        sample['mpc_solve_time_ms'] = msg.mpc_solve_time_ms
        if msg.mpc_solve_time_ms > 0:
            self.diag_stats.mpc_solve_times.append(msg.mpc_solve_time_ms)
        else:
            self.diag_stats.mpc_no_solve_count += 1
        
        # MPC 成功/失败
        if hasattr(msg, 'mpc_success'):
            if msg.mpc_success:
                self.diag_stats.mpc_success_count += 1
            else:
                self.diag_stats.mpc_fail_count += 1
        
        # MPC 健康监控
        if hasattr(msg, 'mpc_health_consecutive_near_timeout'):
            self.diag_stats.mpc_consecutive_near_timeouts.append(msg.mpc_health_consecutive_near_timeout)
        if hasattr(msg, 'mpc_health_degradation_warning') and msg.mpc_health_degradation_warning:
            self.diag_stats.mpc_degradation_warning_count += 1
        if hasattr(msg, 'mpc_health_kkt_residual'):
            self.diag_stats.mpc_kkt_residuals.append(msg.mpc_health_kkt_residual)
        
        # 备份控制器
        if hasattr(msg, 'backup_active') and msg.backup_active:
            self.diag_stats.backup_active_count += 1
        
        # 超时信息 - 使用顶层字段 (DiagnosticsV2.msg 格式)
        if hasattr(msg, 'timeout_last_odom_age_ms'):
            self.diag_stats.odom_ages.append(msg.timeout_last_odom_age_ms)
        if hasattr(msg, 'timeout_last_traj_age_ms'):
            self.diag_stats.traj_ages.append(msg.timeout_last_traj_age_ms)
        if hasattr(msg, 'timeout_last_imu_age_ms'):
            self.diag_stats.imu_ages.append(msg.timeout_last_imu_age_ms)
        if hasattr(msg, 'timeout_odom') and msg.timeout_odom:
            self.diag_stats.odom_timeout_count += 1
        if hasattr(msg, 'timeout_traj') and msg.timeout_traj:
            self.diag_stats.traj_timeout_count += 1
        if hasattr(msg, 'timeout_traj_grace_exceeded') and msg.timeout_traj_grace_exceeded:
            self.diag_stats.traj_grace_exceeded_count += 1
        if hasattr(msg, 'timeout_imu') and msg.timeout_imu:
            self.diag_stats.imu_timeout_count += 1
        
        # 跟踪误差 - 使用顶层字段
        if hasattr(msg, 'tracking_lateral_error'):
            self.diag_stats.lateral_errors.append(abs(msg.tracking_lateral_error))
        if hasattr(msg, 'tracking_longitudinal_error'):
            self.diag_stats.longitudinal_errors.append(abs(msg.tracking_longitudinal_error))
        if hasattr(msg, 'tracking_heading_error'):
            self.diag_stats.heading_errors.append(abs(msg.tracking_heading_error))
        if hasattr(msg, 'tracking_prediction_error'):
            self.diag_stats.prediction_errors.append(abs(msg.tracking_prediction_error))
        
        # 坐标变换
        if hasattr(msg, 'transform_tf2_available'):
            if msg.transform_tf2_available:
                self.diag_stats.tf_available_count += 1
            else:
                self.diag_stats.tf_unavailable_count += 1
        if hasattr(msg, 'transform_fallback_duration_ms') and msg.transform_fallback_duration_ms > 0:
            self.diag_stats.tf_fallback_durations.append(msg.transform_fallback_duration_ms)
        
        # 安全信息
        if hasattr(msg, 'safety_check_passed') and not msg.safety_check_passed:
            self.diag_stats.safety_failed_count += 1
        if hasattr(msg, 'emergency_stop') and msg.emergency_stop:
            self.diag_stats.emergency_stop_count += 1
        
        self.diag_stats.samples.append(sample)
    
    # =========================================================================
    # 分析与调优
    # =========================================================================
    def analyze_and_tune(self) -> List[TuningChange]:
        """分析数据并生成调优建议"""
        self.tuning_changes = []
        
        self._log("\n" + "=" * 60)
        self._log("数据分析")
        self._log("=" * 60)
        
        # 1. 分析话题帧率
        self._analyze_topic_rates()
        
        # 2. 分析诊断数据
        self._analyze_diagnostics()
        
        # 3. 生成调优建议
        self._generate_tuning()
        
        return self.tuning_changes
    
    def _analyze_topic_rates(self):
        """分析话题帧率"""
        self._log("\n【话题帧率分析】")
        
        for key, cfg in TOPICS_CONFIG.items():
            topic = cfg['topic']
            if topic in self.topic_stats:
                stats = self.topic_stats[topic]
                expected = cfg.get('expected_hz', 10)
                status = "✓" if stats.avg_hz >= expected * 0.5 else "⚠"
                self._log(f"  {status} {topic}")
                self._log(f"      帧率: {stats.avg_hz:.1f} Hz (期望 >= {expected} Hz)")
                self._log(f"      消息数: {stats.message_count}")
                self._log(f"      p95间隔: {stats.p95_interval_ms:.1f} ms")
            else:
                self._log(f"  ✗ {topic} - 未收到消息")
        
        # TF2
        if self.tf_stats and self.tf_stats.avg_hz > 0:
            self._log(f"\n  TF2 ({TF_CONFIG['source_frame']} → {TF_CONFIG['target_frame']})")
            self._log(f"      帧率: {self.tf_stats.avg_hz:.1f} Hz")
            self._log(f"      p95间隔: {self.tf_stats.p95_interval_ms:.1f} ms")

    def _analyze_diagnostics(self):
        """分析诊断数据"""
        stats = self.diag_stats
        total = len(stats.samples)
        
        if total == 0:
            self._log("\n【诊断数据】未收到诊断消息")
            return
        
        self._log(f"\n【诊断数据分析】(共 {total} 样本)")
        
        # 状态分布
        self._log("\n  状态分布:")
        for state, count in sorted(stats.state_counts.items()):
            name = self.STATE_NAMES.get(state, str(state))
            pct = count / total * 100
            self._log(f"    {name}: {count} ({pct:.1f}%)")
        
        # MPC 统计
        if stats.mpc_solve_times:
            avg_solve = np.mean(stats.mpc_solve_times)
            max_solve = np.max(stats.mpc_solve_times)
            p95_solve = np.percentile(stats.mpc_solve_times, 95)
            self._log(f"\n  MPC 求解时间: avg={avg_solve:.1f}ms, p95={p95_solve:.1f}ms, max={max_solve:.1f}ms")
        if stats.mpc_no_solve_count > 0:
            self._log(f"  MPC 未求解次数: {stats.mpc_no_solve_count}")
        if stats.mpc_success_count + stats.mpc_fail_count > 0:
            success_rate = stats.mpc_success_count / (stats.mpc_success_count + stats.mpc_fail_count) * 100
            self._log(f"  MPC 成功率: {success_rate:.1f}%")
        if stats.mpc_degradation_warning_count > 0:
            self._log(f"  MPC 降级警告: {stats.mpc_degradation_warning_count} 次")
        if stats.backup_active_count > 0:
            pct = stats.backup_active_count / total * 100
            self._log(f"  备份控制器激活: {stats.backup_active_count} ({pct:.1f}%)")
        
        # 超时统计
        if stats.traj_ages:
            avg_age = np.mean(stats.traj_ages)
            max_age = np.max(stats.traj_ages)
            p95_age = np.percentile(stats.traj_ages, 95)
            self._log(f"\n  轨迹延迟: avg={avg_age:.1f}ms, p95={p95_age:.1f}ms, max={max_age:.1f}ms")
        if stats.traj_timeout_count > 0:
            self._log(f"  轨迹超时次数: {stats.traj_timeout_count}")
        if stats.traj_grace_exceeded_count > 0:
            self._log(f"  宽限期超时次数: {stats.traj_grace_exceeded_count}")
        if stats.odom_timeout_count > 0:
            self._log(f"  里程计超时次数: {stats.odom_timeout_count}")
        
        # 坐标变换
        if stats.tf_fallback_durations:
            avg_fallback = np.mean(stats.tf_fallback_durations)
            max_fallback = np.max(stats.tf_fallback_durations)
            self._log(f"\n  TF 降级持续时间: avg={avg_fallback:.1f}ms, max={max_fallback:.1f}ms")
        if stats.tf_unavailable_count > 0:
            pct = stats.tf_unavailable_count / total * 100
            self._log(f"  TF 不可用: {stats.tf_unavailable_count} ({pct:.1f}%)")
        
        # 跟踪误差
        if stats.lateral_errors:
            self._log(f"\n  跟踪误差:")
            self._log(f"    横向: avg={np.mean(stats.lateral_errors)*100:.1f}cm, max={np.max(stats.lateral_errors)*100:.1f}cm")
            self._log(f"    纵向: avg={np.mean(stats.longitudinal_errors)*100:.1f}cm, max={np.max(stats.longitudinal_errors)*100:.1f}cm")
            self._log(f"    航向: avg={np.degrees(np.mean(stats.heading_errors)):.1f}°, max={np.degrees(np.max(stats.heading_errors)):.1f}°")
        if stats.prediction_errors:
            self._log(f"    预测: avg={np.mean(stats.prediction_errors)*100:.1f}cm")
        
        # 安全统计
        if stats.safety_failed_count > 0:
            pct = stats.safety_failed_count / total * 100
            self._log(f"\n  安全检查失败: {stats.safety_failed_count} ({pct:.1f}%)")
        if stats.emergency_stop_count > 0:
            self._log(f"  紧急停止: {stats.emergency_stop_count} 次")
    
    def _generate_tuning(self):
        """生成调优建议"""
        self._log("\n" + "=" * 60)
        self._log("调优建议")
        self._log("=" * 60)
        
        # 1. 基于话题帧率调优超时参数
        self._tune_timeouts()
        
        # 2. 基于诊断数据调优 MPC 参数
        self._tune_mpc()
        
        # 3. 基于跟踪误差调优权重
        self._tune_weights()
        
        # 4. 基于跟踪误差调优阈值
        self._tune_tracking_thresholds()
        
        # 5. 基于 TF 降级调优坐标变换参数
        self._tune_transform()
        
        # 6. 基于状态机统计调优状态机参数
        self._tune_state_machine()
        
        # 7. 低频轨迹专项优化
        self._tune_low_frequency_trajectory()
        
        if not self.tuning_changes:
            self._log("\n✅ 当前配置已经是最优，无需调整")

    def _tune_timeouts(self):
        """调优超时参数"""
        # Odom 超时
        odom_topic = '/odom'
        if odom_topic in self.topic_stats:
            stats = self.topic_stats[odom_topic]
            if stats.avg_hz > 0:
                cfg = TOPICS_CONFIG['odom']
                current = self._get_param(cfg['param'], 500)
                # 建议值 = max(p95间隔, 平均周期) * margin
                period_ms = 1000.0 / stats.avg_hz
                suggested = int(max(stats.p95_interval_ms, period_ms) * cfg['timeout_margin'])
                suggested = max(suggested, 100)  # 最小 100ms
                
                if current < suggested * 0.8:
                    self._add_change(cfg['param'], current, suggested,
                        f"odom 帧率 {stats.avg_hz:.1f}Hz, p95间隔 {stats.p95_interval_ms:.0f}ms",
                        'warning')
        
        # 轨迹超时
        traj_topic = '/controller/input/trajectory'
        if traj_topic in self.topic_stats:
            stats = self.topic_stats[traj_topic]
            if stats.avg_hz > 0:
                cfg = TOPICS_CONFIG['trajectory']
                
                # timeout
                current_timeout = self._get_param(cfg['param'], 1000)
                period_ms = 1000.0 / stats.avg_hz
                suggested_timeout = int(max(stats.p95_interval_ms, period_ms) * cfg['timeout_margin'])
                suggested_timeout = max(suggested_timeout, 500)  # 最小 500ms
                
                if current_timeout < suggested_timeout * 0.8:
                    self._add_change(cfg['param'], current_timeout, suggested_timeout,
                        f"轨迹帧率 {stats.avg_hz:.1f}Hz, p95间隔 {stats.p95_interval_ms:.0f}ms",
                        'warning')
                
                # grace
                current_grace = self._get_param(cfg['grace_param'], 600)
                suggested_grace = int(max(stats.p95_interval_ms, period_ms) * cfg['grace_margin'])
                suggested_grace = max(suggested_grace, 300)  # 最小 300ms
                
                if current_grace < suggested_grace * 0.8:
                    self._add_change(cfg['grace_param'], current_grace, suggested_grace,
                        f"轨迹帧率 {stats.avg_hz:.1f}Hz, 宽限期需要更长",
                        'warning')
        
        # 基于诊断数据的超时调优
        stats = self.diag_stats
        total = len(stats.samples)
        
        if total > 0:
            # 如果有轨迹超时，增加超时时间
            if stats.traj_timeout_count > 0:
                timeout_rate = stats.traj_timeout_count / total * 100
                if timeout_rate > 1:  # 超过 1% 超时率
                    current = self._get_param('watchdog.traj_timeout_ms', 1000)
                    if stats.traj_ages:
                        p99_age = np.percentile(stats.traj_ages, 99)
                        suggested = int(p99_age * 1.5)
                        suggested = max(suggested, current + 500)
                        self._add_change('watchdog.traj_timeout_ms', current, suggested,
                            f"轨迹超时率 {timeout_rate:.1f}%, p99延迟 {p99_age:.0f}ms",
                            'critical')
            
            # 如果有宽限期超时，增加宽限期
            if stats.traj_grace_exceeded_count > 0:
                grace_rate = stats.traj_grace_exceeded_count / total * 100
                if grace_rate > 1:
                    current = self._get_param('watchdog.traj_grace_ms', 600)
                    suggested = int(current * 1.5)
                    self._add_change('watchdog.traj_grace_ms', current, suggested,
                        f"宽限期超时率 {grace_rate:.1f}%",
                        'warning')
        
        # TF 超时
        if self.tf_stats and self.tf_stats.avg_hz > 0:
            current = self._get_param(TF_CONFIG['param'], 50)
            period_ms = 1000.0 / self.tf_stats.avg_hz
            suggested = int(max(self.tf_stats.p95_interval_ms, period_ms) * TF_CONFIG['timeout_margin'])
            suggested = max(suggested, 30)  # 最小 30ms
            
            if current < suggested * 0.8:
                self._add_change(TF_CONFIG['param'], current, suggested,
                    f"TF 帧率 {self.tf_stats.avg_hz:.1f}Hz",
                    'warning')

    def _tune_mpc(self):
        """调优 MPC 参数"""
        stats = self.diag_stats
        total = len(stats.samples)
        
        if total == 0:
            return
        
        # MPC 求解时间分析
        if stats.mpc_solve_times:
            avg_solve = np.mean(stats.mpc_solve_times)
            p95_solve = np.percentile(stats.mpc_solve_times, 95)
            p99_solve = np.percentile(stats.mpc_solve_times, 99)
            
            # 警告阈值
            current_warning = self._get_param('mpc.health_monitor.time_warning_thresh_ms', 20)
            if p95_solve > current_warning * 0.8:
                suggested = int(p95_solve * 1.5)
                self._add_change('mpc.health_monitor.time_warning_thresh_ms', 
                    current_warning, suggested,
                    f"MPC p95求解时间 {p95_solve:.1f}ms 接近警告阈值",
                    'info')
            
            # 临界阈值
            current_critical = self._get_param('mpc.health_monitor.time_critical_thresh_ms', 40)
            if p99_solve > current_critical * 0.8:
                suggested = int(p99_solve * 1.5)
                self._add_change('mpc.health_monitor.time_critical_thresh_ms',
                    current_critical, suggested,
                    f"MPC p99求解时间 {p99_solve:.1f}ms 接近临界阈值",
                    'warning')
        
        # MPC 连续超时分析
        if stats.mpc_consecutive_near_timeouts:
            max_consecutive = max(stats.mpc_consecutive_near_timeouts)
            current_limit = self._get_param('mpc.health_monitor.consecutive_warning_limit', 10)
            if max_consecutive > current_limit * 0.8:
                suggested = int(max_consecutive * 1.5)
                self._add_change('mpc.health_monitor.consecutive_warning_limit',
                    current_limit, suggested,
                    f"连续接近超时最大值 {max_consecutive} 接近限制",
                    'info')
        
        # MPC 成功率分析 - 调优 horizon
        if stats.mpc_success_count + stats.mpc_fail_count > 0:
            success_rate = stats.mpc_success_count / (stats.mpc_success_count + stats.mpc_fail_count)
            if success_rate < 0.9:  # 成功率低于 90%
                current_horizon = self._get_param('mpc.horizon', 7)
                if current_horizon > 4:
                    suggested = current_horizon - 1
                    self._add_change('mpc.horizon', current_horizon, suggested,
                        f"MPC 成功率 {success_rate*100:.1f}% 较低，减小预测时域",
                        'warning')
        
        # MPC 降级率分析
        degraded_count = stats.state_counts.get(3, 0)  # MPC_DEGRADED
        if degraded_count > 0:
            degraded_rate = degraded_count / total * 100
            if degraded_rate > 5:
                # 增加恢复容错
                current = self._get_param('safety.state_machine.mpc_recovery_tolerance', 1)
                suggested = min(current + 1, 3)
                if suggested > current:
                    self._add_change('safety.state_machine.mpc_recovery_tolerance',
                        current, suggested,
                        f"MPC 降级率 {degraded_rate:.1f}%，增加恢复容错",
                        'warning')
    
    def _tune_weights(self):
        """调优 MPC 权重"""
        stats = self.diag_stats
        
        if not stats.lateral_errors or not stats.longitudinal_errors:
            return
        
        avg_lateral = np.mean(stats.lateral_errors)
        avg_longitudinal = np.mean(stats.longitudinal_errors)
        avg_heading = np.mean(stats.heading_errors) if stats.heading_errors else 0
        
        # 纵向误差过大，增加速度权重
        if avg_longitudinal > 0.5:  # > 50cm
            current = self._get_param('mpc.weights.velocity', 6.0)
            # 误差越大，权重增加越多
            factor = min(avg_longitudinal / 0.3, 1.5)  # 最多增加 50%
            suggested = round(current * factor, 1)
            if suggested > current:
                self._add_change('mpc.weights.velocity', current, suggested,
                    f"纵向误差 avg={avg_longitudinal*100:.1f}cm 过大",
                    'warning')
        
        # 横向误差过大，增加位置权重
        if avg_lateral > 0.15:  # > 15cm
            current = self._get_param('mpc.weights.position', 15.0)
            factor = min(avg_lateral / 0.1, 1.3)
            suggested = round(current * factor, 1)
            if suggested > current:
                self._add_change('mpc.weights.position', current, suggested,
                    f"横向误差 avg={avg_lateral*100:.1f}cm 过大",
                    'warning')
        
        # 航向误差过大，增加航向权重
        if avg_heading > 0.2:  # > ~11°
            current = self._get_param('mpc.weights.heading', 8.0)
            factor = min(avg_heading / 0.15, 1.3)
            suggested = round(current * factor, 1)
            if suggested > current:
                self._add_change('mpc.weights.heading', current, suggested,
                    f"航向误差 avg={np.degrees(avg_heading):.1f}° 过大",
                    'warning')
    
    def _add_change(self, param: str, old_val: Any, new_val: Any, reason: str, severity: str):
        """添加调优变更"""
        # 检查是否已存在相同参数的变更
        for change in self.tuning_changes:
            if change.param_path == param:
                # 保留更激进的变更
                if severity == 'critical' or new_val > change.new_value:
                    change.new_value = new_val
                    change.reason = reason
                    change.severity = severity
                return
        
        change = TuningChange(param, old_val, new_val, reason, severity)
        self.tuning_changes.append(change)
        
        severity_icon = {'critical': '🔴', 'warning': '🟡', 'info': '🔵'}
        self._log(f"\n{severity_icon.get(severity, '•')} {param}")
        self._log(f"    当前值: {old_val} → 建议值: {new_val}")
        self._log(f"    原因: {reason}")
    
    def _tune_tracking_thresholds(self):
        """调优跟踪误差阈值"""
        stats = self.diag_stats
        
        if not stats.lateral_errors:
            return
        
        # 横向误差阈值
        p95_lateral = np.percentile(stats.lateral_errors, 95)
        current_lateral_thresh = self._get_param('tracking.lateral_thresh', 0.25)
        if p95_lateral > current_lateral_thresh * 0.8:
            suggested = round(p95_lateral * 1.3, 2)
            self._add_change('tracking.lateral_thresh', current_lateral_thresh, suggested,
                f"横向误差 p95={p95_lateral*100:.1f}cm 接近阈值",
                'info')
        
        # 纵向误差阈值
        if stats.longitudinal_errors:
            p95_longitudinal = np.percentile(stats.longitudinal_errors, 95)
            current_longitudinal_thresh = self._get_param('tracking.longitudinal_thresh', 0.6)
            if p95_longitudinal > current_longitudinal_thresh * 0.8:
                suggested = round(p95_longitudinal * 1.3, 2)
                self._add_change('tracking.longitudinal_thresh', current_longitudinal_thresh, suggested,
                    f"纵向误差 p95={p95_longitudinal*100:.1f}cm 接近阈值",
                    'info')
        
        # 航向误差阈值
        if stats.heading_errors:
            p95_heading = np.percentile(stats.heading_errors, 95)
            current_heading_thresh = self._get_param('tracking.heading_thresh', 0.5)
            if p95_heading > current_heading_thresh * 0.8:
                suggested = round(p95_heading * 1.3, 2)
                self._add_change('tracking.heading_thresh', current_heading_thresh, suggested,
                    f"航向误差 p95={np.degrees(p95_heading):.1f}° 接近阈值",
                    'info')
        
        # 预测误差阈值
        if stats.prediction_errors:
            p95_prediction = np.percentile(stats.prediction_errors, 95)
            current_prediction_thresh = self._get_param('tracking.prediction_thresh', 0.5)
            if p95_prediction > current_prediction_thresh * 0.8:
                suggested = round(p95_prediction * 1.3, 2)
                self._add_change('tracking.prediction_thresh', current_prediction_thresh, suggested,
                    f"预测误差 p95={p95_prediction*100:.1f}cm 接近阈值",
                    'info')
    
    def _tune_transform(self):
        """调优坐标变换参数"""
        stats = self.diag_stats
        
        if not stats.tf_fallback_durations:
            return
        
        p95_fallback = np.percentile(stats.tf_fallback_durations, 95)
        max_fallback = np.max(stats.tf_fallback_durations)
        
        # 降级警告阈值
        current_limit = self._get_param('transform.fallback_duration_limit_ms', 500)
        if p95_fallback > current_limit * 0.8:
            suggested = int(p95_fallback * 1.5)
            self._add_change('transform.fallback_duration_limit_ms', current_limit, suggested,
                f"TF 降级持续时间 p95={p95_fallback:.0f}ms 接近警告阈值",
                'warning')
        
        # 降级临界阈值
        current_critical = self._get_param('transform.fallback_critical_limit_ms', 1000)
        if max_fallback > current_critical * 0.8:
            suggested = int(max_fallback * 1.5)
            self._add_change('transform.fallback_critical_limit_ms', current_critical, suggested,
                f"TF 降级持续时间 max={max_fallback:.0f}ms 接近临界阈值",
                'warning')
    
    def _tune_state_machine(self):
        """调优状态机参数"""
        stats = self.diag_stats
        total = len(stats.samples)
        
        if total == 0:
            return
        
        # MPC 失败率分析
        if stats.mpc_success_count + stats.mpc_fail_count > 0:
            fail_rate = stats.mpc_fail_count / (stats.mpc_success_count + stats.mpc_fail_count)
            current_thresh = self._get_param('safety.state_machine.mpc_fail_ratio_thresh', 0.5)
            
            # 如果实际失败率接近阈值，适当放宽
            if fail_rate > current_thresh * 0.7 and fail_rate < current_thresh:
                suggested = round(fail_rate * 1.3, 2)
                suggested = min(suggested, 0.7)  # 不超过 70%
                self._add_change('safety.state_machine.mpc_fail_ratio_thresh',
                    current_thresh, suggested,
                    f"MPC 失败率 {fail_rate*100:.1f}% 接近阈值",
                    'info')
        
        # 备份控制器激活率分析
        backup_count = stats.state_counts.get(4, 0)  # BACKUP_ACTIVE
        if backup_count > 0:
            backup_rate = backup_count / total * 100
            if backup_rate > 10:  # 备份激活率超过 10%
                # 可能需要调整 MPC 恢复阈值
                current_recovery = self._get_param('safety.state_machine.mpc_recovery_thresh', 5)
                suggested = max(current_recovery - 1, 3)
                if suggested < current_recovery:
                    self._add_change('safety.state_machine.mpc_recovery_thresh',
                        current_recovery, suggested,
                        f"备份控制器激活率 {backup_rate:.1f}% 过高，降低恢复阈值",
                        'warning')

    def _tune_low_frequency_trajectory(self):
        """低频轨迹专项优化
        
        当轨迹频率低于 5Hz 时，自动调优以下参数:
        - consistency.temporal_window_size: 减少历史窗口，加快响应
        - backup.lookahead_dist: 增加前瞻距离补偿延迟
        - backup.min_lookahead: 配合增加的前瞻距离
        - backup.max_lookahead: 允许更大动态前瞻
        - safety.state_machine.mpc_fail_thresh: 放宽失败阈值
        """
        # 检查轨迹频率
        traj_topic = '/controller/input/trajectory'
        if traj_topic not in self.topic_stats:
            return
        
        traj_stats = self.topic_stats[traj_topic]
        traj_hz = traj_stats.avg_hz
        
        # 只有当轨迹频率低于 5Hz 时才进行低频优化
        LOW_FREQ_THRESHOLD = 5.0
        if traj_hz <= 0 or traj_hz >= LOW_FREQ_THRESHOLD:
            return
        
        self._log(f"\n【低频轨迹专项优化】(轨迹频率: {traj_hz:.1f}Hz < {LOW_FREQ_THRESHOLD}Hz)")
        
        # 计算轨迹周期 (ms)
        traj_period_ms = 1000.0 / traj_hz
        
        # 1. 一致性检查时序窗口
        # 低频轨迹下，减少历史窗口以加快对轨迹变化的响应
        # 目标: 保持约 2-3 秒的历史数据
        current_window = self._get_param('consistency.temporal_window_size', 10)
        target_history_sec = 2.5  # 目标历史时间
        suggested_window = max(int(target_history_sec * traj_hz), 4)  # 最小 4 个样本
        
        if current_window > suggested_window * 1.3:
            self._add_change('consistency.temporal_window_size', current_window, suggested_window,
                f"低频轨迹({traj_hz:.1f}Hz)下减少历史窗口到~{target_history_sec}秒",
                'info')
        
        # 2. 备份控制器前瞻距离
        # 低频轨迹下，需要更大的前瞻距离来补偿轨迹更新延迟
        # 前瞻距离应该能覆盖至少 1-2 个轨迹周期的行驶距离
        current_lookahead = self._get_param('backup.lookahead_dist', 0.5)
        v_max = self._get_param('constraints.v_max', 0.5)
        
        # 计算建议的前瞻距离: 至少覆盖 1.5 个轨迹周期
        min_lookahead_for_freq = v_max * (traj_period_ms / 1000.0) * 1.5
        suggested_lookahead = max(current_lookahead, round(min_lookahead_for_freq + 0.2, 1))
        suggested_lookahead = min(suggested_lookahead, 1.5)  # 不超过 1.5m
        
        if suggested_lookahead > current_lookahead * 1.2:
            self._add_change('backup.lookahead_dist', current_lookahead, suggested_lookahead,
                f"低频轨迹({traj_hz:.1f}Hz)需要更大前瞻距离补偿延迟",
                'info')
            
            # 同步调整 min_lookahead 和 max_lookahead
            current_min = self._get_param('backup.min_lookahead', 0.3)
            suggested_min = round(suggested_lookahead * 0.6, 1)
            if suggested_min > current_min:
                self._add_change('backup.min_lookahead', current_min, suggested_min,
                    f"配合增加的前瞻距离",
                    'info')
            
            current_max = self._get_param('backup.max_lookahead', 1.5)
            suggested_max = round(suggested_lookahead * 2.5, 1)
            suggested_max = min(suggested_max, 3.0)  # 不超过 3m
            if suggested_max > current_max:
                self._add_change('backup.max_lookahead', current_max, suggested_max,
                    f"允许更大的动态前瞻范围",
                    'info')
        
        # 3. MPC 失败阈值
        # 低频轨迹下，MPC 可能更容易因为轨迹点不足而"失败"
        # 适当放宽失败阈值，减少不必要的备份控制器切换
        stats = self.diag_stats
        total = len(stats.samples)
        
        if total > 0:
            # 检查 MPC 降级和备份激活情况
            degraded_count = stats.state_counts.get(3, 0)  # MPC_DEGRADED
            backup_count = stats.state_counts.get(4, 0)  # BACKUP_ACTIVE
            
            # 如果降级或备份激活率较高，放宽失败阈值
            problem_rate = (degraded_count + backup_count) / total * 100
            
            if problem_rate > 3:  # 超过 3% 的时间处于非正常状态
                current_fail_thresh = self._get_param('safety.state_machine.mpc_fail_thresh', 3)
                suggested_fail_thresh = min(current_fail_thresh + 1, 5)  # 最多增加到 5
                
                if suggested_fail_thresh > current_fail_thresh:
                    self._add_change('safety.state_machine.mpc_fail_thresh',
                        current_fail_thresh, suggested_fail_thresh,
                        f"低频轨迹下放宽MPC失败阈值(降级+备份率{problem_rate:.1f}%)",
                        'info')
        
        # 4. 轨迹低速阈值
        # 低频轨迹下，可能需要更低的低速阈值来避免误判
        current_low_speed = self._get_param('trajectory.low_speed_thresh', 0.05)
        if traj_hz < 3:  # 非常低的频率
            suggested_low_speed = 0.03
            if current_low_speed > suggested_low_speed:
                self._add_change('trajectory.low_speed_thresh', current_low_speed, suggested_low_speed,
                    f"极低频轨迹({traj_hz:.1f}Hz)下降低低速阈值",
                    'info')

    # =========================================================================
    # 输出
    # =========================================================================
    def generate_tuned_config(self, output_path: str = None) -> str:
        """生成调优后的配置文件"""
        if not self.tuning_changes:
            self._log("\n无需生成新配置，当前配置已是最优")
            return None
        
        # 复制原配置
        tuned_config = copy.deepcopy(self.config)
        
        # 应用变更
        for change in self.tuning_changes:
            self._set_param(tuned_config, change.param_path, change.new_value)
        
        # 更新调优记录
        now = datetime.now().strftime("%Y-%m-%d %H:%M")
        
        # 确定输出路径
        if output_path is None:
            output_dir = Path(DEFAULT_OUTPUT_DIR)
            output_dir.mkdir(exist_ok=True)
            output_path = output_dir / "tuned_turtlebot1.yaml"
        else:
            output_path = Path(output_path)
            output_path.parent.mkdir(parents=True, exist_ok=True)
        
        # 生成 YAML 内容
        yaml_content = self._generate_yaml_with_comments(tuned_config, now)
        
        with open(output_path, 'w', encoding='utf-8') as f:
            f.write(yaml_content)
        
        self._log(f"\n✅ 调优配置已保存到: {output_path}")
        return str(output_path)
    
    def _generate_yaml_with_comments(self, config: Dict, timestamp: str) -> str:
        """生成带注释的 YAML"""
        # 读取原文件作为模板
        if self.config_path.exists():
            with open(self.config_path, 'r', encoding='utf-8') as f:
                original_content = f.read()
        else:
            original_content = ""
        
        # 生成调优记录头
        header = f"""# =============================================================================
# 自动调优配置 - 由 auto_tune.py 生成
# 生成时间: {timestamp}
# =============================================================================
# 
# 调优变更:
"""
        for change in self.tuning_changes:
            header += f"#   - {change.param_path}: {change.old_value} → {change.new_value}\n"
            header += f"#     原因: {change.reason}\n"
        
        header += "#\n# =============================================================================\n\n"
        
        # 生成 YAML
        yaml_str = yaml.dump(config, default_flow_style=False, allow_unicode=True, sort_keys=False)
        
        return header + yaml_str
    
    def apply_to_config(self) -> bool:
        """直接应用调优到原配置文件"""
        if not self.tuning_changes:
            self._log("\n无需应用，当前配置已是最优")
            return False
        
        # 备份原文件
        backup_path = self.config_path.with_suffix('.yaml.bak')
        shutil.copy(self.config_path, backup_path)
        self._log(f"\n已备份原配置到: {backup_path}")
        
        # 读取原文件
        with open(self.config_path, 'r', encoding='utf-8') as f:
            content = f.read()
        
        # 逐个替换参数值
        for change in self.tuning_changes:
            content = self._replace_param_in_yaml(content, change)
        
        # 写回文件
        with open(self.config_path, 'w', encoding='utf-8') as f:
            f.write(content)
        
        self._log(f"✅ 调优已应用到: {self.config_path}")
        return True
    
    def _replace_param_in_yaml(self, content: str, change: TuningChange) -> str:
        """在 YAML 内容中替换参数值"""
        import re
        
        # 获取参数名 (最后一部分)
        param_name = change.param_path.split('.')[-1]
        
        # 构建正则表达式匹配参数行
        # 匹配: param_name: value 或 param_name: value # comment
        pattern = rf'(\s*{param_name}:\s*)({re.escape(str(change.old_value))})(\s*(?:#.*)?$)'
        
        def replacer(match):
            return f"{match.group(1)}{change.new_value}{match.group(3)}"
        
        new_content = re.sub(pattern, replacer, content, flags=re.MULTILINE)
        
        return new_content
    
    def save_report(self, output_path: str = None):
        """保存分析报告"""
        if output_path is None:
            output_dir = Path(DEFAULT_OUTPUT_DIR)
            output_dir.mkdir(exist_ok=True)
            output_path = output_dir / "auto_tune_report.txt"
        
        with open(output_path, 'w', encoding='utf-8') as f:
            f.write('\n'.join(self.report_lines))
        
        self._log(f"\n报告已保存到: {output_path}")


# =============================================================================
# 主函数
# =============================================================================
def main():
    parser = argparse.ArgumentParser(
        description='一键自动调优工具 - 收集数据、分析、生成调优配置',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  # 收集 30 秒数据并生成调优配置
  python -m tools.tuning.auto_tune --duration 30
  
  # 指定输出文件
  python -m tools.tuning.auto_tune --duration 30 --output my_tuned.yaml
  
  # 直接应用到配置文件
  python -m tools.tuning.auto_tune --duration 30 --apply
  
  # 使用自定义配置文件
  python -m tools.tuning.auto_tune --config path/to/config.yaml --duration 30
"""
    )
    parser.add_argument('--duration', type=float, default=30,
                        help='数据收集持续时间(秒)，默认 30 秒')
    parser.add_argument('--config', type=str, default=DEFAULT_CONFIG_PATH,
                        help=f'配置文件路径，默认 {DEFAULT_CONFIG_PATH}')
    parser.add_argument('--output', type=str,
                        help='输出配置文件路径，默认 tuning_output/tuned_turtlebot1.yaml')
    parser.add_argument('--apply', action='store_true',
                        help='直接应用调优到原配置文件')
    parser.add_argument('--no-report', action='store_true',
                        help='不保存分析报告')
    
    args = parser.parse_args()
    
    print("=" * 60)
    print("一键自动调优工具 v1.0")
    print("=" * 60)
    
    # 创建调优器
    tuner = AutoTuner(args.config)
    
    # 收集数据
    if not tuner.collect_data(args.duration):
        print("\n❌ 数据收集失败")
        sys.exit(1)
    
    # 分析并生成调优建议
    changes = tuner.analyze_and_tune()
    
    # 输出结果
    if changes:
        if args.apply:
            tuner.apply_to_config()
        else:
            tuner.generate_tuned_config(args.output)
    
    # 保存报告
    if not args.no_report:
        tuner.save_report()
    
    print("\n" + "=" * 60)
    print("调优完成!")
    print("=" * 60)
    
    if changes:
        print(f"\n共 {len(changes)} 项调优建议:")
        for change in changes:
            print(f"  • {change.param_path}: {change.old_value} → {change.new_value}")
        
        if not args.apply:
            print(f"\n提示: 使用 --apply 参数可直接应用到配置文件")
    else:
        print("\n当前配置已是最优，无需调整")


if __name__ == '__main__':
    main()
