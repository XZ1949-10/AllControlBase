#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
TurtleBot1 配置诊断与调优工具

使用方法:
  # 从 ROS bag 文件分析
  python -m tools.tuning.run_diagnostics --bag /path/to/recording.bag
  
  # 从 JSON 诊断数据分析
  python -m tools.tuning.run_diagnostics --json /path/to/diagnostics.json
  
  # 实时收集并分析 (需要 ROS 环境)
  python -m tools.tuning.run_diagnostics --live --duration 60
  
  # 指定输出目录
  python -m tools.tuning.run_diagnostics --bag data.bag --output ./tuned_configs

前提条件:
  1. Python 3.7+
  2. numpy, pyyaml 包
  3. 对于 --bag 模式: rosbag 包
  4. 对于 --live 模式: ROS 环境 (source /opt/ros/noetic/setup.bash)

输出:
  - tuned_turtlebot1.yaml: 优化后的配置文件
  - diagnostics_report.txt: 诊断报告
  - analysis_summary.json: 分析摘要
"""

import argparse
import sys
import os
import json
import yaml
from pathlib import Path
from datetime import datetime

# 添加项目根目录到路径
PROJECT_ROOT = Path(__file__).parent.parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from tools.tuning.diagnostics_analyzer import DiagnosticsAnalyzer, AnalysisResult, TuningCategory
from tools.tuning.config_generator import ConfigGenerator
from tools.tuning.data_collector import (
    CollectionConfig, 
    RosbagCollector, 
    LiveCollector, 
    JsonFileCollector,
    save_samples_to_json
)


# 默认配置文件路径
DEFAULT_CONFIG_PATH = PROJECT_ROOT / "controller_ros" / "config" / "turtlebot1.yaml"


def print_banner():
    """打印工具横幅"""
    banner = """
╔══════════════════════════════════════════════════════════════════╗
║         TurtleBot1 配置诊断与调优工具 v1.0                       ║
║                                                                  ║
║  功能: 分析控制器诊断数据，自动生成优化配置                      ║
╚══════════════════════════════════════════════════════════════════╝
"""
    print(banner)


def print_prerequisites():
    """打印前提条件"""
    print("\n" + "=" * 60)
    print("前提条件检查")
    print("=" * 60)
    
    # 检查 numpy
    try:
        import numpy as np
        print(f"✓ numpy: {np.__version__}")
    except ImportError:
        print("✗ numpy: 未安装 (pip install numpy)")
        return False
    
    # 检查 yaml
    try:
        import yaml
        print(f"✓ pyyaml: {yaml.__version__}")
    except ImportError:
        print("✗ pyyaml: 未安装 (pip install pyyaml)")
        return False
    
    # 检查配置文件
    if DEFAULT_CONFIG_PATH.exists():
        print(f"✓ 配置文件: {DEFAULT_CONFIG_PATH}")
    else:
        print(f"✗ 配置文件不存在: {DEFAULT_CONFIG_PATH}")
        return False
    
    print("=" * 60)
    return True


def load_config(config_path: str = None) -> dict:
    """加载配置文件"""
    path = Path(config_path) if config_path else DEFAULT_CONFIG_PATH
    
    if not path.exists():
        print(f"错误: 配置文件不存在: {path}")
        sys.exit(1)
    
    with open(path, 'r', encoding='utf-8') as f:
        config = yaml.safe_load(f)
    
    print(f"已加载配置: {path}")
    return config


def progress_bar(current: int, total: int, width: int = 40):
    """显示进度条"""
    if total == 0:
        return
    progress = current / total
    filled = int(width * progress)
    bar = '█' * filled + '░' * (width - filled)
    print(f"\r进度: [{bar}] {current}/{total} ({progress*100:.1f}%)", end='', flush=True)


def print_analysis_results(results: list, summary: dict):
    """打印分析结果"""
    print("\n" + "=" * 60)
    print("分析摘要")
    print("=" * 60)
    
    # 打印统计摘要
    print(f"\n样本数: {summary.get('total_samples', 'N/A')}")
    
    mpc = summary.get('mpc', {})
    if mpc:
        print(f"\nMPC 性能:")
        print(f"  - 成功率: {mpc.get('success_rate', 'N/A')}%")
        print(f"  - 平均求解时间: {mpc.get('avg_solve_time_ms', 'N/A')}ms")
        print(f"  - 95%分位求解时间: {mpc.get('p95_solve_time_ms', 'N/A')}ms")
        print(f"  - 备用控制器激活率: {mpc.get('backup_rate', 'N/A')}%")
    
    tracking = summary.get('tracking', {})
    if tracking:
        print(f"\n跟踪误差:")
        if 'lateral' in tracking:
            lat = tracking['lateral']
            print(f"  - 横向: avg={lat.get('avg_cm', 'N/A')}cm, max={lat.get('max_cm', 'N/A')}cm")
        if 'longitudinal' in tracking:
            lon = tracking['longitudinal']
            print(f"  - 纵向: avg={lon.get('avg_cm', 'N/A')}cm, max={lon.get('max_cm', 'N/A')}cm")
        if 'heading' in tracking:
            head = tracking['heading']
            print(f"  - 航向: avg={head.get('avg_deg', 'N/A')}°, max={head.get('max_deg', 'N/A')}°")
    
    timeout = summary.get('timeout', {})
    if timeout:
        print(f"\n超时统计:")
        print(f"  - 里程计超时次数: {timeout.get('odom_timeout_count', 0)}")
        print(f"  - 轨迹超时次数: {timeout.get('traj_timeout_count', 0)}")
        print(f"  - 宽限期超时次数: {timeout.get('traj_grace_exceeded_count', 0)}")
    
    # 打印优化建议
    print("\n" + "=" * 60)
    print("优化建议")
    print("=" * 60)
    
    if not results:
        print("\n✓ 当前配置表现良好，无需调整")
        return
    
    # 按调优分类和严重程度分组
    tunable_results = [r for r in results if r.tuning_category == TuningCategory.TUNABLE]
    design_results = [r for r in results if r.tuning_category == TuningCategory.DESIGN]
    safety_results = [r for r in results if r.tuning_category == TuningCategory.SAFETY]
    diagnostic_results = [r for r in results if r.tuning_category == TuningCategory.DIAGNOSTIC]
    
    # 可调优参数
    if tunable_results:
        critical = [r for r in tunable_results if r.severity == 'critical']
        warning = [r for r in tunable_results if r.severity == 'warning']
        info = [r for r in tunable_results if r.severity == 'info']
        
        print("\n" + "-" * 40)
        print("可调优参数 (建议采纳)")
        print("-" * 40)
        
        if critical:
            print(f"\n🔴 严重问题 ({len(critical)}项):")
            for r in critical:
                print(f"  [{r.parameter}]")
                print(f"    当前值: {r.current_value} → 建议值: {r.suggested_value}")
                print(f"    原因: {r.reason}")
        
        if warning:
            print(f"\n🟡 警告 ({len(warning)}项):")
            for r in warning:
                print(f"  [{r.parameter}]")
                print(f"    当前值: {r.current_value} → 建议值: {r.suggested_value}")
                print(f"    原因: {r.reason}")
        
        if info:
            print(f"\n🔵 建议 ({len(info)}项):")
            for r in info:
                print(f"  [{r.parameter}]")
                print(f"    当前值: {r.current_value} → 建议值: {r.suggested_value}")
                print(f"    原因: {r.reason}")
    
    # 设计参数（仅诊断信息）
    if design_results:
        print("\n" + "-" * 40)
        print("设计参数 (不建议自动调优)")
        print("-" * 40)
        for r in design_results:
            print(f"  ⚪ [{r.parameter}]")
            print(f"    {r.reason}")
    
    # 安全参数（仅诊断信息）
    if safety_results:
        print("\n" + "-" * 40)
        print("安全参数 (不建议自动调整)")
        print("-" * 40)
        for r in safety_results:
            # omega_max=0 是配置错误，需要修复
            if r.category == 'config_error':
                print(f"  🔴 [{r.parameter}] (配置错误，需修复)")
                print(f"    当前值: {r.current_value} → 建议值: {r.suggested_value}")
                print(f"    原因: {r.reason}")
            else:
                print(f"  ⚪ [{r.parameter}]")
                print(f"    {r.reason}")


def run_analysis(samples: list, config: dict, output_dir: Path, config_path: str):
    """运行分析并生成输出"""
    print("\n" + "=" * 60)
    print("开始分析")
    print("=" * 60)
    
    # 创建分析器
    analyzer = DiagnosticsAnalyzer(config)
    
    # 添加样本
    print(f"\n处理 {len(samples)} 个样本...")
    for i, sample in enumerate(samples):
        analyzer.add_sample(sample)
        if (i + 1) % 500 == 0:
            print(f"  已处理: {i + 1}/{len(samples)}")
    
    # 执行分析
    print("\n执行分析...")
    results = analyzer.analyze()
    summary = analyzer.get_summary()
    
    # 打印结果
    print_analysis_results(results, summary)
    
    # 生成优化配置
    print("\n" + "=" * 60)
    print("生成优化配置")
    print("=" * 60)
    
    generator = ConfigGenerator(config, config_path)
    
    # 应用所有 warning 和 critical 级别的建议
    optimized_config = generator.apply_results(
        results, 
        min_confidence=0.6,
        severity_filter=['warning', 'critical']
    )
    
    # 创建输出目录
    output_dir.mkdir(parents=True, exist_ok=True)
    
    # 生成优化后的 YAML 文件
    output_yaml = output_dir / "tuned_turtlebot1.yaml"
    generator.generate_yaml(optimized_config, str(output_yaml), summary)
    print(f"\n✓ 优化配置已保存: {output_yaml}")
    
    # 保存分析摘要
    summary_file = output_dir / "analysis_summary.json"
    with open(summary_file, 'w', encoding='utf-8') as f:
        json.dump({
            'timestamp': datetime.now().isoformat(),
            'sample_count': len(samples),
            'summary': summary,
            'results': [
                {
                    'category': r.category,
                    'severity': r.severity,
                    'parameter': r.parameter,
                    'current_value': r.current_value,
                    'suggested_value': r.suggested_value,
                    'reason': r.reason,
                    'confidence': r.confidence,
                    'tuning_category': r.tuning_category.value
                }
                for r in results
            ]
        }, f, indent=2, ensure_ascii=False)
    print(f"✓ 分析摘要已保存: {summary_file}")
    
    # 保存诊断报告
    report_file = output_dir / "diagnostics_report.txt"
    with open(report_file, 'w', encoding='utf-8') as f:
        f.write("TurtleBot1 配置诊断报告\n")
        f.write(f"生成时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.write("=" * 60 + "\n\n")
        
        f.write("分析摘要:\n")
        f.write(json.dumps(summary, indent=2, ensure_ascii=False))
        f.write("\n\n")
        
        f.write("优化建议:\n")
        for r in results:
            f.write(f"\n[{r.severity.upper()}] {r.parameter}\n")
            f.write(f"  当前值: {r.current_value}\n")
            f.write(f"  建议值: {r.suggested_value}\n")
            f.write(f"  原因: {r.reason}\n")
            f.write(f"  置信度: {r.confidence*100:.0f}%\n")
        
        f.write("\n" + "=" * 60 + "\n")
        f.write(generator.get_change_report())
    
    print(f"✓ 诊断报告已保存: {report_file}")
    
    return results, summary


def main():
    """主函数"""
    parser = argparse.ArgumentParser(
        description='TurtleBot1 配置诊断与调优工具',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    
    # 数据源选项（互斥）
    source_group = parser.add_mutually_exclusive_group(required=True)
    source_group.add_argument('--bag', type=str, help='ROS bag 文件路径')
    source_group.add_argument('--json', type=str, help='JSON 诊断数据文件路径')
    source_group.add_argument('--live', action='store_true', help='从实时 ROS 话题收集')
    source_group.add_argument('--demo', action='store_true', help='使用演示数据运行')
    
    # 其他选项
    parser.add_argument('--config', type=str, default=None,
                        help=f'配置文件路径 (默认: {DEFAULT_CONFIG_PATH})')
    parser.add_argument('--output', type=str, default='./tuning_output',
                        help='输出目录 (默认: ./tuning_output)')
    parser.add_argument('--duration', type=float, default=60.0,
                        help='实时收集持续时间（秒）(默认: 60)')
    parser.add_argument('--topic', type=str, default='/controller/diagnostics',
                        help='诊断话题名称 (默认: /controller/diagnostics)')
    parser.add_argument('--max-samples', type=int, default=10000,
                        help='最大样本数 (默认: 10000)')
    
    args = parser.parse_args()
    
    # 打印横幅
    print_banner()
    
    # 检查前提条件
    if not print_prerequisites():
        print("\n请安装缺失的依赖后重试")
        sys.exit(1)
    
    # 加载配置
    config = load_config(args.config)
    config_path = args.config or str(DEFAULT_CONFIG_PATH)
    
    # 收集配置
    collection_config = CollectionConfig(
        diagnostics_topic=args.topic,
        max_samples=args.max_samples
    )
    
    # 收集数据
    samples = []
    
    if args.bag:
        print(f"\n从 ROS bag 文件收集数据: {args.bag}")
        collector = RosbagCollector(args.bag, collection_config)
        count = collector.collect(progress_callback=progress_bar)
        print()  # 换行
        samples = collector.get_samples()
        
    elif args.json:
        print(f"\n从 JSON 文件加载数据: {args.json}")
        collector = JsonFileCollector(args.json, collection_config)
        count = collector.collect()
        samples = collector.get_samples()
        
    elif args.live:
        print(f"\n从实时话题收集数据...")
        print("提示: 确保控制器正在运行并发布诊断数据")
        
        def live_progress(samples, elapsed):
            print(f"\r已收集: {samples} 样本, 已用时: {elapsed:.1f}秒", end='', flush=True)
        
        collector = LiveCollector(collection_config)
        count = collector.collect(
            duration_sec=args.duration,
            progress_callback=live_progress
        )
        print()  # 换行
        samples = collector.get_samples()
        
        # 保存收集的数据
        if samples:
            output_dir = Path(args.output)
            output_dir.mkdir(parents=True, exist_ok=True)
            save_samples_to_json(samples, str(output_dir / "collected_diagnostics.json"))
    
    elif args.demo:
        print("\n使用演示数据...")
        samples = generate_demo_data()
    
    # 检查样本数
    if not samples:
        print("\n错误: 未收集到任何数据")
        print("请检查:")
        print("  1. 数据源是否正确")
        print("  2. 诊断话题是否正确")
        print("  3. 控制器是否正在运行")
        sys.exit(1)
    
    print(f"\n收集到 {len(samples)} 个样本")
    
    # 运行分析
    output_dir = Path(args.output)
    results, summary = run_analysis(samples, config, output_dir, config_path)
    
    # 完成
    print("\n" + "=" * 60)
    print("诊断完成!")
    print("=" * 60)
    print(f"\n输出文件位于: {output_dir.absolute()}")
    print("\n下一步:")
    print("  1. 查看 diagnostics_report.txt 了解详细分析")
    print("  2. 检查 tuned_turtlebot1.yaml 中的优化配置")
    print("  3. 将优化配置复制到 controller_ros/config/ 目录")
    print("  4. 重新启动控制器测试效果")


def generate_demo_data() -> list:
    """生成演示数据 - 模拟真实控制器行为
    
    生成的数据严格遵循 DiagnosticsV2.to_ros_msg() 的输出格式。
    
    生成的数据具有以下特点：
    1. 时间相关性 - 连续帧的误差相近
    2. 状态转换 - 模拟 MPC 偶尔失败后恢复
    3. 周期性波动 - 模拟轨迹跟踪的周期性误差
    """
    import numpy as np
    from universal_controller.core.enums import ControllerState
    
    samples = []
    np.random.seed(42)
    
    # 状态变量（模拟时间相关性）
    current_lateral_error = 0.05
    current_longitudinal_error = 0.10
    current_heading_error = 0.05
    current_solve_time = 10.0
    mpc_fail_streak = 0
    state = ControllerState.NORMAL
    
    for i in range(500):
        # 模拟 MPC 求解时间的时间相关性（带随机扰动）
        current_solve_time = 0.9 * current_solve_time + 0.1 * (np.random.exponential(8) + 5)
        current_solve_time = np.clip(current_solve_time, 3, 50)
        
        # 模拟 MPC 成功/失败（连续失败更可能继续失败）
        if mpc_fail_streak > 0:
            mpc_success = np.random.random() > 0.3  # 失败后恢复概率 70%
        else:
            mpc_success = np.random.random() > 0.08  # 正常失败率 8%
        
        if not mpc_success:
            mpc_fail_streak += 1
        else:
            mpc_fail_streak = 0
        
        # 状态转换逻辑 (使用枚举)
        if mpc_fail_streak >= 5:
            state = ControllerState.BACKUP_ACTIVE
        elif mpc_fail_streak >= 3:
            state = ControllerState.MPC_DEGRADED
        elif mpc_fail_streak == 0 and state != ControllerState.NORMAL:
            state = ControllerState.NORMAL
        
        backup_active = (state == ControllerState.BACKUP_ACTIVE)
        
        # 模拟跟踪误差的时间相关性（带周期性波动）
        phase = i * 0.1  # 模拟轨迹周期
        
        # 误差平滑变化
        target_lateral = 0.05 + 0.03 * np.sin(phase * 0.5) + np.random.normal(0, 0.01)
        target_longitudinal = 0.10 + 0.05 * np.sin(phase * 0.3) + np.random.normal(0, 0.02)
        target_heading = 0.05 + 0.03 * np.sin(phase * 0.7) + np.random.normal(0, 0.01)
        
        current_lateral_error = 0.8 * current_lateral_error + 0.2 * abs(target_lateral)
        current_longitudinal_error = 0.8 * current_longitudinal_error + 0.2 * abs(target_longitudinal)
        current_heading_error = 0.8 * current_heading_error + 0.2 * abs(target_heading)
        
        # 备用控制器激活时误差会增大
        if backup_active:
            current_lateral_error *= 1.5
            current_longitudinal_error *= 1.5
        
        # 计算跟踪质量评分 (简化版)
        tracking_quality_score = max(0, 100 - current_lateral_error * 200 - current_longitudinal_error * 100 - current_heading_error * 50)
        if tracking_quality_score >= 90:
            tracking_quality_rating = 'excellent'
        elif tracking_quality_score >= 70:
            tracking_quality_rating = 'good'
        elif tracking_quality_score >= 50:
            tracking_quality_rating = 'fair'
        else:
            tracking_quality_rating = 'poor'
        
        # 模拟超时事件（偶发）
        odom_timeout = np.random.random() > 0.98
        traj_timeout = np.random.random() > 0.94
        traj_grace_exceeded = traj_timeout and np.random.random() > 0.9
        imu_timeout = np.random.random() > 0.99
        
        # 模拟数据延迟（有时间相关性）
        base_odom_age = 30 + 20 * np.sin(i * 0.05)
        base_traj_age = 150 + 100 * np.sin(i * 0.03)
        base_imu_age = 10 + 5 * np.sin(i * 0.08)
        
        # 生成与 DiagnosticsV2.to_ros_msg() 完全一致的格式
        sample = {
            'header': {
                'stamp': i * 0.05,  # 20Hz
                'frame_id': ''
            },
            'state': int(state),  # 转换为 int 以匹配 ROS 消息格式
            'mpc_success': mpc_success,
            'mpc_solve_time_ms': current_solve_time,
            'backup_active': backup_active,
            
            # MPC 健康状态
            'mpc_health': {
                'kkt_residual': np.random.exponential(0.0001),
                'condition_number': np.random.exponential(1e5),
                'consecutive_near_timeout': mpc_fail_streak,
                'degradation_warning': mpc_fail_streak >= 2,
                'can_recover': mpc_fail_streak < 5
            },
            
            # 一致性指标
            'consistency': {
                'curvature': np.clip(0.7 + np.random.normal(0, 0.1), 0.3, 1.0),
                'velocity_dir': np.clip(0.8 + np.random.normal(0, 0.1), 0.4, 1.0),
                'temporal': np.clip(0.7 + np.random.normal(0, 0.1), 0.3, 1.0),
                'alpha_soft': np.clip(0.6 + np.random.normal(0, 0.15), 0.1, 1.0),
                'data_valid': np.random.random() > 0.02
            },
            
            # 状态估计器健康
            'estimator_health': {
                'covariance_norm': np.random.exponential(0.1),
                'innovation_norm': np.random.exponential(0.05),
                'slip_probability': np.clip(np.random.exponential(0.1), 0, 1),
                'imu_drift_detected': np.random.random() > 0.98,
                'imu_bias': [np.random.normal(0, 0.001) for _ in range(3)],
                'imu_available': True
            },
            
            # 跟踪误差和质量评估
            'tracking': {
                'lateral_error': current_lateral_error,
                'longitudinal_error': current_longitudinal_error,
                'heading_error': current_heading_error,
                'prediction_error': abs(np.random.normal(0.03, 0.01)),
                'quality_score': tracking_quality_score,
                'quality_rating': tracking_quality_rating
            },
            
            # 坐标变换状态
            'transform': {
                'tf2_available': True,
                'tf2_injected': True,
                'fallback_duration_ms': 0 if np.random.random() > 0.05 else np.random.exponential(50),
                'accumulated_drift': np.random.exponential(0.01),
                'source_frame': 'base_link',
                'target_frame': 'odom',
                'error_message': '' if np.random.random() > 0.02 else 'TF2 temporarily unavailable'
            },
            
            # 超时状态
            'timeout': {
                'odom_timeout': odom_timeout,
                'traj_timeout': traj_timeout,
                'traj_grace_exceeded': traj_grace_exceeded,
                'imu_timeout': imu_timeout,
                'last_odom_age_ms': base_odom_age + np.random.exponential(20),
                'last_traj_age_ms': base_traj_age + np.random.exponential(50),
                'last_imu_age_ms': base_imu_age + np.random.exponential(5),
                'in_startup_grace': i < 10
            },
            
            # 控制命令
            'cmd': {
                'vx': np.clip(0.25 + 0.1 * np.sin(phase * 0.2) + np.random.normal(0, 0.02), 0, 0.5),
                'vy': 0.0,
                'vz': 0.0,
                'omega': np.clip(0.2 * np.sin(phase * 0.5) + np.random.normal(0, 0.05), -1.0, 1.0),
                'frame_id': 'base_link'
            },
            
            # 过渡进度
            'transition_progress': 1.0 if state == ControllerState.NORMAL else np.clip(0.5 + np.random.normal(0, 0.1), 0, 1),
            
            # 安全状态 (顶层字段，与 DiagnosticsV2.to_ros_msg() 一致)
            'safety_check_passed': np.random.random() > 0.02,
            'emergency_stop': np.random.random() > 0.995,
            
            # ROS 节点层字段 (由 controller_ros/node/base_node.py 添加)
            # 注意: 这些字段在非 ROS 环境下不存在，分析器会使用默认值 0
            'error_message': '',
            'consecutive_errors': mpc_fail_streak
        }
        samples.append(sample)
    
    return samples


if __name__ == '__main__':
    main()
