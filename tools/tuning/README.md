# TurtleBot1 配置诊断与调优工具 v3.0

自动分析控制器诊断数据，识别性能问题并生成优化后的配置文件。

## 设计原则

### 参数分类

工具将参数分为三类，采用不同的处理策略：

| 分类 | 说明 | 处理策略 |
|------|------|----------|
| **可调优参数** (TUNABLE) | 基于运行数据可安全调整 | 生成调优建议，自动应用 |
| **设计参数** (DESIGN) | 需要系统辨识或专业知识 | 仅报告状态，不自动调整 |
| **安全参数** (SAFETY) | 涉及安全，不应自动放宽 | 仅检测配置错误，不自动放宽 |

### 可调优参数

以下参数可以基于运行数据安全调整：

- **超时配置**: `watchdog.odom_timeout_ms`, `watchdog.traj_timeout_ms`, `watchdog.traj_grace_ms`
- **MPC 健康监控**: `mpc.health_monitor.time_*_thresh_ms`, `mpc.health_monitor.consecutive_*`
- **MPC 预测时域**: `mpc.horizon`, `mpc.horizon_degraded`
- **MPC 跟踪权重**: `mpc.weights.position`, `mpc.weights.velocity`, `mpc.weights.heading`
- **状态机参数**: `safety.state_machine.mpc_*`
- **跟踪质量阈值**: `tracking.*_thresh` (仅影响显示)
- **坐标变换阈值**: `transform.fallback_*_limit_ms`
- **备份控制器**: `backup.lookahead_dist`, `backup.kp_heading`

### 设计参数（不自动调优）

以下参数需要系统辨识或专业知识，工具仅报告状态：

- **EKF 噪声参数**: `ekf.process_noise.*`, `ekf.measurement_noise.*`
- **EKF 自适应参数**: `ekf.adaptive.*`
- **一致性检查阈值**: `consistency.*_thresh`
- **过渡配置**: `transition.*`
- **MPC 控制输入权重**: `mpc.weights.control_accel`, `mpc.weights.control_alpha`

### 安全参数（不自动放宽）

以下参数涉及安全，工具仅检测配置错误（如 `omega_max=0`）：

- **速度/加速度约束**: `constraints.*`
- **安全配置**: `safety.emergency_decel`, `safety.margins.*`
- **EKF 异常检测阈值**: `ekf.anomaly_detection.*`


## 功能特性

- 📊 **全面诊断分析**: 覆盖 turtlebot1.yaml 中所有参数
- 🔧 **智能调优**: 只对可调优参数生成建议
- 🛡️ **安全保护**: 不自动放宽安全参数
- 📝 **配置生成**: 生成与原始 YAML 结构完全一致的优化配置
- 📈 **详细报告**: 诊断报告、分析摘要、变更日志

## 前提条件

### 必需依赖

```bash
pip install numpy pyyaml
```

### 可选依赖

```bash
# 用于读取 ROS bag 文件
pip install rosbag bagpy

# 用于实时数据收集 (需要 ROS 环境)
source /opt/ros/noetic/setup.bash
```

## 使用方法

### 1. 从 ROS bag 文件分析

```bash
python -m tools.tuning.run_diagnostics --bag /path/to/recording.bag
```

### 2. 从 JSON 诊断数据分析

```bash
python -m tools.tuning.run_diagnostics --json /path/to/diagnostics.json
```

### 3. 实时收集并分析

```bash
# 需要先启动 ROS 和控制器
roslaunch controller_ros turtlebot1.launch

# 在另一个终端运行诊断工具
python -m tools.tuning.run_diagnostics --live --duration 60
```

### 4. 使用演示数据测试

```bash
python -m tools.tuning.run_diagnostics --demo
```

## 命令行参数

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `--bag` | ROS bag 文件路径 | - |
| `--json` | JSON 诊断数据文件路径 | - |
| `--live` | 从实时 ROS 话题收集 | - |
| `--demo` | 使用演示数据运行 | - |
| `--config` | 配置文件路径 | `controller_ros/config/turtlebot1.yaml` |
| `--output` | 输出目录 | `./tuning_output` |
| `--duration` | 实时收集持续时间（秒） | 60 |
| `--topic` | 诊断话题名称 | `/controller/diagnostics` |
| `--max-samples` | 最大样本数 | 10000 |

## 输出文件

```
tuning_output/
├── tuned_turtlebot1.yaml      # 优化后的配置文件
├── diagnostics_report.txt     # 详细诊断报告
├── analysis_summary.json      # 分析摘要 (JSON 格式)
└── collected_diagnostics.json # 收集的诊断数据 (仅 --live 模式)
```


## 分析内容

### 配置错误检测（最高优先级）

必须修复的硬性错误：

- `omega_max = 0`: 机器人无法转向
- `mpc.dt ≠ trajectory.default_dt_sec`: 时间步长不一致
- `horizon_degraded >= horizon`: 降级时域配置错误
- 超时配置小于控制周期
- MPC 求解时间超过控制周期

### 超时配置分析

基于实际延迟统计：

- 使用 95% 分位数 + 裕度计算建议值
- 检测超时率过高的情况
- 分析宽限期超时

### MPC 性能分析

- 求解时间统计（平均、95%分位、99%分位）
- 成功率分析
- KKT 残差和条件数监控
- 健康监控阈值建议

### 轨迹跟踪分析

- 横向误差 → position 权重建议
- 纵向误差 → velocity 权重建议
- 航向误差 → heading 权重建议
- 使用保守的调整策略（最多增加 30%）

### 状态机分析

- MPC 降级频率与成功率关系
- 备用控制器激活率与恢复条件
- Alpha 禁用阈值配置

### 诊断信息（不生成调优建议）

以下内容仅报告状态，供用户参考：

- EKF 协方差和新息范数
- 打滑检测频率
- 一致性检查得分
- 速度/角速度约束使用情况
- 紧急停止和安全检查失败统计

## 示例输出

```
============================================================
分析摘要
============================================================

样本数: 500

MPC 性能:
  - 成功率: 89.0%
  - 平均求解时间: 12.9ms
  - 95%分位求解时间: 15.67ms
  - 备用控制器激活率: 0.0%

跟踪误差:
  - 横向: avg=5.03cm, max=8.6cm
  - 纵向: avg=10.67cm, max=16.31cm
  - 航向: avg=2.94°, max=4.85°

============================================================
优化建议
============================================================

----------------------------------------
可调优参数 (建议采纳)
----------------------------------------

🔴 严重问题 (1项):
  [watchdog.traj_grace_ms]
    当前值: 600 → 建议值: 900
    原因: 轨迹宽限期超时(0.6%)，可能导致安全停止。

🟡 警告 (1项):
  [mpc.horizon]
    当前值: 7 → 建议值: 6
    原因: MPC成功率(89.0%)较低，建议减小预测时域。

----------------------------------------
安全参数 (不建议自动调整)
----------------------------------------
  ⚪ [safety.emergency_decel]
    [诊断信息] 紧急停止发生2次(0.4%)。紧急减速度是安全参数，不建议自动调整。
```

## 注意事项

1. **数据质量**: 建议收集至少 100 个样本以获得准确分析
2. **运行场景**: 尽量在典型运行场景下收集数据
3. **配置验证**: 生成的配置需要在实际环境中验证
4. **渐进调优**: 建议逐步应用优化建议，而非一次性全部应用
5. **安全参数**: 安全相关参数不会自动放宽，如需调整请手动修改

## v3.0 更新内容

- 移除对设计参数的自动调优（EKF噪声、一致性阈值等）
- 移除对安全参数的自动放宽（速度/加速度限制等）
- 移除基于命令变化率的控制平滑度分析（数据不可靠）
- 统一参数分类，明确标注不建议调优的参数
- 简化置信度计算，使用更保守的策略
- 使用 99% 分位数代替最大值计算临界阈值（避免异常值影响）
