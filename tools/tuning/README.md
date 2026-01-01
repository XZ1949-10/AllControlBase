# TurtleBot1 配置诊断与调优工具 v4.2

自动分析控制器诊断数据，识别性能问题并生成优化后的配置文件。

## 功能特性

- 📊 **全面诊断分析**: 覆盖所有 turtlebot1.yaml 中定义的可调优参数
- 🔧 **智能调优**: 只对可调优参数生成建议
- 🛡️ **安全保护**: 不自动放宽安全参数
- 📝 **配置生成**: 生成与 turtlebot1.yaml 结构完全一致的优化配置
- 📈 **详细报告**: 诊断报告、分析摘要、变更日志
- 🆕 **一键调优**: 自动收集、分析、生成配置 (v4.0 新增)
- 🆕 **帧率分析**: 分析话题帧率、MPC 降级原因 (v3.6 新增)
- 🆕 **低频轨迹优化**: 自动检测并优化低频轨迹场景 (v4.2 新增)

## 快速开始

### 🚀 一键自动调优 (推荐)

```bash
# 一键完成: 收集数据 → 分析 → 生成调优配置
python -m tools.tuning.auto_tune --duration 30

# 直接应用到配置文件
python -m tools.tuning.auto_tune --duration 30 --apply
```

输出示例:
```
============================================================
一键自动调优工具 v1.0
============================================================

开始收集数据 (30秒)...
监控话题:
  - /odom
  - /controller/input/trajectory
  - /mobile_base/sensors/imu_data
  - /controller/diagnostics
  - TF2: base_footprint → odom

============================================================
调优建议
============================================================

🟡 watchdog.traj_timeout_ms
    当前值: 1500 → 建议值: 2000
    原因: 轨迹帧率 2.3Hz, p95间隔 850ms

✅ 调优配置已保存到: tuning_output/tuned_turtlebot1.yaml
```

### 帧率分析

```bash
# 分析话题帧率和 MPC 降级原因
python -m tools.tuning.analyze_frame_rate --json tuning_output/collected_diagnostics.json
```

### 参数调优

```bash
# 从 JSON 诊断数据分析
python -m tools.tuning.run_diagnostics --json /path/to/diagnostics.json

# 实时收集并分析
python -m tools.tuning.run_diagnostics --live --duration 60
```

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

#### turtlebot1.yaml 中定义的参数

- **超时配置** (watchdog):
  - `watchdog.odom_timeout_ms`, `watchdog.traj_timeout_ms`, `watchdog.traj_grace_ms`
  - `watchdog.imu_timeout_ms`, `watchdog.startup_grace_ms`, `watchdog.absolute_startup_timeout_ms`
- **MPC 健康监控** (mpc.health_monitor):
  - `mpc.health_monitor.time_warning_thresh_ms`, `mpc.health_monitor.time_critical_thresh_ms`
  - `mpc.health_monitor.time_recovery_thresh_ms`, `mpc.health_monitor.consecutive_warning_limit`
  - `mpc.health_monitor.consecutive_recovery_limit`
- **MPC 预测时域**: `mpc.horizon`, `mpc.horizon_degraded`, `mpc.dt`
- **MPC 跟踪权重**: `mpc.weights.position`, `mpc.weights.velocity`, `mpc.weights.heading`
- **MPC Fallback**: `mpc.fallback.lookahead_steps`
- **状态机参数** (safety.state_machine):
  - `safety.state_machine.mpc_fail_thresh`, `safety.state_machine.mpc_fail_window_size`
  - `safety.state_machine.mpc_recovery_thresh`, `safety.state_machine.mpc_recovery_tolerance`
- **跟踪质量阈值** (tracking):
  - `tracking.lateral_thresh`, `tracking.longitudinal_thresh`, `tracking.heading_thresh`
- **坐标变换配置**: `transform.timeout_ms`
- **备份控制器** (backup):
  - `backup.lookahead_dist`, `backup.min_lookahead`, `backup.max_lookahead`
  - `backup.lookahead_ratio`, `backup.kp_heading`, `backup.heading_error_thresh`
  - `backup.max_curvature`, `backup.default_speed_ratio`, `backup.min_distance_thresh`
- **轨迹配置** (trajectory):
  - `trajectory.low_speed_thresh`, `trajectory.min_points`, `trajectory.max_points`
  - `trajectory.max_point_distance`, `trajectory.default_dt_sec`
- **诊断配置**: `diagnostics.publish_rate`
- **低速保护**: `constraints.v_low_thresh`

#### controller_params.yaml 中定义的参数 (ROS 层)

基于 TF2 降级统计可靠调优：

- **TF2 降级限制** (transform):
  - `transform.fallback_duration_limit_ms`: 降级警告阈值，基于 95% 分位降级持续时间调优
  - `transform.fallback_critical_limit_ms`: 降级临界阈值，基于 99% 分位降级持续时间调优

#### internal_params.yaml 中定义的参数 (算法层)

基于运行数据可靠调优：

- **状态机内部参数** (safety.state_machine):
  - `safety.state_machine.mpc_fail_ratio_thresh`: MPC 失败率阈值，基于实际失败率统计调优
  - `safety.state_machine.mpc_recovery_success_ratio`: MPC 恢复成功率要求，基于实际恢复率统计调优
  - `safety.state_machine.degraded_state_timeout`: MPC_DEGRADED 状态超时，基于状态持续时间统计调优
  - `safety.state_machine.backup_state_timeout`: BACKUP_ACTIVE 状态超时，基于状态持续时间统计调优
- **跟踪质量评估** (tracking):
  - `tracking.prediction_thresh`: 预测误差阈值，基于预测误差统计调优

### 设计参数（不自动调优）

以下参数需要系统辨识或专业知识，工具仅报告状态：

- **一致性检查参数** (consistency):
  - `consistency.alpha_min`, `consistency.kappa_thresh`, `consistency.v_dir_thresh`
  - `consistency.temporal_smooth_thresh`, `consistency.max_curvature`, `consistency.temporal_window_size`
  - `consistency.weights.kappa`, `consistency.weights.velocity`, `consistency.weights.temporal`
- **MPC 控制输入权重**: `mpc.weights.control_accel`, `mpc.weights.control_alpha`
- **状态机设计参数**: `safety.state_machine.alpha_disable_thresh`

### 安全参数（不自动放宽）

以下参数涉及安全，工具仅检测配置错误（如 `omega_max=0`）：

- **速度约束**: `constraints.v_max`, `constraints.v_min`, `constraints.omega_max`, `constraints.omega_max_low`
- **加速度约束**: `constraints.a_max`, `constraints.alpha_max`
- **安全配置**: `safety.emergency_decel`, `safety.v_stop_thresh`, `safety.stopping_timeout`


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

## 工具说明

本工具包含四个分析脚本：

| 工具 | 用途 | 数据源 |
|------|------|--------|
| `auto_tune.py` | **🚀 一键自动调优** | 话题 + 诊断消息 |
| `run_diagnostics.py` | 参数调优 | 诊断消息 |
| `analyze_frame_rate.py` | MPC 降级原因分析 | 诊断消息 |
| `analyze_topic_rates.py` | 话题帧率分析 | 直接订阅话题 |

### 话题与参数对应关系

```
话题帧率 → 超时参数调优:
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

输入话题:
  /odom                         → watchdog.odom_timeout_ms
  /controller/input/trajectory  → watchdog.traj_timeout_ms, traj_grace_ms
  /mobile_base/sensors/imu_data → watchdog.imu_timeout_ms
  TF2 (base_footprint → odom)   → transform.timeout_ms

输出话题:
  /controller/diagnostics       → diagnostics.publish_rate
  /cmd_unified                  → cmd_vel_adapter.publish_rate

调优公式:
  timeout_ms >= (1000 / 实际帧率) × 2
  grace_ms   >= (1000 / 实际帧率) × 1.5
```

## 使用方法

### 0. 一键自动调优 (推荐)

```bash
# 收集 30 秒数据并生成调优配置
python -m tools.tuning.auto_tune --duration 30

# 指定输出文件
python -m tools.tuning.auto_tune --duration 30 --output my_tuned.yaml

# 直接应用到配置文件 (会自动备份原文件)
python -m tools.tuning.auto_tune --duration 30 --apply

# 使用自定义配置文件
python -m tools.tuning.auto_tune --config path/to/config.yaml --duration 30
```

### 1. 话题帧率分析 (推荐先运行)

```bash
# 实时分析话题帧率 (需要 ROS 环境)
python -m tools.tuning.analyze_topic_rates --live --duration 30

# 从 ROS bag 分析
python -m tools.tuning.analyze_topic_rates --bag /path/to/recording.bag
```

### 2. MPC 降级原因分析

```bash
python -m tools.tuning.analyze_frame_rate --json tuning_output/collected_diagnostics.json
```

### 3. 从 ROS bag 文件分析

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
roslaunch controller_ros platforms/turtlebot1.launch

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
| `--config` | 配置文件路径 | `controller_ros/config/platforms/turtlebot1.yaml` |
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

必须修复的硬性错误（静态配置检查）：

- `omega_max = 0`: 机器人无法转向
- `mpc.dt ≠ trajectory.default_dt_sec`: 时间步长不一致
- `horizon_degraded >= horizon`: 降级时域配置错误
- 超时配置小于控制周期
- 跟踪权重总和不等于 1.0

### 超时配置分析

基于实际延迟统计：

- 使用 95% 分位数 + 裕度计算建议值
- 检测超时率过高的情况
- 分析宽限期超时

### MPC 性能分析

- 求解时间统计（平均、95%分位、99%分位）
- 成功率分析（综合判断是否需要减小 horizon）
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

## v4.0 更新内容

- **新增一键自动调优工具 `auto_tune.py`**:
  - 一个命令完成: 数据收集 → 分析 → 生成调优配置
  - 自动订阅话题收集帧率数据
  - 自动收集诊断消息分析 MPC 性能
  - 基于实际数据自动计算最优超时参数
  - 支持 `--apply` 直接应用到配置文件
  - 自动备份原配置文件
- **调优公式**:
  - `timeout_ms = max(p95间隔, 平均周期) × 2`
  - `grace_ms = max(p95间隔, 平均周期) × 1.5`
- **智能权重调优**:
  - 基于跟踪误差自动调整 MPC 权重
  - 纵向误差 > 50cm → 增加 velocity 权重
  - 横向误差 > 15cm → 增加 position 权重
  - 航向误差 > 11° → 增加 heading 权重

## v4.2 更新内容

- **新增低频轨迹专项优化** (`_tune_low_frequency_trajectory`):
  - 自动检测轨迹频率是否低于 5Hz
  - 当检测到低频轨迹时，自动优化以下参数:
    - `consistency.temporal_window_size`: 减少历史窗口，加快对轨迹变化的响应
    - `backup.lookahead_dist`: 增加前瞻距离补偿轨迹更新延迟
    - `backup.min_lookahead`: 配合增加的前瞻距离
    - `backup.max_lookahead`: 允许更大的动态前瞻范围
    - `safety.state_machine.mpc_fail_thresh`: 放宽 MPC 失败阈值，减少不必要的备份控制器切换
    - `trajectory.low_speed_thresh`: 极低频轨迹下降低低速阈值
  - 优化公式:
    - `temporal_window_size = max(int(2.5秒 × 轨迹频率), 4)`
    - `lookahead_dist = max(当前值, v_max × 轨迹周期 × 1.5 + 0.2)`
- **改进诊断报告**:
  - 新增【低频轨迹专项优化】章节
  - 显示轨迹频率和对应的优化建议

## v4.1 更新内容

- **扩展诊断数据收集**:
  - 新增 MPC 健康监控数据: `mpc_health_consecutive_near_timeout`, `mpc_health_degradation_warning`, `mpc_health_kkt_residual`
  - 新增坐标变换数据: `transform_fallback_duration_ms`, `transform_tf2_available`
  - 新增跟踪预测误差: `tracking_prediction_error`
  - 新增备份控制器统计: `backup_active`
  - 新增紧急停止统计: `emergency_stop`
- **新增调优参数**:
  - `mpc.health_monitor.time_critical_thresh_ms`: 基于 p99 求解时间调优
  - `mpc.health_monitor.consecutive_warning_limit`: 基于连续超时统计调优
  - `mpc.horizon`: 基于 MPC 成功率调优
  - `tracking.lateral_thresh`, `tracking.longitudinal_thresh`, `tracking.heading_thresh`: 基于跟踪误差 p95 调优
  - `tracking.prediction_thresh`: 基于预测误差 p95 调优
  - `transform.fallback_duration_limit_ms`, `transform.fallback_critical_limit_ms`: 基于 TF 降级持续时间调优
  - `safety.state_machine.mpc_fail_ratio_thresh`: 基于 MPC 失败率调优
  - `safety.state_machine.mpc_recovery_thresh`: 基于备份控制器激活率调优
- **改进诊断报告**:
  - 显示 MPC 成功率和降级警告次数
  - 显示 TF 降级持续时间统计
  - 显示备份控制器激活率
  - 显示紧急停止次数

## v3.5 更新内容

- **扩展支持 controller_params.yaml 和 internal_params.yaml 参数**:
  - 新增 TF2 降级限制参数调优 (`transform.fallback_duration_limit_ms`, `transform.fallback_critical_limit_ms`)
  - 新增状态机内部参数调优 (`mpc_fail_ratio_thresh`, `mpc_recovery_success_ratio`, `degraded_state_timeout`, `backup_state_timeout`)
  - 新增跟踪预测误差阈值调优 (`tracking.prediction_thresh`)
- **新增统计字段**:
  - `degraded_state_durations`: 记录 MPC_DEGRADED 状态持续时间
  - `backup_state_durations`: 记录 BACKUP_ACTIVE 状态持续时间
  - `mpc_fail_ratios`: 记录滑动窗口内的 MPC 失败率
  - `mpc_recovery_ratios`: 记录 MPC 恢复成功率
- **新增分析方法**:
  - `_analyze_tf2_fallback_limits()`: 分析 TF2 降级持续时间限制
  - `_analyze_mpc_ratios()`: 分析 MPC 失败/恢复比率阈值
  - `_analyze_state_timeouts()`: 分析状态超时参数
- **摘要输出增强**:
  - 添加状态持续时间统计 (avg, max, p95)
  - 添加 MPC 失败/恢复比率统计
  - 添加 TF2 降级持续时间 p95 分位数

## v3.4 更新内容

- **参数与 turtlebot1.yaml 完全同步**:
  - 移除不存在于 turtlebot1.yaml 的参数（`tracking.prediction_thresh`, `transform.buffer_warmup_*`）
  - 添加 turtlebot1.yaml 中存在但之前遗漏的参数（`mpc.health_monitor.consecutive_recovery_limit`, `mpc.fallback.lookahead_steps` 等）
  - 添加完整的 backup 参数（`lookahead_ratio`, `heading_error_thresh`, `max_curvature` 等）
  - 添加完整的 trajectory 参数（`min_points`, `max_points`, `max_point_distance`, `default_dt_sec`）
- **ConfigGenerator SECTION_ORDER 修复**:
  - 节顺序与 turtlebot1.yaml 完全一致
  - `topics` 移至最后（原来错误地放在第二位）
  - 添加 `constraints` 节（原来遗漏）
- **设计参数分类完善**:
  - 添加完整的 consistency 参数到 DESIGN_PARAMS
  - 添加 `safety.state_machine.alpha_disable_thresh` 到 DESIGN_PARAMS
- **安全参数分类完善**:
  - 添加 `safety.v_stop_thresh` 和 `safety.stopping_timeout`
- **注释格式统一**:
  - 使用 `[turtlebot1.yaml]` 标记所有参数来源
  - 移除混乱的"默认值"和"turtlebot1"混合注释

## v3.3 更新内容

- **参数分类优化**:
  - 将 `constraints.v_low_thresh` 从 SAFETY 移到 TUNABLE（低速阈值不是安全限制）
  - 添加 `tracking.heading_thresh` 和 `tracking.prediction_thresh` 到 TUNABLE
- **配置统一**:
  - `cmd_vel_adapter` 不再重复定义加速度限制，统一从 `constraints.a_max/alpha_max` 读取
  - 移除 `turtlebot1.yaml` 中的 `cmd_vel_adapter.max_linear_accel` 和 `max_angular_accel`
- **注释增强**: 为每个参数分类添加详细的默认值来源说明
- **航向/预测误差分析**: 现在会从配置读取阈值并生成阈值调整建议

## v3.2 更新内容

- **参数路径统一**: 所有参数路径与 `turtlebot1.yaml` 完全一致
- **简化设计参数**: 只保留 `consistency.weights.*` 和 `mpc.weights.control_*`
- **简化安全参数**: 只保留 `constraints.*` 和 `safety.emergency_decel`
- **修复控制频率默认值**: 默认值从 20Hz 改为 50Hz（与 universal_controller 默认配置一致）
- **优化诊断信息**: 对于不在 turtlebot1.yaml 中的参数，只报告诊断信息而不生成调优建议

## v3.1 更新内容

- 统一状态统计：移除冗余的 `backup_active_flag_count`，统一使用基于状态枚举的 `backup_active_state_count`
- 修复重复建议：MPC horizon 建议现在只生成一个，综合考虑成功率和求解时间
- 分离配置检查：静态配置错误检测与基于运行数据的性能分析分离
- 简化摘要输出：移除冗余的 `backup_active_state_rate` 字段

## v3.0 更新内容

- 移除对设计参数的自动调优（EKF噪声、一致性阈值等）
- 移除对安全参数的自动放宽（速度/加速度限制等）
- 移除基于命令变化率的控制平滑度分析（数据不可靠）
- 统一参数分类，明确标注不建议调优的参数
- 简化置信度计算，使用更保守的策略
- 使用 99% 分位数代替最大值计算临界阈值（避免异常值影响）
