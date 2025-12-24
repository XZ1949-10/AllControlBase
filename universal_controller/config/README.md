# 配置系统说明

## 概述

Universal Controller 使用分层配置架构，支持：
- Python 默认配置（代码中定义）
- YAML 文件覆盖（平台特定配置）
- ROS 参数服务器覆盖（运行时配置）

## 配置优先级

从低到高：
1. `universal_controller/config/*.py` - 默认配置
2. YAML 配置文件 - 平台特定覆盖
3. ROS 参数服务器 - 运行时覆盖

## 配置文件结构

```
universal_controller/config/
├── __init__.py           # 配置模块入口，导出主要接口
├── default_config.py     # 配置聚合，合并所有子模块
├── platform_config.py    # 平台运动学配置
├── system_config.py      # 系统基础配置 (ctrl_freq, watchdog, diagnostics, tracking)
├── mpc_config.py         # MPC 控制器配置
├── safety_config.py      # 安全和约束配置
├── ekf_config.py         # EKF 状态估计器配置
├── attitude_config.py    # 姿态控制配置 (无人机)
├── modules_config.py     # 其他模块配置 (consistency, transform, transition, backup)
├── trajectory_config.py  # 轨迹配置
├── mock_config.py        # Mock 配置 (测试用)
└── validation.py         # 配置验证逻辑
```

## 配置键名约定

### YAML 与 Python 的映射

| YAML 路径 | Python 路径 | 说明 |
|-----------|-------------|------|
| `tf/source_frame` | `transform.source_frame` | TF 配置映射到 transform |
| `tf/target_frame` | `transform.target_frame` | |
| `tf/timeout_ms` | `transform.timeout_ms` | 统一使用 timeout_ms |
| `tf/expected_source_frames` | `transform.expected_source_frames` | |

### 配置访问方式

推荐使用 `get_config_value` 函数：

```python
from universal_controller.config import get_config_value, DEFAULT_CONFIG

# 获取配置值，支持点分隔路径
horizon = get_config_value(config, 'mpc.horizon', default=20)

# 使用 fallback_config 获取默认值
value = get_config_value(config, 'mpc.horizon', fallback_config=DEFAULT_CONFIG)
```

## 配置验证

### 范围验证

每个配置模块定义自己的验证规则：

```python
MPC_VALIDATION_RULES = {
    'mpc.horizon': (1, 100, 'MPC 预测时域'),
    'mpc.dt': (0.001, 1.0, 'MPC 时间步长 (秒)'),
}
```

### 逻辑一致性验证

`validate_logical_consistency()` 检查配置参数之间的逻辑关系：
- `min_lookahead < max_lookahead`
- `horizon_degraded <= horizon`
- `v_min <= v_max`

### 使用验证

```python
from universal_controller.config import validate_config, ConfigValidationError

try:
    errors = validate_config(config, raise_on_error=True)
except ConfigValidationError as e:
    print(f"配置错误: {e}")
```

## 添加新配置项

1. 在对应的 `*_config.py` 中添加默认值
2. 在同一文件的 `*_VALIDATION_RULES` 中添加验证规则
3. 如果需要在 YAML 中覆盖，确保键名一致
4. 更新相关文档

## 特殊配置说明

### low_speed_thresh

低速阈值的唯一定义点是 `trajectory.low_speed_thresh`。
`consistency` 模块会自动从 `trajectory` 配置读取此值，确保一致性。

### traj_grace_ms

轨迹宽限期是超时**之后**的额外等待时间：
- 安全停止延迟 = `traj_timeout_ms` + `traj_grace_ms`
- 例如: 1000ms 超时 + 500ms 宽限 = 1.5秒后触发停止

### expected_source_frames

预期的轨迹源坐标系列表，用于验证网络输出：
- `base_link`: 标准机体坐标系
- `base_footprint`: TurtleBot 等平台使用
- `base_link_0`: 推理时刻冻结的机体坐标系
- `''`: 空字符串，使用默认 source_frame
- `odom`: 已经在里程计坐标系的轨迹
