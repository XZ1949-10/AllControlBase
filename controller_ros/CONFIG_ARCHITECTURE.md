# Controller ROS 配置架构

## 配置优先级（从低到高）

```
1. universal_controller/config/*.py  - Python 代码中的默认值
2. controller_params.yaml            - ROS 基础配置
3. {platform}.yaml                   - 平台特定配置
4. launch 文件 <param> 标签          - 运行时覆盖
```

## 配置文件职责

### controller_params.yaml
- **职责**: 定义 ROS 节点层面的通用配置
- **内容**: 话题名、TF 配置、超时配置、诊断配置
- **使用者**: 所有平台

### {platform}.yaml (differential, omni, ackermann, quadrotor, turtlebot1)
- **职责**: 定义平台特定的配置覆盖
- **内容**: 只包含与默认值不同的配置项
- **原则**: 最小化覆盖，未定义的配置使用默认值

### visualizer_params.yaml
- **职责**: 定义可视化节点的配置
- **内容**: 显示参数、手柄配置、相机配置
- **使用者**: visualizer_node.py

## 配置加载流程

### 控制器节点 (controller_node.py)
```
ParamLoader.load()
    ├── 深拷贝 DEFAULT_CONFIG
    ├── 递归遍历配置结构
    ├── 从 ROS 参数服务器读取覆盖值
    ├── 加载 TF 配置
    └── 映射 tf -> transform
```

### Dashboard 节点 (dashboard_node.py)
```
ParamLoader.load()
    └── 与控制器节点相同的加载逻辑
```

### Visualizer 节点 (visualizer_node.py)
```
VisualizerParamLoader.load()
    ├── 深拷贝 VISUALIZER_DEFAULTS
    ├── 递归遍历配置结构
    └── 优先级: 私有参数 > 全局参数 > 默认值
```

## 配置键名映射

| YAML 路径 | Python 配置路径 | 说明 |
|-----------|-----------------|------|
| system/ctrl_freq | config['system']['ctrl_freq'] | 控制频率 |
| tf/source_frame | config['tf']['source_frame'] | TF 源坐标系 |
| tf/source_frame | config['transform']['source_frame'] | 自动映射 |
| topics/odom | topics['odom'] | 话题配置独立加载 |

## 设计原则

1. **单一数据源**: 默认值只在 `universal_controller/config/*.py` 中定义
2. **最小覆盖**: 平台配置只覆盖必要的参数
3. **统一加载**: 所有节点使用相同的 ParamLoader 接口
4. **类型安全**: ParamLoader 自动进行类型转换
5. **向后兼容**: TF 配置自动映射到 transform 配置

## 添加新配置项

1. 在 `universal_controller/config/*.py` 中添加默认值
2. 在 `controller_params.yaml` 中添加 ROS 层配置（如需要）
3. 在平台配置文件中添加覆盖值（如需要）
4. ParamLoader 会自动加载新配置项（无需修改加载代码）
