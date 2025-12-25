# TurtleBot1 运行可视化模块

## 功能

1. **轨迹可视化** - 在俯视图上显示网络输出的轨迹
2. **速度监控** - 实时显示底盘线速度和角速度
3. **手柄控制** - Xbox 手柄控制机器人，LB 键切换控制模式

## 架构

```
visualizer/
├── __init__.py           # 模块入口
├── models.py             # 数据模型 (与 ROS 解耦)
├── main_window.py        # 主窗口
├── adapters/             # 数据适配器
│   ├── joy_adapter.py    # 手柄消息适配
│   ├── image_adapter.py  # 图像消息适配
│   └── velocity_adapter.py # 速度数据适配
├── handlers/             # 业务逻辑处理器
│   ├── joystick_handler.py # 手柄控制逻辑
│   └── data_aggregator.py  # 数据聚合
├── widgets/              # UI 组件
│   ├── trajectory_view.py  # 轨迹视图
│   ├── velocity_panel.py   # 速度面板
│   ├── velocity_plot.py    # 速度曲线
│   ├── joystick_panel.py   # 手柄面板
│   └── status_bar.py       # 状态栏
└── node/                 # ROS 节点
    ├── ros_bridge.py     # ROS1/ROS2 兼容层
    └── visualizer_node.py # 可视化节点
```

## 使用方法

### ROS1 (Noetic)

```bash
# 启动可视化器
rosrun controller_ros visualizer_node.py

# 或使用 launch 文件
roslaunch controller_ros visualizer.launch
```

### ROS2 (Humble/Iron)

```bash
# 启动可视化器
ros2 run controller_ros visualizer_node.py

# 或使用 launch 文件
ros2 launch controller_ros visualizer.launch.py
```

## 依赖

```bash
# Python 依赖
pip install PyQt5 numpy

# 可选 (速度曲线)
pip install pyqtgraph

# 可选 (相机图像)
pip install opencv-python
```

## 手柄控制

| 按键/摇杆 | 功能 |
|-----------|------|
| LB (左肩键) | 按住启用手柄控制，松开切回网络控制 |
| 左摇杆 Y 轴 | 前进/后退 |
| 右摇杆 X 轴 | 左转/右转 |

## 话题

### 订阅

| 话题 | 类型 | 说明 |
|------|------|------|
| `/odom` | nav_msgs/Odometry | 里程计 |
| `/nn/local_trajectory` | LocalTrajectoryV4 | 网络轨迹 |
| `/cmd_unified` | UnifiedCmd | 控制命令 |
| `/controller/diagnostics` | DiagnosticsV2 | 诊断信息 |
| `/joy` | sensor_msgs/Joy | 手柄输入 |

### 发布

| 话题 | 类型 | 说明 |
|------|------|------|
| `/joy_cmd_vel` | geometry_msgs/Twist | 手柄控制命令 (由 cmd_vel_adapter 选择) |
| `/visualizer/control_mode` | std_msgs/Bool | 控制模式 (true=手柄, false=网络) |
| `/controller/emergency_stop` | std_msgs/Empty | 紧急停止 |

### 控制流程

```
                    ┌─────────────────┐
                    │  可视化器        │
                    │  (visualizer)   │
                    └────────┬────────┘
                             │
              ┌──────────────┴──────────────┐
              │                             │
              ▼                             ▼
    /joy_cmd_vel                  /visualizer/control_mode
    (手柄命令)                     (模式切换)
              │                             │
              └──────────────┬──────────────┘
                             │
                             ▼
                    ┌─────────────────┐
                    │ cmd_vel_adapter │
                    │ (命令选择器)     │
                    └────────┬────────┘
                             │
         ┌───────────────────┼───────────────────┐
         │                   │                   │
         ▼                   ▼                   ▼
  /cmd_unified         control_mode          /cmd_vel
  (控制器输出)         选择逻辑              (底盘输入)
```

## 配置

配置文件: `config/visualizer_params.yaml`

```yaml
# 手柄配置
joystick:
  enable_button: 4      # LB 键
  max_linear: 0.5       # 最大线速度 (m/s)
  max_angular: 1.0      # 最大角速度 (rad/s)
  deadzone: 0.1         # 摇杆死区

# 显示配置
display:
  update_rate: 30       # GUI 刷新率 (Hz)
  velocity_history_sec: 10  # 速度历史时长 (秒)
```
