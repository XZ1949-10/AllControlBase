# TurtleBot1 运行可视化模块

## 模块定位

本模块是**运行时可视化工具**，面向操作员和演示场景，提供：
- 轨迹和机器人位置的实时可视化
- 手柄控制接口
- 相机图像叠加

**与 Dashboard 的区别**：
| 特性 | Visualizer (本模块) | Dashboard |
|------|---------------------|-----------|
| 位置 | `controller_ros/visualizer/` | `universal_controller/dashboard/` |
| 用途 | 运行时操作/演示 | 开发调试/状态监控 |
| 数据源 | ROS 话题 (odom, trajectory, joy) | 控制器诊断数据 |
| ROS 依赖 | 必需 | 可选 (支持独立运行) |
| 主要功能 | 轨迹显示、手柄控制、相机叠加 | MPC 健康、状态机、超时监控 |
| 目标用户 | 操作员 | 开发者 |

## 功能

1. **轨迹可视化** - 俯视图或相机图像叠加显示轨迹
2. **速度监控** - 实时显示底盘线速度和角速度
3. **手柄控制** - Xbox 手柄控制机器人，LB 键切换控制模式
4. **相机轨迹叠加** - 使用单应性变换将轨迹投影到相机图像

## 架构

```
visualizer/
├── __init__.py           # 模块入口
├── models.py             # 数据模型 (与 ROS 解耦)
├── main_window.py        # 主窗口
├── homography.py         # 单应性变换 (相机轨迹投影)
├── adapters/             # 数据适配器
│   ├── joy_adapter.py    # 手柄消息适配
│   ├── image_adapter.py  # 图像消息适配
│   └── velocity_adapter.py # 速度数据适配
├── handlers/             # 业务逻辑处理器
│   ├── joystick_handler.py # 手柄控制逻辑
│   └── data_aggregator.py  # 数据聚合
├── widgets/              # UI 组件
│   ├── trajectory_view.py  # 轨迹视图 (支持俯视图和相机投影)
│   ├── velocity_panel.py   # 速度面板
│   ├── velocity_plot.py    # 速度曲线
│   ├── joystick_panel.py   # 手柄面板
│   └── status_bar.py       # 状态栏
└── node/                 # ROS 节点
    ├── ros_bridge.py     # ROS1/ROS2 兼容层
    └── visualizer_node.py # 可视化节点
```

## 使用方法

### 基本启动 (俯视图模式)

```bash
# 使用 launch 文件
roslaunch controller_ros tools/visualizer.launch

# 或通过 turtlebot1.launch
roslaunch controller_ros platforms/turtlebot1.launch visualizer:=true
```

### 相机轨迹叠加模式

需要先进行单应性标定：

```bash
# 1. 运行标定工具
roslaunch controller_ros tools/trajectory_visualizer.launch calibration_mode:=true

# 2. 按照提示完成标定 (点击4个地面标记点，输入坐标)

# 3. 启动带相机的可视化器
roslaunch controller_ros platforms/turtlebot1.launch \
    visualizer:=true \
    camera_image:=/usb_cam/image_raw
```

### 标定步骤

1. 在机器人前方地面放置 4 个标记点
2. 测量标记点相对于 `base_footprint` 的坐标 (x, y)
3. 启动标定模式
4. 按顺序点击图像中的 4 个标记点
5. 在终端输入对应的地面坐标 (格式: `x y`)
6. 按 `s` 保存标定结果
7. 标定文件保存在 `config/homography_calib.yaml`

推荐标定点:
- Point 1: (1.5, 0.0) - 正前方 1.5m
- Point 2: (2.5, 0.0) - 正前方 2.5m
- Point 3: (2.0, -0.5) - 前右方
- Point 4: (2.0, 0.5) - 前左方

## 依赖

```bash
# Python 依赖
pip install PyQt5 numpy

# 可选 (速度曲线)
pip install pyqtgraph

# 相机图像
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
| `/usb_cam/image_raw` | sensor_msgs/Image | 相机图像 (可选) |

### 发布

| 话题 | 类型 | 说明 |
|------|------|------|
| `/joy_cmd_vel` | geometry_msgs/Twist | 手柄控制命令 |
| `/visualizer/control_mode` | std_msgs/Bool | 控制模式 |
| `/controller/emergency_stop` | std_msgs/Empty | 紧急停止 |

## 配置

配置文件: `config/tools/visualizer_params.yaml`

```yaml
# 话题配置
topics:
  camera_image: ""              # 相机话题 (留空使用俯视图)

# 相机配置
camera:
  calibration_file: ""          # 单应性标定文件

# 手柄配置
joystick:
  enable_button: 4              # LB 键
  max_linear: 0.5               # 最大线速度 (m/s)
  max_angular: 1.0              # 最大角速度 (rad/s)

# 显示配置
display:
  update_rate: 30               # GUI 刷新率 (Hz)
```

## 相关文件

| 文件 | 说明 |
|------|------|
| `visualizer.launch` | 独立启动可视化器 |
| `trajectory_visualizer.launch` | 单应性标定工具 |
| `config/tools/visualizer_params.yaml` | 可视化器配置 |
| `config/homography_calib.yaml` | 单应性标定结果 |
