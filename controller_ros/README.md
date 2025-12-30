# controller_ros

ROS 胶水层 - 将 `universal_controller` 纯算法库与 ROS 生态系统集成。

**支持版本**: ROS1 Noetic (主要) / ROS2 Humble (备用)

## 概述

本包实现了 ROS 胶水层，负责：
- 订阅传感器数据 (`/odom`, `/imu`, `/nn/local_trajectory`)
- 消息格式转换 (ROS 消息 ↔ universal_controller 数据类型)
- TF2 集成 (管理坐标变换，注入到 universal_controller)
- 调用控制算法 (封装 `ControllerManager.update()`)
- 发布统一输出 (`/cmd_unified`, `/controller/diagnostics`, `/controller/state`)
- 紧急停止处理 (`/controller/emergency_stop`)
- 姿态控制接口 (四旋翼平台)

## 架构

```
┌─────────────────────────────────────────────────────────────────┐
│                    controller_ros (ROS 胶水层)                   │
├─────────────────────────────────────────────────────────────────┤
│  输入层 (Subscribers)                                           │
│    ├── OdomSubscriber (/odom)                                   │
│    ├── IMUSubscriber (/imu)                                     │
│    ├── TrajSubscriber (/nn/local_trajectory)                    │
│    └── EmergencyStopSubscriber (/controller/emergency_stop)     │
├─────────────────────────────────────────────────────────────────┤
│  适配器层 (Adapters)                                            │
│    ├── OdomAdapter (ROS → UC)                                   │
│    ├── ImuAdapter (ROS → UC)                                    │
│    ├── TrajectoryAdapter (ROS → UC)                             │
│    ├── OutputAdapter (UC → ROS)                                 │
│    └── AttitudeAdapter (UC → ROS, 四旋翼)                       │
├─────────────────────────────────────────────────────────────────┤
│  桥接层 (Bridge)                                                │
│    ├── ControllerBridge (封装 ControllerManager)                │
│    └── TFBridge (TF2 管理，注入到 coord_transformer)            │
├─────────────────────────────────────────────────────────────────┤
│  输出层 (Publishers)                                            │
│    ├── /cmd_unified (UnifiedCmd)                                │
│    ├── /controller/diagnostics (DiagnosticsV2)                  │
│    ├── /controller/state (Int32)                                │
│    └── /controller/attitude_cmd (AttitudeCmd, 四旋翼)           │
└─────────────────────────────────────────────────────────────────┘
```

## ROS 版本支持

本包支持 ROS1 和 ROS2 双版本，采用**统一架构**策略：

| 版本 | 入口文件 | IO 管理 | 状态 |
|------|---------|---------|------|
| ROS1 Noetic | `scripts/controller_node.py` | `ROS1PublisherManager`, `ROS1ServiceManager` | 主要支持 |
| ROS2 Humble | `src/controller_ros/node/controller_node.py` | `PublisherManager`, `ServiceManager` | 备用支持 |

**架构设计**:
- 两个版本共享 `ControllerNodeBase` 基类，包含所有共享逻辑
- ROS1 和 ROS2 各有对应的 `PublisherManager` 和 `ServiceManager`
- 适配器层 (`adapters/`) 完全共享
- 桥接层 (`bridge/`) 完全共享
- 数据管理 (`io/data_manager.py`) 完全共享

```
                    ┌─────────────────────────────┐
                    │    ControllerNodeBase       │
                    │    (共享控制逻辑)            │
                    └─────────────┬───────────────┘
                                  │
              ┌───────────────────┼───────────────────┐
              │                   │                   │
    ┌─────────▼─────────┐ ┌──────▼──────┐ ┌─────────▼─────────┐
    │ ControllerNodeROS1│ │  共享模块   │ │  ControllerNode   │
    │   (scripts/)      │ │ adapters/   │ │    (ROS2)         │
    │                   │ │ bridge/     │ │                   │
    │ ROS1Publisher     │ │ io/data_mgr │ │ PublisherManager  │
    │ ROS1ServiceMgr    │ │             │ │ ServiceManager    │
    └───────────────────┘ └─────────────┘ └───────────────────┘
```

## 安装

### 依赖

- ROS1 Noetic (Ubuntu 20.04) 或 ROS2 Humble (Ubuntu 22.04)
- universal_controller (纯算法库)
- numpy, scipy
- tf2_ros (可选，用于坐标变换)

### 安装 universal_controller

`universal_controller` 是纯 Python 算法库，需要确保其在 Python 路径中：

**方法 1: 设置 PYTHONPATH (推荐开发时使用)**
```bash
export PYTHONPATH=$PYTHONPATH:/path/to/AllControlBase
```

**方法 2: 安装为 Python 包**
```bash
cd /path/to/universal_controller
pip install -e .
```

### 构建 (ROS1)

```bash
cd ~/catkin_ws/src
ln -s /path/to/controller_ros .

export PYTHONPATH=$PYTHONPATH:/path/to/universal_controller/..

cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## 使用

### 启动

```bash
# 默认配置 (差速车)
roslaunch controller_ros controller.launch

# 指定平台
roslaunch controller_ros controller.launch platform:=omni
roslaunch controller_ros controller.launch platform:=quadrotor
roslaunch controller_ros controller.launch platform:=ackermann

# 使用仿真时间
roslaunch controller_ros controller.launch use_sim_time:=true
```

### 话题

话题命名规范:
- 输入话题: `/controller/input/<name>`
- 输出话题: `/controller/<name>`

#### 订阅 (输入)

| 话题 | 类型 | 说明 |
|------|------|------|
| `/controller/input/odom` | nav_msgs/Odometry | 里程计 |
| `/controller/input/trajectory` | controller_ros/LocalTrajectoryV4 | 轨迹输入 |
| `/controller/emergency_stop` | std_msgs/Empty | 紧急停止信号 |

注意: IMU 话题默认禁用，需要时在配置中启用。

#### 发布 (输出)

| 话题 | 类型 | 说明 |
|------|------|------|
| `/controller/cmd` | controller_ros/UnifiedCmd | 统一控制命令 |
| `/controller/diagnostics` | controller_ros/DiagnosticsV2 | 诊断信息 (降频发布) |
| `/controller/state` | std_msgs/Int32 | 控制器状态 (每次控制循环发布) |
| `/controller/debug_path` | nav_msgs/Path | 调试路径 (用于 RViz 可视化) |
| `/controller/attitude_cmd` | controller_ros/AttitudeCmd | 姿态命令 (仅四旋翼) |

**状态值说明** (ControllerState 枚举):
- 0: INIT - 初始化
- 1: NORMAL - 正常运行
- 2: SOFT_DISABLED - Soft Head 禁用
- 3: MPC_DEGRADED - MPC 降级
- 4: BACKUP_ACTIVE - 备用控制器激活
- 5: STOPPING - 正在停止
- 6: STOPPED - 已停止

### 服务

| 服务 | 类型 | 说明 |
|------|------|------|
| `/controller/reset` | std_srvs/Trigger | 重置控制器 |
| `/controller/set_state` | controller_ros/SetControllerState | 设置控制器状态 (仅支持 STOPPING) |
| `/controller/get_diagnostics` | controller_ros/GetDiagnostics | 获取诊断信息 |
| `/controller/set_hover_yaw` | controller_ros/SetHoverYaw | 设置悬停航向 (仅四旋翼) |
| `/controller/get_attitude_rate_limits` | controller_ros/GetAttitudeRateLimits | 获取姿态角速度限制 (仅四旋翼) |

### 紧急停止

发送空消息到 `/controller/emergency_stop` 话题即可触发紧急停止：

```bash
rostopic pub /controller/emergency_stop std_msgs/Empty "{}"
```

紧急停止后，控制器会：
1. 立即发布零速度命令
2. 请求进入 STOPPING 状态
3. 在诊断信息中标记 `emergency_stop=true`

通过 `/controller/reset` 服务可以清除紧急停止状态。

## TF2 集成

本包支持 TF2 坐标变换集成：

1. **自动检测**: 启动时自动检测 tf2_ros 是否可用
2. **回调注入**: 将 TF2 查找回调注入到 `universal_controller` 的 `RobustCoordinateTransformer`
3. **降级处理**: TF2 不可用时自动降级到 odom 积分
4. **运行时重试**: 如果初始化时 TF2 未就绪，会在运行时周期性重试

```python
# TF2 注入流程
TFBridge.inject_to_transformer(coord_transformer)
    └── coord_transformer.set_tf2_lookup_callback(tf_bridge.lookup_transform)
```

## 配置

### 参数文件

配置文件位于 `config/controller_params.yaml`：

```yaml
# 系统配置
system:
  ctrl_freq: 50                   # 控制频率 (Hz)
  platform: "differential"        # 平台类型

# 话题配置
topics:
  odom: "/odom"
  imu: "/imu"
  trajectory: "/nn/local_trajectory"
  cmd_unified: "/cmd_unified"
  diagnostics: "/controller/diagnostics"
  emergency_stop: "/controller/emergency_stop"
  attitude_cmd: "/controller/attitude_cmd"

# TF 配置
tf:
  source_frame: "base_link"
  target_frame: "odom"
  timeout_ms: 10

# 超时配置
watchdog:
  odom_timeout_ms: 200
  traj_timeout_ms: 500
  imu_timeout_ms: 100

# MPC 配置
mpc:
  horizon: 20
  dt: 0.1

# 姿态控制配置 (四旋翼)
attitude:
  mass: 1.5
  roll_max: 0.5
  pitch_max: 0.5
```

### 配置映射

ROS 参数与 `universal_controller` 配置的映射关系：

| ROS 参数 | UC 配置 |
|----------|---------|
| `system.ctrl_freq` | `system.ctrl_freq` |
| `system.platform` | `system.platform` |
| `watchdog.*` | `watchdog.*` |
| `mpc.*` | `mpc.*` |
| `attitude.*` | `attitude.*` |
| `tf.*` | `transform.*` |

## 平台适配

本包输出统一的 `/cmd_unified` 消息，由独立的平台适配层转换为平台特定命令：

```
/cmd_unified (UnifiedCmd)
    │
    ├── 差速车适配器 → /cmd_vel (Twist)
    ├── 全向车适配器 → /cmd_vel (Twist)
    ├── 阿克曼适配器 → /ackermann_cmd (Ackermann)
    └── 四旋翼适配器 → /mavros/setpoint (TwistStamped)
```

### 四旋翼平台

四旋翼平台额外提供姿态控制接口：

- **姿态命令话题**: `/controller/attitude_cmd` (AttitudeCmd)
- **设置悬停航向**: `/controller/set_hover_yaw` 服务
- **获取角速度限制**: `/controller/get_attitude_rate_limits` 服务

姿态命令包含：
- `roll`, `pitch`, `yaw`: 姿态角 (rad)
- `thrust`: 推力 (归一化，1.0 = 悬停)
- `yaw_mode`: 航向模式 (0=跟随速度, 1=固定, 2=朝向目标, 3=手动)
- `is_hovering`: 是否处于悬停状态

## Dashboard 可视化

本包支持启动 Dashboard 可视化监控界面，实时显示控制器状态。

### 架构说明 (v3.20 更新)

Dashboard 数据源已重构为清晰的分层架构：

```
universal_controller/dashboard/     # ROS 无关的 Dashboard 核心
├── models.py                       # 数据模型定义
├── data_source.py                  # DashboardDataSource (直接访问 ControllerManager)
├── main_window.py                  # UI 实现
└── ros_data_source.py              # 兼容性包装器 (已弃用)

controller_ros/dashboard/           # ROS 数据源适配器 (v3.20 新增)
└── ros_data_source.py              # ROSDashboardDataSource (订阅 ROS 话题)
```

**迁移原因**: `ROSDashboardDataSource` 依赖 `controller_ros.msg.DiagnosticsV2`，
将其放在 `universal_controller` 中违反了分层架构原则。

**迁移指南**:
```python
# 旧代码 (已弃用，会发出警告)
from universal_controller.dashboard import ROSDashboardDataSource

# 新代码 (推荐)
from controller_ros.dashboard import ROSDashboardDataSource
```

### Dashboard vs Visualizer

本项目提供两个可视化工具，面向不同使用场景：

| 特性 | Dashboard | Visualizer |
|------|-----------|------------|
| 位置 | `universal_controller/dashboard/` | `controller_ros/visualizer/` |
| 用途 | 开发调试/状态监控 | 运行时操作/演示 |
| 数据源 | 控制器诊断数据 | ROS 话题 (odom, trajectory, joy) |
| 主要功能 | MPC 健康、状态机、超时监控、一致性分析 | 轨迹显示、手柄控制、相机叠加 |
| 目标用户 | 开发者 | 操作员 |
| 启动方式 | `roslaunch controller_ros dashboard.launch` | `roslaunch controller_ros visualizer.launch` |

### 启动方式

```bash
# 方式 1: 控制器 + Dashboard 一起启动
roslaunch controller_ros controller.launch dashboard:=true

# 方式 2: 单独启动 Dashboard (需要先启动控制器)
roslaunch controller_ros dashboard.launch

# 方式 3: 直接运行 Dashboard 节点
rosrun controller_ros dashboard_node.py
```

### 依赖

Dashboard 需要 PyQt5:
```bash
pip install PyQt5
```

### 功能

Dashboard 订阅 `/controller/diagnostics` 话题，显示：
- 系统状态和 7 级降级状态
- MPC 健康状态 (求解时间、KKT残差、条件数)
- 一致性分析 (α_soft、曲率、速度方向、时序平滑)
- 超时监控 (Odom/Traj/IMU)
- 跟踪误差
- 控制输出
- 状态估计 (EKF)
- 运行统计
- 警告日志

## 测试

```bash
cd controller_ros
python -m pytest test/ -v
```

## 目录结构

```
controller_ros/
├── package.xml              # ROS1 包描述
├── CMakeLists.txt           # catkin 构建配置
├── setup.py                 # Python 包配置
├── config/
│   ├── controller_params.yaml
│   ├── differential.yaml
│   ├── omni.yaml
│   ├── quadrotor.yaml
│   └── ackermann.yaml
├── launch/
│   ├── controller.launch    # ROS1 launch 文件 (支持 dashboard:=true)
│   ├── dashboard.launch     # Dashboard 独立启动
│   └── controller.launch.py # ROS2 launch 文件 (备用)
├── msg/
│   ├── LocalTrajectoryV4.msg
│   ├── UnifiedCmd.msg
│   ├── DiagnosticsV2.msg
│   └── AttitudeCmd.msg      # 姿态命令 (四旋翼)
├── srv/
│   ├── SetControllerState.srv
│   ├── GetDiagnostics.srv
│   ├── SetHoverYaw.srv      # 设置悬停航向 (四旋翼)
│   └── GetAttitudeRateLimits.srv  # 获取角速度限制 (四旋翼)
├── scripts/
│   ├── controller_node.py   # 主节点 (ROS1)
│   └── dashboard_node.py    # Dashboard 节点 (ROS1)
├── src/controller_ros/      # Python 模块
│   ├── adapters/            # 消息适配器
│   │   ├── odom_adapter.py
│   │   ├── imu_adapter.py
│   │   ├── trajectory_adapter.py
│   │   ├── output_adapter.py
│   │   └── attitude_adapter.py  # 姿态适配器 (四旋翼)
│   ├── bridge/              # 控制器和 TF2 桥接
│   ├── dashboard/           # ROS Dashboard 数据源 (v3.20 新增)
│   │   └── ros_data_source.py  # ROSDashboardDataSource
│   ├── io/                  # 订阅/发布管理
│   ├── lifecycle/           # 生命周期管理
│   ├── node/                # 节点基类和 ROS2 节点
│   └── utils/               # 工具函数和 ROS 兼容层
└── test/
    ├── test_adapters.py
    ├── test_bridge.py
    └── test_diagnostics_publisher.py
```

## 许可证

MIT License
