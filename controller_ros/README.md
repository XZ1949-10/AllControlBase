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

## 架构

```
┌─────────────────────────────────────────────────────────────────┐
│                    controller_ros (ROS 胶水层)                   │
├─────────────────────────────────────────────────────────────────┤
│  输入层 (Subscribers)                                           │
│    ├── OdomSubscriber (/odom)                                   │
│    ├── IMUSubscriber (/imu)                                     │
│    └── TrajSubscriber (/nn/local_trajectory)                    │
├─────────────────────────────────────────────────────────────────┤
│  适配器层 (Adapters)                                            │
│    ├── OdomAdapter (ROS → UC)                                   │
│    ├── ImuAdapter (ROS → UC)                                    │
│    ├── TrajectoryAdapter (ROS → UC)                             │
│    └── OutputAdapter (UC → ROS)                                 │
├─────────────────────────────────────────────────────────────────┤
│  桥接层 (Bridge)                                                │
│    ├── ControllerBridge (封装 ControllerManager)                │
│    └── TFBridge (TF2 管理，注入到 coord_transformer)            │
├─────────────────────────────────────────────────────────────────┤
│  输出层 (Publishers)                                            │
│    ├── /cmd_unified (UnifiedCmd)                                │
│    ├── /controller/diagnostics (DiagnosticsV2)                  │
│    └── /controller/state (Int32)                                │
└─────────────────────────────────────────────────────────────────┘
```

## ROS 版本支持

本包支持 ROS1 和 ROS2 双版本，采用分离实现策略：

| 版本 | 入口文件 | 模块位置 | 状态 |
|------|---------|---------|------|
| ROS1 Noetic | `scripts/controller_node.py` | 独立实现 | 主要支持 |
| ROS2 Humble | `src/controller_ros/node/controller_node.py` | 模块化实现 | 备用支持 |

**设计说明**:
- ROS1 版本在 `scripts/` 中是独立的单文件实现，便于直接运行
- ROS2 版本在 `src/` 中是模块化实现，使用 `io/`, `bridge/`, `adapters/` 等子模块
- 两个版本共享 `universal_controller` 算法库，但 ROS 接口层分别实现
- 构建系统使用 catkin (ROS1)，ROS2 版本需要单独配置 ament_python

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
# 假设 universal_controller 和 controller_ros 在同一父目录下
export PYTHONPATH=$PYTHONPATH:/path/to/AllControlBase
```

**方法 2: 安装为 Python 包**
```bash
cd /path/to/universal_controller
pip install -e .
```

**方法 3: 在 .bashrc 中永久设置**
```bash
echo 'export PYTHONPATH=$PYTHONPATH:/path/to/AllControlBase' >> ~/.bashrc
source ~/.bashrc
```

### 构建 (ROS1)

```bash
# 在 catkin 工作空间中
cd ~/catkin_ws/src
ln -s /path/to/controller_ros .

# 确保 universal_controller 在 Python 路径中
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

#### 订阅 (输入)

| 话题 | 类型 | 说明 |
|------|------|------|
| `/odom` | nav_msgs/Odometry | 里程计 |
| `/imu` | sensor_msgs/Imu | IMU (可选) |
| `/nn/local_trajectory` | controller_ros/LocalTrajectoryV4 | 网络预测轨迹 |

#### 发布 (输出)

| 话题 | 类型 | 说明 |
|------|------|------|
| `/cmd_unified` | controller_ros/UnifiedCmd | 统一控制命令 |
| `/controller/diagnostics` | controller_ros/DiagnosticsV2 | 诊断信息 (降频发布) |
| `/controller/state` | std_msgs/Int32 | 控制器状态 (每次控制循环发布) |

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

## TF2 集成

本包支持 TF2 坐标变换集成：

1. **自动检测**: 启动时自动检测 tf2_ros 是否可用
2. **回调注入**: 将 TF2 查找回调注入到 `universal_controller` 的 `RobustCoordinateTransformer`
3. **降级处理**: TF2 不可用时自动降级到 odom 积分

```python
# TF2 注入流程
TFBridge.inject_to_transformer(coord_transformer)
    └── coord_transformer.set_tf2_lookup_callback(tf_bridge.lookup_transform)
```

## 配置

### 参数文件

```yaml
# config/controller_params.yaml
node:
  control_rate: 50.0

topics:
  odom: "/odom"
  imu: "/imu"
  trajectory: "/nn/local_trajectory"
  cmd_unified: "/cmd_unified"
  diagnostics: "/controller/diagnostics"

platform:
  type: "differential"  # differential/omni/ackermann/quadrotor

tf:
  source_frame: "base_link"
  target_frame: "odom"
  timeout_sec: 0.01

time_sync:
  max_odom_age_ms: 100
  max_traj_age_ms: 200
  max_imu_age_ms: 50
```

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

## 测试

```bash
# 运行单元测试 (不需要 ROS 环境)
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
│   ├── controller.launch    # ROS1 launch 文件
│   └── controller.launch.py # ROS2 launch 文件 (备用)
├── msg/
│   ├── LocalTrajectoryV4.msg
│   ├── UnifiedCmd.msg
│   └── DiagnosticsV2.msg
├── srv/
│   ├── SetControllerState.srv
│   └── GetDiagnostics.srv
├── scripts/
│   └── controller_node.py   # 主节点 (ROS1)
├── src/controller_ros/      # Python 模块
│   ├── adapters/            # 消息适配器
│   ├── bridge/              # 控制器和 TF2 桥接
│   ├── io/                  # 订阅/发布管理
│   ├── node/                # ROS2 节点 (备用)
│   └── utils/               # 工具函数和 ROS 兼容层
└── test/
    ├── test_adapters.py
    └── test_bridge.py
```

## 许可证

MIT License
