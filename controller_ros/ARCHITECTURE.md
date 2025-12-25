# controller_ros 架构文档

## 概述

`controller_ros` 是 ROS 胶水层，将 `universal_controller` 纯算法库与 ROS 生态系统集成。

## 架构设计原则

### 1. 统一架构 (ROS1/ROS2)

ROS1 和 ROS2 版本采用统一架构，最大化代码复用：

```
                    ┌─────────────────────────────┐
                    │    ControllerNodeBase       │
                    │    (共享控制逻辑)            │
                    │    - 控制循环核心            │
                    │    - TF2 注入               │
                    │    - 紧急停止处理            │
                    │    - 姿态控制接口            │
                    └─────────────┬───────────────┘
                                  │
              ┌───────────────────┼───────────────────┐
              │                   │                   │
    ┌─────────▼─────────┐ ┌──────▼──────┐ ┌─────────▼─────────┐
    │ ControllerNodeROS1│ │  共享模块   │ │  ControllerNode   │
    │   (scripts/)      │ │             │ │    (ROS2)         │
    │                   │ │ adapters/   │ │                   │
    │ ROS1Publisher     │ │ bridge/     │ │ PublisherManager  │
    │ ROS1ServiceMgr    │ │ io/data_mgr │ │ ServiceManager    │
    └───────────────────┘ └─────────────┘ └───────────────────┘
```

### 2. 分层架构

```
┌─────────────────────────────────────────────────────────────────┐
│                         节点层 (Node)                           │
│  - ControllerNodeBase: 共享逻辑基类                             │
│  - ControllerNodeROS1: ROS1 实现                                │
│  - ControllerNode: ROS2 实现                                    │
├─────────────────────────────────────────────────────────────────┤
│                         IO 层 (IO)                              │
│  - DataManager: 数据缓存管理 (ROS 无关)                         │
│  - ROS1PublisherManager / PublisherManager: 发布管理            │
│  - ROS1ServiceManager / ServiceManager: 服务管理                │
├─────────────────────────────────────────────────────────────────┤
│                        桥接层 (Bridge)                          │
│  - ControllerBridge: 封装 ControllerManager                     │
│  - TFBridge: TF2 管理和注入                                     │
├─────────────────────────────────────────────────────────────────┤
│                       适配器层 (Adapters)                       │
│  - OdomAdapter: nav_msgs/Odometry ↔ UC Odometry                │
│  - ImuAdapter: sensor_msgs/Imu ↔ UC Imu                        │
│  - TrajectoryAdapter: LocalTrajectoryV4 ↔ UC Trajectory        │
│  - OutputAdapter: UC ControlOutput → UnifiedCmd                 │
│  - AttitudeAdapter: UC AttitudeCommand → AttitudeCmd           │
├─────────────────────────────────────────────────────────────────┤
│                        工具层 (Utils)                           │
│  - ParamLoader: 参数加载                                        │
│  - TimeSync: 时间同步                                           │
│  - DiagnosticsThrottler: 诊断节流                               │
│  - TF2InjectionManager: TF2 注入管理                            │
│  - ros_compat: ROS1/ROS2 兼容层                                 │
└─────────────────────────────────────────────────────────────────┘
```

## 模块说明

### 节点层 (node/)

| 文件 | 说明 |
|------|------|
| `base_node.py` | 节点基类，包含所有共享逻辑 |
| `controller_node.py` | ROS2 节点实现 |
| `scripts/controller_node.py` | ROS1 节点实现 |

**共享逻辑 (ControllerNodeBase)**:
- 控制循环核心 (`_control_loop_core`)
- TF2 注入 (`_inject_tf2_to_controller`)
- 紧急停止处理 (`_handle_emergency_stop`)
- 姿态控制接口 (`_try_publish_attitude_command`)
- 服务处理 (`_handle_reset`, `_handle_set_state`, etc.)

### IO 层 (io/)

| 文件 | 说明 |
|------|------|
| `data_manager.py` | 数据缓存管理，ROS 无关 |
| `publishers.py` | ROS2 发布管理器 |
| `services.py` | ROS2 服务管理器 |
| `ros1_publishers.py` | ROS1 发布管理器 |
| `ros1_services.py` | ROS1 服务管理器 |

**设计说明**:
- `DataManager` 完全 ROS 无关，可独立测试
- ROS1/ROS2 各有对应的 Manager 类，接口对齐
- Manager 类封装发布/服务创建和消息转换

### 桥接层 (bridge/)

| 文件 | 说明 |
|------|------|
| `controller_bridge.py` | 封装 ControllerManager 调用 |
| `tf_bridge.py` | TF2 管理和注入 |

**职责分离**:
- `ControllerBridge`: 隔离 ROS 层与算法库
- `TFBridge`: 管理 TF2 buffer，提供查找回调

### 适配器层 (adapters/)

| 文件 | 说明 |
|------|------|
| `odom_adapter.py` | 里程计消息转换 |
| `imu_adapter.py` | IMU 消息转换 |
| `trajectory_adapter.py` | 轨迹消息转换 |
| `output_adapter.py` | 控制输出转换 |
| `attitude_adapter.py` | 姿态命令转换 (四旋翼) |

**接口统一**:
- 所有适配器实现 `IMsgConverter` 接口
- `to_uc()`: ROS → UC
- `to_ros()`: UC → ROS

## 可视化模块

### Dashboard vs Visualizer

| 特性 | Dashboard | Visualizer |
|------|-----------|------------|
| 位置 | `universal_controller/dashboard/` | `controller_ros/visualizer/` |
| 用途 | 开发调试/状态监控 | 运行时操作/演示 |
| 数据源 | 控制器诊断数据 | ROS 话题 |
| ROS 依赖 | 可选 | 必需 |
| 主要功能 | MPC 健康、状态机、超时监控 | 轨迹显示、手柄控制、相机叠加 |
| 目标用户 | 开发者 | 操作员 |

### Visualizer 架构

```
visualizer/
├── adapters/           # 数据适配器 (ROS → 可视化数据)
├── handlers/           # 业务逻辑处理器
├── widgets/            # UI 组件 (PyQt5)
├── node/               # ROS 节点
├── homography.py       # 单应性变换
├── main_window.py      # 主窗口
└── models.py           # 数据模型
```

## 四旋翼平台支持

四旋翼平台额外提供：

**话题**:
- `/controller/attitude_cmd` (AttitudeCmd)

**服务**:
- `/controller/set_hover_yaw` (SetHoverYaw)
- `/controller/get_attitude_rate_limits` (GetAttitudeRateLimits)

**实现**:
- `AttitudeAdapter`: 姿态命令转换
- `ControllerNodeBase._try_publish_attitude_command()`: 姿态命令发布
- `PublisherManager.publish_attitude_cmd()`: ROS2 发布
- `ROS1PublisherManager.publish_attitude_cmd()`: ROS1 发布

## 测试

```bash
# 运行所有测试
python -m pytest controller_ros/test/ -v

# 运行特定测试
python -m pytest controller_ros/test/test_data_manager.py -v
python -m pytest controller_ros/test/test_adapters.py -v
python -m pytest controller_ros/test/test_bridge.py -v
```

## 版本历史

### v1.1.0 (2024-12-25)
- 统一 ROS1/ROS2 架构
- 添加 `ROS1PublisherManager` 和 `ROS1ServiceManager`
- 补充 ROS2 四旋翼平台支持
- 更新文档，明确 Dashboard 和 Visualizer 定位

### v1.0.0
- 初始版本
