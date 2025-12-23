# TurtleBot1 + ROS Noetic 完整部署指南

> 版本: v1.0.0 | 最后更新: 2024-12-23

本文档提供在 TurtleBot1 机器人上部署 `universal_controller` + `controller_ros` 的完整流程。

---

## 🚀 快速开始 (最佳部署方案)

如果你想快速部署，按以下步骤执行：

```bash
# ========== 1. 部署 universal_controller (最佳方案: pip 可编辑安装) ==========
cd /path/to/AllControlBase/universal_controller

# 安装依赖
pip install numpy scipy PyYAML

# 可编辑安装 (推荐！修改代码无需重装)
pip install -e .

# 验证
python3 -c "from universal_controller import ControllerManager; print('OK')"

# ========== 2. 部署 controller_ros ==========
cd ~/catkin_ws/src
ln -s /path/to/AllControlBase/controller_ros .
cd ~/catkin_ws
catkin_make
source devel/setup.bash

# ========== 3. 启动 ==========
# 终端1: TurtleBot 驱动
roslaunch turtlebot_bringup minimal.launch

# 终端2: 控制器
roslaunch controller_ros turtlebot1.launch

# 终端3: 你的轨迹发布器
rosrun your_package trajectory_publisher.py
```

---

## 目录

1. [系统架构概览](#1-系统架构概览)
2. [必需的输入话题](#2-必需的输入话题)
3. [部署流程](#3-部署流程)
4. [轨迹格式与发布](#4-轨迹格式与发布)
5. [cmd_vel 适配器](#5-cmd_vel-适配器)
6. [完整启动流程](#6-完整启动流程)
7. [最佳效果配置](#7-最佳效果配置)
8. [故障排除](#8-故障排除)

---

## 1. 系统架构概览

### 1.1 整体数据流

```
┌─────────────────────────────────────────────────────────────────────────┐
│                           你的神经网络                                   │
│                    (输出局部轨迹 base_link 坐标系)                        │
└─────────────────────────────────┬───────────────────────────────────────┘
                                  │ /nn/local_trajectory
                                  ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                    controller_ros (ROS 胶水层)                           │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐                   │
│  │ /odom 订阅   │  │ /imu 订阅    │  │ TF2 查询     │                   │
│  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘                   │
│         └─────────────────┼─────────────────┘                           │
│                           ▼                                             │
│              ┌────────────────────────┐                                 │
│              │  universal_controller  │                                 │
│              │    (MPC 算法核心)       │                                 │
│              └────────────┬───────────┘                                 │
│                           ▼                                             │
│              ┌────────────────────────┐                                 │
│              │   /cmd_unified 发布    │                                 │
│              └────────────┬───────────┘                                 │
└───────────────────────────┼─────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                    cmd_vel 适配器                                        │
│              /cmd_unified → /cmd_vel (geometry_msgs/Twist)              │
└─────────────────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                         TurtleBot1 底盘                                  │
└─────────────────────────────────────────────────────────────────────────┘
```

### 1.2 坐标系说明

本控制器**不需要建图或定位**，使用以下两个坐标系：

```
base_link (机体坐标系)              odom (里程计坐标系)
┌───────────────┐                   ┌─────────────────────┐
│       ↑ X     │                   │                     │
│       │       │    坐标变换        │    机器人轨迹       │
│    ←──┼──→    │  ───────────→     │    ○──○──○──○       │
│     Y │       │  base_link→odom   │                     │
│       ↓       │                   │    启动位置 ●       │
└───────────────┘                   └─────────────────────┘

- 原点在机器人中心                   - 从启动位置开始累积
- X轴朝前                            - 会有漂移（正常）
- 随机器人移动                       - 不需要建图/定位
```

### 1.3 数据流详解

```
网络输出轨迹 (base_link, 局部坐标，当前位置为原点)
    ↓
坐标变换 (base_link → odom)
    ↓
控制器计算 (在 odom 坐标系下)
    ↓
控制输出: vx, omega (base_link 坐标系)
    ↓
cmd_vel 适配器转换
    ↓
TurtleBot 执行
```

---

## 2. 必需的输入话题

### 2.1 输入话题列表

| 话题 | 类型 | 必需 | 说明 |
|------|------|------|------|
| `/odom` | `nav_msgs/Odometry` | ✅ 必需 | TurtleBot 里程计 |
| `/imu` | `sensor_msgs/Imu` | ❌ 可选 | IMU 数据 (TurtleBot1 可能没有) |
| `/nn/local_trajectory` | `controller_ros/LocalTrajectoryV4` | ✅ 必需 | 神经网络输出的轨迹 |
| TF: `base_link` → `odom` | tf2 | ✅ 必需 | 坐标变换 (TurtleBot 自动发布) |

### 2.2 输出话题列表

| 话题 | 类型 | 说明 |
|------|------|------|
| `/cmd_unified` | `controller_ros/UnifiedCmd` | 控制器统一输出 |
| `/cmd_vel` | `geometry_msgs/Twist` | TurtleBot 速度命令 |
| `/controller/diagnostics` | `controller_ros/DiagnosticsV2` | 诊断信息 |
| `/controller/state` | `std_msgs/Int32` | 控制器状态 |

### 2.3 服务列表

| 服务 | 类型 | 说明 |
|------|------|------|
| `/controller/reset` | `std_srvs/Trigger` | 重置控制器 |
| `/controller/set_state` | `controller_ros/SetControllerState` | 设置控制器状态 |
| `/controller/get_diagnostics` | `controller_ros/GetDiagnostics` | 获取诊断信息 |

---

## 3. 部署流程

### 3.1 环境准备

```bash
# 确保 ROS Noetic 已安装
source /opt/ros/noetic/setup.bash

# 创建 catkin 工作空间 (如果没有)
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 3.2 部署 universal_controller (纯算法库) ⭐ 最佳方案

`universal_controller` 是纯 Python 算法库，不依赖 ROS。

#### 🏆 推荐方案：pip 可编辑安装 + 依赖完整安装

这是**效果最好**的部署方案，原因：
- ✅ 自动处理所有依赖
- ✅ 支持实时修改代码无需重新安装
- ✅ 与 ROS 环境完美兼容
- ✅ 支持 ACADOS 高性能 MPC 求解器

```bash
# 1. 进入 universal_controller 目录
cd /home/user/AllControlBase/universal_controller

# 2. 安装所有依赖
pip install numpy scipy PyYAML

# 3. (可选但强烈推荐) 安装 ACADOS 以获得最佳 MPC 性能
# ACADOS 可以将 MPC 求解时间从 50ms 降低到 5-15ms
# 安装方法见下方 "ACADOS 安装" 部分

# 4. 可编辑安装 universal_controller
pip install -e .

# 5. 验证安装
python3 -c "
from universal_controller import ControllerManager, DEFAULT_CONFIG
from universal_controller.core.enums import ControllerState
print('✅ universal_controller 安装成功!')
print(f'   支持平台: differential, omni, ackermann, quadrotor')
print(f'   控制器状态: {[s.name for s in ControllerState]}')
"
```

#### ACADOS 安装 (可选，推荐)

ACADOS 是高性能 MPC 求解器，可显著提升控制性能：

```bash
# 1. 安装依赖
sudo apt install cmake build-essential

# 2. 克隆 ACADOS
cd ~
git clone https://github.com/acados/acados.git
cd acados
git submodule update --recursive --init

# 3. 编译
mkdir -p build && cd build
cmake -DACADOS_WITH_QPOASES=ON ..
make -j4
sudo make install

# 4. 设置环境变量
echo 'export ACADOS_SOURCE_DIR=~/acados' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/acados/lib' >> ~/.bashrc
source ~/.bashrc

# 5. 安装 Python 接口
pip install ~/acados/interfaces/acados_template
```

#### 备选方案对比

| 方案 | 优点 | 缺点 | 推荐场景 |
|------|------|------|----------|
| **pip install -e . (推荐)** | 自动依赖、可编辑、稳定 | 需要 pip | 生产环境、长期使用 |
| PYTHONPATH | 简单快速 | 依赖需手动装、易出错 | 临时测试 |
| pip install . | 稳定 | 修改代码需重装 | 只运行不修改 |

#### 备选方案 1: PYTHONPATH (仅临时测试用)

```bash
# 假设代码在 /home/user/AllControlBase
export PYTHONPATH=$PYTHONPATH:/home/user/AllControlBase

# 添加到 .bashrc 永久生效
echo 'export PYTHONPATH=$PYTHONPATH:/home/user/AllControlBase' >> ~/.bashrc
source ~/.bashrc

# 手动安装依赖
pip install numpy scipy PyYAML
```

#### 验证安装

```bash
# 基础验证
python3 -c "from universal_controller import ControllerManager; print('universal_controller OK')"

# 完整验证
python3 -c "
from universal_controller import ControllerManager, DEFAULT_CONFIG
from universal_controller.core.ros_compat import ROS_AVAILABLE, TF2_AVAILABLE

config = DEFAULT_CONFIG.copy()
config['system']['platform'] = 'differential'
manager = ControllerManager(config)
manager.initialize_default_components()

print('✅ ControllerManager 初始化成功')
print(f'   ROS 可用: {ROS_AVAILABLE}')
print(f'   TF2 可用: {TF2_AVAILABLE}')
print(f'   平台: {config[\"system\"][\"platform\"]}')
print(f'   控制频率: {config[\"system\"][\"ctrl_freq\"]} Hz')

manager.shutdown()
print('✅ 验证完成!')
"
```

### 3.3 部署 controller_ros (ROS 胶水层)

```bash
# 链接到 catkin 工作空间
cd ~/catkin_ws/src
ln -s /home/user/AllControlBase/controller_ros .

# 确保 PYTHONPATH 包含 universal_controller
export PYTHONPATH=$PYTHONPATH:/home/user/AllControlBase

# 编译
cd ~/catkin_ws
catkin_make

# 加载环境
source devel/setup.bash
```

### 3.4 验证编译

```bash
# 检查包是否可用
rospack find controller_ros

# 检查消息是否编译成功
rosmsg show controller_ros/LocalTrajectoryV4
rosmsg show controller_ros/UnifiedCmd
rosmsg show controller_ros/DiagnosticsV2
```

---

## 4. 轨迹格式与发布

### 4.1 LocalTrajectoryV4 消息格式

```
std_msgs/Header header
  - stamp: 时间戳
  - frame_id: "base_link" (重要!)

uint8 mode
  - 0: MODE_TRACK (跟踪模式)
  - 1: MODE_STOP (停止模式)
  - 2: MODE_HOVER (悬停模式)
  - 3: MODE_EMERGENCY (紧急模式)

geometry_msgs/Point[] points
  - 轨迹点序列 [x, y, z]
  - z 对于地面机器人通常为 0

float32[] velocities_flat
  - 速度数组 (可选)
  - 格式: [vx0, vy0, vz0, wz0, vx1, vy1, vz1, wz1, ...]
  - 长度 = len(points) * 4

float32 dt_sec
  - 轨迹点时间间隔 (秒)

float32 confidence
  - 轨迹置信度 [0, 1]

bool soft_enabled
  - 是否启用 Soft 约束
```

### 4.2 神经网络输出要求

你的神经网络输出需要满足：

1. **坐标系**: `base_link` (机器人当前位置为原点，X轴朝前)
2. **轨迹点数**: 建议 10-30 个点
3. **时间间隔**: 建议 0.1s (与 MPC dt 匹配)
4. **数据内容**:
   - `points`: 必需，[x, y, z] 坐标序列
   - `velocities_flat`: 可选，速度序列
   - `confidence`: 可选，轨迹置信度 (0-1)

### 4.3 轨迹发布器示例代码

创建文件 `trajectory_publisher.py`:

```python
#!/usr/bin/env python3
"""
轨迹发布器示例 - 将神经网络输出转换为 LocalTrajectoryV4
"""
import rospy
import numpy as np
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from controller_ros.msg import LocalTrajectoryV4


class TrajectoryPublisher:
    def __init__(self):
        rospy.init_node('trajectory_publisher')
        self.pub = rospy.Publisher('/nn/local_trajectory', LocalTrajectoryV4, queue_size=1)
        self.rate = rospy.Rate(10)  # 10Hz 发布频率
    
    def publish_trajectory(self, positions, velocities=None, dt=0.1, confidence=1.0):
        """
        发布轨迹
        
        Args:
            positions: numpy array, shape (N, 2) 或 (N, 3)
                       [x, y] 或 [x, y, theta]
                       坐标系: base_link (机器人当前位置为原点，X轴朝前)
            velocities: numpy array, shape (N, 2), [vx, vy] (可选)
            dt: 轨迹点时间间隔
            confidence: 轨迹置信度 [0, 1]
        """
        msg = LocalTrajectoryV4()
        
        # Header
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_link"  # 重要：必须是 base_link
        
        # 轨迹模式
        msg.mode = 0  # MODE_TRACK
        
        # 轨迹点
        msg.points = []
        for i in range(len(positions)):
            p = Point()
            p.x = float(positions[i, 0])
            p.y = float(positions[i, 1])
            p.z = 0.0  # 地面机器人 z=0
            msg.points.append(p)
        
        # 速度 (可选)
        if velocities is not None:
            msg.velocities_flat = []
            for i in range(len(velocities)):
                msg.velocities_flat.extend([
                    float(velocities[i, 0]),  # vx
                    float(velocities[i, 1]) if velocities.shape[1] > 1 else 0.0,  # vy
                    0.0,  # vz
                    0.0   # wz
                ])
            msg.soft_enabled = True
        else:
            msg.velocities_flat = []
            msg.soft_enabled = False
        
        # 时间间隔
        msg.dt_sec = dt
        
        # 置信度
        msg.confidence = confidence
        
        self.pub.publish(msg)
    
    def run_example(self):
        """运行示例：发布直线轨迹"""
        while not rospy.is_shutdown():
            # 示例：生成一条直线轨迹 (向前 2 米)
            positions = np.array([
                [0.0, 0.0],
                [0.2, 0.0],
                [0.4, 0.0],
                [0.6, 0.0],
                [0.8, 0.0],
                [1.0, 0.0],
                [1.2, 0.0],
                [1.4, 0.0],
                [1.6, 0.0],
                [1.8, 0.0],
                [2.0, 0.0],
            ])
            
            self.publish_trajectory(positions, dt=0.1)
            self.rate.sleep()


if __name__ == '__main__':
    try:
        pub = TrajectoryPublisher()
        pub.run_example()
    except rospy.ROSInterruptException:
        pass
```

### 4.4 集成你的神经网络

```python
#!/usr/bin/env python3
"""
神经网络轨迹发布器模板
"""
import rospy
import numpy as np
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from controller_ros.msg import LocalTrajectoryV4
# from your_network import YourNetwork  # 导入你的网络


class NNTrajectoryPublisher:
    def __init__(self):
        rospy.init_node('nn_trajectory_publisher')
        self.pub = rospy.Publisher('/nn/local_trajectory', LocalTrajectoryV4, queue_size=1)
        
        # 初始化你的网络
        # self.network = YourNetwork()
        
        self.rate = rospy.Rate(10)  # 10Hz
    
    def network_inference(self, sensor_data):
        """
        调用你的神经网络进行推理
        
        Returns:
            positions: numpy array, shape (N, 2), [x, y] 坐标
            velocities: numpy array, shape (N, 2), [vx, vy] 速度 (可选)
            confidence: float, 置信度 [0, 1]
        """
        # TODO: 替换为你的网络推理代码
        # output = self.network.predict(sensor_data)
        # positions = output['positions']
        # velocities = output.get('velocities', None)
        # confidence = output.get('confidence', 1.0)
        
        # 示例：返回直线轨迹
        positions = np.array([[i * 0.2, 0.0] for i in range(15)])
        velocities = None
        confidence = 1.0
        
        return positions, velocities, confidence
    
    def publish_trajectory(self, positions, velocities=None, dt=0.1, confidence=1.0):
        msg = LocalTrajectoryV4()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_link"
        msg.mode = 0  # MODE_TRACK
        
        msg.points = []
        for i in range(len(positions)):
            p = Point()
            p.x = float(positions[i, 0])
            p.y = float(positions[i, 1])
            p.z = 0.0
            msg.points.append(p)
        
        if velocities is not None:
            msg.velocities_flat = []
            for i in range(len(velocities)):
                msg.velocities_flat.extend([
                    float(velocities[i, 0]),
                    float(velocities[i, 1]) if velocities.shape[1] > 1 else 0.0,
                    0.0, 0.0
                ])
            msg.soft_enabled = True
        else:
            msg.velocities_flat = []
            msg.soft_enabled = False
        
        msg.dt_sec = dt
        msg.confidence = confidence
        self.pub.publish(msg)
    
    def run(self):
        while not rospy.is_shutdown():
            # 获取传感器数据 (根据你的需求)
            sensor_data = None  # TODO: 获取传感器数据
            
            # 网络推理
            positions, velocities, confidence = self.network_inference(sensor_data)
            
            # 发布轨迹
            self.publish_trajectory(positions, velocities, confidence=confidence)
            
            self.rate.sleep()


if __name__ == '__main__':
    try:
        node = NNTrajectoryPublisher()
        node.run()
    except rospy.ROSInterruptException:
        pass
```

---

## 5. cmd_vel 适配器

TurtleBot1 订阅 `/cmd_vel` (geometry_msgs/Twist)，需要将 `/cmd_unified` 转换。

### 5.1 适配器原理

```
/cmd_unified (UnifiedCmd)          /cmd_vel (Twist)
┌─────────────────────┐            ┌─────────────────────┐
│ vx: 线速度 x        │            │ linear.x: 线速度    │
│ vy: 线速度 y        │  ────→     │ linear.y: 0         │
│ vz: 线速度 z        │            │ linear.z: 0         │
│ omega: 角速度       │            │ angular.z: 角速度   │
└─────────────────────┘            └─────────────────────┘

差速车只使用 vx 和 omega
```

### 5.2 适配器已包含在 turtlebot1.launch

`turtlebot1.launch` 已经包含了 `cmd_vel_adapter.py` 节点，会自动：
- 订阅 `/cmd_unified`
- 发布 `/cmd_vel`
- 应用速度限制保护

---

## 6. 完整启动流程

### 6.1 启动命令

```bash
# 终端 1: 启动 TurtleBot1 底层驱动
roslaunch turtlebot_bringup minimal.launch

# 终端 2: 启动控制器
source ~/catkin_ws/devel/setup.bash
export PYTHONPATH=$PYTHONPATH:/home/user/AllControlBase
roslaunch controller_ros turtlebot1.launch

# 终端 3: 启动你的神经网络轨迹发布器
rosrun your_package trajectory_publisher.py

# (可选) 终端 4: 启动 Dashboard 监控
roslaunch controller_ros turtlebot1.launch dashboard:=true
```

### 6.2 一键启动脚本

创建 `start_controller.sh`:

```bash
#!/bin/bash
# TurtleBot1 控制器启动脚本

# 设置环境
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export PYTHONPATH=$PYTHONPATH:/home/user/AllControlBase

# 启动控制器
roslaunch controller_ros turtlebot1.launch dashboard:=false
```

### 6.3 验证话题

```bash
# 检查必需话题是否存在
rostopic list | grep -E "odom|trajectory|cmd"

# 应该看到:
# /odom                          (TurtleBot 发布)
# /nn/local_trajectory           (你的网络发布)
# /cmd_unified                   (控制器发布)
# /cmd_vel                       (适配器发布)
# /controller/diagnostics        (控制器发布)
# /controller/state              (控制器发布)

# 检查 TF
rosrun tf tf_echo odom base_link

# 监控控制器状态
rostopic echo /controller/state

# 监控输出命令
rostopic echo /cmd_vel
```

### 6.4 测试轨迹发布

```bash
# 使用示例轨迹发布器测试
rosrun controller_ros trajectory_publisher_example.py
```

---

## 7. 最佳效果配置

### 7.1 TurtleBot1 专用配置

配置文件: `controller_ros/config/turtlebot1.yaml`

| 参数 | 推荐值 | 说明 |
|------|--------|------|
| `system.ctrl_freq` | 20 Hz | TurtleBot1 适合较低频率 |
| `system.platform` | "differential" | 差速车平台 |
| `mpc.horizon` | 15 | 1.5秒预测时域 |
| `mpc.horizon_degraded` | 8 | 降级模式 0.8秒 |
| `mpc.dt` | 0.1 | 100ms 步长 |
| `constraints.v_max` | 0.5 m/s | TurtleBot1 安全速度 |
| `constraints.v_min` | -0.2 m/s | 倒车限制 |
| `constraints.omega_max` | 1.0 rad/s | 安全角速度 |
| `watchdog.odom_timeout_ms` | 500 | 适应较低 odom 频率 |
| `watchdog.traj_timeout_ms` | 1000 | 轨迹超时放宽 |
| `safety.enable_backup_controller` | true | 启用 Pure Pursuit 备用 |

### 7.2 轨迹发布建议

- **发布频率**: 10-20 Hz
- **轨迹点数**: 15-20 个点
- **时间跨度**: 1.5-2.0 秒
- **坐标系**: 始终使用 `base_link`

### 7.3 性能优化建议

1. **降低控制频率**: TurtleBot1 建议 20Hz
2. **减小 MPC horizon**: 15 个点足够
3. **启用备用控制器**: 提高鲁棒性
4. **适当放宽超时**: 适应较低传感器频率
5. **启用 EKF 状态估计**: 提高状态估计精度

### 7.4 配置文件完整示例

```yaml
# turtlebot1.yaml - TurtleBot1 专用配置

system:
  ctrl_freq: 20                   # 控制频率 (Hz)
  platform: "differential"        # 差速车

node:
  use_sim_time: false

topics:
  odom: "/odom"
  imu: ""                         # TurtleBot1 可能没有 IMU
  trajectory: "/nn/local_trajectory"
  emergency_stop: "/controller/emergency_stop"
  cmd_unified: "/cmd_unified"
  diagnostics: "/controller/diagnostics"
  state: "/controller/state"

tf:
  source_frame: "base_link"
  target_frame: "odom"
  timeout_ms: 50
  buffer_warmup_timeout_sec: 5.0
  buffer_warmup_interval_sec: 0.2

watchdog:
  odom_timeout_ms: 500
  traj_timeout_ms: 1000
  imu_timeout_ms: 0               # 禁用 IMU 超时
  startup_grace_ms: 3000

diagnostics:
  publish_rate: 5

mpc:
  horizon: 15
  horizon_degraded: 8
  dt: 0.1

constraints:
  v_max: 0.5
  v_min: -0.2
  omega_max: 1.0
  a_max: 0.5
  alpha_max: 1.0

consistency:
  enable_soft_head: true
  alpha_soft_threshold: 0.6
  curvature_weight: 0.3
  velocity_direction_weight: 0.3
  temporal_smoothness_weight: 0.4

safety:
  enable_backup_controller: true
  backup_lookahead: 0.5
  emergency_decel: 1.0

estimator:
  use_ekf: true
  process_noise: 0.1
  measurement_noise: 0.05
```

---

## 8. 故障排除

### 8.1 常见问题

| 问题 | 可能原因 | 解决方案 |
|------|----------|----------|
| 控制器无输出 | 缺少 odom 或轨迹 | 检查 `rostopic hz /odom /nn/local_trajectory` |
| TF 查询失败 | TF 树不完整 | 运行 `rosrun tf view_frames` 检查 |
| 机器人不动 | cmd_vel 未订阅 | 检查 TurtleBot 驱动是否启动 |
| 运动抖动 | 控制频率过高 | 降低 `ctrl_freq` 到 20Hz |
| 轨迹跟踪偏差大 | MPC 参数不当 | 调整 `horizon` 和 `dt` |
| ImportError | PYTHONPATH 未设置 | 检查 `echo $PYTHONPATH` |
| 消息类型错误 | 包未编译 | 重新 `catkin_make` |

### 8.2 诊断命令

```bash
# 检查话题频率
rostopic hz /odom
rostopic hz /nn/local_trajectory
rostopic hz /cmd_unified
rostopic hz /cmd_vel

# 检查话题内容
rostopic echo /controller/state
rostopic echo /controller/diagnostics

# 检查 TF 树
rosrun tf view_frames
evince frames.pdf

# 检查节点状态
rosnode list
rosnode info /universal_controller_node
```

### 8.3 紧急停止

```bash
# 发送紧急停止
rostopic pub /controller/emergency_stop std_msgs/Empty "{}"

# 重置控制器
rosservice call /controller/reset

# 手动停止机器人
rostopic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.0}" -1
```

### 8.4 日志查看

```bash
# 查看控制器日志
roslaunch controller_ros turtlebot1.launch 2>&1 | tee controller.log

# 过滤警告和错误
grep -E "WARN|ERROR" controller.log
```

### 8.5 控制器状态说明

| 值 | 状态 | 说明 |
|----|------|------|
| 0 | INIT | 初始化 |
| 1 | NORMAL | 正常运行 |
| 2 | SOFT_DISABLED | Soft Head 禁用 |
| 3 | MPC_DEGRADED | MPC 降级 |
| 4 | BACKUP_ACTIVE | 备用控制器激活 |
| 5 | STOPPING | 正在停止 |
| 6 | STOPPED | 已停止 |

---

## 附录 A: 文件清单

| 文件 | 说明 |
|------|------|
| `config/turtlebot1.yaml` | TurtleBot1 专用配置 |
| `config/controller_params.yaml` | 基础配置 |
| `launch/turtlebot1.launch` | TurtleBot1 启动文件 |
| `launch/controller.launch` | 通用启动文件 |
| `scripts/controller_node.py` | 控制器主节点 |
| `scripts/cmd_vel_adapter.py` | cmd_vel 适配器 |
| `scripts/dashboard_node.py` | Dashboard 节点 |
| `msg/LocalTrajectoryV4.msg` | 轨迹消息定义 |
| `msg/UnifiedCmd.msg` | 统一命令消息定义 |
| `msg/DiagnosticsV2.msg` | 诊断消息定义 |

---

## 附录 B: 快速检查清单

### 部署前检查

- [ ] ROS Noetic 已安装
- [ ] catkin 工作空间已创建
- [ ] universal_controller 在 PYTHONPATH 中
- [ ] controller_ros 已编译
- [ ] TurtleBot 驱动可正常启动
- [ ] /odom 话题有数据
- [ ] TF base_link → odom 可用

### 运行时检查

- [ ] 控制器节点已启动
- [ ] cmd_vel 适配器已启动
- [ ] 轨迹发布器已启动
- [ ] /cmd_vel 有输出
- [ ] 机器人响应命令

---

## 附录 C: 依赖安装

```bash
# Python 依赖
pip install numpy scipy PyYAML

# Dashboard 依赖 (可选)
pip install PyQt5 matplotlib

# ROS 依赖
sudo apt install ros-noetic-tf2-ros ros-noetic-nav-msgs ros-noetic-sensor-msgs
```

---

## 许可证

MIT License
