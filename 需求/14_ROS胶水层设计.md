# ROS 胶水层设计文档

> 版本: v1.1 | 日期: 2024-12-22

## 1. 设计目标

将 `universal_controller` 纯算法库与 ROS 生态系统集成，遵循以下原则：

- **单一职责**: 每个模块只做一件事
- **高内聚低耦合**: 模块内部紧密相关，模块间依赖最小化
- **依赖倒置**: 依赖抽象接口而非具体实现
- **开闭原则**: 对扩展开放，对修改关闭
- **平台无关**: 控制器输出统一格式，平台适配层独立

## 2. 整体架构

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              系统整体架构                                    │
└─────────────────────────────────────────────────────────────────────────────┘

                         ┌─────────────────────────────┐
                         │       感知/规划层            │
                         │   (网络推理、轨迹预测)        │
                         └─────────────┬───────────────┘
                                       │
                                       ▼
                         ┌─────────────────────────────┐
                         │  /nn/local_trajectory       │
                         │  (LocalTrajectoryV4)        │
                         └─────────────┬───────────────┘
                                       │
┌──────────────────────────────────────┼──────────────────────────────────────┐
│                                      │                                      │
│   /odom ─────────────────────────────┤                                      │
│   (nav_msgs/Odometry)                │                                      │
│                                      ▼                                      │
│   /imu ──────────────────────► ┌───────────────┐                            │
│   (sensor_msgs/Imu)            │               │                            │
│                                │  controller   │     /cmd_unified           │
│   /tf ───────────────────────► │     _ros      │ ──► (UnifiedCmd)           │
│   (TF2 变换)                   │               │                            │
│                                │  (ROS胶水层)  │     /controller/diagnostics│
│                                │               │ ──► (DiagnosticsV2)        │
│                                └───────────────┘                            │
│                                                                             │
│                        ROS 胶水层 (本文档设计范围)                            │
└──────────────────────────────────────┬──────────────────────────────────────┘
                                       │
                                       ▼
                         ┌─────────────────────────────┐
                         │      /cmd_unified           │
                         │      (统一控制命令)          │
                         └─────────────┬───────────────┘
                                       │
┌──────────────────────────────────────┼──────────────────────────────────────┐
│                                      │                                      │
│                                      ▼                                      │
│   ┌─────────────┐    ┌─────────────┐    ┌─────────────┐    ┌─────────────┐  │
│   │  差速车      │    │  全向车      │    │  阿克曼车   │    │  四旋翼     │  │
│   │  适配器      │    │  适配器      │    │  适配器     │    │  适配器     │  │
│   └──────┬──────┘    └──────┬──────┘    └──────┬──────┘    └──────┬──────┘  │
│          │                  │                  │                  │         │
│          ▼                  ▼                  ▼                  ▼         │
│     /cmd_vel           /cmd_vel         /ackermann_cmd    /mavros/setpoint  │
│     (Twist)            (Twist)          (Ackermann)       (TwistStamped)    │
│                                                                             │
│                        平台适配层 (独立开发，不在本文档范围)                   │
└─────────────────────────────────────────────────────────────────────────────┘
                                       │
                                       ▼
                         ┌─────────────────────────────┐
                         │        机器人硬件            │
                         │   (电机驱动、飞控等)          │
                         └─────────────────────────────┘
```

## 3. ROS 胶水层内部架构

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           ROS 胶水层 (controller_ros)                        │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │                         输入层 (Subscribers)                         │    │
│  │  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐               │    │
│  │  │ OdomSubscriber│  │ IMUSubscriber │  │ TrajSubscriber│              │    │
│  │  └───────┬──────┘  └───────┬──────┘  └───────┬──────┘               │    │
│  └──────────┼─────────────────┼─────────────────┼───────────────────────┘    │
│             │                 │                 │                            │
│             ▼                 ▼                 ▼                            │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │                       适配器层 (Adapters)                            │    │
│  │  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐               │    │
│  │  │ OdomAdapter  │  │  IMUAdapter  │  │ TrajAdapter  │               │    │
│  │  │ ROS→UC转换   │  │  ROS→UC转换  │  │  ROS→UC转换  │               │    │
│  │  └───────┬──────┘  └───────┬──────┘  └───────┬──────┘               │    │
│  └──────────┼─────────────────┼─────────────────┼───────────────────────┘    │
│             │                 │                 │                            │
│             └─────────────────┼─────────────────┘                            │
│                               ▼                                              │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │                        TF 桥接层 (TF Bridge)                         │    │
│  │                    管理 TF2 Buffer，提供坐标变换                       │    │
│  └────────────────────────────┬────────────────────────────────────────┘    │
│                               │                                              │
│                               ▼                                              │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │                      控制器桥接层 (Controller Bridge)                 │    │
│  │                    封装 ControllerManager 调用                        │    │
│  │  ┌─────────────────────────────────────────────────────────────┐    │    │
│  │  │                  universal_controller                        │    │    │
│  │  │                     (纯算法库)                                │    │    │
│  │  │  ┌─────────┐ ┌─────────┐ ┌─────────┐ ┌─────────┐            │    │    │
│  │  │  │  EKF    │ │   MPC   │ │ Safety  │ │ Consist │            │    │    │
│  │  │  │ 状态估计 │ │  控制器  │ │ 安全监控 │ │ 一致性  │            │    │    │
│  │  │  └─────────┘ └─────────┘ └─────────┘ └─────────┘            │    │    │
│  │  └─────────────────────────────────────────────────────────────┘    │    │
│  └────────────────────────────┬────────────────────────────────────────┘    │
│                               │                                              │
│                               ▼                                              │
│  ┌─────────────────────────────────────────────────────────────────────┐    │
│  │                        输出层 (Publishers)                           │    │
│  │  ┌────────────────────────┐  ┌────────────────────────┐             │    │
│  │  │   /cmd_unified         │  │  /controller/diagnostics│             │    │
│  │  │   (UnifiedCmd)         │  │  (DiagnosticsV2)        │             │    │
│  │  └────────────────────────┘  └────────────────────────┘             │    │
│  └─────────────────────────────────────────────────────────────────────┘    │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

## 4. 设计边界

### 4.1 ROS 胶水层职责 (本文档范围)

| 职责 | 说明 |
|------|------|
| 订阅传感器数据 | `/odom`, `/imu`, `/nn/local_trajectory` |
| 消息格式转换 | ROS 消息 ↔ universal_controller 数据类型 |
| TF2 集成 | 管理坐标变换，注入到控制器 |
| 调用控制算法 | 封装 `ControllerManager.update()` |
| 发布统一输出 | `/cmd_unified`, `/controller/diagnostics` |

### 4.2 ROS 胶水层不做的事

| 不做 | 原因 |
|------|------|
| 平台特定命令转换 | 由独立的平台适配层负责 |
| 电机控制/PWM 输出 | 由平台驱动负责 |
| 视觉/感知处理 | 由上游节点负责 |
| 全局规划 | 由规划层负责 |

### 4.3 平台适配层职责 (独立开发)

| 职责 | 说明 |
|------|------|
| 订阅 `/cmd_unified` | 接收统一控制命令 |
| 平台特定转换 | UnifiedCmd → Twist/Ackermann/MAVROS |
| 执行器整形 | 速度→PWM，油门刹车互斥等 |
| 平台状态机 | Offboard 切换、安全互锁等 |

## 5. 模块划分

### 5.1 目录结构

```
controller_ros/
├── package.xml                      # ROS 包描述
├── setup.py                         # Python 包配置 (ROS2)
├── CMakeLists.txt                   # 构建配置 (ROS1/ROS2)
│
├── config/
│   ├── controller_params.yaml       # 控制器参数
│   ├── differential.yaml            # 差速车配置
│   ├── omni.yaml                    # 全向车配置
│   ├── quadrotor.yaml               # 四旋翼配置
│   └── ackermann.yaml               # 阿克曼车配置
│
├── launch/
│   ├── controller.launch.py         # ROS2 启动文件
│   └── controller.launch            # ROS1 启动文件
│
├── msg/                             # 自定义消息 (复用 universal_controller/msg)
│   ├── LocalTrajectoryV4.msg        # 网络输出轨迹
│   ├── UnifiedCmd.msg               # 统一控制命令 (输出)
│   └── DiagnosticsV2.msg            # 诊断消息 (输出)
│
├── srv/                             # 服务定义
│   ├── SetControllerState.srv       # 设置控制器状态
│   └── GetDiagnostics.srv           # 获取诊断信息
│
└── src/
    └── controller_ros/
        ├── __init__.py
        │
        ├── node/                    # 节点层 (最外层)
        │   ├── __init__.py
        │   └── controller_node.py   # 主节点，组装所有组件
        │
        ├── adapters/                # 适配器层 (消息转换)
        │   ├── __init__.py
        │   ├── base.py              # 适配器基类接口
        │   ├── odom_adapter.py      # Odometry: ROS → UC
        │   ├── imu_adapter.py       # IMU: ROS → UC
        │   ├── trajectory_adapter.py # Trajectory: ROS → UC
        │   └── output_adapter.py    # ControlOutput: UC → ROS (UnifiedCmd)
        │
        ├── bridge/                  # 桥接层 (核心逻辑)
        │   ├── __init__.py
        │   ├── controller_bridge.py # 封装 ControllerManager
        │   └── tf_bridge.py         # TF2 桥接
        │
        ├── io/                      # IO 层 (订阅/发布)
        │   ├── __init__.py
        │   ├── subscribers.py       # 订阅管理 (odom, imu, trajectory)
        │   ├── publishers.py        # 发布管理 (cmd_unified, diagnostics)
        │   └── services.py          # 服务管理
        │
        └── utils/                   # 工具层
            ├── __init__.py
            ├── time_sync.py         # 时间同步
            ├── param_loader.py      # 参数加载
            └── ros_compat.py        # ROS1/ROS2 兼容层
```

注意：**没有 `platform/` 目录**，平台适配是独立的 ROS 包，不在本包范围内。

## 6. 各层职责详解

### 6.1 节点层 (node/)

**职责**: 组装所有组件，管理生命周期

```python
# controller_node.py
class ControllerNode:
    """
    控制器 ROS 节点
    
    职责:
    - 初始化 ROS 节点
    - 加载配置参数
    - 组装各层组件
    - 管理控制循环
    - 处理生命周期
    
    输出:
    - /cmd_unified (UnifiedCmd)
    - /controller/diagnostics (DiagnosticsV2)
    """
    
    def __init__(self):
        # 1. 初始化 ROS
        # 2. 加载参数
        # 3. 创建组件
        # 4. 启动控制循环
        pass
    
    def spin(self):
        """主循环"""
        pass
    
    def shutdown(self):
        """清理资源"""
        pass
```

### 6.2 适配器层 (adapters/)

**职责**: ROS 消息与 universal_controller 数据类型的双向转换

```python
# base.py - 核心转换接口
class IMsgConverter(ABC):
    """消息转换器接口"""
    
    @abstractmethod
    def to_uc(self, ros_msg) -> Any:
        """ROS 消息 → UC 数据类型"""
        pass
    
    @abstractmethod
    def to_ros(self, uc_data) -> Any:
        """UC 数据类型 → ROS 消息"""
        pass

# odom_adapter.py
class OdomAdapter(IMsgConverter):
    """
    Odometry 适配器
    
    ROS: nav_msgs/Odometry
    UC:  universal_controller.core.data_types.Odometry
    """
    
    def to_uc(self, ros_odom: nav_msgs.Odometry) -> uc.Odometry:
        return uc.Odometry(
            header=uc.Header(
                stamp=ros_odom.header.stamp.to_sec(),
                frame_id=ros_odom.header.frame_id
            ),
            pose_position=uc.Point3D(
                ros_odom.pose.pose.position.x,
                ros_odom.pose.pose.position.y,
                ros_odom.pose.pose.position.z
            ),
            pose_orientation=(
                ros_odom.pose.pose.orientation.x,
                ros_odom.pose.pose.orientation.y,
                ros_odom.pose.pose.orientation.z,
                ros_odom.pose.pose.orientation.w
            ),
            twist_linear=(
                ros_odom.twist.twist.linear.x,
                ros_odom.twist.twist.linear.y,
                ros_odom.twist.twist.linear.z
            ),
            twist_angular=(
                ros_odom.twist.twist.angular.x,
                ros_odom.twist.twist.angular.y,
                ros_odom.twist.twist.angular.z
            )
        )

# trajectory_adapter.py
class TrajectoryAdapter(IMsgConverter):
    """
    轨迹适配器
    
    ROS: controller_msgs/LocalTrajectoryV4 (自定义)
    UC:  universal_controller.core.data_types.Trajectory
    """
    
    def to_uc(self, ros_traj) -> uc.Trajectory:
        points = [
            uc.Point3D(p.x, p.y, p.z) 
            for p in ros_traj.points
        ]
        
        velocities = None
        if ros_traj.soft_enabled and len(ros_traj.velocities_flat) > 0:
            velocities = np.array(ros_traj.velocities_flat).reshape(-1, 4)
        
        return uc.Trajectory(
            header=uc.Header(
                stamp=ros_traj.header.stamp.to_sec(),
                frame_id=ros_traj.header.frame_id
            ),
            points=points,
            velocities=velocities,
            dt_sec=ros_traj.dt_sec,
            confidence=ros_traj.confidence,
            mode=uc.TrajectoryMode(ros_traj.mode),
            soft_enabled=ros_traj.soft_enabled
        )

# output_adapter.py
class OutputAdapter(IMsgConverter):
    """
    控制输出适配器
    
    UC:  universal_controller.core.data_types.ControlOutput
    ROS: controller_msgs/UnifiedCmd
    """
    
    def to_ros(self, uc_cmd: uc.ControlOutput) -> UnifiedCmd:
        msg = UnifiedCmd()
        msg.header.stamp = self._get_current_time()
        msg.header.frame_id = uc_cmd.frame_id
        msg.vx = uc_cmd.vx
        msg.vy = uc_cmd.vy
        msg.vz = uc_cmd.vz
        msg.omega = uc_cmd.omega
        msg.success = uc_cmd.success
        msg.solve_time_ms = uc_cmd.solve_time_ms
        return msg
```

### 6.3 桥接层 (bridge/)

**职责**: 封装 universal_controller 调用，隔离 ROS 与算法库

```python
# controller_bridge.py
class ControllerBridge:
    """
    控制器桥接
    
    职责:
    - 封装 ControllerManager 调用
    - 管理控制器生命周期
    - 提供统一的更新接口
    """
    
    def __init__(self, config: Dict[str, Any]):
        self._manager = ControllerManager(config)
        self._manager.initialize_default_components()
    
    def update(self, odom: uc.Odometry, 
               trajectory: uc.Trajectory,
               imu: Optional[uc.Imu] = None) -> uc.ControlOutput:
        """执行一次控制更新"""
        return self._manager.update(odom, trajectory, imu)
    
    def get_state(self) -> ControllerState:
        return self._manager.get_state()
    
    def get_diagnostics(self) -> Dict[str, Any]:
        return self._manager.get_last_published_diagnostics()
    
    def reset(self):
        self._manager.reset()
    
    def shutdown(self):
        self._manager.shutdown()

# tf_bridge.py
class TFBridge:
    """
    TF2 桥接
    
    职责:
    - 管理 TF2 Buffer 和 Listener
    - 提供坐标变换查询
    - 将变换注入到 universal_controller
    """
    
    def __init__(self):
        self._buffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._buffer)
    
    def lookup_transform(self, target_frame: str, source_frame: str,
                        time: rospy.Time) -> Optional[TransformStamped]:
        try:
            return self._buffer.lookup_transform(
                target_frame, source_frame, time,
                timeout=rospy.Duration(0.01)
            )
        except (tf2_ros.LookupException, 
                tf2_ros.ExtrapolationException):
            return None
    
    def inject_to_controller(self, transformer: ICoordinateTransformer):
        """将 TF2 变换注入到控制器的坐标变换器"""
        # 设置变换回调或直接注入 buffer
        pass
```

### 6.4 IO 层 (io/)

**职责**: 管理 ROS 订阅和发布

```python
# subscribers.py
class SubscriberManager:
    """
    订阅管理器
    
    职责:
    - 创建和管理所有订阅
    - 数据缓存和时间戳管理
    - 提供最新数据访问
    """
    
    def __init__(self, node, adapters: Dict[str, IMsgConverter]):
        self._adapters = adapters
        self._latest_data: Dict[str, Any] = {}
        self._timestamps: Dict[str, float] = {}
        
        # 创建订阅
        self._odom_sub = node.create_subscription(
            Odometry, '/odom', self._odom_callback, 10)
        self._imu_sub = node.create_subscription(
            Imu, '/imu', self._imu_callback, 10)
        self._traj_sub = node.create_subscription(
            LocalTrajectoryV4, '/nn/local_trajectory', self._traj_callback, 10)
    
    def _odom_callback(self, msg):
        self._latest_data['odom'] = self._adapters['odom'].to_uc(msg)
        self._timestamps['odom'] = time.time()
    
    def get_latest_odom(self) -> Optional[uc.Odometry]:
        return self._latest_data.get('odom')
    
    def get_data_ages(self) -> Dict[str, float]:
        """获取各数据的年龄 (秒)"""
        now = time.time()
        return {k: now - v for k, v in self._timestamps.items()}

# publishers.py
class PublisherManager:
    """
    发布管理器
    
    职责:
    - 创建和管理所有发布器
    - 发布统一控制命令和诊断
    
    注意: 不做平台特定转换，只发布 UnifiedCmd
    """
    
    def __init__(self, node, output_adapter: OutputAdapter):
        self._output_adapter = output_adapter
        
        # 统一控制命令发布 (核心输出)
        self._cmd_pub = node.create_publisher(
            UnifiedCmd, '/cmd_unified', 10)
        
        # 诊断发布
        self._diag_pub = node.create_publisher(
            DiagnosticsV2, '/controller/diagnostics', 10)
        
        # 可视化发布 (调试用)
        self._path_pub = node.create_publisher(
            Path, '/controller/debug_path', 10)
    
    def publish_cmd(self, cmd: uc.ControlOutput):
        """发布统一控制命令"""
        ros_msg = self._output_adapter.to_ros(cmd)
        self._cmd_pub.publish(ros_msg)
    
    def publish_diagnostics(self, diag: Dict[str, Any]):
        """发布诊断信息"""
        # 转换并发布诊断
        pass
```

## 7. 数据流详解

```
┌──────────────────────────────────────────────────────────────────────────┐
│                              数据流                                       │
└──────────────────────────────────────────────────────────────────────────┘

1. 输入数据流:
   
   /odom (nav_msgs/Odometry)
       │
       ▼
   OdomAdapter.to_uc()
       │
       ▼
   uc.Odometry ─────────────────────┐
                                    │
   /imu (sensor_msgs/Imu)           │
       │                            │
       ▼                            │
   ImuAdapter.to_uc()               │
       │                            │
       ▼                            │
   uc.Imu ──────────────────────────┤
                                    │
   /nn/local_trajectory             │
   (LocalTrajectoryV4)              │
       │                            │
       ▼                            │
   TrajectoryAdapter.to_uc()        │
       │                            │
       ▼                            │
   uc.Trajectory ───────────────────┤
                                    │
                                    ▼
                          ControllerBridge.update()
                                    │
                                    ▼
                          uc.ControlOutput
                                    │
                                    ▼
                          OutputAdapter.to_ros()
                                    │
                                    ▼
                          /cmd_unified (UnifiedCmd)
                                    │
                                    │  (由独立的平台适配层订阅)
                                    ▼
                          平台适配节点 (不在本包范围)
                                    │
                                    ▼
                          /cmd_vel 或其他平台特定话题

2. TF 数据流:
   
   TF2 Buffer
       │
       ▼
   TFBridge.lookup_transform()
       │
       ▼
   RobustCoordinateTransformer (在 universal_controller 内部)
       │
       ▼
   轨迹坐标变换 (base_link → odom)
```

## 8. 话题和服务规划

### 8.1 订阅话题 (输入)

| 话题名 | 消息类型 | 说明 | QoS |
|--------|----------|------|-----|
| `/odom` | nav_msgs/Odometry | 里程计 | Reliable, 10 |
| `/imu` | sensor_msgs/Imu | IMU (可选) | Reliable, 10 |
| `/nn/local_trajectory` | controller_msgs/LocalTrajectoryV4 | 网络预测轨迹 | Reliable, 1 |

### 8.2 发布话题 (输出)

| 话题名 | 消息类型 | 说明 | QoS |
|--------|----------|------|-----|
| `/cmd_unified` | controller_msgs/UnifiedCmd | **统一控制命令 (核心输出)** | Reliable, 1 |
| `/controller/diagnostics` | controller_msgs/DiagnosticsV2 | **诊断信息 (核心输出)** | Reliable, 10 |
| `/controller/state` | std_msgs/Int32 | 控制器状态 | Reliable, 1 |
| `/controller/debug_path` | nav_msgs/Path | 调试轨迹 (可选) | Best Effort, 1 |

### 8.3 服务

| 服务名 | 类型 | 说明 |
|--------|------|------|
| `/controller/set_state` | SetControllerState | 设置控制器状态 |
| `/controller/reset` | std_srvs/Trigger | 重置控制器 |
| `/controller/get_diagnostics` | GetDiagnostics | 获取详细诊断 |

### 8.4 TF 坐标系

| 坐标系 | 说明 |
|--------|------|
| `odom` | 里程计坐标系 (控制器工作坐标系) |
| `base_link` | 机体坐标系 |
| `base_link_0` | 网络推理时刻的机体坐标系 |

### 8.5 输出消息格式

```yaml
# /cmd_unified (controller_msgs/UnifiedCmd)
# 这是 ROS 胶水层的核心输出，由下游平台适配层消费

header:
  stamp: {sec: 1703232000, nanosec: 0}
  frame_id: "odom"              # 或 "base_link"，取决于平台配置

# 速度命令 (统一格式)
vx: 1.0                         # 前向/X方向速度 (m/s)
vy: 0.0                         # 横向/Y方向速度 (m/s)，差速车为0
vz: 0.0                         # 垂直/Z方向速度 (m/s)，地面车为0
omega: 0.1                      # 偏航角速度 (rad/s)

# 元信息
success: true                   # 控制计算是否成功
solve_time_ms: 5.2              # MPC 求解时间
```

## 9. 配置参数

```yaml
# controller_params.yaml
controller_ros:
  # 节点配置
  node:
    control_rate: 50.0          # 控制频率 (Hz)
    use_sim_time: false         # 是否使用仿真时间
  
  # 话题配置
  topics:
    # 输入
    odom: "/odom"
    imu: "/imu"
    trajectory: "/nn/local_trajectory"
    # 输出 (统一格式)
    cmd_unified: "/cmd_unified"
    diagnostics: "/controller/diagnostics"
  
  # 平台配置 (影响 frame_id 和约束)
  platform:
    type: "differential"        # differential/omni/ackermann/quadrotor
    
  # TF 配置
  tf:
    source_frame: "base_link"
    target_frame: "odom"
    timeout_ms: 10
    
  # 时间同步配置
  time_sync:
    max_odom_age_ms: 100
    max_traj_age_ms: 200
    max_imu_age_ms: 50

# 控制器算法参数 (传递给 universal_controller)
controller:
  system:
    ctrl_freq: 50
    platform: "differential"
  mpc:
    horizon: 20
    dt: 0.1
  # ... 其他 universal_controller 配置
```

## 10. 时序图

```
┌─────────┐  ┌──────────┐  ┌───────────┐  ┌────────────┐  ┌──────────┐
│  Odom   │  │   IMU    │  │ Trajectory│  │ Controller │  │  Output  │
│Subscriber│  │Subscriber│  │ Subscriber│  │   Bridge   │  │ Publisher│
└────┬────┘  └────┬─────┘  └─────┬─────┘  └──────┬─────┘  └────┬─────┘
     │            │              │               │              │
     │ odom_cb    │              │               │              │
     ├───────────►│              │               │              │
     │            │ imu_cb       │               │              │
     │            ├─────────────►│               │              │
     │            │              │ traj_cb       │              │
     │            │              ├──────────────►│              │
     │            │              │               │              │
     │            │              │    ┌──────────┴──────────┐   │
     │            │              │    │  Control Loop (50Hz)│   │
     │            │              │    └──────────┬──────────┘   │
     │            │              │               │              │
     │            │              │  get_latest() │              │
     │◄───────────┼──────────────┼───────────────┤              │
     │            │◄─────────────┼───────────────┤              │
     │            │              │◄──────────────┤              │
     │            │              │               │              │
     │            │              │    update()   │              │
     │            │              │    ┌──────────┴──────────┐   │
     │            │              │    │ ControllerManager   │   │
     │            │              │    │     .update()       │   │
     │            │              │    └──────────┬──────────┘   │
     │            │              │               │              │
     │            │              │               │ publish      │
     │            │              │               │ /cmd_unified │
     │            │              │               ├─────────────►│
     │            │              │               │              │
     │            │              │               │              │
                                                               │
                                                               ▼
                                              ┌────────────────────────┐
                                              │   平台适配层 (独立)     │
                                              │   订阅 /cmd_unified    │
                                              │   发布 /cmd_vel 等     │
                                              └────────────────────────┘
```

## 11. 错误处理策略

### 11.1 数据超时

```python
class TimeoutHandler:
    """超时处理器"""
    
    def check_data_freshness(self, ages: Dict[str, float]) -> List[str]:
        """检查数据新鲜度，返回超时的数据源"""
        timeouts = []
        if ages.get('odom', float('inf')) > self.max_odom_age:
            timeouts.append('odom')
        if ages.get('trajectory', float('inf')) > self.max_traj_age:
            timeouts.append('trajectory')
        return timeouts
    
    def handle_timeout(self, timeouts: List[str]):
        """处理超时 - 通过 /cmd_unified 发布停止命令"""
        if 'odom' in timeouts:
            # 里程计超时，发布零速命令
            self._publish_stop_cmd()
        elif 'trajectory' in timeouts:
            # 轨迹超时，控制器内部会切换到备用控制器
            pass
```

### 11.2 TF 异常

```python
class TFErrorHandler:
    """TF 错误处理器"""
    
    def handle_tf_error(self, error_type: str):
        if error_type == 'lookup_failed':
            # 使用 odom 积分降级 (由 universal_controller 内部处理)
            pass
        elif error_type == 'extrapolation':
            # 使用最近的有效变换
            pass
```

## 12. 测试策略

### 12.1 单元测试

- 适配器层: 测试消息转换正确性
- 桥接层: 测试控制器调用逻辑
- IO 层: 测试订阅/发布功能

### 12.2 集成测试

- 使用 rosbag 回放测试完整数据流
- 验证 `/cmd_unified` 输出格式正确

### 12.3 Mock 测试

```python
# 使用 universal_controller 提供的 mock 模块
from universal_controller.mock import MockOdometry, MockTrajectory

def test_controller_bridge():
    bridge = ControllerBridge(config)
    odom = MockOdometry(vx=1.0)
    traj = MockTrajectory(num_points=10)
    
    cmd = bridge.update(odom, traj)
    assert cmd.success
```

## 13. 部署建议

### 13.1 启动顺序

1. 启动传感器驱动 (odom, imu)
2. 启动网络推理节点
3. 启动控制器节点 (controller_ros)
4. 启动平台适配节点 (独立包)
5. 启动平台驱动

### 13.2 资源配置

```yaml
# 建议的 CPU 亲和性配置
controller_node:
  cpu_affinity: [2, 3]  # 绑定到特定 CPU 核心
  priority: 90          # 实时优先级
```

## 14. 平台适配层设计指南 (独立开发)

平台适配层是独立的 ROS 包，不在本文档范围内，但这里提供设计指南：

### 14.1 平台适配层职责

```
┌─────────────────────────────────────────────────────────────────┐
│                      平台适配层 (独立包)                         │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│   /cmd_unified ──► ┌─────────────────┐ ──► /cmd_vel            │
│   (UnifiedCmd)     │  平台适配节点    │     (Twist)             │
│                    │                 │                          │
│                    │  - 消息格式转换  │ ──► /ackermann_cmd      │
│                    │  - 执行器整形    │     (Ackermann)         │
│                    │  - 安全互锁      │                          │
│                    │  - 状态机管理    │ ──► /mavros/setpoint    │
│                    └─────────────────┘     (TwistStamped)       │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### 14.2 各平台适配器示例

```python
# 差速车适配器 (differential_adapter_node.py)
class DifferentialAdapterNode:
    def __init__(self):
        self._cmd_sub = self.create_subscription(
            UnifiedCmd, '/cmd_unified', self._cmd_callback, 10)
        self._cmd_pub = self.create_publisher(
            Twist, '/cmd_vel', 10)
    
    def _cmd_callback(self, msg: UnifiedCmd):
        twist = Twist()
        twist.linear.x = msg.vx
        twist.angular.z = msg.omega
        self._cmd_pub.publish(twist)

# 四旋翼适配器 (quadrotor_adapter_node.py)
class QuadrotorAdapterNode:
    def __init__(self):
        self._cmd_sub = self.create_subscription(
            UnifiedCmd, '/cmd_unified', self._cmd_callback, 10)
        self._cmd_pub = self.create_publisher(
            TwistStamped, '/mavros/setpoint_velocity/cmd_vel', 10)
        
        # MAVROS 状态机管理
        self._arm_client = ...
        self._offboard_client = ...
    
    def _cmd_callback(self, msg: UnifiedCmd):
        # 检查 Offboard 模式
        if not self._is_offboard:
            return
        
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = 'world'
        twist.twist.linear.x = msg.vx
        twist.twist.linear.y = msg.vy
        twist.twist.linear.z = msg.vz
        twist.twist.angular.z = msg.omega
        self._cmd_pub.publish(twist)
```

### 14.3 平台适配层目录结构

```
platform_adapters/                    # 可以是一个包含多个适配器的包
├── differential_adapter/             # 或者每个平台一个独立包
│   ├── package.xml
│   ├── setup.py
│   └── src/
│       └── differential_adapter_node.py
│
├── ackermann_adapter/
│   └── ...
│
└── quadrotor_adapter/
    ├── package.xml
    ├── setup.py
    └── src/
        ├── quadrotor_adapter_node.py
        └── mavros_state_machine.py   # MAVROS 状态机管理
```

## 15. 总结

本设计遵循以下软件工程原则:

| 原则 | 实现方式 |
|------|----------|
| 单一职责 | ROS 胶水层只负责"接收-转换-调用-发布"，不做平台适配 |
| 开闭原则 | 通过接口和继承支持扩展 |
| 依赖倒置 | 依赖抽象接口 (IMsgConverter) |
| 接口隔离 | 小而专注的接口 |
| 高内聚 | 相关功能聚合在同一模块 |
| 低耦合 | 模块间通过接口通信，平台适配完全独立 |

### 核心设计决策

| 决策 | 原因 |
|------|------|
| 输出 `/cmd_unified` 而非平台特定话题 | 保持控制器与平台解耦，符合需求文档 |
| 平台适配层独立 | 不同机器人可复用同一控制器节点 |
| 使用 `UnifiedCmd` 消息 | 统一格式，包含所有平台需要的信息 |

### 系统边界清晰

```
┌─────────────────────────────────────────────────────────────────┐
│  controller_ros (本文档范围)                                     │
│                                                                 │
│  输入: /odom, /imu, /nn/local_trajectory                        │
│  输出: /cmd_unified, /controller/diagnostics                    │
│                                                                 │
│  职责: 消息转换 + 调用算法 + 发布统一命令                          │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│  platform_adapter_xxx (独立开发)                                 │
│                                                                 │
│  输入: /cmd_unified                                             │
│  输出: /cmd_vel, /ackermann_cmd, /mavros/setpoint 等            │
│                                                                 │
│  职责: 平台特定转换 + 执行器整形 + 安全互锁                        │
└─────────────────────────────────────────────────────────────────┘
```

这种分层架构使得:
- `universal_controller` 保持纯净，不依赖 ROS
- `controller_ros` 与平台无关，可复用
- 平台适配层可独立开发和部署
- 易于测试和维护


## 14. 核心代码示例

### 14.1 主节点实现

```python
# src/controller_ros/node/controller_node.py
"""
控制器 ROS 节点

职责:
- 组装所有组件
- 管理控制循环
- 处理生命周期
"""
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

from ..adapters import OdomAdapter, ImuAdapter, TrajectoryAdapter, CmdAdapter
from ..bridge import ControllerBridge, TFBridge
from ..io import SubscriberManager, PublisherManager
from ..platform import create_platform_adapter
from ..utils import ParamLoader


class ControllerNode(Node):
    """控制器主节点"""
    
    def __init__(self):
        super().__init__('universal_controller_node')
        
        # 1. 加载参数
        self._load_parameters()
        
        # 2. 创建回调组 (用于并发控制)
        self._sensor_cb_group = ReentrantCallbackGroup()
        self._control_cb_group = MutuallyExclusiveCallbackGroup()
        
        # 3. 创建适配器
        self._adapters = {
            'odom': OdomAdapter(),
            'imu': ImuAdapter(),
            'trajectory': TrajectoryAdapter(),
            'cmd': CmdAdapter()
        }
        
        # 4. 创建平台适配器
        self._platform_adapter = create_platform_adapter(
            self._params['platform']['type']
        )
        
        # 5. 创建 TF 桥接
        self._tf_bridge = TFBridge(self)
        
        # 6. 创建控制器桥接
        self._controller_bridge = ControllerBridge(
            self._params['controller']
        )
        
        # 7. 创建订阅/发布管理器
        self._subscribers = SubscriberManager(
            self, self._adapters, self._sensor_cb_group
        )
        self._publishers = PublisherManager(
            self, self._platform_adapter
        )
        
        # 8. 创建控制定时器
        control_period = 1.0 / self._params['node']['control_rate']
        self._control_timer = self.create_timer(
            control_period,
            self._control_callback,
            callback_group=self._control_cb_group
        )
        
        self.get_logger().info('Controller node initialized')
    
    def _load_parameters(self):
        """加载参数"""
        self._params = ParamLoader.load(self)
    
    def _control_callback(self):
        """控制循环回调"""
        # 1. 获取最新数据
        odom = self._subscribers.get_latest_odom()
        imu = self._subscribers.get_latest_imu()
        trajectory = self._subscribers.get_latest_trajectory()
        
        # 2. 检查数据有效性
        if odom is None or trajectory is None:
            self.get_logger().warn_throttle(
                1.0, 'Waiting for odom and trajectory data'
            )
            return
        
        # 3. 执行控制更新
        try:
            cmd = self._controller_bridge.update(odom, trajectory, imu)
        except Exception as e:
            self.get_logger().error(f'Controller update failed: {e}')
            return
        
        # 4. 发布控制命令
        self._publishers.publish_cmd(cmd)
        
        # 5. 发布诊断 (降频)
        if self._should_publish_diagnostics():
            diag = self._controller_bridge.get_diagnostics()
            self._publishers.publish_diagnostics(diag)
    
    def _should_publish_diagnostics(self) -> bool:
        """判断是否应该发布诊断 (10Hz)"""
        # 实现降频逻辑
        return True
    
    def shutdown(self):
        """清理资源"""
        self._controller_bridge.shutdown()
        self.get_logger().info('Controller node shutdown')


def main(args=None):
    rclpy.init(args=args)
    
    node = ControllerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 14.2 消息适配器实现

```python
# src/controller_ros/adapters/odom_adapter.py
"""里程计适配器"""
from nav_msgs.msg import Odometry as RosOdometry
from universal_controller.core.data_types import (
    Odometry as UcOdometry, Header, Point3D
)
from .base import IMsgConverter


class OdomAdapter(IMsgConverter):
    """
    Odometry 消息适配器
    
    ROS 消息: nav_msgs/Odometry
    UC 数据类型: universal_controller.core.data_types.Odometry
    """
    
    def to_uc(self, ros_msg: RosOdometry) -> UcOdometry:
        """ROS Odometry → UC Odometry"""
        return UcOdometry(
            header=Header(
                stamp=self._ros_time_to_sec(ros_msg.header.stamp),
                frame_id=ros_msg.header.frame_id
            ),
            pose_position=Point3D(
                x=ros_msg.pose.pose.position.x,
                y=ros_msg.pose.pose.position.y,
                z=ros_msg.pose.pose.position.z
            ),
            pose_orientation=(
                ros_msg.pose.pose.orientation.x,
                ros_msg.pose.pose.orientation.y,
                ros_msg.pose.pose.orientation.z,
                ros_msg.pose.pose.orientation.w
            ),
            twist_linear=(
                ros_msg.twist.twist.linear.x,
                ros_msg.twist.twist.linear.y,
                ros_msg.twist.twist.linear.z
            ),
            twist_angular=(
                ros_msg.twist.twist.angular.x,
                ros_msg.twist.twist.angular.y,
                ros_msg.twist.twist.angular.z
            )
        )
    
    def to_ros(self, uc_data: UcOdometry) -> RosOdometry:
        """UC Odometry → ROS Odometry (通常不需要)"""
        raise NotImplementedError("Odom is input-only")
    
    def _ros_time_to_sec(self, stamp) -> float:
        """ROS 时间戳转秒"""
        return stamp.sec + stamp.nanosec * 1e-9


# src/controller_ros/adapters/trajectory_adapter.py
"""轨迹适配器"""
import numpy as np
from controller_msgs.msg import LocalTrajectoryV4 as RosTraj
from universal_controller.core.data_types import (
    Trajectory as UcTrajectory, Header, Point3D
)
from universal_controller.core.enums import TrajectoryMode
from .base import IMsgConverter


class TrajectoryAdapter(IMsgConverter):
    """
    轨迹消息适配器
    
    ROS 消息: controller_msgs/LocalTrajectoryV4
    UC 数据类型: universal_controller.core.data_types.Trajectory
    """
    
    def to_uc(self, ros_msg: RosTraj) -> UcTrajectory:
        """ROS LocalTrajectoryV4 → UC Trajectory"""
        # 转换轨迹点
        points = [
            Point3D(x=p.x, y=p.y, z=p.z)
            for p in ros_msg.points
        ]
        
        # 转换速度数组
        velocities = None
        if ros_msg.soft_enabled and len(ros_msg.velocities_flat) > 0:
            # 从扁平数组重建 [N, 4] 数组
            velocities = np.array(ros_msg.velocities_flat).reshape(-1, 4)
        
        return UcTrajectory(
            header=Header(
                stamp=self._ros_time_to_sec(ros_msg.header.stamp),
                frame_id=ros_msg.header.frame_id
            ),
            points=points,
            velocities=velocities,
            dt_sec=ros_msg.dt_sec,
            confidence=ros_msg.confidence,
            mode=TrajectoryMode(ros_msg.mode),
            soft_enabled=ros_msg.soft_enabled
        )
    
    def to_ros(self, uc_data: UcTrajectory) -> RosTraj:
        """UC Trajectory → ROS LocalTrajectoryV4"""
        from geometry_msgs.msg import Point
        from std_msgs.msg import Header as RosHeader
        
        ros_msg = RosTraj()
        ros_msg.header.stamp = self._sec_to_ros_time(uc_data.header.stamp)
        ros_msg.header.frame_id = uc_data.header.frame_id
        ros_msg.mode = int(uc_data.mode)
        ros_msg.dt_sec = uc_data.dt_sec
        ros_msg.confidence = uc_data.confidence
        ros_msg.soft_enabled = uc_data.soft_enabled
        
        ros_msg.points = [
            Point(x=p.x, y=p.y, z=p.z)
            for p in uc_data.points
        ]
        
        if uc_data.velocities is not None:
            ros_msg.velocities_flat = uc_data.velocities.flatten().tolist()
        
        return ros_msg
    
    def _ros_time_to_sec(self, stamp) -> float:
        return stamp.sec + stamp.nanosec * 1e-9
    
    def _sec_to_ros_time(self, sec: float):
        from builtin_interfaces.msg import Time
        t = Time()
        t.sec = int(sec)
        t.nanosec = int((sec - t.sec) * 1e9)
        return t
```

### 14.3 平台适配器实现

```python
# src/controller_ros/platform/base_adapter.py
"""平台适配器基类"""
from abc import ABC, abstractmethod
from typing import Any
from universal_controller.core.data_types import ControlOutput


class BasePlatformAdapter(ABC):
    """平台适配器基类"""
    
    @abstractmethod
    def to_ros_cmd(self, cmd: ControlOutput) -> Any:
        """将 UC 控制输出转换为 ROS 消息"""
        pass
    
    @abstractmethod
    def get_output_frame(self) -> str:
        """获取输出坐标系"""
        pass
    
    @abstractmethod
    def get_output_topic(self) -> str:
        """获取输出话题"""
        pass


# src/controller_ros/platform/differential_adapter.py
"""差速车适配器"""
from geometry_msgs.msg import Twist
from universal_controller.core.data_types import ControlOutput
from .base_adapter import BasePlatformAdapter


class DifferentialAdapter(BasePlatformAdapter):
    """
    差速车平台适配器
    
    输出: geometry_msgs/Twist
    坐标系: base_link
    """
    
    def to_ros_cmd(self, cmd: ControlOutput) -> Twist:
        twist = Twist()
        twist.linear.x = cmd.vx
        twist.linear.y = 0.0  # 差速车无横向速度
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = cmd.omega
        return twist
    
    def get_output_frame(self) -> str:
        return 'base_link'
    
    def get_output_topic(self) -> str:
        return '/cmd_vel'


# src/controller_ros/platform/quadrotor_adapter.py
"""四旋翼适配器"""
from geometry_msgs.msg import TwistStamped
from universal_controller.core.data_types import ControlOutput
from .base_adapter import BasePlatformAdapter


class QuadrotorAdapter(BasePlatformAdapter):
    """
    四旋翼平台适配器
    
    输出: geometry_msgs/TwistStamped (MAVROS 格式)
    坐标系: world
    """
    
    def __init__(self):
        self._seq = 0
    
    def to_ros_cmd(self, cmd: ControlOutput) -> TwistStamped:
        from std_msgs.msg import Header
        import time
        
        msg = TwistStamped()
        msg.header.stamp = self._get_current_time()
        msg.header.frame_id = 'world'
        
        msg.twist.linear.x = cmd.vx
        msg.twist.linear.y = cmd.vy
        msg.twist.linear.z = cmd.vz
        msg.twist.angular.z = cmd.omega
        
        return msg
    
    def get_output_frame(self) -> str:
        return 'world'
    
    def get_output_topic(self) -> str:
        return '/mavros/setpoint_velocity/cmd_vel'
    
    def _get_current_time(self):
        from builtin_interfaces.msg import Time
        import time
        t = Time()
        now = time.time()
        t.sec = int(now)
        t.nanosec = int((now - t.sec) * 1e9)
        return t


# src/controller_ros/platform/__init__.py
"""平台适配器工厂"""
from .differential_adapter import DifferentialAdapter
from .omni_adapter import OmniAdapter
from .ackermann_adapter import AckermannAdapter
from .quadrotor_adapter import QuadrotorAdapter


def create_platform_adapter(platform_type: str) -> BasePlatformAdapter:
    """
    创建平台适配器
    
    Args:
        platform_type: 平台类型 (differential/omni/ackermann/quadrotor)
    
    Returns:
        对应的平台适配器实例
    """
    adapters = {
        'differential': DifferentialAdapter,
        'omni': OmniAdapter,
        'ackermann': AckermannAdapter,
        'quadrotor': QuadrotorAdapter
    }
    
    if platform_type not in adapters:
        raise ValueError(f"Unknown platform type: {platform_type}")
    
    return adapters[platform_type]()
```

### 14.4 控制器桥接实现

```python
# src/controller_ros/bridge/controller_bridge.py
"""控制器桥接"""
from typing import Dict, Any, Optional
import logging

from universal_controller.manager.controller_manager import ControllerManager
from universal_controller.core.data_types import (
    Odometry, Imu, Trajectory, ControlOutput
)
from universal_controller.core.enums import ControllerState


logger = logging.getLogger(__name__)


class ControllerBridge:
    """
    控制器桥接层
    
    职责:
    - 封装 ControllerManager 调用
    - 管理控制器生命周期
    - 提供统一的更新接口
    - 隔离 ROS 层与算法库
    """
    
    def __init__(self, config: Dict[str, Any]):
        """
        初始化控制器桥接
        
        Args:
            config: 控制器配置字典，将传递给 ControllerManager
        """
        self._config = config
        self._manager: Optional[ControllerManager] = None
        self._initialized = False
        
        self._initialize()
    
    def _initialize(self):
        """初始化控制器"""
        try:
            self._manager = ControllerManager(self._config)
            self._manager.initialize_default_components()
            self._initialized = True
            logger.info("Controller bridge initialized successfully")
        except Exception as e:
            logger.error(f"Controller initialization failed: {e}")
            raise
    
    def update(self, odom: Odometry, trajectory: Trajectory,
               imu: Optional[Imu] = None) -> ControlOutput:
        """
        执行一次控制更新
        
        Args:
            odom: 里程计数据 (UC 格式)
            trajectory: 轨迹数据 (UC 格式)
            imu: IMU 数据 (UC 格式，可选)
        
        Returns:
            控制输出 (UC 格式)
        
        Raises:
            RuntimeError: 控制器未初始化
        """
        if not self._initialized or self._manager is None:
            raise RuntimeError("Controller not initialized")
        
        return self._manager.update(odom, trajectory, imu)
    
    def get_state(self) -> ControllerState:
        """获取控制器状态"""
        if self._manager is None:
            return ControllerState.INIT
        return self._manager.get_state()
    
    def get_diagnostics(self) -> Optional[Dict[str, Any]]:
        """获取诊断信息"""
        if self._manager is None:
            return None
        return self._manager.get_last_published_diagnostics()
    
    def reset(self):
        """重置控制器"""
        if self._manager is not None:
            self._manager.reset()
            logger.info("Controller reset")
    
    def shutdown(self):
        """关闭控制器"""
        if self._manager is not None:
            self._manager.shutdown()
            self._initialized = False
            logger.info("Controller shutdown")
    
    @property
    def is_initialized(self) -> bool:
        """控制器是否已初始化"""
        return self._initialized
```

### 14.5 订阅管理器实现

```python
# src/controller_ros/io/subscribers.py
"""订阅管理器"""
from typing import Dict, Any, Optional
import threading
import time

from rclpy.node import Node
from rclpy.callback_groups import CallbackGroup
from nav_msgs.msg import Odometry as RosOdometry
from sensor_msgs.msg import Imu as RosImu

from universal_controller.core.data_types import Odometry, Imu, Trajectory
from ..adapters import IMsgConverter


class SubscriberManager:
    """
    订阅管理器
    
    职责:
    - 创建和管理所有订阅
    - 线程安全的数据缓存
    - 提供最新数据访问
    - 数据时间戳管理
    """
    
    def __init__(self, node: Node, adapters: Dict[str, IMsgConverter],
                 callback_group: Optional[CallbackGroup] = None):
        self._node = node
        self._adapters = adapters
        self._callback_group = callback_group
        
        # 线程安全的数据存储
        self._lock = threading.Lock()
        self._latest_data: Dict[str, Any] = {}
        self._timestamps: Dict[str, float] = {}
        
        # 创建订阅
        self._create_subscriptions()
    
    def _create_subscriptions(self):
        """创建所有订阅"""
        # 里程计订阅
        self._odom_sub = self._node.create_subscription(
            RosOdometry,
            self._node.get_parameter('topics.odom').value,
            self._odom_callback,
            10,
            callback_group=self._callback_group
        )
        
        # IMU 订阅
        self._imu_sub = self._node.create_subscription(
            RosImu,
            self._node.get_parameter('topics.imu').value,
            self._imu_callback,
            10,
            callback_group=self._callback_group
        )
        
        # 轨迹订阅 (需要自定义消息类型)
        # self._traj_sub = ...
    
    def _odom_callback(self, msg: RosOdometry):
        """里程计回调"""
        uc_odom = self._adapters['odom'].to_uc(msg)
        with self._lock:
            self._latest_data['odom'] = uc_odom
            self._timestamps['odom'] = time.time()
    
    def _imu_callback(self, msg: RosImu):
        """IMU 回调"""
        uc_imu = self._adapters['imu'].to_uc(msg)
        with self._lock:
            self._latest_data['imu'] = uc_imu
            self._timestamps['imu'] = time.time()
    
    def _traj_callback(self, msg):
        """轨迹回调"""
        uc_traj = self._adapters['trajectory'].to_uc(msg)
        with self._lock:
            self._latest_data['trajectory'] = uc_traj
            self._timestamps['trajectory'] = time.time()
    
    def get_latest_odom(self) -> Optional[Odometry]:
        """获取最新里程计"""
        with self._lock:
            return self._latest_data.get('odom')
    
    def get_latest_imu(self) -> Optional[Imu]:
        """获取最新 IMU"""
        with self._lock:
            return self._latest_data.get('imu')
    
    def get_latest_trajectory(self) -> Optional[Trajectory]:
        """获取最新轨迹"""
        with self._lock:
            return self._latest_data.get('trajectory')
    
    def get_data_ages(self) -> Dict[str, float]:
        """
        获取各数据的年龄 (秒)
        
        Returns:
            字典，键为数据名，值为距上次更新的秒数
        """
        now = time.time()
        with self._lock:
            return {k: now - v for k, v in self._timestamps.items()}
    
    def is_data_fresh(self, max_ages: Dict[str, float]) -> bool:
        """
        检查数据是否新鲜
        
        Args:
            max_ages: 各数据的最大允许年龄
        
        Returns:
            所有数据都在允许年龄内返回 True
        """
        ages = self.get_data_ages()
        for key, max_age in max_ages.items():
            if ages.get(key, float('inf')) > max_age:
                return False
        return True
```

## 15. 依赖关系图

```
┌─────────────────────────────────────────────────────────────────┐
│                        依赖关系                                  │
└─────────────────────────────────────────────────────────────────┘

controller_ros (ROS 胶水层)
    │
    ├── rclpy / rospy (ROS 核心)
    │
    ├── nav_msgs, sensor_msgs, geometry_msgs (标准消息)
    │
    ├── tf2_ros (坐标变换)
    │
    └── universal_controller (纯算法库)
            │
            ├── numpy
            ├── scipy
            └── PyYAML

依赖方向: controller_ros → universal_controller
         (单向依赖，算法库不依赖 ROS)
```

这个设计确保了:
1. `universal_controller` 保持纯净，可独立测试
2. ROS 相关代码完全隔离在胶水层
3. 易于支持 ROS1/ROS2 双版本
4. 平台扩展只需添加新的适配器
