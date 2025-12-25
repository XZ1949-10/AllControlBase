# 通用控制器 (Universal Controller)

> 版本: v3.18.1 | 日期: 2024-12-25

基于 MPC 的通用轨迹跟踪控制器，支持多平台部署。

## 坐标系说明 (重要)

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

### 数据流

```
网络输出轨迹 (base_link, 局部坐标，当前位置为原点)
    ↓
坐标变换 (base_link → odom)
    ↓
控制器计算 (在 odom 坐标系下)
    ↓
控制输出:
    - 差速车/阿克曼车: base_link (vx, omega)
    - 全向车/四旋翼: odom (vx, vy, omega)
```

### 关键点

- **odom 就是你的"世界坐标系"**，不需要建图
- **网络输出**的轨迹应该设置 `frame_id='base_link'`
- 如果轨迹已经在 odom 坐标系，会跳过变换直接使用

## 特性

- **跨平台部署**: 统一状态空间支持地面车辆和无人机
- **Hard/Soft 融合**: 多维一致性门控，硬主干保证可用性，软建议提升平滑性
- **安全可靠**: 热备控制器 + 渐进式降级，异常时安全停车
- **高实时性**: ACADOS 求解器，15ms 内完成优化
- **插件化架构**: 核心模块可替换，便于扩展和维护
- **ROS 兼容**: 支持 ROS 环境和独立运行模式
- **完整 TF2 支持**: 多跳链式变换查找 (最多 10 跳)
- **轨迹可视化**: 实时 2D/3D 轨迹显示，支持 Hard/Soft 叠加
- **模块化架构**: ROS 兼容层独立到 `compat/` 模块，测试数据生成器在 `tests/`

## ROS 兼容性

本项目支持两种运行模式：

### 1. ROS 环境
当检测到 ROS 环境时，自动使用真实的 ROS 组件：
- `rospy` - ROS Python 客户端
- `tf2_ros` - TF2 坐标变换
- `nav_msgs`, `sensor_msgs`, `geometry_msgs` - 标准消息类型
- 自动发布诊断到 `/controller/diagnostics`
- 自动发布控制命令到 `/cmd_unified`

### 2. 独立模式
在非 ROS 环境下，使用内置的模拟实现：
- 模拟 `rospy.Time`, `rospy.Duration`
- 模拟 TF2 Buffer 和变换查找 (支持多跳链式查找)
- 兼容的数据类型定义
- 诊断回调机制

检查运行模式：
```python
from universal_controller.core.ros_compat import ROS_AVAILABLE, TF2_AVAILABLE
print(f"ROS: {ROS_AVAILABLE}, TF2: {TF2_AVAILABLE}")
```

### ROS 消息定义

项目包含 ROS 消息定义文件 (`msg/` 目录)：
- `DiagnosticsV2.msg` - 诊断消息
- `LocalTrajectoryV4.msg` - 轨迹消息
- `UnifiedCmd.msg` - 统一控制命令

在 ROS 环境中使用时，将 `msg/` 目录复制到 catkin 工作空间并编译。

## 支持平台

- 阿克曼转向车辆 (Ackermann)
- 差速驱动车辆 (Differential)
- 全向移动车辆 (Omni)
- 四旋翼无人机 (Quadrotor)

## 项目结构

```
universal_controller/
├── __init__.py              # 包入口，导出主要类
├── main.py                  # 主程序入口
├── requirements.txt         # 依赖列表
├── README.md                # 本文件
│
├── config/                  # 配置模块
│   ├── __init__.py
│   └── default_config.py    # 默认配置和平台配置
│
├── core/                    # 核心模块
│   ├── __init__.py
│   ├── data_types.py        # 数据类型定义
│   ├── enums.py             # 枚举定义
│   ├── interfaces.py        # 接口定义
│   └── ros_compat.py        # ROS 兼容层
│
├── estimator/               # 状态估计
│   ├── __init__.py
│   └── adaptive_ekf.py      # 自适应 EKF 状态估计器
│
├── tracker/                 # 轨迹跟踪
│   ├── __init__.py
│   ├── mpc_controller.py    # MPC 控制器
│   ├── pure_pursuit.py      # Pure Pursuit 备用控制器
│   └── attitude_controller.py # 无人机姿态控制器 (F14)
│
├── consistency/             # 一致性检查
│   ├── __init__.py
│   └── weighted_analyzer.py # 加权一致性分析器
│
├── safety/                  # 安全模块
│   ├── __init__.py
│   ├── safety_monitor.py    # 安全监控器
│   ├── state_machine.py     # 状态机
│   └── timeout_monitor.py   # 超时监控器
│
├── health/                  # 健康监控
│   ├── __init__.py
│   └── mpc_health_monitor.py # MPC 健康监控器
│
├── transition/              # 平滑过渡
│   ├── __init__.py
│   └── smooth_transition.py # 平滑过渡控制器
│
├── transform/               # 坐标变换
│   ├── __init__.py
│   └── robust_transformer.py # 鲁棒坐标变换器
│
├── manager/                 # 控制器管理
│   ├── __init__.py
│   └── controller_manager.py # 控制器管理器
│
├── compat/                  # ROS 兼容层模块 (生产代码，用于独立运行模式)
│   ├── __init__.py
│   └── ros_compat_impl.py   # 独立运行实现 (StandaloneRospy, StandaloneTF2Buffer)
│
├── dashboard/               # 可视化界面
│   ├── __init__.py
│   ├── main_window.py       # 主窗口
│   ├── data_source.py       # 数据源接口
│   ├── styles.py            # 样式定义
│   ├── run_dashboard.py     # 启动脚本
│   ├── panels/              # 面板组件
│   │   ├── system_info.py   # 系统信息
│   │   ├── state_panel.py   # 状态面板
│   │   ├── mpc_health.py    # MPC健康
│   │   ├── degradation.py   # 降级状态
│   │   ├── timeout.py       # 超时监控
│   │   ├── consistency.py   # 一致性分析
│   │   ├── safety.py        # 安全约束
│   │   ├── trajectory.py    # 轨迹信息
│   │   ├── tracking.py      # 跟踪误差
│   │   ├── control.py       # 控制输出
│   │   ├── estimator.py     # 状态估计
│   │   ├── statistics.py    # 运行统计
│   │   └── alerts.py        # 警告提醒
│   └── widgets/             # 自定义控件
│       ├── progress_bar.py  # 进度条
│       ├── status_led.py    # 状态指示灯
│       └── state_indicator.py # 状态指示器
│
└── tests/                   # 测试模块
    ├── __init__.py
    ├── fixtures/            # 测试夹具
    │   ├── __init__.py
    │   ├── test_data_generator.py  # 测试数据生成 (轨迹、里程计、IMU)
    │   └── mock_diagnostics.py     # 模拟诊断数据 (仅用于 Dashboard 测试)
    ├── test_core.py         # 核心模块测试
    ├── test_components.py   # 组件测试
    ├── test_tf2_transform.py # TF2 变换测试 (F3)
    ├── test_attitude_controller.py # 姿态控制器测试 (F14)
    ├── test_integration.py  # 集成测试
    └── run_all_tests.py     # 运行所有测试
```

## 快速开始

### 安装依赖

```bash
pip install numpy
# 可选: 安装 ACADOS 以获得更好的 MPC 性能
```

### 基本使用

```python
from universal_controller import ControllerManager, DEFAULT_CONFIG

# 创建配置
config = DEFAULT_CONFIG.copy()
config['system']['platform'] = 'differential'  # 差速车

# 创建控制器管理器
manager = ControllerManager(config)
manager.initialize_default_components()

# 控制循环
cmd = manager.update(odom, trajectory)
print(f"vx={cmd.vx}, omega={cmd.omega}")

# 清理
manager.shutdown()
```

### 运行示例

```bash
python -m universal_controller.main
```

### 运行测试

```bash
python universal_controller/tests/run_all_tests.py
```

### 运行可视化 Dashboard

Dashboard 支持两种运行模式：

**独立模式 (使用模拟数据)**
```bash
python -m universal_controller.dashboard.run_dashboard
```

**ROS 模式 (订阅控制器话题)**
```bash
# 需要先启动控制器
roslaunch controller_ros controller.launch

# 然后启动 Dashboard
roslaunch controller_ros dashboard.launch
# 或
roslaunch controller_ros controller.launch dashboard:=true
```

Dashboard 提供实时监控界面，显示：
- 系统状态和 7 级降级状态
- MPC 健康状态 (求解时间、KKT残差、条件数)
- 一致性分析 (α_soft、曲率、速度方向、时序平滑)
- 超时监控 (Odom/Traj/IMU)
- **轨迹可视化** (2D/3D 视图，Hard轨迹、Soft速度向量、历史轨迹)
- 跟踪误差
- 控制输出
- 状态估计 (EKF)
- 运行统计
- 警告日志

依赖: `pip install PyQt5 matplotlib`

## 配置说明

主要配置参数：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `system.ctrl_freq` | 50 | 控制频率 (Hz) |
| `system.platform` | differential | 平台类型 |
| `mpc.horizon` | 20 | MPC 预测 horizon |
| `mpc.dt` | 0.02 | 时间步长 (s) |
| `constraints.v_max` | 2.0 | 最大速度 (m/s) |
| `constraints.omega_max` | 2.0 | 最大角速度 (rad/s) |

详细配置请参考 `config/default_config.py`。

## 状态机

控制器支持 7 种状态：

1. **INIT** - 初始化状态
2. **NORMAL** - 正常运行
3. **SOFT_DISABLED** - Soft Head 禁用
4. **MPC_DEGRADED** - MPC 降级
5. **BACKUP_ACTIVE** - 备用控制器激活
6. **STOPPING** - 停车中
7. **STOPPED** - 已停车

## 版本历史

- **v3.18.1** - 架构清理与模块职责统一:
  - 删除空的 `visualization/` 目录，避免与 `dashboard/` 和 `controller_ros/visualizer/` 混淆
  - 简化 `mock/` 模块，仅保留最小向后兼容层，标注将在 v4.0 移除
  - 清理 `compat/` 模块，不再导出数据类型（应从 `core.data_types` 导入）
  - 更新 REFACTORING_NOTES.md 记录架构变更
  - 所有 145 个测试通过
- **v3.18.0** - MPC 权重命名统一、接口修复:
  - 重命名 `control_v`/`control_omega` 为 `control_accel`/`control_alpha`
  - 修复 `ITrajectoryTracker.set_horizon()` 返回值为 `bool`
  - 修复 `ControllerBridge.notify_xxx_received()` 调用链
- **v3.17.12** - 新增轨迹可视化面板:
  - 在 Dashboard 中添加 `TrajectoryViewPanel` 实时轨迹可视化
  - 支持 2D/3D 视图切换
  - 显示 Hard 轨迹点、Soft 速度向量、历史轨迹
  - 标记当前位置、航向、最近点、前视点
  - 支持跟随机器人、自动缩放、网格显示等选项
  - 在 `DashboardDataSource` 中添加 `get_trajectory_data()` 接口
- **v3.17.11** - Bug 修复与代码健壮性增强:
  - 修复 `euler_from_quaternion` 未对输入四元数归一化的问题，添加数值稳定性保护
  - 修复 EKF `_compute_jacobian` 使用预测后状态而非当前状态计算 Jacobian 的问题
  - 修复 `ControllerManager.update` 中缺少 EKF 预测步骤的问题
  - 修复 MPC fallback 求解器中目标点在车辆后方时曲率计算错误的问题
  - 修复 Pure Pursuit `_compute_target_velocity` 对不同数组形状的处理
  - 修复姿态控制器悬停检测未考虑垂直速度的问题
  - 优化 MPC 健康监控器恢复条件，添加备选恢复路径
- **v3.17.10** - Bug 修复:
  - 修复状态机 `mpc_fail_count` 在所有状态转换中未正确重置的问题
  - 修复姿态控制器 `_compute_attitude_and_thrust` 中 `cos_roll` 除零保护阈值过小的问题 (1e-6 -> 0.1)
  - 修复 `Trajectory.copy()` 方法对空 velocities 数组的处理，保持原始状态一致性
  - 修复测试文件 `test_attitude_controller.py` 中断言逻辑过于宽松的问题
  - 修复测试文件 `test_tf2_transform.py` 中 Mock 类导入路径错误
  - 添加 `test_state_machine_mpc_fail_count_reset` 测试用例验证状态机计数器重置
- **v3.17.9** - 重构模拟数据模块:
  - 移除测试文件中内嵌的 `create_test_trajectory`、`create_test_odom` 函数
  - 统一使用 `mock/test_data_generator.py` 中的测试数据生成器
  - 修复 `Velocity3D` 类型引用错误，改用 numpy 数组
  - 更新 `mock/__init__.py` 导出所有模拟类和函数
  - 修复 `transform/robust_transformer.py` 中的 mock 类型导入路径
- **v3.17.8** - 增强 TF2 链式查找支持多跳 (BFS 算法，最多 10 跳)，完善诊断发布机制，添加 ROS 消息定义文件
- **v3.17.7** - 实现真实 TF2 集成，添加 ROS 兼容层，支持独立运行模式
- **v3.17.6** - 修复诊断发布类型错误，添加 `_last_published_diagnostics`
- **v3.17.5** - 添加 DiagnosticsV2 dataclass，组件延迟绑定
- **v3.17.4** - 添加组件注入方法，修复 theta_ref 未定义问题
- **v3.17.3** - 修复 IMU 超时检测，MPC horizon 动态调整
- **v3.17** - 完善 ACADOS 集成，添加诊断发布功能

## 许可证

MIT License
