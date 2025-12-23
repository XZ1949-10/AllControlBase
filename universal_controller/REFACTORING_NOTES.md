# 代码重构说明

## 重构日期: 2024-12-23 (更新)

## 最新更新: 模拟数据控制机制

### 问题: Dashboard 和其他模块可能自动使用模拟数据
- **需求**: 除非明确配置允许，否则所有模块都不应使用模拟数据
- **解决方案**:
  1. 新增 `config/mock_config.py` 配置文件，控制模拟数据使用
  2. 新增 `DataAvailability` 数据模型，标记各类数据的可用性
  3. 修改 Dashboard 面板，当数据不可用时显示"无数据"而非默认值
  4. 修改数据源，不再自动生成模拟数据

### 新增配置项 (mock_config.py)
```python
MOCK_CONFIG = {
    'allow_mock_data': False,  # 全局开关，默认禁用
    'dashboard': {
        'allow_mock_diagnostics': False,
        'allow_mock_trajectory': False,
        'allow_mock_position': False,
    },
    'ros_compat': {
        'allow_standalone_mode': True,  # 允许独立运行模式
        'allow_mock_tf2': False,
    },
    'controller': {
        'allow_mock_odom': False,
        'allow_mock_imu': False,
        'allow_mock_trajectory': False,
    },
}
```

### 数据可用性模型 (DataAvailability)
```python
@dataclass
class DataAvailability:
    diagnostics_available: bool = False
    trajectory_available: bool = False
    position_available: bool = False
    odom_available: bool = False
    imu_data_available: bool = False
    mpc_data_available: bool = False
    consistency_data_available: bool = False
    tracking_data_available: bool = False
    estimator_data_available: bool = False
    transform_data_available: bool = False
    last_update_time: float = 0.0
    data_age_ms: float = 0.0
```

### 修改的文件
- `config/mock_config.py` - 新增模拟数据配置
- `config/default_config.py` - 集成 mock 配置
- `config/__init__.py` - 导出 mock 配置
- `dashboard/models.py` - 新增 DataAvailability 模型
- `dashboard/data_source.py` - 添加数据可用性检测
- `dashboard/ros_data_source.py` - 添加数据可用性检测
- `dashboard/styles.py` - 添加"不可用"状态样式
- `dashboard/widgets/status_led.py` - 支持 None 状态
- `dashboard/main_window.py` - 状态栏显示数据连接状态
- `dashboard/panels/mpc_health.py` - 添加 `_show_unavailable()` 方法
- `dashboard/panels/consistency.py` - 添加 `_show_unavailable()` 方法
- `dashboard/panels/tracking.py` - 添加 `_show_unavailable()` 方法
- `dashboard/panels/estimator.py` - 添加 `_show_unavailable()` 方法
- `dashboard/panels/timeout.py` - 添加 `_show_unavailable()` 方法
- `dashboard/panels/control.py` - 添加 `_show_unavailable()` 方法
- `dashboard/panels/safety.py` - 添加 `_show_unavailable()` 方法
- `dashboard/panels/state_panel.py` - 添加 `_show_unavailable()` 方法
- `dashboard/panels/degradation.py` - 添加 `_show_unavailable()` 方法
- `dashboard/panels/statistics.py` - 添加数据可用性检查
- `dashboard/panels/trajectory.py` - 添加 `_show_unavailable()` 方法
- `dashboard/panels/system_info.py` - 添加 `_show_unavailable_dynamic()` 方法
- `tests/run_dashboard_mock.py` - 明确标记为测试模式

### 面板数据可用性检查逻辑
每个面板在 `update_display()` 方法中检查相应的数据可用性标志：
- `mpc_health.py`: 检查 `data.availability.mpc_data_available`
- `consistency.py`: 检查 `data.availability.consistency_data_available`
- `tracking.py`: 检查 `data.availability.tracking_data_available`
- `estimator.py`: 检查 `data.availability.estimator_data_available`
- `timeout.py`: 检查 `data.availability.diagnostics_available`
- `control.py`: 检查 `data.availability.diagnostics_available`
- `safety.py`: 检查 `data.availability.diagnostics_available`
- `state_panel.py`: 检查 `data.availability.diagnostics_available`
- `degradation.py`: 检查 `data.availability.diagnostics_available`
- `statistics.py`: 部分数据始终显示，状态统计检查 `diagnostics_available`
- `trajectory.py`: 检查 `data.availability.trajectory_available`
- `system_info.py`: 环境和配置始终显示，动态数据检查 `diagnostics_available`

---

## 问题分析与修复

### 问题 1: Mock 命名混淆 ✅ 已修复
- **位置**: `compat/ros_compat_impl.py`
- **问题**: 类名使用 `Mock` 前缀，但这是生产代码的一部分（用于非 ROS 环境运行）
- **解决**: 重命名为 `Standalone*` 前缀，保留 `Mock*` 别名以保持向后兼容

### 问题 2: 代码重复 ✅ 已修复
- **位置**: `mock/` vs `compat/`
- **问题**: `mock/ros_mock.py` 和 `compat/ros_compat_impl.py` 有重复实现
- **解决**: 删除 `mock/` 中的重复文件，统一使用 `compat/` 中的实现

### 问题 3: 模拟诊断数据在生产路径 ✅ 已修复
- **位置**: `compat/diagnostics_generator.py`
- **问题**: 生成模拟数据的代码不应该在 `compat/` 目录
- **解决**: 
  - 移动到 `tests/fixtures/mock_diagnostics.py`
  - 删除 `compat/diagnostics_generator.py`

### 问题 4: 测试数据生成器位置不当 ✅ 已修复
- **位置**: `mock/test_data_generator.py` 和 `tests/test_data_generator.py`
- **问题**: 测试数据生成器应该只在 `tests/` 目录
- **解决**: 统一移动到 `tests/fixtures/test_data_generator.py`

### 问题 5: mock 模块混合导出测试代码 ✅ 已修复
- **位置**: `mock/__init__.py`
- **问题**: mock 模块从 tests/fixtures 导入测试数据生成器，导致生产代码可能依赖测试代码
- **解决**: 
  - mock 模块只导出 ROS 兼容层别名
  - 测试数据生成器不再从 mock 模块导出
  - 在 ROS 环境中导入 mock 模块会发出更强的警告

## 当前目录结构

```
universal_controller/
├── compat/                           # ROS 兼容层 (生产代码，用于独立运行模式)
│   ├── __init__.py                   # 导出 Standalone* 类和 Mock* 别名
│   └── ros_compat_impl.py            # StandaloneRospy, StandaloneTF2Buffer 等
│
├── core/
│   └── ros_compat.py                 # 自动检测 ROS 环境，选择真实或独立实现
│
├── mock/                             # 向后兼容模块 (已弃用)
│   └── __init__.py                   # 仅导出 ROS 兼容层别名，带弃用警告
│
└── tests/
    ├── __init__.py
    ├── fixtures/                     # 测试夹具
    │   ├── __init__.py               # 导出所有测试数据生成器
    │   ├── test_data_generator.py    # 测试数据生成 (轨迹、里程计、IMU)
    │   └── mock_diagnostics.py       # 模拟诊断数据 (仅用于 Dashboard 测试)
    ├── run_dashboard_mock.py         # Dashboard 测试模式启动脚本
    └── test_*.py                     # 测试文件
```

## 数据隔离原则

### 真实数据流 (ROS 环境)
```
真实传感器 → ROS 话题 → controller_ros → DataManager → ControllerManager
                                              ↓
                                        真实 TF2 查询
```

### 独立运行模式 (非 ROS 环境)
```
测试数据生成器 → Standalone* 类 → ControllerManager
(tests/fixtures/)   (compat/)
```

## 命名约定

### 生产代码 (compat/)
- `StandaloneRospy` - rospy 的独立运行替代实现
- `StandaloneTF2Buffer` - tf2_ros.Buffer 的独立运行替代实现
- `StandaloneTF2Ros` - tf2_ros 模块的独立运行替代实现
- `StandaloneTFTransformations` - tf.transformations 的独立运行替代实现

### 向后兼容别名 (仅用于迁移)
- `MockRospy` → `StandaloneRospy`
- `MockTF2BufferCore` → `StandaloneTF2Buffer`
- `MockTF2Ros` → `StandaloneTF2Ros`
- `MockTFTransformations` → `StandaloneTFTransformations`

## 迁移指南

### 旧代码 (已弃用)
```python
# 不推荐
from universal_controller.mock import MockRospy, create_test_trajectory
from universal_controller.compat.diagnostics_generator import generate_mock_diagnostics
```

### 新代码 (推荐)

**生产代码 (ROS 兼容层):**
```python
from universal_controller.compat import StandaloneRospy
# 或使用向后兼容别名
from universal_controller.compat import MockRospy
```

**测试代码:**
```python
from universal_controller.tests.fixtures import (
    create_test_trajectory,
    create_test_odom,
    generate_mock_diagnostics,
)
```

## 弃用警告

以下导入路径已弃用，会显示警告：

1. `from universal_controller.mock import ...`
   - 非 ROS 环境: DeprecationWarning
   - ROS 环境: UserWarning (更强的警告)

2. 测试数据生成器不再从 mock 模块导出
   - 请直接从 `tests.fixtures` 导入

## 注意事项

1. **生产代码不应导入 `tests/` 模块**
   - `tests/fixtures/` 仅用于测试
   - 生产代码应使用 `compat/` 中的实现

2. **ROS 胶水层 (controller_ros)**
   - 应该使用真实的 ROS 模块 (rospy, tf2_ros)
   - 不应该导入 `mock/` 或 `tests/fixtures/`
   - ROS 环境下自动使用真实实现

3. **Dashboard 模式**
   - ROS 模式: 使用 `ROSDashboardDataSource` 订阅真实话题
   - 测试模式: 使用 `tests/run_dashboard_mock.py` 生成模拟数据

4. **自动检测机制**
   - `core/ros_compat.py` 自动检测 ROS 环境
   - ROS 可用时使用真实模块
   - ROS 不可用时使用 `compat/` 中的独立实现
