"""
controller_ros - ROS 胶水层

将 universal_controller 纯算法库与 ROS 生态系统集成。

模块结构:
- node/: 节点层 (主节点)
- adapters/: 适配器层 (消息转换)
- bridge/: 桥接层 (控制器封装)
- io/: IO 层 (订阅/发布)
- utils/: 工具层
- lifecycle/: 生命周期管理
- dashboard/: ROS Dashboard 数据源 (v3.20 新增)
"""

__version__ = "1.0.0"
