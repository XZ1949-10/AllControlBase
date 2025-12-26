#!/usr/bin/env python3
"""
ROS 数据源适配器 - 兼容性包装器

.. deprecated:: 3.20
    此模块已迁移到 controller_ros.dashboard.ros_data_source。
    请更新导入路径：
    
    旧代码:
        from universal_controller.dashboard import ROSDashboardDataSource
    
    新代码:
        from controller_ros.dashboard import ROSDashboardDataSource

迁移原因：
---------
ROSDashboardDataSource 依赖 controller_ros.msg.DiagnosticsV2 消息类型，
这违反了分层架构原则（算法核心层不应依赖 ROS 适配层）。

迁移后的架构：
- universal_controller.dashboard: 包含 Dashboard UI 和数据模型（ROS 无关）
  - DashboardDataSource: 直接访问 ControllerManager
  - models.py: 数据模型定义
  - main_window.py: UI 实现
  
- controller_ros.dashboard: 包含 ROS 数据源适配器
  - ROSDashboardDataSource: 订阅 ROS 话题获取数据

此文件保留是为了向后兼容，但会发出弃用警告。
"""

import warnings

# 发出弃用警告
warnings.warn(
    "Importing ROSDashboardDataSource from universal_controller.dashboard is deprecated. "
    "Please import from controller_ros.dashboard instead:\n"
    "  from controller_ros.dashboard import ROSDashboardDataSource",
    DeprecationWarning,
    stacklevel=2
)

# 尝试从新位置导入
try:
    from controller_ros.dashboard.ros_data_source import ROSDashboardDataSource
except ImportError:
    # 如果 controller_ros 不可用，提供一个占位类
    class ROSDashboardDataSource:
        """
        占位类 - controller_ros 包不可用
        
        请确保 controller_ros 包已安装并正确配置。
        """
        def __init__(self, *args, **kwargs):
            raise ImportError(
                "ROSDashboardDataSource has been moved to controller_ros.dashboard. "
                "Please install and source the controller_ros package, then import from:\n"
                "  from controller_ros.dashboard import ROSDashboardDataSource"
            )

__all__ = ['ROSDashboardDataSource']
