"""
ROS Dashboard 数据源模块

提供 ROSDashboardDataSource，通过订阅 ROS 话题获取诊断数据，
转换为 universal_controller.dashboard 的统一 DisplayData 模型。

架构说明：
---------
- universal_controller.dashboard: 包含 Dashboard UI 和数据模型（ROS 无关）
- controller_ros.dashboard: 包含 ROS 数据源适配器（本模块）

这种分离保持了 universal_controller 的纯净性，同时允许 ROS 层
提供特定的数据源实现。

使用示例：
    # ROS1
    from controller_ros.dashboard import ROSDashboardDataSource
    from universal_controller.dashboard import DashboardWindow
    
    data_source = ROSDashboardDataSource(config)
    window = DashboardWindow(data_source)
    
    # ROS2
    data_source = ROSDashboardDataSource(config, node=my_node)
    window = DashboardWindow(data_source)
"""

from .ros_data_source import ROSDashboardDataSource

__all__ = ['ROSDashboardDataSource']
