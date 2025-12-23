"""
Universal Controller Dashboard - 可视化监控界面

提供实时监控界面，显示：
- 系统状态和降级级别
- MPC 健康状态
- 一致性分析
- 跟踪误差
- 警告提醒

支持两种数据源:
- DashboardDataSource: 直接从 ControllerManager 获取数据 (独立模式)
- ROSDashboardDataSource: 订阅 ROS 话题获取数据 (ROS 模式)
"""

# 使用延迟导入避免循环导入问题
def __getattr__(name):
    if name == 'DashboardWindow':
        from .main_window import DashboardWindow
        return DashboardWindow
    elif name == 'DashboardDataSource':
        from .data_source import DashboardDataSource
        return DashboardDataSource
    elif name == 'ROSDashboardDataSource':
        from .ros_data_source import ROSDashboardDataSource
        return ROSDashboardDataSource
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")

__all__ = ['DashboardWindow', 'DashboardDataSource', 'ROSDashboardDataSource']
