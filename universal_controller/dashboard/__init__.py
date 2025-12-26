"""
Universal Controller Dashboard - 可视化监控界面

提供实时监控界面，显示：
- 系统状态和降级级别
- MPC 健康状态
- 一致性分析
- 跟踪误差
- 警告提醒

数据源:
-------
- DashboardDataSource: 直接从 ControllerManager 获取数据 (独立模式)
- ROSDashboardDataSource: 订阅 ROS 话题获取数据 (ROS 模式)
  注意: ROSDashboardDataSource 已迁移到 controller_ros.dashboard

架构说明:
--------
v3.20 重构：将 ROSDashboardDataSource 迁移到 controller_ros.dashboard

原因：ROSDashboardDataSource 依赖 controller_ros.msg，违反了分层架构原则。
迁移后，universal_controller 保持 ROS 无关性。

迁移指南：
    # 旧代码
    from universal_controller.dashboard import ROSDashboardDataSource
    
    # 新代码
    from controller_ros.dashboard import ROSDashboardDataSource
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
        # 发出弃用警告并尝试从新位置导入
        import warnings
        warnings.warn(
            "Importing ROSDashboardDataSource from universal_controller.dashboard is deprecated. "
            "Please import from controller_ros.dashboard instead.",
            DeprecationWarning,
            stacklevel=2
        )
        try:
            from controller_ros.dashboard import ROSDashboardDataSource
            return ROSDashboardDataSource
        except ImportError:
            # 回退到本地兼容性模块
            from .ros_data_source import ROSDashboardDataSource
            return ROSDashboardDataSource
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")

__all__ = ['DashboardWindow', 'DashboardDataSource', 'ROSDashboardDataSource']
