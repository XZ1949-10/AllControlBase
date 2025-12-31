"""
Dashboard 主窗口 - 使用统一数据模型
"""

import sys
import os

# 支持直接运行此文件
if __name__ == '__main__' and __package__ is None:
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
    __package__ = 'universal_controller.dashboard'

import time
from PyQt5.QtWidgets import (
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QLabel,
    QScrollArea,
    QFrame,
)
from PyQt5.QtCore import QTimer

from .styles import MAIN_STYLE, COLORS
from .data_source import DashboardDataSource
from .models import DisplayData, ControllerStateEnum
from .panels import (
    SystemInfoPanel,
    StatePanel,
    MPCHealthPanel,
    DegradationPanel,
    TimeoutPanel,
    ConsistencyPanel,
    SafetyPanel,
    TrajectoryPanel,
    TrajectoryViewPanel,
    TrackingPanel,
    ControlPanel,
    EstimatorPanel,
    StatisticsPanel,
    AlertsPanel,
)

STATE_COLORS = {
    ControllerStateEnum.INIT: COLORS['INIT'],
    ControllerStateEnum.NORMAL: COLORS['NORMAL'],
    ControllerStateEnum.SOFT_DISABLED: COLORS['SOFT_DISABLED'],
    ControllerStateEnum.MPC_DEGRADED: COLORS['MPC_DEGRADED'],
    ControllerStateEnum.BACKUP_ACTIVE: COLORS['BACKUP_ACTIVE'],
    ControllerStateEnum.STOPPING: COLORS['STOPPING'],
    ControllerStateEnum.STOPPED: COLORS['STOPPED'],
}


class DashboardWindow(QMainWindow):
    def __init__(self, data_source: DashboardDataSource = None):
        super().__init__()
        self.data_source = data_source or DashboardDataSource()
        self._last_data: DisplayData = None
        self._setup_ui()
        self._setup_timer()
        self.alerts_panel.add_alert('info', '系统启动，Dashboard 初始化完成')

    def _setup_ui(self):
        self.setWindowTitle('Universal Controller Dashboard')
        self.setMinimumSize(1400, 900)
        self.setStyleSheet(MAIN_STYLE)
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)
        main_layout.setSpacing(10)
        main_layout.setContentsMargins(10, 10, 10, 10)
        self.status_bar = self._create_status_bar()
        main_layout.addWidget(self.status_bar)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll_content = QWidget()
        scroll_layout = QVBoxLayout(scroll_content)
        scroll_layout.setSpacing(10)
        self.system_info_panel = SystemInfoPanel()
        scroll_layout.addWidget(self.system_info_panel)
        row1 = QHBoxLayout()
        row1.setSpacing(10)
        self.state_panel = StatePanel()
        self.state_panel.setFixedWidth(280)
        row1.addWidget(self.state_panel)
        self.mpc_health_panel = MPCHealthPanel()
        row1.addWidget(self.mpc_health_panel)
        scroll_layout.addLayout(row1)
        row2 = QHBoxLayout()
        row2.setSpacing(10)
        self.degradation_panel = DegradationPanel()
        row2.addWidget(self.degradation_panel)
        # 获取配置传递给 TimeoutPanel
        config = getattr(self.data_source, '_config', {})
        self.timeout_panel = TimeoutPanel(config=config)
        row2.addWidget(self.timeout_panel)
        scroll_layout.addLayout(row2)
        row3 = QHBoxLayout()
        row3.setSpacing(10)
        self.consistency_panel = ConsistencyPanel(config=config)
        row3.addWidget(self.consistency_panel)
        self.safety_panel = SafetyPanel(config=config)
        row3.addWidget(self.safety_panel)
        scroll_layout.addLayout(row3)
        row4 = QHBoxLayout()
        row4.setSpacing(10)
        self.trajectory_panel = TrajectoryPanel()
        row4.addWidget(self.trajectory_panel)
        self.tracking_panel = TrackingPanel(config=config)
        row4.addWidget(self.tracking_panel)
        scroll_layout.addLayout(row4)
        self.trajectory_view_panel = TrajectoryViewPanel()
        self.trajectory_view_panel.setMinimumHeight(450)
        scroll_layout.addWidget(self.trajectory_view_panel)
        row5 = QHBoxLayout()
        row5.setSpacing(10)
        self.control_panel = ControlPanel()
        row5.addWidget(self.control_panel)
        self.estimator_panel = EstimatorPanel()
        row5.addWidget(self.estimator_panel)
        scroll_layout.addLayout(row5)
        self.statistics_panel = StatisticsPanel()
        scroll_layout.addWidget(self.statistics_panel)
        self.alerts_panel = AlertsPanel()
        scroll_layout.addWidget(self.alerts_panel)
        scroll.setWidget(scroll_content)
        main_layout.addWidget(scroll)

    def _create_status_bar(self) -> QWidget:
        bar = QFrame()
        bar.setFixedHeight(40)
        bar.setStyleSheet(f"QFrame {{ background-color: {COLORS['bg_header']}; border-radius: 5px; }}")
        layout = QHBoxLayout(bar)
        layout.setContentsMargins(15, 5, 15, 5)
        layout.setSpacing(20)
        title = QLabel('Universal Controller Dashboard')
        title.setStyleSheet('font-size: 14px; font-weight: bold; color: #2196F3;')
        layout.addWidget(title)
        self.mock_mode_label = QLabel('[模拟模式]')
        self.mock_mode_label.setStyleSheet('color: #FF9800; font-weight: bold;')
        self.mock_mode_label.setVisible(False)
        layout.addWidget(self.mock_mode_label)
        layout.addStretch()
        layout.addWidget(QLabel('状态:'))
        self.status_state_label = QLabel('[INIT]')
        self.status_state_label.setStyleSheet(f'color: {COLORS["INIT"]}; font-weight: bold;')
        layout.addWidget(self.status_state_label)
        layout.addWidget(QLabel('控制器:'))
        self.status_ctrl_label = QLabel('[MPC]')
        self.status_ctrl_label.setStyleSheet(f'color: {COLORS["success"]};')
        layout.addWidget(self.status_ctrl_label)
        layout.addWidget(QLabel('平台:'))
        self.status_platform_label = QLabel('[差速车]')
        layout.addWidget(self.status_platform_label)
        self.status_ros_label = QLabel('ROS: ?')
        layout.addWidget(self.status_ros_label)
        self.status_acados_label = QLabel('ACADOS: ?')
        layout.addWidget(self.status_acados_label)
        self.status_tf2_label = QLabel('TF2: ?')
        layout.addWidget(self.status_tf2_label)
        self.status_imu_label = QLabel('IMU: ?')
        layout.addWidget(self.status_imu_label)
        layout.addStretch()
        self.status_runtime_label = QLabel('运行: 00:00:00')
        self.status_runtime_label.setStyleSheet('color: #B0B0B0;')
        layout.addWidget(self.status_runtime_label)
        self.status_freq_label = QLabel('[0.0 Hz]')
        self.status_freq_label.setStyleSheet('color: #4CAF50; font-weight: bold;')
        layout.addWidget(self.status_freq_label)
        # 版本号从 DisplayData 获取，初始显示空字符串
        self.version_label = QLabel('')
        self.version_label.setStyleSheet('color: #606060;')
        layout.addWidget(self.version_label)
        return bar

    def _setup_timer(self):
        self.refresh_timer = QTimer()
        self.refresh_timer.timeout.connect(self._refresh)
        self.refresh_timer.start(50)

    def _refresh(self):
        data = self.data_source.get_display_data()
        self._update_status_bar(data)
        self.system_info_panel.update_display(data)
        self.state_panel.update_display(data)
        self.mpc_health_panel.update_display(data)
        self.degradation_panel.update_display(data)
        self.timeout_panel.update_display(data)
        self.consistency_panel.update_display(data)
        self.safety_panel.update_display(data)
        self.trajectory_panel.update_display(data)
        self.trajectory_view_panel.update_display(data)
        self.tracking_panel.update_display(data)
        self.control_panel.update_display(data)
        self.estimator_panel.update_display(data)
        self.statistics_panel.update_display(data)
        self.alerts_panel.check_alerts(data, self._last_data)
        self._last_data = data

    def _update_status_bar(self, data: DisplayData):
        env = data.environment
        ctrl = data.controller
        stats = data.statistics
        avail = data.availability
        
        self.mock_mode_label.setVisible(env.is_mock_mode)
        
        # 检查数据是否可用
        if not avail.diagnostics_available:
            self.status_state_label.setText('[无数据]')
            self.status_state_label.setStyleSheet(f'color: {COLORS["unavailable"]}; font-weight: bold;')
            self.status_ctrl_label.setText('[--]')
            self.status_ctrl_label.setStyleSheet(f'color: {COLORS["unavailable"]};')
        else:
            color = STATE_COLORS.get(ctrl.state, COLORS['disabled'])
            self.status_state_label.setText(f'[{ctrl.state_name}]')
            self.status_state_label.setStyleSheet(f'color: {color}; font-weight: bold;')
            ctrl_color = COLORS['warning'] if ctrl.backup_active else COLORS['success']
            self.status_ctrl_label.setText(f'[{ctrl.current_controller}]')
            self.status_ctrl_label.setStyleSheet(f'color: {ctrl_color};')
        
        self.status_platform_label.setText(f'[{data.platform.platform_display}]')
        self._set_indicator(self.status_ros_label, 'ROS', env.ros_available)
        self._set_indicator(self.status_acados_label, 'ACADOS', env.acados_available)
        self._set_indicator(self.status_tf2_label, 'TF2', env.tf2_available)
        self._set_indicator(self.status_imu_label, 'IMU', env.imu_available)
        self.status_runtime_label.setText(f'运行: {stats.elapsed_time_str}')
        self.status_freq_label.setText(f'[{stats.actual_freq:.1f} Hz]')
        self.version_label.setText(data.version)

    def _set_indicator(self, label: QLabel, name: str, status: bool):
        if status:
            label.setText(f'{name}: ✓')
            label.setStyleSheet(f'color: {COLORS["success"]};')
        else:
            label.setText(f'{name}: ✗')
            label.setStyleSheet(f'color: {COLORS["error"]};')

    def closeEvent(self, event):
        self.refresh_timer.stop()
        event.accept()


def run_dashboard(controller_manager, config: dict = None):
    """
    启动 Dashboard 界面
    
    Args:
        controller_manager: ControllerManager 实例（必需）
        config: 配置字典
    
    注意：
        此函数需要传入有效的 controller_manager。
        如需测试界面，请使用 tests/run_dashboard_mock.py
    """
    import sys
    from PyQt5.QtWidgets import QApplication
    
    if controller_manager is None:
        raise ValueError(
            "controller_manager is required. "
            "For testing with mock data, use: python -m universal_controller.tests.run_dashboard_mock"
        )
    
    app = QApplication(sys.argv)
    data_source = DashboardDataSource(controller_manager, config)
    window = DashboardWindow(data_source)
    window.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    print("错误: 此模块不能直接运行")
    print("请使用以下方式之一:")
    print("  1. ROS 模式: roslaunch controller_ros core/controller.launch dashboard:=true")
    print("  2. 测试模式: python -m universal_controller.tests.run_dashboard_mock")
    sys.exit(1)
