"""
超时监控面板
"""

from PyQt5.QtWidgets import QGroupBox, QVBoxLayout, QHBoxLayout, QLabel, QGridLayout
from PyQt5.QtCore import Qt
from ..widgets.progress_bar import ColorProgressBar
from ..widgets.status_led import StatusLED
from ..styles import COLORS


class TimeoutPanel(QGroupBox):
    """超时监控面板"""
    
    def __init__(self, parent=None):
        super().__init__('超时监控', parent)
        self._setup_ui()
    
    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(8)
        
        # 数据源超时状态标题
        title1 = QLabel('数据源超时状态')
        title1.setStyleSheet('color: #2196F3; font-weight: bold; border-bottom: 1px solid #3D3D3D; padding-bottom: 3px;')
        layout.addWidget(title1)
        
        # Odom
        self.odom_led, self.odom_progress = self._add_timeout_row(layout, 'Odom', 200)
        
        # Traj
        self.traj_led, self.traj_progress = self._add_timeout_row(layout, 'Traj', 200)
        
        # IMU
        self.imu_led, self.imu_progress = self._add_timeout_row(layout, 'IMU', 100)
        
        layout.addSpacing(10)
        
        # 宽限期状态标题
        title2 = QLabel('宽限期状态')
        title2.setStyleSheet('color: #2196F3; font-weight: bold; border-bottom: 1px solid #3D3D3D; padding-bottom: 3px;')
        layout.addWidget(title2)
        
        # 启动宽限期
        self.startup_grace_led = StatusLED('启动宽限期')
        layout.addWidget(self.startup_grace_led)
        
        # 轨迹宽限期
        self.traj_grace_led = StatusLED('轨迹宽限期')
        layout.addWidget(self.traj_grace_led)
        
        layout.addSpacing(10)
        
        # 超时配置标题
        title3 = QLabel('超时配置')
        title3.setStyleSheet('color: #2196F3; font-weight: bold; border-bottom: 1px solid #3D3D3D; padding-bottom: 3px;')
        layout.addWidget(title3)
        
        # 配置信息
        config_grid = QGridLayout()
        config_grid.setSpacing(3)
        
        configs = [
            ('Odom超时阈值:', '200 ms'),
            ('Traj超时阈值:', '200 ms'),
            ('Traj宽限期:', '100 ms'),
            ('IMU超时阈值:', '100 ms'),
            ('启动宽限期:', '1000 ms'),
        ]
        
        for i, (label, value) in enumerate(configs):
            config_grid.addWidget(QLabel(label), i, 0)
            value_label = QLabel(value)
            value_label.setStyleSheet('color: #B0B0B0;')
            config_grid.addWidget(value_label, i, 1)
        
        layout.addLayout(config_grid)
        layout.addStretch()
    
    def _add_timeout_row(self, parent_layout, name: str, threshold: int):
        """添加超时行"""
        row_layout = QHBoxLayout()
        row_layout.setSpacing(10)
        
        # LED 和名称
        led = StatusLED(f'{name}:')
        led.setFixedWidth(80)
        row_layout.addWidget(led)
        
        # 进度条
        progress = ColorProgressBar(show_percent=True)
        progress.set_unit(' ms')
        row_layout.addWidget(progress, 1)
        
        parent_layout.addLayout(row_layout)
        
        return led, progress

    def update_display(self, data):
        """使用统一数据模型更新显示"""
        from ..models import DisplayData
        if not isinstance(data, DisplayData):
            return

        # 检查数据可用性
        if not data.availability.diagnostics_available:
            self._show_unavailable()
            return

        t = data.timeout
        env = data.environment

        # Odom
        self.odom_led.set_status(not t.odom_timeout, '✓ 正常' if not t.odom_timeout else '✗ 超时')
        self.odom_progress.set_value(t.last_odom_age_ms, 200, 200)

        # Traj
        self.traj_led.set_status(not t.traj_timeout, '✓ 正常' if not t.traj_timeout else '✗ 超时')
        self.traj_progress.set_value(t.last_traj_age_ms, 200, 200)

        # IMU - 考虑 mock 模式
        if env.is_mock_mode or not data.availability.imu_data_available:
            self.imu_led.set_status(None, '无数据')
            self.imu_progress.set_value(0, 100, 100)
        else:
            self.imu_led.set_status(not t.imu_timeout, '✓ 正常' if not t.imu_timeout else '✗ 超时')
            self.imu_progress.set_value(t.last_imu_age_ms, 100, 100)

        # 宽限期
        self.startup_grace_led.set_status(not t.in_startup_grace, '✓ 已过' if not t.in_startup_grace else '○ 进行中')
        self.traj_grace_led.set_status(not t.traj_grace_exceeded, '○ 未触发' if not t.traj_grace_exceeded else '⚠ 已超过')

    def _show_unavailable(self):
        """显示数据不可用状态"""
        # LED 显示不可用
        self.odom_led.set_status(None, '无数据')
        self.traj_led.set_status(None, '无数据')
        self.imu_led.set_status(None, '无数据')
        self.startup_grace_led.set_status(None, '无数据')
        self.traj_grace_led.set_status(None, '无数据')
        
        # 进度条显示为空
        self.odom_progress.set_value(0, 200, 200)
        self.traj_progress.set_value(0, 200, 200)
        self.imu_progress.set_value(0, 100, 100)
