"""
超时监控面板

配置来源: universal_controller/config/system_config.py -> WATCHDOG_CONFIG
YAML 覆盖: controller_ros/config/turtlebot1.yaml -> watchdog 节
"""

from PyQt5.QtWidgets import QGroupBox, QVBoxLayout, QHBoxLayout, QLabel, QGridLayout
from PyQt5.QtCore import Qt
from ..widgets.progress_bar import ColorProgressBar
from ..widgets.status_led import StatusLED
from ..styles import COLORS

# 从统一配置模块导入默认值
from ...config import WATCHDOG_CONFIG


class TimeoutPanel(QGroupBox):
    """超时监控面板"""
    
    def __init__(self, parent=None, config=None):
        super().__init__('超时监控', parent)
        self._config = config or {}
        self._load_config()
        self._setup_ui()
    
    def _load_config(self):
        """从配置加载超时阈值，使用 WATCHDOG_CONFIG 作为默认值"""
        watchdog = self._config.get('watchdog', {})
        self._odom_timeout_ms = watchdog.get('odom_timeout_ms', WATCHDOG_CONFIG['odom_timeout_ms'])
        self._traj_timeout_ms = watchdog.get('traj_timeout_ms', WATCHDOG_CONFIG['traj_timeout_ms'])
        self._traj_grace_ms = watchdog.get('traj_grace_ms', WATCHDOG_CONFIG['traj_grace_ms'])
        self._imu_timeout_ms = watchdog.get('imu_timeout_ms', WATCHDOG_CONFIG['imu_timeout_ms'])
        self._startup_grace_ms = watchdog.get('startup_grace_ms', WATCHDOG_CONFIG['startup_grace_ms'])
    
    def set_config(self, config):
        """更新配置"""
        self._config = config or {}
        self._load_config()
        self._update_config_labels()
    
    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(8)
        
        # 数据源超时状态标题
        title1 = QLabel('数据源超时状态')
        title1.setStyleSheet('color: #2196F3; font-weight: bold; border-bottom: 1px solid #3D3D3D; padding-bottom: 3px;')
        layout.addWidget(title1)
        
        # Odom
        self.odom_led, self.odom_progress = self._add_timeout_row(layout, 'Odom', self._odom_timeout_ms)
        
        # Traj
        self.traj_led, self.traj_progress = self._add_timeout_row(layout, 'Traj', self._traj_timeout_ms)
        
        # IMU
        imu_thresh = self._imu_timeout_ms if self._imu_timeout_ms > 0 else 100
        self.imu_led, self.imu_progress = self._add_timeout_row(layout, 'IMU', imu_thresh)
        
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
        
        # 创建配置标签（保存引用以便后续更新）
        self._config_labels = {}
        configs = [
            ('odom', 'Odom超时阈值:', self._format_timeout(self._odom_timeout_ms)),
            ('traj', 'Traj超时阈值:', self._format_timeout(self._traj_timeout_ms)),
            ('traj_grace', 'Traj宽限期:', f'{self._traj_grace_ms} ms'),
            ('imu', 'IMU超时阈值:', self._format_timeout(self._imu_timeout_ms)),
            ('startup', '启动宽限期:', f'{self._startup_grace_ms} ms'),
        ]
        
        for i, (key, label, value) in enumerate(configs):
            config_grid.addWidget(QLabel(label), i, 0)
            value_label = QLabel(value)
            value_label.setStyleSheet('color: #B0B0B0;')
            config_grid.addWidget(value_label, i, 1)
            self._config_labels[key] = value_label
        
        layout.addLayout(config_grid)
        layout.addStretch()
    
    def _format_timeout(self, timeout_ms):
        """格式化超时值显示"""
        if timeout_ms <= 0:
            return '禁用'
        return f'{timeout_ms} ms'
    
    def _update_config_labels(self):
        """更新配置标签显示"""
        if hasattr(self, '_config_labels'):
            self._config_labels['odom'].setText(self._format_timeout(self._odom_timeout_ms))
            self._config_labels['traj'].setText(self._format_timeout(self._traj_timeout_ms))
            self._config_labels['traj_grace'].setText(f'{self._traj_grace_ms} ms')
            self._config_labels['imu'].setText(self._format_timeout(self._imu_timeout_ms))
            self._config_labels['startup'].setText(f'{self._startup_grace_ms} ms')
    
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

        # 使用配置的超时阈值
        odom_thresh = max(self._odom_timeout_ms, 1)  # 避免除零
        traj_thresh = max(self._traj_timeout_ms, 1)
        imu_thresh = max(self._imu_timeout_ms, 100) if self._imu_timeout_ms > 0 else 100

        # Odom
        self.odom_led.set_status(not t.odom_timeout, '✓ 正常' if not t.odom_timeout else '✗ 超时')
        self.odom_progress.set_value(t.last_odom_age_ms, odom_thresh, odom_thresh)

        # Traj
        self.traj_led.set_status(not t.traj_timeout, '✓ 正常' if not t.traj_timeout else '✗ 超时')
        self.traj_progress.set_value(t.last_traj_age_ms, traj_thresh, traj_thresh)

        # IMU - 考虑 mock 模式和禁用状态
        if env.is_mock_mode or not data.availability.imu_data_available or self._imu_timeout_ms <= 0:
            imu_status = '禁用' if self._imu_timeout_ms <= 0 else '无数据'
            self.imu_led.set_status(None, imu_status)
            self.imu_progress.set_value(0, imu_thresh, imu_thresh)
        else:
            self.imu_led.set_status(not t.imu_timeout, '✓ 正常' if not t.imu_timeout else '✗ 超时')
            self.imu_progress.set_value(t.last_imu_age_ms, imu_thresh, imu_thresh)

        # 宽限期
        self.startup_grace_led.set_status(not t.in_startup_grace, '✓ 已过' if not t.in_startup_grace else '○ 进行中')
        self.traj_grace_led.set_status(not t.traj_grace_exceeded, '○ 未触发' if not t.traj_grace_exceeded else '⚠ 已超过')

    def _show_unavailable(self):
        """显示数据不可用状态"""
        odom_thresh = max(self._odom_timeout_ms, 1)
        traj_thresh = max(self._traj_timeout_ms, 1)
        imu_thresh = max(self._imu_timeout_ms, 100) if self._imu_timeout_ms > 0 else 100
        
        # LED 显示不可用
        self.odom_led.set_status(None, '无数据')
        self.traj_led.set_status(None, '无数据')
        self.imu_led.set_status(None, '无数据')
        self.startup_grace_led.set_status(None, '无数据')
        self.traj_grace_led.set_status(None, '无数据')
        
        # 进度条显示为空
        self.odom_progress.set_value(0, odom_thresh, odom_thresh)
        self.traj_progress.set_value(0, traj_thresh, traj_thresh)
        self.imu_progress.set_value(0, imu_thresh, imu_thresh)
