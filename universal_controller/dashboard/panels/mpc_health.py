"""
MPC 健康状态面板
"""

from PyQt5.QtWidgets import QGroupBox, QVBoxLayout, QHBoxLayout, QLabel, QGridLayout
from PyQt5.QtCore import Qt
from ..widgets.progress_bar import ColorProgressBar
from ..widgets.status_led import StatusLED
from ..styles import COLORS


class MPCHealthPanel(QGroupBox):
    """MPC 健康状态面板"""
    
    def __init__(self, parent=None):
        super().__init__('MPC 健康状态', parent)
        self._setup_ui()
    
    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(8)
        
        # 求解时间
        self._add_metric_row(layout, '求解时间', 'solve_time', 15, 'ms')
        
        # KKT 残差
        self._add_metric_row(layout, 'KKT 残差', 'kkt', 0.001, '')
        
        # 条件数
        self._add_metric_row(layout, '条件数', 'condition', 1e8, '')
        
        # 分隔线
        layout.addSpacing(5)
        
        # 连续警告次数
        warn_row = QHBoxLayout()
        warn_row.addWidget(QLabel('连续警告次数:'))
        self.warn_count_label = QLabel('0 / 3')
        warn_row.addWidget(self.warn_count_label)
        warn_row.addStretch()
        layout.addLayout(warn_row)
        
        # 连续良好次数
        good_row = QHBoxLayout()
        good_row.addWidget(QLabel('连续良好次数:'))
        self.good_count_label = QLabel('0')
        good_row.addWidget(self.good_count_label)
        good_row.addStretch()
        layout.addLayout(good_row)
        
        # Horizon
        horizon_row = QHBoxLayout()
        horizon_row.addWidget(QLabel('Horizon:'))
        self.horizon_label = QLabel('20 (正常) / 10 (降级)')
        horizon_row.addWidget(self.horizon_label)
        horizon_row.addStretch()
        layout.addLayout(horizon_row)
        
        layout.addSpacing(5)
        
        # 状态指示
        status_layout = QGridLayout()
        
        self.health_led = StatusLED('健康状态')
        self.warning_led = StatusLED('降级警告')
        self.recover_led = StatusLED('可恢复')
        
        status_layout.addWidget(self.health_led, 0, 0)
        status_layout.addWidget(self.warning_led, 0, 1)
        status_layout.addWidget(self.recover_led, 1, 0)
        
        layout.addLayout(status_layout)
    
    def _add_metric_row(self, parent_layout, label: str, name: str, threshold: float, unit: str):
        """添加指标行"""
        row_layout = QVBoxLayout()
        row_layout.setSpacing(2)
        
        # 标签
        label_widget = QLabel(label)
        row_layout.addWidget(label_widget)
        
        # 进度条
        progress = ColorProgressBar(show_percent=True)
        progress.set_unit(unit)
        progress.setObjectName(f'{name}_progress')
        row_layout.addWidget(progress)
        
        # 保存引用
        setattr(self, f'{name}_progress', progress)
        setattr(self, f'{name}_threshold', threshold)
        
        parent_layout.addLayout(row_layout)

    def update_display(self, data):
        """使用统一数据模型更新显示"""
        from ..models import DisplayData
        if not isinstance(data, DisplayData):
            return

        # 检查数据可用性
        if not data.availability.mpc_data_available:
            self._show_unavailable()
            return

        mpc = data.mpc_health
        platform = data.platform

        # 求解时间
        self.solve_time_progress.set_value(mpc.solve_time_ms, 15, 15)

        # KKT 残差
        self.kkt_progress.set_value(mpc.kkt_residual * 1000, 1, 1)

        # 条件数
        import math
        cond_log = math.log10(max(mpc.condition_number, 1))
        self.condition_progress.set_value(cond_log, 8, 8)

        # 连续警告次数
        self.warn_count_label.setText(f'{mpc.consecutive_near_timeout} / 3')
        if mpc.consecutive_near_timeout > 0:
            self.warn_count_label.setStyleSheet(f'color: {COLORS["warning"]};')
        else:
            self.warn_count_label.setStyleSheet('color: #FFFFFF;')

        # 连续良好次数
        self.good_count_label.setText('0')
        self.good_count_label.setStyleSheet('color: #FFFFFF;')

        # Horizon
        self.horizon_label.setText(f'{platform.mpc_horizon} (正常) / {platform.mpc_horizon_degraded} (降级)')

        # 状态指示
        self.health_led.set_status(mpc.healthy, '✓ 健康' if mpc.healthy else '✗ 异常')
        self.warning_led.set_status(not mpc.degradation_warning, '○ 无' if not mpc.degradation_warning else '⚠ 警告')
        self.recover_led.set_status(mpc.can_recover, '✓ 是' if mpc.can_recover else '✗ 否')

    def _show_unavailable(self):
        """显示数据不可用状态"""
        unavailable_style = f'color: {COLORS["unavailable"]};'
        
        # 进度条显示为空
        self.solve_time_progress.set_value(0, 15, 15)
        self.kkt_progress.set_value(0, 1, 1)
        self.condition_progress.set_value(0, 8, 8)
        
        # 标签显示不可用
        self.warn_count_label.setText('--')
        self.warn_count_label.setStyleSheet(unavailable_style)
        self.good_count_label.setText('--')
        self.good_count_label.setStyleSheet(unavailable_style)
        self.horizon_label.setText('无数据')
        self.horizon_label.setStyleSheet(unavailable_style)
        
        # LED 显示不可用
        self.health_led.set_status(None, '无数据')
        self.warning_led.set_status(None, '无数据')
        self.recover_led.set_status(None, '无数据')
