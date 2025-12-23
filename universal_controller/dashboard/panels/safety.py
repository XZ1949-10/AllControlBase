"""
安全约束状态面板
"""

from PyQt5.QtWidgets import QGroupBox, QVBoxLayout, QHBoxLayout, QLabel, QGridLayout
from PyQt5.QtCore import Qt
from ..widgets.progress_bar import ColorProgressBar
from ..widgets.status_led import StatusLED
from ..styles import COLORS


class SafetyPanel(QGroupBox):
    """安全约束状态面板"""
    
    def __init__(self, parent=None):
        super().__init__('安全约束状态', parent)
        self._setup_ui()
    
    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(8)
        
        # 速度约束
        vel_title = QLabel('速度约束')
        vel_title.setStyleSheet('color: #2196F3; font-weight: bold; border-bottom: 1px solid #3D3D3D; padding-bottom: 3px;')
        layout.addWidget(vel_title)
        
        self.v_max_progress = self._add_constraint_row(layout, 'v_max', 2.0, 'm/s')
        self.omega_max_progress = self._add_constraint_row(layout, 'omega_max', 2.0, 'rad/s')
        
        layout.addSpacing(5)
        
        # 加速度约束
        accel_title = QLabel('加速度约束')
        accel_title.setStyleSheet('color: #2196F3; font-weight: bold; border-bottom: 1px solid #3D3D3D; padding-bottom: 3px;')
        layout.addWidget(accel_title)
        
        self.a_max_progress = self._add_constraint_row(layout, 'a_max', 1.5, 'm/s²')
        self.az_max_progress = self._add_constraint_row(layout, 'az_max', 1.0, 'm/s²')
        self.alpha_max_progress = self._add_constraint_row(layout, 'alpha_max', 3.0, 'rad/s²')
        
        layout.addSpacing(5)
        
        # 低速保护
        low_title = QLabel('低速保护')
        low_title.setStyleSheet('color: #2196F3; font-weight: bold; border-bottom: 1px solid #3D3D3D; padding-bottom: 3px;')
        layout.addWidget(low_title)
        
        low_grid = QGridLayout()
        low_grid.setSpacing(3)
        
        low_grid.addWidget(QLabel('低速阈值:'), 0, 0)
        low_grid.addWidget(QLabel('0.1 m/s'), 0, 1)
        
        low_grid.addWidget(QLabel('低速角速度限制:'), 1, 0)
        low_grid.addWidget(QLabel('1.0 rad/s'), 1, 1)
        
        self.low_speed_led = StatusLED('当前状态')
        low_grid.addWidget(self.low_speed_led, 2, 0, 1, 2)
        
        layout.addLayout(low_grid)
        
        layout.addSpacing(5)
        
        # 安全裕度
        margin_title = QLabel('安全裕度')
        margin_title.setStyleSheet('color: #2196F3; font-weight: bold; border-bottom: 1px solid #3D3D3D; padding-bottom: 3px;')
        layout.addWidget(margin_title)
        
        margin_grid = QGridLayout()
        margin_grid.setSpacing(3)
        
        margin_grid.addWidget(QLabel('速度裕度:'), 0, 0)
        margin_grid.addWidget(QLabel('110% (1.1x)'), 0, 1)
        
        margin_grid.addWidget(QLabel('加速度裕度:'), 1, 0)
        margin_grid.addWidget(QLabel('150% (1.5x)'), 1, 1)
        
        layout.addLayout(margin_grid)
        
        layout.addSpacing(5)
        
        # 安全检查状态
        self.safety_led = StatusLED('安全检查')
        layout.addWidget(self.safety_led)
        
        last_row = QHBoxLayout()
        last_row.addWidget(QLabel('上次限制:'))
        self.last_limit_label = QLabel('无')
        self.last_limit_label.setStyleSheet('color: #B0B0B0;')
        last_row.addWidget(self.last_limit_label)
        last_row.addStretch()
        layout.addLayout(last_row)
        
        layout.addStretch()
    
    def _add_constraint_row(self, parent_layout, name: str, max_val: float, unit: str):
        """添加约束行"""
        row_layout = QHBoxLayout()
        row_layout.setSpacing(5)
        
        # 名称和最大值
        label = QLabel(f'{name}: {max_val} {unit}')
        label.setFixedWidth(150)
        row_layout.addWidget(label)
        
        # 当前值标签
        current_label = QLabel('当前: 0.00')
        current_label.setObjectName(f'{name}_current')
        current_label.setFixedWidth(100)
        row_layout.addWidget(current_label)
        
        # 进度条
        progress = ColorProgressBar(show_text=False)
        row_layout.addWidget(progress, 1)
        
        parent_layout.addLayout(row_layout)
        
        setattr(self, f'{name}_current_label', current_label)
        
        return progress

    def update_display(self, data):
        """使用统一数据模型更新显示"""
        from ..models import DisplayData
        import math
        if not isinstance(data, DisplayData):
            return

        # 检查数据可用性
        if not data.availability.diagnostics_available:
            self._show_unavailable()
            return

        safety = data.safety
        cmd = data.command

        # 速度
        v = math.sqrt(cmd.vx**2 + cmd.vy**2)
        self.v_max_progress.set_value(v, safety.v_max)
        self.v_max_current_label.setText(f'当前: {v:.2f}')

        # 角速度
        omega = abs(cmd.omega)
        self.omega_max_progress.set_value(omega, safety.omega_max)
        self.omega_max_current_label.setText(f'当前: {omega:.2f}')

        # 加速度 (使用占位值)
        self.a_max_progress.set_value(0.5, safety.a_max)
        self.a_max_current_label.setText('当前: 0.50')

        self.az_max_progress.set_value(0.0, 1.0)
        self.az_max_current_label.setText('当前: 0.00')

        self.alpha_max_progress.set_value(0.3, 3.0)
        self.alpha_max_current_label.setText('当前: 0.30')

        # 低速保护
        low_speed = safety.low_speed_protection_active
        self.low_speed_led.set_status(not low_speed, '○ 未触发' if not low_speed else '● 已触发')

        # 安全检查
        safety_ok = safety.safety_check_passed
        self.safety_led.set_status(safety_ok, '✓ 全部通过' if safety_ok else '✗ 检查失败')

    def _show_unavailable(self):
        """显示数据不可用状态"""
        unavailable_style = f'color: {COLORS["unavailable"]};'
        
        # 进度条显示为空
        self.v_max_progress.set_value(0, 2.0)
        self.omega_max_progress.set_value(0, 2.0)
        self.a_max_progress.set_value(0, 1.5)
        self.az_max_progress.set_value(0, 1.0)
        self.alpha_max_progress.set_value(0, 3.0)
        
        # 当前值标签显示不可用
        self.v_max_current_label.setText('当前: --')
        self.v_max_current_label.setStyleSheet(unavailable_style)
        self.omega_max_current_label.setText('当前: --')
        self.omega_max_current_label.setStyleSheet(unavailable_style)
        self.a_max_current_label.setText('当前: --')
        self.a_max_current_label.setStyleSheet(unavailable_style)
        self.az_max_current_label.setText('当前: --')
        self.az_max_current_label.setStyleSheet(unavailable_style)
        self.alpha_max_current_label.setText('当前: --')
        self.alpha_max_current_label.setStyleSheet(unavailable_style)
        
        # LED 显示不可用
        self.low_speed_led.set_status(None, '无数据')
        self.safety_led.set_status(None, '无数据')
