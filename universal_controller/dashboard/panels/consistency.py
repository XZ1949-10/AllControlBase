"""
一致性分析面板
"""

from PyQt5.QtWidgets import QGroupBox, QVBoxLayout, QHBoxLayout, QLabel, QGridLayout
from PyQt5.QtCore import Qt
from ..widgets.progress_bar import ColorProgressBar
from ..widgets.status_led import StatusLED
from ..styles import COLORS


class ConsistencyPanel(QGroupBox):
    """一致性分析面板"""
    
    def __init__(self, parent=None):
        super().__init__('一致性分析', parent)
        self._setup_ui()
    
    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(8)
        
        # 融合权重 α_soft
        self._add_metric('融合权重 α_soft', 'alpha', layout)
        
        # 曲率一致性
        self._add_metric('曲率一致性 (κ)', 'kappa', layout)
        
        # 速度方向一致性
        self._add_metric('速度方向一致性', 'velocity', layout)
        
        # 时序平滑性
        self._add_metric('时序平滑性', 'temporal', layout)
        
        layout.addSpacing(10)
        
        # 权重配置
        weight_title = QLabel('权重配置')
        weight_title.setStyleSheet('color: #2196F3; font-weight: bold; border-bottom: 1px solid #3D3D3D; padding-bottom: 3px;')
        layout.addWidget(weight_title)
        
        weight_grid = QGridLayout()
        weight_grid.setSpacing(3)
        
        weights = [
            ('曲率权重 (w1):', '1.0'),
            ('速度方向权重 (w2):', '1.5'),
            ('时序权重 (w3):', '0.8'),
        ]
        
        for i, (label, value) in enumerate(weights):
            weight_grid.addWidget(QLabel(label), i, 0)
            value_label = QLabel(value)
            value_label.setStyleSheet('color: #B0B0B0;')
            weight_grid.addWidget(value_label, i, 1)
        
        layout.addLayout(weight_grid)
        
        layout.addSpacing(10)
        
        # 状态
        status_title = QLabel('状态')
        status_title.setStyleSheet('color: #2196F3; font-weight: bold; border-bottom: 1px solid #3D3D3D; padding-bottom: 3px;')
        layout.addWidget(status_title)
        
        self.soft_led = StatusLED('Soft Head')
        layout.addWidget(self.soft_led)
        
        self.valid_led = StatusLED('数据有效')
        layout.addWidget(self.valid_led)
        
        # 禁用阈值
        thresh_row = QHBoxLayout()
        thresh_row.addWidget(QLabel('禁用阈值:'))
        self.thresh_label = QLabel('0.1')
        self.thresh_label.setStyleSheet('color: #B0B0B0;')
        thresh_row.addWidget(self.thresh_label)
        thresh_row.addStretch()
        layout.addLayout(thresh_row)
        
        # 当前 > 阈值
        compare_row = QHBoxLayout()
        compare_row.addWidget(QLabel('当前 > 阈值:'))
        self.compare_label = QLabel('--')
        compare_row.addWidget(self.compare_label)
        compare_row.addStretch()
        layout.addLayout(compare_row)
        
        layout.addStretch()
    
    def _add_metric(self, label: str, name: str, parent_layout):
        """添加指标"""
        row_layout = QVBoxLayout()
        row_layout.setSpacing(2)
        
        label_widget = QLabel(label)
        row_layout.addWidget(label_widget)
        
        progress = ColorProgressBar(show_percent=False)
        progress.setObjectName(f'{name}_progress')
        row_layout.addWidget(progress)
        
        setattr(self, f'{name}_progress', progress)
        
        parent_layout.addLayout(row_layout)

    def update_display(self, data):
        """使用统一数据模型更新显示"""
        from ..models import DisplayData
        if not isinstance(data, DisplayData):
            return

        # 检查数据可用性
        if not data.availability.consistency_data_available:
            self._show_unavailable()
            return

        c = data.consistency

        self.alpha_progress.set_value(c.alpha_soft, 1.0, 0.1)
        self.kappa_progress.set_value(c.curvature, 1.0)
        self.velocity_progress.set_value(c.velocity_dir, 1.0)
        self.temporal_progress.set_value(c.temporal, 1.0)

        soft_enabled = c.alpha_soft > 0.1
        self.soft_led.set_status(soft_enabled, '✓ 启用' if soft_enabled else '✗ 禁用')
        self.valid_led.set_status(c.data_valid, '✓ 是' if c.data_valid else '✗ 否')

        if c.alpha_soft > 0.1:
            self.compare_label.setText(f'✓ 是 ({c.alpha_soft:.2f} > 0.1)')
            self.compare_label.setStyleSheet(f'color: {COLORS["success"]};')
        else:
            self.compare_label.setText(f'✗ 否 ({c.alpha_soft:.2f} ≤ 0.1)')
            self.compare_label.setStyleSheet(f'color: {COLORS["error"]};')

    def _show_unavailable(self):
        """显示数据不可用状态"""
        unavailable_style = f'color: {COLORS["unavailable"]};'
        
        # 进度条显示为空
        self.alpha_progress.set_value(0, 1.0, 0.1)
        self.kappa_progress.set_value(0, 1.0)
        self.velocity_progress.set_value(0, 1.0)
        self.temporal_progress.set_value(0, 1.0)
        
        # LED 显示不可用
        self.soft_led.set_status(None, '无数据')
        self.valid_led.set_status(None, '无数据')
        
        # 标签显示不可用
        self.compare_label.setText('无数据')
        self.compare_label.setStyleSheet(unavailable_style)
