"""
跟踪误差面板
"""

from PyQt5.QtWidgets import QGroupBox, QVBoxLayout, QHBoxLayout, QLabel
from PyQt5.QtCore import Qt
from ..widgets.progress_bar import ColorProgressBar
from ..styles import COLORS


class TrackingPanel(QGroupBox):
    """跟踪误差面板"""
    
    def __init__(self, parent=None):
        super().__init__('跟踪误差', parent)
        self._setup_ui()
    
    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(8)
        
        # 横向误差
        self.lateral_progress = self._add_error_row(layout, '横向误差 (Lateral)', 0.3, 'm')
        
        # 纵向误差
        self.longitudinal_progress = self._add_error_row(layout, '纵向误差 (Longitudinal)', 0.5, 'm')
        
        # 航向误差
        self.heading_progress = self._add_error_row(layout, '航向误差 (Heading)', 0.5, 'rad')
        
        # 预测误差
        self.prediction_progress = self._add_error_row(layout, '预测误差 (Prediction)', 0.5, 'm')
        
        layout.addSpacing(10)
        
        # 误差趋势
        trend_title = QLabel('误差趋势')
        trend_title.setStyleSheet('color: #2196F3; font-weight: bold; border-bottom: 1px solid #3D3D3D; padding-bottom: 3px;')
        layout.addWidget(trend_title)
        
        self.lateral_trend = self._add_trend_row(layout, '横向:')
        self.longitudinal_trend = self._add_trend_row(layout, '纵向:')
        self.heading_trend = self._add_trend_row(layout, '航向:')
        
        layout.addSpacing(10)
        
        # 跟踪质量评分
        quality_title = QLabel('跟踪质量评分')
        quality_title.setStyleSheet('color: #2196F3; font-weight: bold; border-bottom: 1px solid #3D3D3D; padding-bottom: 3px;')
        layout.addWidget(quality_title)
        
        self.quality_progress = ColorProgressBar(show_percent=True)
        layout.addWidget(self.quality_progress)
        
        rating_row = QHBoxLayout()
        rating_row.addWidget(QLabel('评级:'))
        self.rating_label = QLabel('良好 (Good)')
        self.rating_label.setStyleSheet(f'color: {COLORS["success"]}; font-weight: bold;')
        rating_row.addWidget(self.rating_label)
        rating_row.addStretch()
        layout.addLayout(rating_row)
        
        layout.addStretch()
    
    def _add_error_row(self, parent_layout, label: str, threshold: float, unit: str):
        """添加误差行"""
        row_layout = QVBoxLayout()
        row_layout.setSpacing(2)
        
        label_widget = QLabel(label)
        row_layout.addWidget(label_widget)
        
        progress = ColorProgressBar(show_percent=True)
        progress.set_unit(f' {unit}')
        row_layout.addWidget(progress)
        
        parent_layout.addLayout(row_layout)
        
        return progress
    
    def _add_trend_row(self, parent_layout, label: str):
        """添加趋势行"""
        row = QHBoxLayout()
        row.addWidget(QLabel(label))
        
        trend_label = QLabel('→ 稳定')
        trend_label.setStyleSheet('color: #B0B0B0;')
        row.addWidget(trend_label)
        row.addStretch()
        
        parent_layout.addLayout(row)
        
        return trend_label

    def update_display(self, data):
        """使用统一数据模型更新显示"""
        from ..models import DisplayData
        if not isinstance(data, DisplayData):
            return

        # 检查数据可用性
        if not data.availability.tracking_data_available:
            self._show_unavailable()
            return

        t = data.tracking

        self.lateral_progress.set_value(t.lateral_error, 0.3, 0.3)
        self.longitudinal_progress.set_value(t.longitudinal_error, 0.5, 0.5)
        self.heading_progress.set_value(t.heading_error, 0.5, 0.5)
        self.prediction_progress.set_value(t.prediction_error, 0.5, 0.5)

        lateral_score = max(0, 1 - t.lateral_error / 0.3)
        longitudinal_score = max(0, 1 - t.longitudinal_error / 0.5)
        heading_score = max(0, 1 - t.heading_error / 0.5)
        quality = (lateral_score * 0.4 + longitudinal_score * 0.4 + heading_score * 0.2) * 100
        self.quality_progress.set_value(quality, 100)

        if quality >= 90:
            rating, color = '优秀 (Excellent)', COLORS['success']
        elif quality >= 70:
            rating, color = '良好 (Good)', COLORS['success']
        elif quality >= 50:
            rating, color = '一般 (Fair)', COLORS['warning']
        else:
            rating, color = '较差 (Poor)', COLORS['error']
        self.rating_label.setText(rating)
        self.rating_label.setStyleSheet(f'color: {color}; font-weight: bold;')

    def _show_unavailable(self):
        """显示数据不可用状态"""
        unavailable_style = f'color: {COLORS["unavailable"]};'
        
        # 进度条显示为空
        self.lateral_progress.set_value(0, 0.3, 0.3)
        self.longitudinal_progress.set_value(0, 0.5, 0.5)
        self.heading_progress.set_value(0, 0.5, 0.5)
        self.prediction_progress.set_value(0, 0.5, 0.5)
        self.quality_progress.set_value(0, 100)
        
        # 趋势显示不可用
        self.lateral_trend.setText('无数据')
        self.lateral_trend.setStyleSheet(unavailable_style)
        self.longitudinal_trend.setText('无数据')
        self.longitudinal_trend.setStyleSheet(unavailable_style)
        self.heading_trend.setText('无数据')
        self.heading_trend.setStyleSheet(unavailable_style)
        
        # 评级显示不可用
        self.rating_label.setText('无数据')
        self.rating_label.setStyleSheet(f'color: {COLORS["unavailable"]}; font-weight: bold;')
