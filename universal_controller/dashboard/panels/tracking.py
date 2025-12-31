"""
跟踪误差面板

配置来源: universal_controller/config/system_config.py -> TRACKING_CONFIG
YAML 覆盖: controller_ros/config/platforms/turtlebot1.yaml -> tracking 节

显示：
- 横向/纵向/航向/预测误差
- 误差趋势
- 跟踪质量评分
"""

from PyQt5.QtWidgets import QGroupBox, QVBoxLayout, QHBoxLayout, QLabel
from PyQt5.QtCore import Qt
from ..widgets.progress_bar import ColorProgressBar
from ..styles import COLORS, PANEL_TITLE_STYLE, get_quality_color

# 从统一配置模块导入默认值
from ...config import TRACKING_CONFIG


class TrackingPanel(QGroupBox):
    """跟踪误差面板"""
    
    def __init__(self, parent=None, config=None):
        super().__init__('跟踪误差', parent)
        self._config = {}
        self._load_config(config)
        self._setup_ui()
    
    def _load_config(self, config=None):
        """从配置加载参数，使用 TRACKING_CONFIG 作为默认值"""
        tracking_config = (config or {}).get('tracking', {})
        
        # 阈值
        self._config['lateral_thresh'] = tracking_config.get('lateral_thresh', TRACKING_CONFIG['lateral_thresh'])
        self._config['longitudinal_thresh'] = tracking_config.get('longitudinal_thresh', TRACKING_CONFIG['longitudinal_thresh'])
        self._config['heading_thresh'] = tracking_config.get('heading_thresh', TRACKING_CONFIG['heading_thresh'])
        self._config['prediction_thresh'] = tracking_config.get('prediction_thresh', TRACKING_CONFIG['prediction_thresh'])
        
        # 权重
        weights = tracking_config.get('weights', {})
        self._config['weights'] = {
            'lateral': weights.get('lateral', TRACKING_CONFIG['weights']['lateral']),
            'longitudinal': weights.get('longitudinal', TRACKING_CONFIG['weights']['longitudinal']),
            'heading': weights.get('heading', TRACKING_CONFIG['weights']['heading']),
        }
        
        # 评级阈值
        rating = tracking_config.get('rating', {})
        self._config['rating'] = {
            'excellent': rating.get('excellent', TRACKING_CONFIG['rating']['excellent']),
            'good': rating.get('good', TRACKING_CONFIG['rating']['good']),
            'fair': rating.get('fair', TRACKING_CONFIG['rating']['fair']),
        }
    
    def set_config(self, config: dict):
        """设置跟踪配置"""
        self._load_config(config)
    
    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(8)
        
        # 横向误差
        self.lateral_progress = self._add_error_row(layout, '横向误差 (Lateral)', 'm')
        
        # 纵向误差
        self.longitudinal_progress = self._add_error_row(layout, '纵向误差 (Longitudinal)', 'm')
        
        # 航向误差
        self.heading_progress = self._add_error_row(layout, '航向误差 (Heading)', 'rad')
        
        # 预测误差
        self.prediction_progress = self._add_error_row(layout, '预测误差 (Prediction)', 'm')
        
        layout.addSpacing(10)
        
        # 误差趋势
        trend_title = QLabel('误差趋势')
        trend_title.setStyleSheet(PANEL_TITLE_STYLE)
        layout.addWidget(trend_title)
        
        self.lateral_trend = self._add_trend_row(layout, '横向:')
        self.longitudinal_trend = self._add_trend_row(layout, '纵向:')
        self.heading_trend = self._add_trend_row(layout, '航向:')
        
        layout.addSpacing(10)
        
        # 跟踪质量评分
        quality_title = QLabel('跟踪质量评分')
        quality_title.setStyleSheet(PANEL_TITLE_STYLE)
        layout.addWidget(quality_title)
        
        self.quality_progress = ColorProgressBar(show_percent=True)
        self.quality_progress.set_inverted(True)  # 质量评分越高越好
        layout.addWidget(self.quality_progress)
        
        rating_row = QHBoxLayout()
        rating_row.addWidget(QLabel('评级:'))
        self.rating_label = QLabel('良好 (Good)')
        self.rating_label.setStyleSheet(f'color: {COLORS["success"]}; font-weight: bold;')
        rating_row.addWidget(self.rating_label)
        rating_row.addStretch()
        layout.addLayout(rating_row)
        
        layout.addStretch()
    
    def _add_error_row(self, parent_layout, label: str, unit: str):
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
        
        # 获取配置阈值
        lat_thresh = self._config['lateral_thresh']
        lon_thresh = self._config['longitudinal_thresh']
        head_thresh = self._config['heading_thresh']
        pred_thresh = self._config['prediction_thresh']
        
        # 取绝对值 - 修复负值导致评分异常的 BUG
        # 误差的正负只表示方向（左/右、前/后），评分应该只关心误差大小
        lat_error = abs(t.lateral_error)
        lon_error = abs(t.longitudinal_error)
        head_error = abs(t.heading_error)
        pred_error = abs(t.prediction_error)
        
        # 更新进度条显示
        self.lateral_progress.set_value(lat_error, lat_thresh, lat_thresh)
        self.longitudinal_progress.set_value(lon_error, lon_thresh, lon_thresh)
        self.heading_progress.set_value(head_error, head_thresh, head_thresh)
        self.prediction_progress.set_value(pred_error, pred_thresh, pred_thresh)
        
        # 计算各维度评分 (0-1)
        # 误差为 0 时评分为 1，误差达到阈值时评分为 0
        lateral_score = max(0.0, 1.0 - lat_error / lat_thresh)
        longitudinal_score = max(0.0, 1.0 - lon_error / lon_thresh)
        heading_score = max(0.0, 1.0 - head_error / head_thresh)
        
        # 加权计算总评分
        weights = self._config['weights']
        quality = (
            lateral_score * weights['lateral'] +
            longitudinal_score * weights['longitudinal'] +
            heading_score * weights['heading']
        ) * 100
        
        self.quality_progress.set_value(quality, 100)
        
        # 根据评级阈值确定评级
        rating_config = self._config['rating']
        if quality >= rating_config['excellent']:
            rating = '优秀 (Excellent)'
            color = get_quality_color(quality)
        elif quality >= rating_config['good']:
            rating = '良好 (Good)'
            color = get_quality_color(quality)
        elif quality >= rating_config['fair']:
            rating = '一般 (Fair)'
            color = get_quality_color(quality)
        else:
            rating = '较差 (Poor)'
            color = get_quality_color(quality)
        
        self.rating_label.setText(rating)
        self.rating_label.setStyleSheet(f'color: {color}; font-weight: bold;')

    def _show_unavailable(self):
        """显示数据不可用状态"""
        unavailable_style = f'color: {COLORS["unavailable"]};'
        
        # 获取配置阈值
        lat_thresh = self._config['lateral_thresh']
        lon_thresh = self._config['longitudinal_thresh']
        head_thresh = self._config['heading_thresh']
        pred_thresh = self._config['prediction_thresh']
        
        # 进度条显示为空
        self.lateral_progress.set_value(0, lat_thresh, lat_thresh)
        self.longitudinal_progress.set_value(0, lon_thresh, lon_thresh)
        self.heading_progress.set_value(0, head_thresh, head_thresh)
        self.prediction_progress.set_value(0, pred_thresh, pred_thresh)
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
