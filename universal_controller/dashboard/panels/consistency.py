"""
一致性分析面板

配置来源: universal_controller/config/modules_config.py -> CONSISTENCY_CONFIG
YAML 覆盖: controller_ros/config/platforms/turtlebot1.yaml -> consistency 节

显示：
- 融合权重 α_soft
- 曲率/速度方向/时序一致性
- Soft Head 状态
"""

from PyQt5.QtWidgets import QGroupBox, QVBoxLayout, QHBoxLayout, QLabel, QGridLayout
from PyQt5.QtCore import Qt
from ..widgets.progress_bar import ColorProgressBar
from ..widgets.status_led import StatusLED
from ..styles import COLORS, PANEL_TITLE_STYLE

# 从统一配置模块导入默认值
from ...config import CONSISTENCY_CONFIG


class ConsistencyPanel(QGroupBox):
    """一致性分析面板"""
    
    def __init__(self, parent=None, config=None):
        super().__init__('一致性分析', parent)
        self._config = config or {}
        self._load_config()
        self._setup_ui()
    
    def _load_config(self):
        """从配置加载参数，使用 CONSISTENCY_CONFIG 作为默认值"""
        consistency = self._config.get('consistency', {})
        weights = consistency.get('weights', {})
        default_weights = CONSISTENCY_CONFIG.get('weights', {})
        
        # 注意: 这里的权重用于 Dashboard 显示
        # 实际一致性计算使用 WeightedConsistencyAnalyzer 中的指数权重
        self._w_kappa = weights.get('kappa', default_weights.get('kappa', 1.0))
        self._w_velocity = weights.get('velocity', default_weights.get('velocity', 1.5))
        self._w_temporal = weights.get('temporal', default_weights.get('temporal', 0.8))
        self._alpha_min = consistency.get('alpha_min', CONSISTENCY_CONFIG.get('alpha_min', 0.1))
    
    def set_config(self, config):
        """更新配置"""
        self._config = config or {}
        self._load_config()
        self._update_config_labels()
    
    def _update_config_labels(self):
        """更新配置标签显示"""
        if hasattr(self, '_weight_labels'):
            self._weight_labels['kappa'].setText(str(self._w_kappa))
            self._weight_labels['velocity'].setText(str(self._w_velocity))
            self._weight_labels['temporal'].setText(str(self._w_temporal))
        if hasattr(self, 'thresh_label'):
            self.thresh_label.setText(str(self._alpha_min))
    
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
        weight_title.setStyleSheet(PANEL_TITLE_STYLE)
        layout.addWidget(weight_title)
        
        weight_grid = QGridLayout()
        weight_grid.setSpacing(3)
        
        self._weight_labels = {}
        weights = [
            ('kappa', '曲率权重 (w1):', self._w_kappa),
            ('velocity', '速度方向权重 (w2):', self._w_velocity),
            ('temporal', '时序权重 (w3):', self._w_temporal),
        ]
        
        for i, (key, label, value) in enumerate(weights):
            weight_grid.addWidget(QLabel(label), i, 0)
            value_label = QLabel(str(value))
            value_label.setStyleSheet('color: #B0B0B0;')
            weight_grid.addWidget(value_label, i, 1)
            self._weight_labels[key] = value_label
        
        layout.addLayout(weight_grid)
        
        layout.addSpacing(10)
        
        # 状态
        status_title = QLabel('状态')
        status_title.setStyleSheet(PANEL_TITLE_STYLE)
        layout.addWidget(status_title)
        
        self.soft_led = StatusLED('Soft Head')
        layout.addWidget(self.soft_led)
        
        self.valid_led = StatusLED('数据有效')
        layout.addWidget(self.valid_led)
        
        # 禁用阈值
        thresh_row = QHBoxLayout()
        thresh_row.addWidget(QLabel('禁用阈值:'))
        self.thresh_label = QLabel(str(self._alpha_min))
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

        # 检查是否因为 soft_enabled=False 导致一致性检查被跳过
        # 当 alpha=0 且所有指标都是 1.0 时，说明轨迹没有速度信息
        soft_skipped = (c.alpha_soft == 0 and 
                        c.curvature == 1.0 and 
                        c.velocity_dir == 1.0 and 
                        c.temporal == 1.0)
        
        if soft_skipped:
            self._show_soft_disabled()
            return

        self.alpha_progress.set_value(c.alpha_soft, 1.0, 0.1)
        self.kappa_progress.set_value(c.curvature, 1.0)
        self.velocity_progress.set_value(c.velocity_dir, 1.0)
        self.temporal_progress.set_value(c.temporal, 1.0)

        soft_enabled = c.alpha_soft > 0.1
        self.soft_led.set_status(soft_enabled, '✓ 启用' if soft_enabled else '✗ 禁用')
        self.valid_led.set_status(c.data_valid, '✓ 是' if c.data_valid else '✗ 否')

        # 使用配置的阈值
        alpha_min = self._alpha_min
        if c.alpha_soft > alpha_min:
            self.compare_label.setText(f'✓ 是 ({c.alpha_soft:.2f} > {alpha_min})')
            self.compare_label.setStyleSheet(f'color: {COLORS["success"]};')
        else:
            self.compare_label.setText(f'✗ 否 ({c.alpha_soft:.2f} ≤ {alpha_min})')
            self.compare_label.setStyleSheet(f'color: {COLORS["error"]};')

    def _show_soft_disabled(self):
        """显示 Soft 轨迹未启用状态 (轨迹无速度信息)"""
        disabled_style = f'color: {COLORS["disabled"]};'
        
        # 进度条显示为灰色空条
        self.alpha_progress.set_value(0, 1.0, 0.1)
        self.kappa_progress.set_value(0, 1.0)
        self.velocity_progress.set_value(0, 1.0)
        self.temporal_progress.set_value(0, 1.0)
        
        # LED 显示禁用状态
        self.soft_led.set_status(False, '未启用 (轨迹无速度)')
        self.valid_led.set_status(True, '✓ 是 (Hard轨迹)')
        
        # 提示信息
        self.compare_label.setText('N/A (仅Hard轨迹)')
        self.compare_label.setStyleSheet(disabled_style)

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
