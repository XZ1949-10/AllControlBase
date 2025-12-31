"""
自定义进度条控件 - 支持动态颜色和多种显示模式
"""

import math

from PyQt5.QtWidgets import QProgressBar, QWidget, QHBoxLayout, QLabel
from PyQt5.QtCore import Qt
from ..styles import get_progress_color, COLORS


class ColorProgressBar(QWidget):
    """带颜色变化的进度条
    
    支持：
    - 动态颜色（根据值自动变色）
    - 阈值显示
    - 单位显示
    - 百分比显示
    """
    
    def __init__(self, parent=None, show_text=True, show_percent=True):
        super().__init__(parent)
        self.show_text = show_text
        self.show_percent = show_percent
        self._value = 0
        self._max_value = 100
        self._threshold = None
        self._unit = ''
        self._inverted = False  # 是否反转颜色（值越高越好）
        
        self._setup_ui()
    
    def _setup_ui(self):
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(5)
        
        # 进度条
        self.progress = QProgressBar()
        self.progress.setTextVisible(False)
        self.progress.setFixedHeight(18)
        layout.addWidget(self.progress, 1)
        
        # 数值标签
        if self.show_text:
            self.value_label = QLabel()
            self.value_label.setMinimumWidth(140)
            self.value_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            layout.addWidget(self.value_label)
    
    def set_inverted(self, inverted: bool):
        """设置是否反转颜色（用于"值越高越好"的指标）"""
        self._inverted = inverted
    
    def set_value(self, value: float, max_value: float = None, threshold: float = None):
        """设置值"""
        # 处理 NaN 和 Inf 值
        if math.isnan(value) or math.isinf(value):
            value = 0.0
        
        self._value = value
        if max_value is not None:
            self._max_value = max_value
        if threshold is not None:
            self._threshold = threshold
        
        # 计算百分比
        if self._max_value > 0:
            ratio = min(value / self._max_value, 1.0)
        else:
            ratio = 0
        
        # 确保 ratio 是有效数值
        if math.isnan(ratio) or math.isinf(ratio):
            ratio = 0.0
        
        # 更新进度条
        self.progress.setMaximum(1000)
        self.progress.setValue(int(ratio * 1000))
        
        # 更新颜色
        color = get_progress_color(ratio, self._inverted)
        
        # 添加渐变效果
        self.progress.setStyleSheet(f"""
            QProgressBar {{
                border: 1px solid #3D3D3D;
                border-radius: 4px;
                background-color: #1E1E1E;
            }}
            QProgressBar::chunk {{
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 {color}, stop:1 {self._lighten_color(color)});
                border-radius: 3px;
            }}
        """)
        
        # 更新文本
        if self.show_text:
            if self._threshold:
                text = f"{value:.2f}{self._unit} / {self._threshold:.2f}"
            else:
                text = f"{value:.2f}{self._unit}"
            
            if self.show_percent:
                text += f" ({ratio*100:.1f}%)"
            
            self.value_label.setText(text)
    
    def _lighten_color(self, color: str) -> str:
        """将颜色变亮一点（用于渐变效果）"""
        # 简单的颜色变亮处理
        color_map = {
            COLORS['success']: COLORS.get('success_light', '#81C784'),
            COLORS['warning']: COLORS.get('warning_light', '#FFD54F'),
            COLORS['error']: COLORS.get('error_light', '#E57373'),
        }
        return color_map.get(color, color)
    
    def set_unit(self, unit: str):
        """设置单位"""
        self._unit = unit


class SimpleProgressBar(QProgressBar):
    """简单的彩色进度条"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setTextVisible(True)
        self.setFixedHeight(20)
        self._inverted = False
    
    def set_inverted(self, inverted: bool):
        """设置是否反转颜色"""
        self._inverted = inverted
    
    def set_value_with_color(self, value: float, max_value: float = 100):
        """设置值并更新颜色"""
        # 处理 NaN 和 Inf 值
        if math.isnan(value) or math.isinf(value):
            value = 0.0
        
        ratio = value / max_value if max_value > 0 else 0
        ratio = min(ratio, 1.0)
        
        # 确保 ratio 是有效数值
        if math.isnan(ratio) or math.isinf(ratio):
            ratio = 0.0
        
        self.setMaximum(int(max_value * 10))
        self.setValue(int(value * 10))
        
        color = get_progress_color(ratio, self._inverted)
        light_color = self._lighten_color(color)
        
        self.setStyleSheet(f"""
            QProgressBar {{
                border: 1px solid #3D3D3D;
                border-radius: 4px;
                background-color: #1E1E1E;
                text-align: center;
                color: white;
                font-weight: 500;
            }}
            QProgressBar::chunk {{
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 {color}, stop:1 {light_color});
                border-radius: 3px;
            }}
        """)
        self.setFormat(f"{value:.1f} / {max_value:.1f}")
    
    def _lighten_color(self, color: str) -> str:
        """将颜色变亮"""
        color_map = {
            COLORS['success']: COLORS.get('success_light', '#81C784'),
            COLORS['warning']: COLORS.get('warning_light', '#FFD54F'),
            COLORS['error']: COLORS.get('error_light', '#E57373'),
        }
        return color_map.get(color, color)
