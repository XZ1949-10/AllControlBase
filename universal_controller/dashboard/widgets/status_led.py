"""
状态指示灯控件
"""

from PyQt5.QtWidgets import QWidget, QHBoxLayout, QLabel
from PyQt5.QtCore import Qt
from ..styles import COLORS


class StatusLED(QWidget):
    """状态指示灯"""
    
    def __init__(self, text: str = '', parent=None):
        super().__init__(parent)
        self._text = text
        self._status = None
        self._setup_ui()
    
    def _setup_ui(self):
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(5)
        
        # LED 指示灯
        self.led = QLabel()
        self.led.setFixedSize(12, 12)
        layout.addWidget(self.led)
        
        # 文本标签
        self.label = QLabel(self._text)
        layout.addWidget(self.label)
        layout.addStretch()
    
    def set_status(self, status: bool, text: str = None):
        """
        设置状态
        
        Args:
            status: True=成功, False=失败, None=不可用
            text: 显示文本
        """
        self._status = status
        
        if text:
            self.label.setText(text)
        
        if status is None:
            # 数据不可用状态
            color = COLORS.get('unavailable', COLORS['disabled'])
            symbol = '?'
            self.label.setStyleSheet(f'color: {color};')
        elif status:
            color = COLORS['success']
            symbol = '✓'
            self.label.setStyleSheet('color: #FFFFFF;')
        else:
            color = COLORS['error']
            symbol = '✗'
            self.label.setStyleSheet('color: #FFFFFF;')
        
        self.led.setStyleSheet(f"""
            QLabel {{
                background-color: {color};
                border-radius: 6px;
                color: white;
                font-size: 10px;
            }}
        """)
        self.led.setText(symbol)
        self.led.setAlignment(Qt.AlignCenter)
    
    def set_warning(self, text: str = None):
        """设置警告状态"""
        if text:
            self.label.setText(text)
        
        self.led.setStyleSheet(f"""
            QLabel {{
                background-color: {COLORS['warning']};
                border-radius: 6px;
                color: black;
                font-size: 10px;
            }}
        """)
        self.led.setText('!')
        self.led.setAlignment(Qt.AlignCenter)


class StatusIndicator(QWidget):
    """状态指示器 (带值显示)"""
    
    def __init__(self, label: str = '', parent=None):
        super().__init__(parent)
        self._label = label
        self._setup_ui()
    
    def _setup_ui(self):
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(5)
        
        # 标签
        self.name_label = QLabel(self._label)
        self.name_label.setFixedWidth(100)
        layout.addWidget(self.name_label)
        
        # LED
        self.led = QLabel()
        self.led.setFixedSize(14, 14)
        layout.addWidget(self.led)
        
        # 值
        self.value_label = QLabel()
        layout.addWidget(self.value_label)
        layout.addStretch()
    
    def set_value(self, status: bool, value_text: str, detail_text: str = ''):
        """设置值"""
        # 更新 LED
        if status is None:
            color = COLORS['disabled']
        elif status:
            color = COLORS['success']
        else:
            color = COLORS['error']
        
        self.led.setStyleSheet(f"""
            QLabel {{
                background-color: {color};
                border-radius: 7px;
            }}
        """)
        
        # 更新值文本
        text = value_text
        if detail_text:
            text += f"  ({detail_text})"
        self.value_label.setText(text)
