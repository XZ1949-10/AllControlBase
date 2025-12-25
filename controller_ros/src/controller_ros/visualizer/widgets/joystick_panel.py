"""
手柄控制面板

显示手柄状态和控制模式。
"""
from typing import Optional

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, 
    QFrame, QGridLayout, QRadioButton, QButtonGroup
)
from PyQt5.QtCore import Qt, QPointF
from PyQt5.QtGui import QPainter, QPen, QBrush, QColor, QFont

from ..models import JoystickState, ControlMode


class JoystickIndicator(QWidget):
    """摇杆指示器"""
    
    def __init__(self, title: str, subtitle: str = "", parent=None):
        super().__init__(parent)
        
        self._title = title
        self._subtitle = subtitle
        self._x = 0.0
        self._y = 0.0
        self._value_label = ""
        
        self.setMinimumSize(120, 140)
        self.setMaximumSize(180, 180)
    
    def set_position(self, x: float, y: float, label: str = ""):
        """设置摇杆位置 [-1, 1]"""
        self._x = max(-1, min(1, x))
        self._y = max(-1, min(1, y))
        self._value_label = label
        self.update()
    
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        w, h = self.width(), self.height()
        
        # 标题
        painter.setPen(QColor(200, 200, 200))
        font = QFont()
        font.setPointSize(10)
        font.setBold(True)
        painter.setFont(font)
        painter.drawText(0, 0, w, 20, Qt.AlignCenter, self._title)
        
        # 副标题
        if self._subtitle:
            painter.setPen(QColor(150, 150, 150))
            font.setPointSize(8)
            font.setBold(False)
            painter.setFont(font)
            painter.drawText(0, 18, w, 16, Qt.AlignCenter, self._subtitle)
        
        # 摇杆区域
        margin = 10
        title_height = 38 if self._subtitle else 24
        area_size = min(w - 2 * margin, h - title_height - 25)
        area_x = (w - area_size) // 2
        area_y = title_height
        
        # 背景圆
        painter.setPen(QPen(QColor(80, 80, 80), 2))
        painter.setBrush(QBrush(QColor(40, 40, 40)))
        painter.drawEllipse(area_x, area_y, area_size, area_size)
        
        # 十字线
        painter.setPen(QPen(QColor(60, 60, 60), 1))
        cx = area_x + area_size // 2
        cy = area_y + area_size // 2
        painter.drawLine(cx, area_y, cx, area_y + area_size)
        painter.drawLine(area_x, cy, area_x + area_size, cy)
        
        # 摇杆位置
        stick_radius = 12
        stick_x = cx + int(self._x * (area_size // 2 - stick_radius))
        stick_y = cy - int(self._y * (area_size // 2 - stick_radius))  # Y 轴取反
        
        # 摇杆
        painter.setPen(Qt.NoPen)
        painter.setBrush(QBrush(QColor(0, 200, 100)))
        painter.drawEllipse(
            stick_x - stick_radius, 
            stick_y - stick_radius,
            stick_radius * 2, 
            stick_radius * 2
        )
        
        # 数值标签
        if self._value_label:
            painter.setPen(QColor(150, 150, 150))
            font.setPointSize(9)
            painter.setFont(font)
            painter.drawText(0, h - 18, w, 18, Qt.AlignCenter, self._value_label)
        
        painter.end()


class JoystickPanel(QWidget):
    """
    手柄控制面板
    
    显示:
    - 控制模式 (网络/手柄)
    - 左右摇杆状态
    - 使能键状态
    - 手柄连接状态
    """
    
    def __init__(self, parent=None):
        super().__init__(parent)
        
        self._joystick_state: Optional[JoystickState] = None
        self._control_mode = ControlMode.NETWORK
        
        self._init_ui()
    
    def _init_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(10)
        
        # 标题
        title = QLabel("Xbox 手柄控制")
        title.setStyleSheet("color: #ffffff; font-size: 13px; font-weight: bold;")
        layout.addWidget(title)
        
        # 分隔线
        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setStyleSheet("background-color: #555555;")
        layout.addWidget(line)
        
        # 使能键提示
        self._enable_hint = QLabel(
            "[LB 使能键]\n"
            "○ 未按下 → 网络轨迹控制\n"
            "● 按住   → 手柄控制"
        )
        self._enable_hint.setStyleSheet("""
            color: #aaaaaa;
            font-size: 11px;
            padding: 5px;
            background-color: #333333;
            border-radius: 3px;
        """)
        layout.addWidget(self._enable_hint)
        
        # 摇杆显示区域
        sticks_layout = QHBoxLayout()
        sticks_layout.setSpacing(30)
        
        self._left_stick = JoystickIndicator("左摇杆", "前进/后退")
        sticks_layout.addWidget(self._left_stick)
        
        self._right_stick = JoystickIndicator("右摇杆", "转向")
        sticks_layout.addWidget(self._right_stick)
        
        layout.addLayout(sticks_layout)
        
        # 状态显示
        status_layout = QHBoxLayout()
        
        # 控制模式指示
        mode_frame = QFrame()
        mode_frame.setStyleSheet("""
            background-color: #333333;
            border-radius: 3px;
            padding: 5px;
        """)
        mode_layout = QHBoxLayout(mode_frame)
        mode_layout.setContentsMargins(10, 5, 10, 5)
        
        mode_label = QLabel("控制模式:")
        mode_label.setStyleSheet("color: #aaaaaa; font-size: 11px;")
        mode_layout.addWidget(mode_label)
        
        self._network_indicator = QLabel("● 网络轨迹")
        self._network_indicator.setStyleSheet("color: #00ff88; font-size: 11px;")
        mode_layout.addWidget(self._network_indicator)
        
        self._joystick_indicator = QLabel("○ 手柄控制")
        self._joystick_indicator.setStyleSheet("color: #666666; font-size: 11px;")
        mode_layout.addWidget(self._joystick_indicator)
        
        status_layout.addWidget(mode_frame)
        
        # 手柄连接状态
        self._connection_label = QLabel("手柄: 未连接")
        self._connection_label.setStyleSheet("""
            color: #ff6666;
            font-size: 11px;
            padding: 5px 10px;
            background-color: #333333;
            border-radius: 3px;
        """)
        status_layout.addWidget(self._connection_label)
        
        layout.addLayout(status_layout)
        
        # 设置背景
        self.setStyleSheet("""
            JoystickPanel {
                background-color: #2a2a2a;
                border: 1px solid #444444;
                border-radius: 5px;
            }
        """)
    
    def update_joystick(self, state: JoystickState):
        """更新手柄状态"""
        self._joystick_state = state
        
        # 更新摇杆指示器
        self._left_stick.set_position(
            state.left_x, state.left_y,
            f"vx: {state.left_y:.2f}"
        )
        self._right_stick.set_position(
            state.right_x, state.right_y,
            f"ω: {-state.right_x:.2f}"
        )
        
        # 更新连接状态
        if state.connected:
            self._connection_label.setText("手柄: 已连接 ✓")
            self._connection_label.setStyleSheet("""
                color: #00ff88;
                font-size: 11px;
                padding: 5px 10px;
                background-color: #333333;
                border-radius: 3px;
            """)
        else:
            self._connection_label.setText("手柄: 未连接")
            self._connection_label.setStyleSheet("""
                color: #ff6666;
                font-size: 11px;
                padding: 5px 10px;
                background-color: #333333;
                border-radius: 3px;
            """)
    
    def update_control_mode(self, mode: ControlMode):
        """更新控制模式"""
        self._control_mode = mode
        
        if mode == ControlMode.NETWORK:
            self._network_indicator.setStyleSheet("color: #00ff88; font-size: 11px;")
            self._network_indicator.setText("● 网络轨迹")
            self._joystick_indicator.setStyleSheet("color: #666666; font-size: 11px;")
            self._joystick_indicator.setText("○ 手柄控制")
        else:
            self._network_indicator.setStyleSheet("color: #666666; font-size: 11px;")
            self._network_indicator.setText("○ 网络轨迹")
            self._joystick_indicator.setStyleSheet("color: #ffaa00; font-size: 11px;")
            self._joystick_indicator.setText("● 手柄控制")
