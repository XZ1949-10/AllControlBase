"""
状态栏组件

显示控制器状态、连接状态和紧急停止按钮。
"""
from typing import Optional, Callable

from PyQt5.QtWidgets import (
    QWidget, QHBoxLayout, QLabel, QPushButton, QFrame
)
from PyQt5.QtCore import Qt, pyqtSignal

from ..models import ControllerStatus


class StatusIndicator(QWidget):
    """状态指示器"""
    
    def __init__(self, label: str, parent=None):
        super().__init__(parent)
        
        layout = QHBoxLayout(self)
        layout.setContentsMargins(5, 2, 5, 2)
        layout.setSpacing(5)
        
        self._label = QLabel(label)
        self._label.setStyleSheet("color: #aaaaaa; font-size: 11px;")
        layout.addWidget(self._label)
        
        self._status = QLabel("--")
        self._status.setStyleSheet("color: #888888; font-size: 11px;")
        layout.addWidget(self._status)
    
    def set_status(self, text: str, ok: bool = True):
        """设置状态"""
        self._status.setText(text)
        if ok:
            self._status.setStyleSheet("color: #00ff88; font-size: 11px;")
        else:
            self._status.setStyleSheet("color: #ff6666; font-size: 11px;")


class VisualizerStatusBar(QWidget):
    """
    可视化器状态栏
    
    显示:
    - 控制器状态
    - MPC 状态
    - 数据源状态 (里程计、轨迹)
    - 紧急停止按钮
    """
    
    # 信号
    emergency_stop_clicked = pyqtSignal()
    resume_clicked = pyqtSignal()  # 恢复信号
    
    # 状态名称映射
    STATE_NAMES = {
        0: 'INIT',
        1: 'NORMAL',
        2: 'SOFT_DISABLED',
        3: 'MPC_DEGRADED',
        4: 'BACKUP_ACTIVE',
        5: 'STOPPING',
        6: 'STOPPED',
    }
    
    def __init__(self, parent=None):
        super().__init__(parent)
        
        self._init_ui()
    
    def _init_ui(self):
        layout = QHBoxLayout(self)
        layout.setContentsMargins(10, 5, 10, 5)
        layout.setSpacing(15)
        
        # 控制器状态
        self._controller_status = StatusIndicator("控制器:")
        layout.addWidget(self._controller_status)
        
        # 分隔符
        layout.addWidget(self._create_separator())
        
        # MPC 状态
        self._mpc_status = StatusIndicator("MPC:")
        layout.addWidget(self._mpc_status)
        
        # 分隔符
        layout.addWidget(self._create_separator())
        
        # 里程计状态
        self._odom_status = StatusIndicator("里程计:")
        layout.addWidget(self._odom_status)
        
        # 轨迹状态
        self._traj_status = StatusIndicator("轨迹:")
        layout.addWidget(self._traj_status)
        
        layout.addStretch()
        
        # 恢复按钮 (初始隐藏)
        self._resume_button = QPushButton("恢复控制")
        self._resume_button.setStyleSheet("""
            QPushButton {
                background-color: #007700;
                color: white;
                font-weight: bold;
                font-size: 12px;
                padding: 8px 20px;
                border: none;
                border-radius: 4px;
            }
            QPushButton:hover {
                background-color: #009900;
            }
            QPushButton:pressed {
                background-color: #005500;
            }
        """)
        self._resume_button.clicked.connect(self._on_resume_clicked)
        self._resume_button.hide()  # 初始隐藏
        layout.addWidget(self._resume_button)
        
        # 紧急停止按钮
        self._stop_button = QPushButton("紧急停止")
        self._stop_button.setStyleSheet("""
            QPushButton {
                background-color: #cc0000;
                color: white;
                font-weight: bold;
                font-size: 12px;
                padding: 8px 20px;
                border: none;
                border-radius: 4px;
            }
            QPushButton:hover {
                background-color: #ff0000;
            }
            QPushButton:pressed {
                background-color: #990000;
            }
        """)
        self._stop_button.clicked.connect(self._on_stop_clicked)
        layout.addWidget(self._stop_button)
        
        # 设置背景
        self.setStyleSheet("""
            VisualizerStatusBar {
                background-color: #1a1a1a;
                border-top: 1px solid #444444;
            }
        """)
        self.setFixedHeight(45)
    
    def _create_separator(self) -> QFrame:
        """创建分隔符"""
        sep = QFrame()
        sep.setFrameShape(QFrame.VLine)
        sep.setStyleSheet("background-color: #444444;")
        return sep
    
    def _on_stop_clicked(self):
        """紧急停止按钮点击"""
        self.emergency_stop_clicked.emit()
    
    def _on_resume_clicked(self):
        """恢复按钮点击"""
        self.resume_clicked.emit()
    
    def update_status(self, status: ControllerStatus):
        """更新控制器状态"""
        # 控制器状态
        state_name = self.STATE_NAMES.get(status.state, f'UNKNOWN({status.state})')
        is_ok = status.state in (1, 2)  # NORMAL 或 SOFT_DISABLED
        self._controller_status.set_status(state_name, is_ok)
        
        # MPC 状态
        if status.mpc_success:
            mpc_text = f"✓ 成功 ({status.mpc_solve_time_ms:.1f}ms)"
            self._mpc_status.set_status(mpc_text, True)
        elif status.backup_active:
            self._mpc_status.set_status("备份控制", False)
        else:
            self._mpc_status.set_status("✗ 失败", False)
    
    def update_connection(self, odom_ok: bool, traj_ok: bool):
        """更新连接状态"""
        self._odom_status.set_status("✓" if odom_ok else "✗", odom_ok)
        self._traj_status.set_status("✓" if traj_ok else "✗", traj_ok)
    
    def set_emergency_stop_state(self, stopped: bool):
        """设置紧急停止状态"""
        if stopped:
            self._stop_button.setText("已停止")
            self._stop_button.setStyleSheet("""
                QPushButton {
                    background-color: #666666;
                    color: white;
                    font-weight: bold;
                    font-size: 12px;
                    padding: 8px 20px;
                    border: none;
                    border-radius: 4px;
                }
            """)
            self._stop_button.setEnabled(False)
            self._resume_button.show()  # 显示恢复按钮
        else:
            self._stop_button.setText("紧急停止")
            self._stop_button.setStyleSheet("""
                QPushButton {
                    background-color: #cc0000;
                    color: white;
                    font-weight: bold;
                    font-size: 12px;
                    padding: 8px 20px;
                    border: none;
                    border-radius: 4px;
                }
                QPushButton:hover {
                    background-color: #ff0000;
                }
                QPushButton:pressed {
                    background-color: #990000;
                }
            """)
            self._stop_button.setEnabled(True)
            self._resume_button.hide()  # 隐藏恢复按钮
