"""
可视化器主窗口

组装所有 UI 组件，提供统一的界面。
"""
from typing import Optional, Dict, Any

from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QSplitter, QFrame, QLabel
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QFont

from .widgets import (
    TrajectoryView, VelocityPanel, VelocityPlot,
    JoystickPanel, VisualizerStatusBar
)
from .models import VisualizerData, ControlMode
from .handlers import DataAggregator


class VisualizerMainWindow(QMainWindow):
    """
    可视化器主窗口
    
    布局:
    ┌─────────────────────────────────────────────────────────────┐
    │  标题栏                                          [连接状态]  │
    ├─────────────────────────────────┬───────────────────────────┤
    │                                 │                           │
    │      轨迹可视化                  │      速度监控面板          │
    │      (TrajectoryView)           │      (VelocityPanel)      │
    │                                 │                           │
    │                                 ├───────────────────────────┤
    │                                 │                           │
    │                                 │      速度历史曲线          │
    │                                 │      (VelocityPlot)       │
    │                                 │                           │
    ├─────────────────────────────────┴───────────────────────────┤
    │                                                             │
    │                    手柄控制面板 (JoystickPanel)              │
    │                                                             │
    ├─────────────────────────────────────────────────────────────┤
    │  状态栏 (VisualizerStatusBar)                    [紧急停止]  │
    └─────────────────────────────────────────────────────────────┘
    """
    
    # 信号
    emergency_stop_requested = pyqtSignal()
    resume_requested = pyqtSignal()  # 恢复控制信号
    
    def __init__(self, data_aggregator: DataAggregator, config: Dict[str, Any] = None):
        super().__init__()
        
        self._data_aggregator = data_aggregator
        self._config = config or {}
        
        # 配置
        display_config = self._config.get('display', {})
        self._update_rate = display_config.get('update_rate', 30)
        self._velocity_history_sec = display_config.get('velocity_history_sec', 10)
        
        constraints = self._config.get('constraints', {})
        self._max_linear = constraints.get('v_max', 0.5)
        self._max_angular = constraints.get('omega_max', 1.0)
        
        self._init_ui()
        self._init_timer()
    
    def _init_ui(self):
        """初始化 UI"""
        self.setWindowTitle("TurtleBot1 运行可视化")
        self.setMinimumSize(1000, 700)
        
        # 设置深色主题
        self.setStyleSheet("""
            QMainWindow {
                background-color: #1e1e1e;
            }
            QWidget {
                color: #cccccc;
                font-family: "Microsoft YaHei", "SimHei", sans-serif;
            }
            QSplitter::handle {
                background-color: #444444;
            }
        """)
        
        # 中央组件
        central = QWidget()
        self.setCentralWidget(central)
        
        main_layout = QVBoxLayout(central)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)
        
        # 标题栏
        title_bar = self._create_title_bar()
        main_layout.addWidget(title_bar)
        
        # 主内容区域
        content = QWidget()
        content_layout = QVBoxLayout(content)
        content_layout.setContentsMargins(10, 10, 10, 10)
        content_layout.setSpacing(10)
        
        # 上半部分: 轨迹 + 速度
        top_splitter = QSplitter(Qt.Horizontal)
        
        # 轨迹可视化 (传递 display 配置)
        display_config = self._config.get('display', {})
        self._trajectory_view = TrajectoryView(display_config=display_config)
        top_splitter.addWidget(self._trajectory_view)
        
        # 右侧面板
        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.setSpacing(10)
        
        # 速度监控
        self._velocity_panel = VelocityPanel(
            max_linear=self._max_linear,
            max_angular=self._max_angular
        )
        right_layout.addWidget(self._velocity_panel)
        
        # 速度曲线
        self._velocity_plot = VelocityPlot(history_sec=self._velocity_history_sec)
        right_layout.addWidget(self._velocity_plot)
        
        top_splitter.addWidget(right_panel)
        top_splitter.setSizes([600, 400])
        
        content_layout.addWidget(top_splitter, stretch=2)
        
        # 下半部分: 手柄控制
        self._joystick_panel = JoystickPanel()
        content_layout.addWidget(self._joystick_panel, stretch=1)
        
        main_layout.addWidget(content, stretch=1)
        
        # 状态栏
        self._status_bar = VisualizerStatusBar()
        self._status_bar.emergency_stop_clicked.connect(self._on_emergency_stop)
        self._status_bar.resume_clicked.connect(self._on_resume)
        main_layout.addWidget(self._status_bar)
    
    def _create_title_bar(self) -> QWidget:
        """创建标题栏"""
        title_bar = QFrame()
        title_bar.setStyleSheet("""
            QFrame {
                background-color: #252526;
                border-bottom: 1px solid #444444;
            }
        """)
        title_bar.setFixedHeight(40)
        
        layout = QHBoxLayout(title_bar)
        layout.setContentsMargins(15, 0, 15, 0)
        
        # 标题
        title = QLabel("TurtleBot1 运行可视化")
        title.setStyleSheet("color: #ffffff; font-size: 14px; font-weight: bold;")
        layout.addWidget(title)
        
        layout.addStretch()
        
        # 连接状态
        self._connection_label = QLabel("● 已连接")
        self._connection_label.setStyleSheet("color: #00ff88; font-size: 12px;")
        layout.addWidget(self._connection_label)
        
        return title_bar
    
    def _init_timer(self):
        """初始化更新定时器"""
        self._update_timer = QTimer(self)
        self._update_timer.timeout.connect(self._update_display)
        self._update_timer.start(int(1000 / self._update_rate))
    
    def _update_display(self):
        """更新显示"""
        # 获取聚合数据
        data = self._data_aggregator.get_data()
        
        # 更新轨迹视图
        self._trajectory_view.set_trajectory(data.trajectory)
        self._trajectory_view.set_robot_pose(data.robot_pose)
        if data.camera_image is not None:
            self._trajectory_view.set_camera_image(data.camera_image)
        
        # 更新速度面板
        self._velocity_panel.update_velocity(
            data.actual_velocity,
            data.target_velocity
        )
        
        # 更新速度曲线
        history = self._data_aggregator.get_velocity_history(self._velocity_history_sec)
        self._velocity_plot.update_data(history)
        
        # 更新手柄面板
        self._joystick_panel.update_joystick(data.joystick)
        self._joystick_panel.update_control_mode(data.control_mode)
        
        # 更新状态栏 (包含紧急停止/恢复按钮状态更新)
        self._status_bar.update_status(data.controller_status)
        
        # 更新连接状态
        conn_status = self._data_aggregator.get_connection_status()
        self._status_bar.update_connection(
            conn_status.get('odom', False),
            conn_status.get('trajectory', False)
        )
        
        # 更新标题栏连接状态
        if data.ros_connected:
            self._connection_label.setText("● 已连接")
            self._connection_label.setStyleSheet("color: #00ff88; font-size: 12px;")
        else:
            self._connection_label.setText("○ 未连接")
            self._connection_label.setStyleSheet("color: #ff6666; font-size: 12px;")
    
    def _on_emergency_stop(self):
        """紧急停止按钮点击"""
        self.emergency_stop_requested.emit()
    
    def _on_resume(self):
        """恢复按钮点击"""
        self.resume_requested.emit()
    
    def set_emergency_stop_state(self, stopped: bool):
        """设置紧急停止状态"""
        self._status_bar.set_emergency_stop_state(stopped)
    
    def load_homography_calibration(self, calib_file: str) -> bool:
        """
        加载单应性标定文件
        
        Args:
            calib_file: YAML 标定文件路径
            
        Returns:
            是否加载成功
        """
        return self._trajectory_view.load_homography_calibration(calib_file)
    
    def closeEvent(self, event):
        """窗口关闭事件"""
        self._update_timer.stop()
        event.accept()
