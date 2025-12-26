"""
速度历史曲线组件

显示速度随时间变化的曲线图。
"""
from typing import Dict, List, Optional

from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel, QFrame
from PyQt5.QtCore import Qt

# 尝试导入 pyqtgraph，如果不可用则使用简单的替代方案
try:
    import pyqtgraph as pg
    PYQTGRAPH_AVAILABLE = True
except ImportError:
    PYQTGRAPH_AVAILABLE = False


class VelocityPlot(QWidget):
    """
    速度历史曲线
    
    显示线速度和角速度的历史变化曲线。
    如果 pyqtgraph 不可用，显示简单的文本提示。
    """
    
    def __init__(self, history_sec: float = 10.0, parent=None):
        super().__init__(parent)
        
        self._history_sec = history_sec
        self._plot_widget = None
        self._linear_curve = None
        self._angular_curve = None
        
        self._init_ui()
    
    def _init_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(5)
        
        # 标题
        title = QLabel("速度历史曲线")
        title.setStyleSheet("color: #ffffff; font-size: 12px; font-weight: bold;")
        layout.addWidget(title)
        
        if PYQTGRAPH_AVAILABLE:
            self._init_plot(layout)
        else:
            self._init_fallback(layout)
        
        # 图例
        legend_layout = QVBoxLayout()
        legend_layout.setSpacing(2)
        
        linear_legend = QLabel("── vx (线速度)")
        linear_legend.setStyleSheet("color: #00ff88; font-size: 10px;")
        legend_layout.addWidget(linear_legend)
        
        angular_legend = QLabel("── ω (角速度)")
        angular_legend.setStyleSheet("color: #ffaa00; font-size: 10px;")
        legend_layout.addWidget(angular_legend)
        
        layout.addLayout(legend_layout)
        
        # 设置背景
        self.setStyleSheet("""
            VelocityPlot {
                background-color: #2a2a2a;
                border: 1px solid #444444;
                border-radius: 5px;
            }
        """)
    
    def _init_plot(self, layout):
        """初始化 pyqtgraph 绘图"""
        # 配置 pyqtgraph
        pg.setConfigOptions(antialias=True)
        
        # 创建绘图组件
        self._plot_widget = pg.PlotWidget()
        self._plot_widget.setBackground('#1a1a1a')
        self._plot_widget.showGrid(x=True, y=True, alpha=0.3)
        self._plot_widget.setLabel('left', '速度')
        self._plot_widget.setLabel('bottom', '时间', units='s')
        self._plot_widget.setMinimumHeight(150)
        
        # 创建曲线
        self._linear_curve = self._plot_widget.plot(
            pen=pg.mkPen(color='#00ff88', width=2),
            name='vx'
        )
        self._angular_curve = self._plot_widget.plot(
            pen=pg.mkPen(color='#ffaa00', width=2),
            name='ω'
        )
        
        layout.addWidget(self._plot_widget)
    
    def _init_fallback(self, layout):
        """初始化回退方案 (无 pyqtgraph)"""
        fallback_label = QLabel(
            "速度曲线需要 pyqtgraph 库\n"
            "安装: pip install pyqtgraph"
        )
        fallback_label.setStyleSheet("""
            color: #888888;
            font-size: 11px;
            padding: 20px;
        """)
        fallback_label.setAlignment(Qt.AlignCenter)
        
        frame = QFrame()
        frame.setStyleSheet("""
            background-color: #1a1a1a;
            border: 1px solid #333333;
            border-radius: 3px;
        """)
        frame.setMinimumHeight(150)
        
        frame_layout = QVBoxLayout(frame)
        frame_layout.addWidget(fallback_label)
        
        layout.addWidget(frame)
    
    def update_data(self, history: Dict[str, List[float]]):
        """
        更新曲线数据
        
        Args:
            history: 包含 'timestamps', 'linear_x', 'angular_z' 的字典
        """
        if not PYQTGRAPH_AVAILABLE or self._plot_widget is None:
            return
        
        timestamps = history.get('timestamps', [])
        linear_x = history.get('linear_x', [])
        angular_z = history.get('angular_z', [])
        
        if not timestamps:
            return
        
        # 转换为相对时间 (最新时间为 0)
        t0 = timestamps[-1]
        rel_times = [t - t0 for t in timestamps]
        
        # 更新曲线 (使用最小长度确保数据对齐)
        n_linear = min(len(rel_times), len(linear_x))
        if n_linear > 0:
            self._linear_curve.setData(rel_times[:n_linear], linear_x[:n_linear])
        
        n_angular = min(len(rel_times), len(angular_z))
        if n_angular > 0:
            self._angular_curve.setData(rel_times[:n_angular], angular_z[:n_angular])
        
        # 设置 X 轴范围
        self._plot_widget.setXRange(-self._history_sec, 0)
