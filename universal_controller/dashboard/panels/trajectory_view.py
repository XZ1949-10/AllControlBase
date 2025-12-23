"""
轨迹可视化面板 - 显示网络输出的轨迹和当前位置
"""

import numpy as np
from typing import Tuple, Optional, Deque
from collections import deque

from PyQt5.QtWidgets import (
    QGroupBox,
    QVBoxLayout,
    QHBoxLayout,
    QLabel,
    QCheckBox,
    QPushButton,
    QComboBox,
    QWidget,
    QSizePolicy,
    QSlider,
    QFrame,
)
from PyQt5.QtCore import Qt

# matplotlib 嵌入 PyQt5
import matplotlib

matplotlib.use('Qt5Agg')
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.pyplot as plt

# 设置中文字体支持
plt.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

from ..styles import COLORS


class TrajectoryViewPanel(QGroupBox):
    """轨迹可视化面板 - 优化版"""

    def __init__(self, parent=None):
        super().__init__('轨迹可视化', parent)

        # 历史轨迹缓存
        self._history_size = 300
        self._position_history: Deque[Tuple[float, float]] = deque(maxlen=self._history_size)
        self._last_position: Optional[Tuple[float, float]] = None

        # 显示选项
        self._show_hard_traj = True
        self._show_soft_vel = True
        self._show_history = True
        self._show_grid = True
        self._follow_robot = True
        self._view_mode = '2D'

        # 视图范围
        self._view_range = 6.0

        self._setup_ui()

    def _setup_ui(self):
        main_layout = QHBoxLayout(self)
        main_layout.setSpacing(10)
        main_layout.setContentsMargins(10, 15, 10, 10)

        # 左侧: 图表区域 (占主要空间)
        chart_container = QWidget()
        chart_layout = QVBoxLayout(chart_container)
        chart_layout.setContentsMargins(0, 0, 0, 0)
        chart_layout.setSpacing(5)

        # matplotlib 画布 - 更大的尺寸
        self.figure = Figure(facecolor='#2D2D2D')
        self.canvas = FigureCanvas(self.figure)
        self.canvas.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.canvas.setMinimumSize(500, 350)
        self.ax = self.figure.add_subplot(111)
        self._setup_axes()
        chart_layout.addWidget(self.canvas, 1)

        # 底部状态栏
        status_bar = self._create_status_bar()
        chart_layout.addWidget(status_bar)

        main_layout.addWidget(chart_container, 1)

        # 右侧: 控制面板
        control_panel = self._create_control_panel()
        main_layout.addWidget(control_panel)

    def _create_control_panel(self) -> QWidget:
        """创建右侧控制面板"""
        panel = QFrame()
        panel.setFixedWidth(160)
        panel.setStyleSheet("""
            QFrame {
                background-color: #252525;
                border-radius: 5px;
                border: 1px solid #3D3D3D;
            }
        """)

        layout = QVBoxLayout(panel)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(8)

        # 视图模式
        mode_label = QLabel('视图模式')
        mode_label.setStyleSheet('color: #2196F3; font-weight: bold; border: none;')
        layout.addWidget(mode_label)

        self.view_combo = QComboBox()
        self.view_combo.addItems(['2D 平面', '3D 立体'])
        self.view_combo.currentIndexChanged.connect(self._change_view_mode)
        layout.addWidget(self.view_combo)

        layout.addSpacing(5)

        # 显示选项
        opt_label = QLabel('显示选项')
        opt_label.setStyleSheet('color: #2196F3; font-weight: bold; border: none;')
        layout.addWidget(opt_label)

        self.cb_hard = QCheckBox('Hard 轨迹')
        self.cb_hard.setChecked(True)
        self.cb_hard.setStyleSheet('border: none;')
        self.cb_hard.stateChanged.connect(lambda s: setattr(self, '_show_hard_traj', s == Qt.Checked))
        layout.addWidget(self.cb_hard)

        self.cb_soft = QCheckBox('Soft 速度')
        self.cb_soft.setChecked(True)
        self.cb_soft.setStyleSheet('border: none;')
        self.cb_soft.stateChanged.connect(lambda s: setattr(self, '_show_soft_vel', s == Qt.Checked))
        layout.addWidget(self.cb_soft)

        self.cb_history = QCheckBox('历史轨迹')
        self.cb_history.setChecked(True)
        self.cb_history.setStyleSheet('border: none;')
        self.cb_history.stateChanged.connect(lambda s: setattr(self, '_show_history', s == Qt.Checked))
        layout.addWidget(self.cb_history)

        self.cb_grid = QCheckBox('网格线')
        self.cb_grid.setChecked(True)
        self.cb_grid.setStyleSheet('border: none;')
        self.cb_grid.stateChanged.connect(self._toggle_grid)
        layout.addWidget(self.cb_grid)

        self.cb_follow = QCheckBox('跟随机器人')
        self.cb_follow.setChecked(True)
        self.cb_follow.setStyleSheet('border: none;')
        self.cb_follow.stateChanged.connect(lambda s: setattr(self, '_follow_robot', s == Qt.Checked))
        layout.addWidget(self.cb_follow)

        layout.addSpacing(5)

        # 缩放控制
        zoom_label = QLabel('视野范围')
        zoom_label.setStyleSheet('color: #2196F3; font-weight: bold; border: none;')
        layout.addWidget(zoom_label)

        self.zoom_slider = QSlider(Qt.Horizontal)
        self.zoom_slider.setRange(2, 20)
        self.zoom_slider.setValue(6)
        self.zoom_slider.setStyleSheet('border: none;')
        self.zoom_slider.valueChanged.connect(self._on_zoom_changed)
        layout.addWidget(self.zoom_slider)

        self.zoom_value_label = QLabel('±6.0 m')
        self.zoom_value_label.setStyleSheet('color: #B0B0B0; border: none;')
        self.zoom_value_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.zoom_value_label)

        layout.addStretch()

        # 重置按钮
        self.btn_reset = QPushButton('重置视图')
        self.btn_reset.clicked.connect(self._reset_view)
        layout.addWidget(self.btn_reset)

        return panel

    def _create_status_bar(self) -> QWidget:
        """创建底部状态栏"""
        bar = QFrame()
        bar.setFixedHeight(30)
        bar.setStyleSheet("""
            QFrame {
                background-color: #252525;
                border-radius: 3px;
            }
            QLabel {
                color: #B0B0B0;
            }
        """)

        layout = QHBoxLayout(bar)
        layout.setContentsMargins(10, 0, 10, 0)
        layout.setSpacing(20)

        # 位置
        self.pos_label = QLabel('位置: (0.00, 0.00)')
        self.pos_label.setStyleSheet('color: #4CAF50;')
        layout.addWidget(self.pos_label)

        # 航向
        self.heading_label = QLabel('航向: 0.0°')
        self.heading_label.setStyleSheet('color: #2196F3;')
        layout.addWidget(self.heading_label)

        # 轨迹点
        self.points_label = QLabel('轨迹: 0 点')
        layout.addWidget(self.points_label)

        # 最近点
        self.nearest_label = QLabel('最近点: -- m')
        layout.addWidget(self.nearest_label)

        layout.addStretch()

        # Soft 状态
        self.soft_label = QLabel('Soft: --')
        layout.addWidget(self.soft_label)

        return bar

    def _setup_axes(self):
        """设置坐标轴样式"""
        self.ax.set_facecolor('#1A1A1A')
        self.ax.tick_params(colors='#808080', labelsize=9)
        for spine in self.ax.spines.values():
            spine.set_color('#404040')
        self.ax.set_xlabel('X (m)', color='#909090', fontsize=10)
        self.ax.set_ylabel('Y (m)', color='#909090', fontsize=10)
        self.ax.grid(True, color='#353535', linestyle='-', linewidth=0.5, alpha=0.8)
        self.ax.set_aspect('equal', adjustable='box')
        self.figure.subplots_adjust(left=0.1, right=0.95, top=0.95, bottom=0.1)

    def _toggle_grid(self, state):
        self._show_grid = state == Qt.Checked

    def _on_zoom_changed(self, value):
        self._view_range = float(value)
        self.zoom_value_label.setText(f'±{value:.1f} m')

    def _change_view_mode(self, index):
        self._view_mode = '2D' if index == 0 else '3D'
        self.figure.clear()

        if self._view_mode == '3D':
            self.ax = self.figure.add_subplot(111, projection='3d')
            self.ax.set_facecolor('#1A1A1A')
        else:
            self.ax = self.figure.add_subplot(111)
            self._setup_axes()

        self.canvas.draw_idle()

    def _reset_view(self):
        self._view_range = 6.0
        self.zoom_slider.setValue(6)
        self._position_history.clear()
        self._last_position = None

    def update_display(self, data):
        """使用统一数据模型更新显示"""
        from ..models import DisplayData
        if not isinstance(data, DisplayData):
            return

        traj = data.trajectory
        consistency = data.consistency

        # 构建轨迹数据字典
        trajectory_data = {
            'hard_points': traj.hard_points if traj.hard_points else [],
            'soft_velocities': traj.soft_velocities if traj.soft_velocities else [],
            'current_position': traj.current_position,
            'current_heading': traj.current_heading,
            'current_velocity': traj.current_velocity,
            'nearest_idx': traj.nearest_idx,
            'nearest_distance': traj.nearest_distance,
            'lookahead_idx': traj.lookahead_idx,
        }

        # 检查是否有真实轨迹数据
        has_trajectory = bool(trajectory_data['hard_points'])
        
        # 检查是否有真实位置数据 (非零位置)
        pos = trajectory_data.get('current_position')
        has_position = pos is not None and (pos[0] != 0 or pos[1] != 0 or pos[2] != 0)

        self._update_status_bar(trajectory_data, {
            'alpha_soft': consistency.alpha_soft
        }, has_trajectory=has_trajectory, has_position=has_position)

        # 只有有真实位置数据时才更新历史轨迹
        if has_position:
            if self._last_position is None or \
               np.hypot(pos[0] - self._last_position[0], pos[1] - self._last_position[1]) > 0.02:
                self._position_history.append((pos[0], pos[1]))
                self._last_position = (pos[0], pos[1])

        self._draw(trajectory_data, has_trajectory=has_trajectory, has_position=has_position)

    def _update_status_bar(self, traj_data: dict, consistency: dict, 
                           has_trajectory: bool = True, has_position: bool = True):
        pos = traj_data.get('current_position', (0, 0, 0))
        heading = traj_data.get('current_heading', 0)
        hard_points = traj_data.get('hard_points', [])
        nearest_dist = traj_data.get('nearest_distance', None)

        # 位置显示
        if has_position and pos:
            self.pos_label.setText(f'位置: ({pos[0]:.2f}, {pos[1]:.2f})')
            self.pos_label.setStyleSheet('color: #4CAF50;')
            self.heading_label.setText(f'航向: {np.degrees(heading):.1f}°')
            self.heading_label.setStyleSheet('color: #2196F3;')
        else:
            self.pos_label.setText('位置: 无真实数据')
            self.pos_label.setStyleSheet('color: #757575;')
            self.heading_label.setText('航向: 无真实数据')
            self.heading_label.setStyleSheet('color: #757575;')

        # 轨迹显示
        if has_trajectory:
            self.points_label.setText(f'轨迹: {len(hard_points)} 点')
            self.points_label.setStyleSheet('color: #B0B0B0;')
            self.nearest_label.setText(f'最近点: {nearest_dist:.2f} m' if nearest_dist else '最近点: -- m')
        else:
            self.points_label.setText('轨迹: 无真实数据')
            self.points_label.setStyleSheet('color: #757575;')
            self.nearest_label.setText('最近点: 无真实数据')

        # Soft 状态
        alpha = consistency.get('alpha_soft', 0)
        if alpha > 0.1:
            self.soft_label.setText(f'Soft: ✓ α={alpha:.2f}')
            self.soft_label.setStyleSheet('color: #4CAF50;')
        else:
            self.soft_label.setText(f'Soft: ✗ α={alpha:.2f}')
            self.soft_label.setStyleSheet('color: #FFC107;')

    def _draw(self, traj_data: dict, has_trajectory: bool = True, has_position: bool = True):
        self.ax.clear()
        if self._view_mode == '2D':
            self._draw_2d(traj_data, has_trajectory=has_trajectory, has_position=has_position)
        else:
            self._draw_3d(traj_data, has_trajectory=has_trajectory, has_position=has_position)
        self.canvas.draw_idle()

    def _draw_2d(self, traj_data: dict, has_trajectory: bool = True, has_position: bool = True):
        hard_points = traj_data.get('hard_points', [])
        soft_velocities = traj_data.get('soft_velocities', [])
        pos = traj_data.get('current_position', (0, 0, 0))
        heading = traj_data.get('current_heading', 0)
        vel = traj_data.get('current_velocity', (0, 0))
        nearest_idx = traj_data.get('nearest_idx', 0)
        lookahead_idx = traj_data.get('lookahead_idx', 0)

        # 坐标轴样式
        self.ax.set_facecolor('#1A1A1A')
        self.ax.tick_params(colors='#808080', labelsize=9)
        for spine in self.ax.spines.values():
            spine.set_color('#404040')
        self.ax.set_xlabel('X (m)', color='#909090', fontsize=10)
        self.ax.set_ylabel('Y (m)', color='#909090', fontsize=10)

        if self._show_grid:
            self.ax.grid(True, color='#353535', linestyle='-', linewidth=0.5, alpha=0.8)
        else:
            self.ax.grid(False)

        self.ax.set_aspect('equal', adjustable='box')

        # 如果没有任何数据，显示"无数据"提示
        if not has_trajectory and not has_position:
            self.ax.text(0.5, 0.5, '无真实数据', transform=self.ax.transAxes,
                        fontsize=16, color='#757575', ha='center', va='center')
            self.ax.set_xlim(-5, 5)
            self.ax.set_ylim(-5, 5)
            return

        # 历史轨迹 (只有有真实位置数据时才显示)
        if self._show_history and has_position and len(self._position_history) > 1:
            hist = list(self._position_history)
            hx, hy = zip(*hist)
            self.ax.plot(hx, hy, '-', color='#505050', linewidth=2, alpha=0.6)

        # Hard 轨迹 (只有有真实轨迹数据时才显示)
        if self._show_hard_traj and has_trajectory and hard_points:
            xs, ys = zip(*[(p[0], p[1]) for p in hard_points])
            self.ax.plot(xs, ys, 'o-', color='#2196F3', markersize=5, linewidth=2, label='Hard')

            if 0 <= nearest_idx < len(hard_points):
                self.ax.plot(hard_points[nearest_idx][0], hard_points[nearest_idx][1],
                           'o', color='#FF9800', markersize=10, zorder=5)
            if 0 <= lookahead_idx < len(hard_points):
                self.ax.plot(hard_points[lookahead_idx][0], hard_points[lookahead_idx][1],
                           'D', color='#9C27B0', markersize=8, zorder=5)

        # Soft 速度向量 (只有有真实数据时才显示)
        if self._show_soft_vel and has_trajectory and soft_velocities and hard_points:
            n = min(len(hard_points), len(soft_velocities))
            scale = 0.4
            for i in range(0, n, 3):
                px, py = hard_points[i][0], hard_points[i][1]
                vx, vy = soft_velocities[i][0], soft_velocities[i][1]
                if np.hypot(vx, vy) > 0.05:
                    self.ax.annotate('', xy=(px + vx * scale, py + vy * scale), xytext=(px, py),
                                    arrowprops=dict(arrowstyle='->', color='#4CAF50', lw=1.5))

        # 当前位置 (只有有真实位置数据时才显示)
        if has_position and pos:
            self.ax.plot(pos[0], pos[1], 'o', color='#F44336', markersize=14, zorder=10)

            # 航向箭头
            arrow_len = self._view_range * 0.12
            dx, dy = arrow_len * np.cos(heading), arrow_len * np.sin(heading)
            self.ax.annotate('', xy=(pos[0] + dx, pos[1] + dy), xytext=(pos[0], pos[1]),
                            arrowprops=dict(arrowstyle='->', color='#F44336', lw=2.5))

            # 速度向量
            if vel and np.hypot(vel[0], vel[1]) > 0.05:
                vel_scale = 0.6
                self.ax.annotate('', xy=(pos[0] + vel[0] * vel_scale, pos[1] + vel[1] * vel_scale),
                                xytext=(pos[0], pos[1]),
                                arrowprops=dict(arrowstyle='->', color='#FFEB3B', lw=2))

        # 视图范围
        if has_position and pos and self._follow_robot:
            r = self._view_range
            self.ax.set_xlim(pos[0] - r, pos[0] + r)
            self.ax.set_ylim(pos[1] - r, pos[1] + r)
        elif has_trajectory and hard_points:
            # 如果只有轨迹没有位置，以轨迹中心为视图中心
            xs = [p[0] for p in hard_points]
            ys = [p[1] for p in hard_points]
            cx, cy = np.mean(xs), np.mean(ys)
            r = self._view_range
            self.ax.set_xlim(cx - r, cx + r)
            self.ax.set_ylim(cy - r, cy + r)
        else:
            # 默认视图
            self.ax.set_xlim(-self._view_range, self._view_range)
            self.ax.set_ylim(-self._view_range, self._view_range)

    def _draw_3d(self, traj_data: dict, has_trajectory: bool = True, has_position: bool = True):
        hard_points = traj_data.get('hard_points', [])
        pos = traj_data.get('current_position', (0, 0, 0))

        self.ax.set_facecolor('#1A1A1A')
        self.ax.set_xlabel('X (m)', color='#909090')
        self.ax.set_ylabel('Y (m)', color='#909090')
        self.ax.set_zlabel('Z (m)', color='#909090')
        self.ax.tick_params(colors='#808080', labelsize=8)

        # 如果没有任何数据，显示提示
        if not has_trajectory and not has_position:
            self.ax.text2D(0.5, 0.5, '无真实数据', transform=self.ax.transAxes,
                          fontsize=16, color='#757575', ha='center', va='center')
            r = self._view_range
            self.ax.set_xlim(-r, r)
            self.ax.set_ylim(-r, r)
            self.ax.set_zlim(0, r)
            return

        # Hard 轨迹
        if self._show_hard_traj and has_trajectory and hard_points:
            xs, ys, zs = zip(*hard_points)
            self.ax.plot(xs, ys, zs, 'o-', color='#2196F3', markersize=4, linewidth=1.5)

        # 当前位置
        if has_position and pos:
            self.ax.scatter([pos[0]], [pos[1]], [pos[2]], c='#F44336', s=120, marker='o')

        # 视图范围
        if has_position and pos and self._follow_robot:
            r = self._view_range
            self.ax.set_xlim(pos[0] - r, pos[0] + r)
            self.ax.set_ylim(pos[1] - r, pos[1] + r)
            self.ax.set_zlim(max(0, pos[2] - r / 2), pos[2] + r / 2)
        elif has_trajectory and hard_points:
            xs = [p[0] for p in hard_points]
            ys = [p[1] for p in hard_points]
            zs = [p[2] for p in hard_points]
            cx, cy, cz = np.mean(xs), np.mean(ys), np.mean(zs)
            r = self._view_range
            self.ax.set_xlim(cx - r, cx + r)
            self.ax.set_ylim(cy - r, cy + r)
            self.ax.set_zlim(max(0, cz - r / 2), cz + r / 2)
        else:
            r = self._view_range
            self.ax.set_xlim(-r, r)
            self.ax.set_ylim(-r, r)
            self.ax.set_zlim(0, r)

