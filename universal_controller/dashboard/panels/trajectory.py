"""
轨迹信息面板
"""

from PyQt5.QtWidgets import QGroupBox, QVBoxLayout, QHBoxLayout, QLabel, QGridLayout
from PyQt5.QtCore import Qt
from ..widgets.status_led import StatusLED
from ..styles import COLORS


class TrajectoryPanel(QGroupBox):
    """轨迹信息面板"""
    
    def __init__(self, parent=None):
        super().__init__('轨迹信息', parent)
        self._setup_ui()
    
    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(8)
        
        # Hard 轨迹
        hard_title = QLabel('Hard 轨迹 (必选)')
        hard_title.setStyleSheet('color: #2196F3; font-weight: bold; border-bottom: 1px solid #3D3D3D; padding-bottom: 3px;')
        layout.addWidget(hard_title)
        
        hard_grid = QGridLayout()
        hard_grid.setSpacing(3)
        
        hard_grid.addWidget(QLabel('轨迹点数:'), 0, 0)
        self.points_label = QLabel('20')
        hard_grid.addWidget(self.points_label, 0, 1)
        
        hard_grid.addWidget(QLabel('时间步长:'), 1, 0)
        self.dt_label = QLabel('0.1 s')
        hard_grid.addWidget(self.dt_label, 1, 1)
        
        hard_grid.addWidget(QLabel('总时长:'), 2, 0)
        self.duration_label = QLabel('2.0 s')
        hard_grid.addWidget(self.duration_label, 2, 1)
        
        hard_grid.addWidget(QLabel('坐标系:'), 3, 0)
        self.frame_label = QLabel('world')
        hard_grid.addWidget(self.frame_label, 3, 1)
        
        layout.addLayout(hard_grid)
        
        layout.addSpacing(5)
        
        # Soft 建议
        soft_title = QLabel('Soft 建议 (可选)')
        soft_title.setStyleSheet('color: #2196F3; font-weight: bold; border-bottom: 1px solid #3D3D3D; padding-bottom: 3px;')
        layout.addWidget(soft_title)
        
        self.soft_enabled_led = StatusLED('启用状态')
        layout.addWidget(self.soft_enabled_led)
        
        soft_grid = QGridLayout()
        soft_grid.setSpacing(3)
        
        soft_grid.addWidget(QLabel('置信度:'), 0, 0)
        self.confidence_label = QLabel('0.85')
        soft_grid.addWidget(self.confidence_label, 0, 1)
        
        soft_grid.addWidget(QLabel('融合权重 α:'), 1, 0)
        self.alpha_label = QLabel('0.82')
        soft_grid.addWidget(self.alpha_label, 1, 1)
        
        soft_grid.addWidget(QLabel('速度点数:'), 2, 0)
        self.vel_points_label = QLabel('20')
        soft_grid.addWidget(self.vel_points_label, 2, 1)
        
        layout.addLayout(soft_grid)
        
        layout.addSpacing(5)
        
        # 轨迹模式
        mode_title = QLabel('轨迹模式')
        mode_title.setStyleSheet('color: #2196F3; font-weight: bold; border-bottom: 1px solid #3D3D3D; padding-bottom: 3px;')
        layout.addWidget(mode_title)
        
        self.mode_labels = {}
        modes = [
            ('track', 'MODE_TRACK', '跟踪模式'),
            ('stop', 'MODE_STOP', '停车模式'),
            ('hover', 'MODE_HOVER', '悬停模式'),
            ('emergency', 'MODE_EMERGENCY', '紧急模式'),
        ]
        
        for mode_id, mode_name, mode_desc in modes:
            row = QHBoxLayout()
            led = QLabel('○')
            led.setFixedWidth(20)
            led.setObjectName(f'{mode_id}_led')
            row.addWidget(led)
            
            label = QLabel(f'{mode_name}  {mode_desc}')
            label.setObjectName(f'{mode_id}_label')
            row.addWidget(label)
            row.addStretch()
            
            layout.addLayout(row)
            self.mode_labels[mode_id] = (led, label)
        
        layout.addSpacing(5)
        
        # 最近点信息
        nearest_title = QLabel('最近点信息')
        nearest_title.setStyleSheet('color: #2196F3; font-weight: bold; border-bottom: 1px solid #3D3D3D; padding-bottom: 3px;')
        layout.addWidget(nearest_title)
        
        nearest_grid = QGridLayout()
        nearest_grid.setSpacing(3)
        
        nearest_grid.addWidget(QLabel('最近点距离:'), 0, 0)
        self.nearest_dist_label = QLabel('0.12 m')
        nearest_grid.addWidget(self.nearest_dist_label, 0, 1)
        
        nearest_grid.addWidget(QLabel('最近点索引:'), 1, 0)
        self.nearest_idx_label = QLabel('3 / 20')
        nearest_grid.addWidget(self.nearest_idx_label, 1, 1)
        
        nearest_grid.addWidget(QLabel('前视距离:'), 2, 0)
        self.lookahead_label = QLabel('1.2 m')
        nearest_grid.addWidget(self.lookahead_label, 2, 1)
        
        nearest_grid.addWidget(QLabel('前视点索引:'), 3, 0)
        self.lookahead_idx_label = QLabel('8 / 20')
        nearest_grid.addWidget(self.lookahead_idx_label, 3, 1)
        
        layout.addLayout(nearest_grid)
        
        layout.addStretch()

    def update_display(self, data):
        """使用统一数据模型更新显示"""
        from ..models import DisplayData
        if not isinstance(data, DisplayData):
            return

        # 检查数据可用性
        if not data.availability.trajectory_available:
            self._show_unavailable()
            return

        traj = data.trajectory
        consistency = data.consistency

        # Hard 轨迹
        self.points_label.setText(str(traj.num_points))
        self.points_label.setStyleSheet('')
        self.dt_label.setText(f'{traj.dt_sec:.2f} s')
        self.dt_label.setStyleSheet('')
        self.duration_label.setText(f'{traj.total_duration:.1f} s')
        self.duration_label.setStyleSheet('')
        self.frame_label.setText('world')
        self.frame_label.setStyleSheet('')

        # Soft 状态
        alpha = consistency.alpha_soft
        soft_enabled = traj.soft_enabled or alpha > 0.1
        self.soft_enabled_led.set_status(soft_enabled, '✓ 启用' if soft_enabled else '✗ 禁用')
        self.confidence_label.setText(f'{traj.confidence:.2f}')
        self.confidence_label.setStyleSheet('')
        self.alpha_label.setText(f'{alpha:.2f}')
        self.alpha_label.setStyleSheet('')
        self.vel_points_label.setText(str(len(traj.soft_velocities)))
        self.vel_points_label.setStyleSheet('')

        # 轨迹模式
        current_mode = traj.mode.lower() if traj.mode else 'track'
        for mode_id, (led, label) in self.mode_labels.items():
            if mode_id == current_mode:
                led.setText('●')
                led.setStyleSheet(f'color: {COLORS["success"]};')
                label.setStyleSheet('font-weight: bold;')
            else:
                led.setText('○')
                led.setStyleSheet('color: #606060;')
                label.setStyleSheet('color: #808080;')

        # 最近点信息
        self.nearest_dist_label.setText(f'{traj.nearest_distance:.2f} m')
        self.nearest_dist_label.setStyleSheet('')
        self.nearest_idx_label.setText(f'{traj.nearest_idx} / {traj.num_points}')
        self.nearest_idx_label.setStyleSheet('')
        self.lookahead_label.setText('1.2 m')
        self.lookahead_label.setStyleSheet('')
        self.lookahead_idx_label.setText(f'{traj.lookahead_idx} / {traj.num_points}')
        self.lookahead_idx_label.setStyleSheet('')

    def _show_unavailable(self):
        """显示数据不可用状态"""
        unavailable_style = f'color: {COLORS["unavailable"]};'
        
        # Hard 轨迹显示不可用
        self.points_label.setText('--')
        self.points_label.setStyleSheet(unavailable_style)
        self.dt_label.setText('--')
        self.dt_label.setStyleSheet(unavailable_style)
        self.duration_label.setText('--')
        self.duration_label.setStyleSheet(unavailable_style)
        self.frame_label.setText('--')
        self.frame_label.setStyleSheet(unavailable_style)
        
        # Soft 状态显示不可用
        self.soft_enabled_led.set_status(None, '无数据')
        self.confidence_label.setText('--')
        self.confidence_label.setStyleSheet(unavailable_style)
        self.alpha_label.setText('--')
        self.alpha_label.setStyleSheet(unavailable_style)
        self.vel_points_label.setText('--')
        self.vel_points_label.setStyleSheet(unavailable_style)
        
        # 轨迹模式显示不可用
        for mode_id, (led, label) in self.mode_labels.items():
            led.setText('○')
            led.setStyleSheet(unavailable_style)
            label.setStyleSheet(unavailable_style)
        
        # 最近点信息显示不可用
        self.nearest_dist_label.setText('--')
        self.nearest_dist_label.setStyleSheet(unavailable_style)
        self.nearest_idx_label.setText('--')
        self.nearest_idx_label.setStyleSheet(unavailable_style)
        self.lookahead_label.setText('--')
        self.lookahead_label.setStyleSheet(unavailable_style)
        self.lookahead_idx_label.setText('--')
        self.lookahead_idx_label.setStyleSheet(unavailable_style)
