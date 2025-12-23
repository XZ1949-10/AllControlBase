"""
系统状态面板 - 使用统一数据模型
"""

import time
from PyQt5.QtWidgets import QGroupBox, QVBoxLayout, QLabel, QHBoxLayout
from PyQt5.QtCore import Qt
from ..widgets.state_indicator import StateIndicator
from ..styles import COLORS
from ..models import DisplayData


class StatePanel(QGroupBox):
    """系统状态面板"""

    def __init__(self, parent=None):
        super().__init__('系统状态 (State)', parent)
        self._last_state = 0
        self._state_start_time = 0
        self._last_transition = ''
        self._setup_ui()

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(10)

        self.state_indicator = StateIndicator()
        layout.addWidget(self.state_indicator, alignment=Qt.AlignCenter)

        info_layout = QVBoxLayout()
        info_layout.setSpacing(3)

        duration_row = QHBoxLayout()
        duration_row.addWidget(QLabel('状态持续时间:'))
        self.duration_label = QLabel('00:00:00')
        self.duration_label.setStyleSheet('color: #4CAF50;')
        duration_row.addWidget(self.duration_label)
        duration_row.addStretch()
        info_layout.addLayout(duration_row)

        transition_row = QHBoxLayout()
        transition_row.addWidget(QLabel('上次状态变化:'))
        self.transition_label = QLabel('--')
        self.transition_label.setStyleSheet('color: #B0B0B0;')
        transition_row.addWidget(self.transition_label)
        transition_row.addStretch()
        info_layout.addLayout(transition_row)

        layout.addLayout(info_layout)

    def update_display(self, data: DisplayData):
        """使用统一数据模型更新显示"""
        from ..styles import COLORS
        
        # 检查数据可用性
        if not data.availability.diagnostics_available:
            self._show_unavailable()
            return

        ctrl = data.controller
        state = ctrl.state.value

        # 检测状态变化
        if state != self._last_state:
            old_name = self._get_state_name(self._last_state)
            new_name = ctrl.state_name
            self._last_transition = f'{old_name} → {new_name}'
            self._state_start_time = time.time()
            self._last_state = state

        self.state_indicator.set_state(state)

        # 更新持续时间
        duration = time.time() - self._state_start_time if self._state_start_time > 0 else data.statistics.elapsed_time
        hours = int(duration // 3600)
        minutes = int((duration % 3600) // 60)
        seconds = int(duration % 60)
        self.duration_label.setText(f'{hours:02d}:{minutes:02d}:{seconds:02d}')
        self.duration_label.setStyleSheet('color: #4CAF50;')

        if self._last_transition:
            self.transition_label.setText(self._last_transition)
            self.transition_label.setStyleSheet('color: #B0B0B0;')

    def _show_unavailable(self):
        """显示数据不可用状态"""
        from ..styles import COLORS
        unavailable_style = f'color: {COLORS["unavailable"]};'
        
        # 状态指示器显示未知状态
        self.state_indicator.set_state(-1)  # 无效状态
        
        # 持续时间显示不可用
        self.duration_label.setText('--:--:--')
        self.duration_label.setStyleSheet(unavailable_style)
        
        # 状态变化显示不可用
        self.transition_label.setText('无数据')
        self.transition_label.setStyleSheet(unavailable_style)

    def _get_state_name(self, state: int) -> str:
        names = {0: 'INIT', 1: 'NORMAL', 2: 'SOFT_DISABLED', 3: 'MPC_DEGRADED',
                 4: 'BACKUP_ACTIVE', 5: 'STOPPING', 6: 'STOPPED'}
        return names.get(state, 'UNKNOWN')
