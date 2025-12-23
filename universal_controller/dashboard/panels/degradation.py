"""
降级状态面板
"""

from PyQt5.QtWidgets import QGroupBox, QVBoxLayout, QHBoxLayout, QLabel
from PyQt5.QtCore import Qt
from ..widgets.state_indicator import StateMachineIndicator
from ..styles import COLORS


class DegradationPanel(QGroupBox):
    """降级状态面板 (7级状态机)"""
    
    def __init__(self, parent=None):
        super().__init__('降级状态 (7级状态机)', parent)
        self._setup_ui()
    
    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(5)
        
        # 状态机指示器
        self.state_machine = StateMachineIndicator()
        layout.addWidget(self.state_machine)
        
        # 分隔
        layout.addSpacing(10)
        
        # 恢复计数器标题
        counter_title = QLabel('恢复计数器')
        counter_title.setStyleSheet('color: #2196F3; font-weight: bold; border-bottom: 1px solid #3D3D3D; padding-bottom: 3px;')
        layout.addWidget(counter_title)
        
        # Alpha 恢复计数
        alpha_row = QHBoxLayout()
        alpha_row.addWidget(QLabel('Alpha恢复计数:'))
        self.alpha_count_label = QLabel('0 / 5')
        alpha_row.addWidget(self.alpha_count_label)
        alpha_row.addStretch()
        layout.addLayout(alpha_row)
        
        # MPC 恢复计数
        mpc_row = QHBoxLayout()
        mpc_row.addWidget(QLabel('MPC恢复计数:'))
        self.mpc_count_label = QLabel('0 / 5')
        mpc_row.addWidget(self.mpc_count_label)
        mpc_row.addStretch()
        layout.addLayout(mpc_row)
        
        layout.addStretch()

    def update_display(self, data):
        """使用统一数据模型更新显示"""
        from ..models import DisplayData
        if not isinstance(data, DisplayData):
            return

        # 检查数据可用性
        if not data.availability.diagnostics_available:
            self._show_unavailable()
            return

        # 状态机
        self.state_machine.set_state(data.controller.state)

        # 恢复计数器 (目前模型中没有这些字段，显示默认值)
        self.alpha_count_label.setText('0 / 5')
        self.alpha_count_label.setStyleSheet('')
        self.mpc_count_label.setText('0 / 5')
        self.mpc_count_label.setStyleSheet('')

    def _show_unavailable(self):
        """显示数据不可用状态"""
        unavailable_style = f'color: {COLORS["unavailable"]};'
        
        # 状态机显示未知状态
        from ..models import ControllerStateEnum
        self.state_machine.set_state(ControllerStateEnum.INIT)
        
        # 恢复计数器显示不可用
        self.alpha_count_label.setText('--')
        self.alpha_count_label.setStyleSheet(unavailable_style)
        self.mpc_count_label.setText('--')
        self.mpc_count_label.setStyleSheet(unavailable_style)
