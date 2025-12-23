"""
控制输出面板
"""

from PyQt5.QtWidgets import QGroupBox, QVBoxLayout, QHBoxLayout, QLabel, QGridLayout
from PyQt5.QtCore import Qt
from ..widgets.progress_bar import ColorProgressBar
from ..widgets.status_led import StatusLED
from ..styles import COLORS


class ControlPanel(QGroupBox):
    """控制输出面板"""
    
    def __init__(self, parent=None):
        super().__init__('控制输出', parent)
        self._setup_ui()
    
    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(8)
        
        # 速度命令
        vel_title = QLabel('速度命令')
        vel_title.setStyleSheet('color: #2196F3; font-weight: bold; border-bottom: 1px solid #3D3D3D; padding-bottom: 3px;')
        layout.addWidget(vel_title)
        
        self.vx_progress = self._add_velocity_row(layout, 'vx', 2.0, 'm/s')
        self.vy_progress = self._add_velocity_row(layout, 'vy', 1.5, 'm/s')
        self.vz_progress = self._add_velocity_row(layout, 'vz', 2.0, 'm/s')
        self.omega_progress = self._add_velocity_row(layout, 'omega', 2.0, 'rad/s')
        
        layout.addSpacing(5)
        
        # 控制器信息
        ctrl_title = QLabel('控制器信息')
        ctrl_title.setStyleSheet('color: #2196F3; font-weight: bold; border-bottom: 1px solid #3D3D3D; padding-bottom: 3px;')
        layout.addWidget(ctrl_title)
        
        ctrl_grid = QGridLayout()
        ctrl_grid.setSpacing(3)
        
        ctrl_grid.addWidget(QLabel('当前控制器:'), 0, 0)
        self.controller_label = QLabel('MPC')
        self.controller_label.setStyleSheet(f'color: {COLORS["success"]}; font-weight: bold;')
        ctrl_grid.addWidget(self.controller_label, 0, 1)
        
        ctrl_grid.addWidget(QLabel('求解成功:'), 1, 0)
        self.success_label = QLabel('✓ 是')
        self.success_label.setStyleSheet(f'color: {COLORS["success"]};')
        ctrl_grid.addWidget(self.success_label, 1, 1)
        
        ctrl_grid.addWidget(QLabel('求解时间:'), 2, 0)
        self.solve_time_label = QLabel('8.2 ms')
        ctrl_grid.addWidget(self.solve_time_label, 2, 1)
        
        layout.addLayout(ctrl_grid)
        
        layout.addSpacing(5)
        
        # 过渡状态
        trans_title = QLabel('过渡状态')
        trans_title.setStyleSheet('color: #2196F3; font-weight: bold; border-bottom: 1px solid #3D3D3D; padding-bottom: 3px;')
        layout.addWidget(trans_title)
        
        self.transition_led = StatusLED('过渡中')
        layout.addWidget(self.transition_led)
        
        trans_grid = QGridLayout()
        trans_grid.setSpacing(3)
        
        trans_grid.addWidget(QLabel('过渡进度:'), 0, 0)
        self.progress_label = QLabel('100%')
        trans_grid.addWidget(self.progress_label, 0, 1)
        
        trans_grid.addWidget(QLabel('过渡类型:'), 1, 0)
        self.trans_type_label = QLabel('exponential')
        self.trans_type_label.setStyleSheet('color: #B0B0B0;')
        trans_grid.addWidget(self.trans_type_label, 1, 1)
        
        layout.addLayout(trans_grid)
        
        layout.addSpacing(5)
        
        # 安全检查
        safety_title = QLabel('安全检查')
        safety_title.setStyleSheet('color: #2196F3; font-weight: bold; border-bottom: 1px solid #3D3D3D; padding-bottom: 3px;')
        layout.addWidget(safety_title)
        
        self.safety_led = StatusLED('安全检查')
        layout.addWidget(self.safety_led)
        
        self.limit_led = StatusLED('限制应用')
        layout.addWidget(self.limit_led)
        
        # 输出坐标系
        frame_row = QHBoxLayout()
        frame_row.addWidget(QLabel('输出坐标系:'))
        self.frame_label = QLabel('base_link (机体)')
        self.frame_label.setStyleSheet('color: #B0B0B0;')
        frame_row.addWidget(self.frame_label)
        frame_row.addStretch()
        layout.addLayout(frame_row)
        
        layout.addStretch()
    
    def _add_velocity_row(self, parent_layout, name: str, max_val: float, unit: str):
        """添加速度行"""
        row_layout = QHBoxLayout()
        row_layout.setSpacing(5)
        
        # 名称和值
        name_label = QLabel(f'{name}:')
        name_label.setFixedWidth(50)
        row_layout.addWidget(name_label)
        
        value_label = QLabel('0.00')
        value_label.setFixedWidth(80)
        value_label.setObjectName(f'{name}_value')
        row_layout.addWidget(value_label)
        
        unit_label = QLabel(unit)
        unit_label.setFixedWidth(50)
        unit_label.setStyleSheet('color: #B0B0B0;')
        row_layout.addWidget(unit_label)
        
        # 进度条
        progress = ColorProgressBar(show_text=False)
        row_layout.addWidget(progress, 1)
        
        parent_layout.addLayout(row_layout)
        
        setattr(self, f'{name}_value_label', value_label)
        
        return progress

    def update_display(self, data):
        """使用统一数据模型更新显示"""
        from ..models import DisplayData
        if not isinstance(data, DisplayData):
            return

        # 检查数据可用性
        if not data.availability.diagnostics_available:
            self._show_unavailable()
            return

        cmd = data.command
        ctrl = data.controller
        mpc = data.mpc_health
        safety = data.safety

        # 速度命令
        self.vx_progress.set_value(abs(cmd.vx), safety.v_max)
        self.vx_value_label.setText(f'{cmd.vx:.2f}')

        self.vy_progress.set_value(abs(cmd.vy), 1.5)
        self.vy_value_label.setText(f'{cmd.vy:.2f}')

        self.vz_progress.set_value(abs(cmd.vz), 2.0)
        self.vz_value_label.setText(f'{cmd.vz:.2f}')

        self.omega_progress.set_value(abs(cmd.omega), safety.omega_max)
        self.omega_value_label.setText(f'{cmd.omega:.2f}')

        # 控制器信息
        controller = ctrl.current_controller
        color = COLORS['warning'] if ctrl.backup_active else COLORS['success']
        self.controller_label.setText(controller)
        self.controller_label.setStyleSheet(f'color: {color}; font-weight: bold;')

        self.success_label.setText('✓ 是' if ctrl.mpc_success else '✗ 否')
        self.success_label.setStyleSheet(f'color: {COLORS["success"] if ctrl.mpc_success else COLORS["error"]};')

        self.solve_time_label.setText(f'{mpc.solve_time_ms:.1f} ms')

        # 过渡状态
        progress = data.transition_progress
        in_transition = progress < 1.0
        self.transition_led.set_status(not in_transition, '○ 否' if not in_transition else '● 是')
        self.progress_label.setText(f'{progress * 100:.0f}%')

        # 安全检查
        safety_ok = safety.safety_check_passed
        self.safety_led.set_status(safety_ok, '✓ 通过' if safety_ok else '✗ 失败')
        self.limit_led.set_status(True, '○ 无')

        # 坐标系
        self.frame_label.setText(f'{cmd.frame_id} (机体)')

    def _show_unavailable(self):
        """显示数据不可用状态"""
        unavailable_style = f'color: {COLORS["unavailable"]};'
        
        # 速度命令显示不可用
        self.vx_progress.set_value(0, 2.0)
        self.vx_value_label.setText('--')
        self.vx_value_label.setStyleSheet(unavailable_style)
        
        self.vy_progress.set_value(0, 1.5)
        self.vy_value_label.setText('--')
        self.vy_value_label.setStyleSheet(unavailable_style)
        
        self.vz_progress.set_value(0, 2.0)
        self.vz_value_label.setText('--')
        self.vz_value_label.setStyleSheet(unavailable_style)
        
        self.omega_progress.set_value(0, 2.0)
        self.omega_value_label.setText('--')
        self.omega_value_label.setStyleSheet(unavailable_style)
        
        # 控制器信息显示不可用
        self.controller_label.setText('无数据')
        self.controller_label.setStyleSheet(unavailable_style)
        self.success_label.setText('无数据')
        self.success_label.setStyleSheet(unavailable_style)
        self.solve_time_label.setText('无数据')
        self.solve_time_label.setStyleSheet(unavailable_style)
        
        # 过渡状态显示不可用
        self.transition_led.set_status(None, '无数据')
        self.progress_label.setText('--')
        self.progress_label.setStyleSheet(unavailable_style)
        
        # 安全检查显示不可用
        self.safety_led.set_status(None, '无数据')
        self.limit_led.set_status(None, '无数据')
        
        # 坐标系显示不可用
        self.frame_label.setText('无数据')
        self.frame_label.setStyleSheet(unavailable_style)
