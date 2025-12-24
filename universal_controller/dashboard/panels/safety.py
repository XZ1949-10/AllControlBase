"""
安全约束状态面板

配置来源: 
- universal_controller/config/safety_config.py -> CONSTRAINTS_CONFIG, SAFETY_CONFIG
YAML 覆盖: controller_ros/config/turtlebot1.yaml -> constraints, safety 节
"""

from PyQt5.QtWidgets import QGroupBox, QVBoxLayout, QHBoxLayout, QLabel, QGridLayout
from PyQt5.QtCore import Qt
from ..widgets.progress_bar import ColorProgressBar
from ..widgets.status_led import StatusLED
from ..styles import COLORS

# 从统一配置模块导入默认值
from ...config import CONSTRAINTS_CONFIG, SAFETY_CONFIG


class SafetyPanel(QGroupBox):
    """安全约束状态面板"""
    
    def __init__(self, parent=None, config=None):
        super().__init__('安全约束状态', parent)
        self._config = config or {}
        self._load_config()
        self._setup_ui()
    
    def _load_config(self):
        """从配置加载约束参数，使用统一配置作为默认值"""
        # 约束配置
        constraints = self._config.get('constraints', {})
        self._v_max = constraints.get('v_max', CONSTRAINTS_CONFIG['v_max'])
        self._v_min = constraints.get('v_min', CONSTRAINTS_CONFIG.get('v_min', 0.0))
        self._omega_max = constraints.get('omega_max', CONSTRAINTS_CONFIG['omega_max'])
        self._a_max = constraints.get('a_max', CONSTRAINTS_CONFIG['a_max'])
        self._az_max = constraints.get('az_max', CONSTRAINTS_CONFIG['az_max'])
        self._alpha_max = constraints.get('alpha_max', CONSTRAINTS_CONFIG['alpha_max'])
        
        # 安全配置
        safety = self._config.get('safety', {})
        
        # 低速保护配置
        low_speed = safety.get('low_speed', {})
        default_low_speed = SAFETY_CONFIG.get('low_speed', {})
        self._low_speed_thresh = low_speed.get('threshold', default_low_speed.get('threshold', 0.1))
        self._low_speed_omega = low_speed.get('omega_limit', default_low_speed.get('omega_limit', 1.0))
        
        # 安全裕度配置
        margins = safety.get('margins', {})
        default_margins = SAFETY_CONFIG.get('margins', {})
        self._vel_margin = margins.get('velocity', default_margins.get('velocity', 1.1))
        self._accel_margin = margins.get('acceleration', default_margins.get('acceleration', 1.5))
    
    def set_config(self, config):
        """更新配置"""
        self._config = config or {}
        self._load_config()
        self._update_config_labels()
    
    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(8)
        
        # 速度约束
        vel_title = QLabel('速度约束')
        vel_title.setStyleSheet('color: #2196F3; font-weight: bold; border-bottom: 1px solid #3D3D3D; padding-bottom: 3px;')
        layout.addWidget(vel_title)
        
        self.v_max_progress = self._add_constraint_row(layout, 'v_max', self._v_max, 'm/s')
        self.omega_max_progress = self._add_constraint_row(layout, 'omega_max', self._omega_max, 'rad/s')
        
        layout.addSpacing(5)
        
        # 加速度约束
        accel_title = QLabel('加速度约束')
        accel_title.setStyleSheet('color: #2196F3; font-weight: bold; border-bottom: 1px solid #3D3D3D; padding-bottom: 3px;')
        layout.addWidget(accel_title)
        
        self.a_max_progress = self._add_constraint_row(layout, 'a_max', self._a_max, 'm/s²')
        self.az_max_progress = self._add_constraint_row(layout, 'az_max', self._az_max, 'm/s²')
        self.alpha_max_progress = self._add_constraint_row(layout, 'alpha_max', self._alpha_max, 'rad/s²')
        
        layout.addSpacing(5)
        
        # 低速保护
        low_title = QLabel('低速保护')
        low_title.setStyleSheet('color: #2196F3; font-weight: bold; border-bottom: 1px solid #3D3D3D; padding-bottom: 3px;')
        layout.addWidget(low_title)
        
        low_grid = QGridLayout()
        low_grid.setSpacing(3)
        
        low_grid.addWidget(QLabel('低速阈值:'), 0, 0)
        self.low_thresh_label = QLabel(f'{self._low_speed_thresh} m/s')
        self.low_thresh_label.setStyleSheet('color: #B0B0B0;')
        low_grid.addWidget(self.low_thresh_label, 0, 1)
        
        low_grid.addWidget(QLabel('低速角速度限制:'), 1, 0)
        self.low_omega_label = QLabel(f'{self._low_speed_omega} rad/s')
        self.low_omega_label.setStyleSheet('color: #B0B0B0;')
        low_grid.addWidget(self.low_omega_label, 1, 1)
        
        self.low_speed_led = StatusLED('当前状态')
        low_grid.addWidget(self.low_speed_led, 2, 0, 1, 2)
        
        layout.addLayout(low_grid)
        
        layout.addSpacing(5)
        
        # 安全裕度
        margin_title = QLabel('安全裕度')
        margin_title.setStyleSheet('color: #2196F3; font-weight: bold; border-bottom: 1px solid #3D3D3D; padding-bottom: 3px;')
        layout.addWidget(margin_title)
        
        margin_grid = QGridLayout()
        margin_grid.setSpacing(3)
        
        margin_grid.addWidget(QLabel('速度裕度:'), 0, 0)
        self.vel_margin_label = QLabel(f'{self._vel_margin * 100:.0f}% ({self._vel_margin}x)')
        self.vel_margin_label.setStyleSheet('color: #B0B0B0;')
        margin_grid.addWidget(self.vel_margin_label, 0, 1)
        
        margin_grid.addWidget(QLabel('加速度裕度:'), 1, 0)
        self.accel_margin_label = QLabel(f'{self._accel_margin * 100:.0f}% ({self._accel_margin}x)')
        self.accel_margin_label.setStyleSheet('color: #B0B0B0;')
        margin_grid.addWidget(self.accel_margin_label, 1, 1)
        
        layout.addLayout(margin_grid)
        
        layout.addSpacing(5)
        
        # 安全检查状态
        self.safety_led = StatusLED('安全检查')
        layout.addWidget(self.safety_led)
        
        last_row = QHBoxLayout()
        last_row.addWidget(QLabel('上次限制:'))
        self.last_limit_label = QLabel('无')
        self.last_limit_label.setStyleSheet('color: #B0B0B0;')
        last_row.addWidget(self.last_limit_label)
        last_row.addStretch()
        layout.addLayout(last_row)
        
        layout.addStretch()
    
    def _update_config_labels(self):
        """更新配置标签显示"""
        if hasattr(self, 'low_thresh_label'):
            self.low_thresh_label.setText(f'{self._low_speed_thresh} m/s')
            self.low_omega_label.setText(f'{self._low_speed_omega} rad/s')
            self.vel_margin_label.setText(f'{self._vel_margin * 100:.0f}% ({self._vel_margin}x)')
            self.accel_margin_label.setText(f'{self._accel_margin * 100:.0f}% ({self._accel_margin}x)')
    
    def _add_constraint_row(self, parent_layout, name: str, max_val: float, unit: str):
        """添加约束行"""
        row_layout = QHBoxLayout()
        row_layout.setSpacing(5)
        
        # 名称和最大值
        label = QLabel(f'{name}: {max_val} {unit}')
        label.setFixedWidth(150)
        label.setObjectName(f'{name}_label')
        row_layout.addWidget(label)
        
        # 当前值标签
        current_label = QLabel('当前: 0.00')
        current_label.setObjectName(f'{name}_current')
        current_label.setFixedWidth(100)
        row_layout.addWidget(current_label)
        
        # 进度条
        progress = ColorProgressBar(show_text=False)
        row_layout.addWidget(progress, 1)
        
        parent_layout.addLayout(row_layout)
        
        setattr(self, f'{name}_current_label', current_label)
        setattr(self, f'{name}_max_label', label)
        
        return progress

    def update_display(self, data):
        """使用统一数据模型更新显示"""
        from ..models import DisplayData
        import math
        if not isinstance(data, DisplayData):
            return

        # 检查数据可用性
        if not data.availability.diagnostics_available:
            self._show_unavailable()
            return

        safety = data.safety
        cmd = data.command
        
        # 使用配置的约束值（如果 safety 数据中有则优先使用）
        v_max = safety.v_max if safety.v_max > 0 else self._v_max
        omega_max = safety.omega_max if safety.omega_max > 0 else self._omega_max
        a_max = safety.a_max if safety.a_max > 0 else self._a_max

        # 更新约束标签
        self.v_max_max_label.setText(f'v_max: {v_max} m/s')
        self.omega_max_max_label.setText(f'omega_max: {omega_max} rad/s')
        self.a_max_max_label.setText(f'a_max: {a_max} m/s²')

        # 速度 - 取绝对值
        v = math.sqrt(cmd.vx**2 + cmd.vy**2)
        self.v_max_progress.set_value(v, v_max)
        self.v_max_current_label.setText(f'当前: {v:.2f}')
        self.v_max_current_label.setStyleSheet('')

        # 角速度 - 取绝对值
        omega = abs(cmd.omega)
        self.omega_max_progress.set_value(omega, omega_max)
        self.omega_max_current_label.setText(f'当前: {omega:.2f}')
        self.omega_max_current_label.setStyleSheet('')

        # 加速度 - 从 safety 数据获取当前值
        # 注意：当前实现中没有实时加速度数据，显示 N/A
        self.a_max_progress.set_value(0, a_max)
        self.a_max_current_label.setText('当前: N/A')
        self.a_max_current_label.setStyleSheet('color: #808080;')

        self.az_max_progress.set_value(0, self._az_max)
        self.az_max_current_label.setText('当前: N/A')
        self.az_max_current_label.setStyleSheet('color: #808080;')

        self.alpha_max_progress.set_value(0, self._alpha_max)
        self.alpha_max_current_label.setText('当前: N/A')
        self.alpha_max_current_label.setStyleSheet('color: #808080;')

        # 低速保护
        low_speed = safety.low_speed_protection_active
        self.low_speed_led.set_status(not low_speed, '○ 未触发' if not low_speed else '● 已触发')

        # 安全检查
        safety_ok = safety.safety_check_passed
        self.safety_led.set_status(safety_ok, '✓ 全部通过' if safety_ok else '✗ 检查失败')

    def _show_unavailable(self):
        """显示数据不可用状态"""
        unavailable_style = f'color: {COLORS["unavailable"]};'
        
        # 进度条显示为空
        self.v_max_progress.set_value(0, self._v_max)
        self.omega_max_progress.set_value(0, self._omega_max)
        self.a_max_progress.set_value(0, self._a_max)
        self.az_max_progress.set_value(0, self._az_max)
        self.alpha_max_progress.set_value(0, self._alpha_max)
        
        # 当前值标签显示不可用
        self.v_max_current_label.setText('当前: --')
        self.v_max_current_label.setStyleSheet(unavailable_style)
        self.omega_max_current_label.setText('当前: --')
        self.omega_max_current_label.setStyleSheet(unavailable_style)
        self.a_max_current_label.setText('当前: --')
        self.a_max_current_label.setStyleSheet(unavailable_style)
        self.az_max_current_label.setText('当前: --')
        self.az_max_current_label.setStyleSheet(unavailable_style)
        self.alpha_max_current_label.setText('当前: --')
        self.alpha_max_current_label.setStyleSheet(unavailable_style)
        
        # LED 显示不可用
        self.low_speed_led.set_status(None)
        self.safety_led.set_status(None)
