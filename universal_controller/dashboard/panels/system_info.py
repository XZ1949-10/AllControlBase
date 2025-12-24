"""
系统信息面板 - 使用统一数据模型
"""

from PyQt5.QtWidgets import QWidget, QGroupBox, QVBoxLayout, QHBoxLayout, QLabel
from ..styles import COLORS
from ..widgets.status_led import StatusLED
from ..models import DisplayData


class SystemInfoPanel(QGroupBox):
    """系统信息面板"""

    def __init__(self, parent=None):
        super().__init__('系统信息 (System Info)', parent)
        self._setup_ui()

    def _setup_ui(self):
        layout = QHBoxLayout(self)
        layout.setSpacing(20)

        # 运行环境
        env_widget = self._create_section('运行环境')
        env_layout = env_widget.layout()
        self.ros_led = StatusLED('ROS环境')
        self.tf2_led = StatusLED('TF2可用')
        self.acados_led = StatusLED('ACADOS')
        self.imu_led = StatusLED('IMU输入')
        env_layout.addWidget(self.ros_led)
        env_layout.addWidget(self.tf2_led)
        env_layout.addWidget(self.acados_led)
        env_layout.addWidget(self.imu_led)
        env_layout.addStretch()
        layout.addWidget(env_widget)

        # 控制策略
        ctrl_widget = self._create_section('控制策略')
        ctrl_layout = ctrl_widget.layout()
        self.main_ctrl_label = self._create_info_row('主控制器:', 'MPC (Fallback)')
        self.backup_ctrl_label = self._create_info_row('备用控制器:', 'Pure Pursuit')
        self.current_ctrl_label = self._create_info_row('当前使用:', '● MPC')
        self.soft_head_led = StatusLED('Soft Head')
        ctrl_layout.addWidget(self.main_ctrl_label)
        ctrl_layout.addWidget(self.backup_ctrl_label)
        ctrl_layout.addWidget(self.current_ctrl_label)
        ctrl_layout.addWidget(self.soft_head_led)
        ctrl_layout.addStretch()
        layout.addWidget(ctrl_widget)

        # 平台配置
        platform_widget = self._create_section('平台配置')
        platform_layout = platform_widget.layout()
        self.platform_label = self._create_info_row('平台类型:', '差速车')
        self.freq_label = self._create_info_row('控制频率:', '50 Hz')
        self.horizon_label = self._create_info_row('MPC Horizon:', '20')
        self.dt_label = self._create_info_row('时间步长:', '0.02s')
        platform_layout.addWidget(self.platform_label)
        platform_layout.addWidget(self.freq_label)
        platform_layout.addWidget(self.horizon_label)
        platform_layout.addWidget(self.dt_label)
        platform_layout.addStretch()
        layout.addWidget(platform_widget)

        # 功能开关
        feature_widget = self._create_section('功能开关')
        feature_layout = feature_widget.layout()
        self.ekf_led = StatusLED('自适应EKF')
        self.slip_led = StatusLED('打滑检测')
        self.drift_led = StatusLED('漂移校正')
        self.heading_led = StatusLED('航向备选')
        feature_layout.addWidget(self.ekf_led)
        feature_layout.addWidget(self.slip_led)
        feature_layout.addWidget(self.drift_led)
        feature_layout.addWidget(self.heading_led)
        feature_layout.addStretch()
        layout.addWidget(feature_widget)

        # 坐标系配置
        coord_widget = self._create_section('坐标系配置')
        coord_layout = coord_widget.layout()
        self.target_frame_label = self._create_info_row('目标坐标系:', 'odom')
        self.output_frame_label = self._create_info_row('输出坐标系:', 'base_link')
        self.tf2_fallback_led = StatusLED('TF2降级')
        self.fallback_duration_label = self._create_info_row('降级持续:', '0 ms')
        coord_layout.addWidget(self.target_frame_label)
        coord_layout.addWidget(self.output_frame_label)
        coord_layout.addWidget(self.tf2_fallback_led)
        coord_layout.addWidget(self.fallback_duration_label)
        coord_layout.addStretch()
        layout.addWidget(coord_widget)

    def _create_section(self, title: str) -> QWidget:
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(5)
        title_label = QLabel(title)
        title_label.setStyleSheet('color: #2196F3; font-weight: bold; border-bottom: 1px solid #3D3D3D; padding-bottom: 3px;')
        layout.addWidget(title_label)
        return widget

    def _create_info_row(self, label: str, value: str) -> QWidget:
        widget = QWidget()
        layout = QHBoxLayout(widget)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(5)
        label_widget = QLabel(label)
        label_widget.setStyleSheet('color: #B0B0B0;')
        layout.addWidget(label_widget)
        value_widget = QLabel(value)
        value_widget.setObjectName('value')
        layout.addWidget(value_widget)
        layout.addStretch()
        return widget

    def update_display(self, data: DisplayData):
        """使用统一数据模型更新显示"""
        from ..styles import COLORS
        
        env = data.environment
        ctrl = data.controller
        platform = data.platform
        estimator = data.estimator
        transform = data.transform

        # 运行环境 - 直接从统一数据获取 (环境状态始终显示)
        self.ros_led.set_status(env.ros_available)
        self.tf2_led.set_status(env.tf2_available)
        self.acados_led.set_status(env.acados_available)
        self.imu_led.set_status(env.imu_available)

        # 平台配置 - 始终显示 (来自配置)
        self.platform_label.findChild(QLabel, 'value').setText(platform.platform_display)
        self.freq_label.findChild(QLabel, 'value').setText(f'{platform.ctrl_freq} Hz')
        self.horizon_label.findChild(QLabel, 'value').setText(str(platform.mpc_horizon))
        self.dt_label.findChild(QLabel, 'value').setText(f'{platform.mpc_dt}s')

        # 检查诊断数据可用性
        if not data.availability.diagnostics_available:
            self._show_unavailable_dynamic()
            return

        # 控制策略 - 需要诊断数据
        main_ctrl = 'MPC (ACADOS)' if env.acados_available else 'MPC (Fallback)'
        self.main_ctrl_label.findChild(QLabel, 'value').setText(main_ctrl)
        current = f'● {ctrl.current_controller}'
        self.current_ctrl_label.findChild(QLabel, 'value').setText(current)
        self.soft_head_led.set_status(ctrl.soft_head_enabled)

        # 功能开关 - 从统一数据获取
        # 注意：需要传入文本参数，否则 LED 不会更新显示文本
        self.ekf_led.set_status(estimator.ekf_enabled, '✓ 启用' if estimator.ekf_enabled else '✗ 禁用')
        self.slip_led.set_status(estimator.slip_detection_enabled, '✓ 启用' if estimator.slip_detection_enabled else '✗ 禁用')
        self.drift_led.set_status(estimator.drift_correction_enabled, '✓ 启用' if estimator.drift_correction_enabled else '✗ 禁用')
        self.heading_led.set_status(estimator.heading_fallback_enabled, '✓ 启用' if estimator.heading_fallback_enabled else '✗ 禁用')

        # 坐标系
        self.target_frame_label.findChild(QLabel, 'value').setText(transform.target_frame)
        self.output_frame_label.findChild(QLabel, 'value').setText(transform.output_frame)
        self.tf2_fallback_led.set_status(not transform.fallback_active)
        self.fallback_duration_label.findChild(QLabel, 'value').setText(f'{transform.fallback_duration_ms:.0f} ms')

    def _show_unavailable_dynamic(self):
        """显示动态数据不可用状态 (保留静态配置)"""
        from ..styles import COLORS
        unavailable_style = f'color: {COLORS["unavailable"]};'
        
        # 控制策略显示不可用
        self.current_ctrl_label.findChild(QLabel, 'value').setText('无数据')
        self.current_ctrl_label.findChild(QLabel, 'value').setStyleSheet(unavailable_style)
        self.soft_head_led.set_status(None, '无数据')
        
        # 功能开关显示不可用
        self.ekf_led.set_status(None, '无数据')
        self.slip_led.set_status(None, '无数据')
        self.drift_led.set_status(None, '无数据')
        self.heading_led.set_status(None, '无数据')
        
        # 坐标系显示不可用
        self.tf2_fallback_led.set_status(None, '无数据')
        self.fallback_duration_label.findChild(QLabel, 'value').setText('--')
        self.fallback_duration_label.findChild(QLabel, 'value').setStyleSheet(unavailable_style)
