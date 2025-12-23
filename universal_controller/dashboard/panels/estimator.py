"""
状态估计面板
"""

from PyQt5.QtWidgets import QGroupBox, QVBoxLayout, QHBoxLayout, QLabel, QGridLayout
from PyQt5.QtCore import Qt
from ..widgets.progress_bar import ColorProgressBar
from ..widgets.status_led import StatusLED
from ..styles import COLORS


class EstimatorPanel(QGroupBox):
    """状态估计 (EKF) 面板"""
    
    def __init__(self, parent=None):
        super().__init__('状态估计 (EKF)', parent)
        self._setup_ui()
    
    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setSpacing(8)
        
        # 位置估计
        pos_title = QLabel('位置估计')
        pos_title.setStyleSheet('color: #2196F3; font-weight: bold; border-bottom: 1px solid #3D3D3D; padding-bottom: 3px;')
        layout.addWidget(pos_title)
        
        pos_grid = QGridLayout()
        pos_grid.setSpacing(3)
        
        pos_grid.addWidget(QLabel('x:'), 0, 0)
        self.x_label = QLabel('0.00 m')
        pos_grid.addWidget(self.x_label, 0, 1)
        
        pos_grid.addWidget(QLabel('y:'), 1, 0)
        self.y_label = QLabel('0.00 m')
        pos_grid.addWidget(self.y_label, 1, 1)
        
        pos_grid.addWidget(QLabel('z:'), 2, 0)
        self.z_label = QLabel('0.00 m')
        pos_grid.addWidget(self.z_label, 2, 1)
        
        layout.addLayout(pos_grid)
        
        layout.addSpacing(5)
        
        # 航向估计
        heading_title = QLabel('航向估计')
        heading_title.setStyleSheet('color: #2196F3; font-weight: bold; border-bottom: 1px solid #3D3D3D; padding-bottom: 3px;')
        layout.addWidget(heading_title)
        
        heading_grid = QGridLayout()
        heading_grid.setSpacing(3)
        
        heading_grid.addWidget(QLabel('θ:'), 0, 0)
        self.theta_label = QLabel('0.00 rad (0.0°)')
        heading_grid.addWidget(self.theta_label, 0, 1)
        
        heading_grid.addWidget(QLabel('来源:'), 1, 0)
        self.source_label = QLabel('EKF 融合')
        heading_grid.addWidget(self.source_label, 1, 1)
        
        heading_grid.addWidget(QLabel('备选:'), 2, 0)
        self.fallback_label = QLabel('○ 未使用')
        self.fallback_label.setStyleSheet('color: #B0B0B0;')
        heading_grid.addWidget(self.fallback_label, 2, 1)
        
        layout.addLayout(heading_grid)
        
        layout.addSpacing(5)
        
        # 速度估计
        vel_title = QLabel('速度估计 (世界坐标系)')
        vel_title.setStyleSheet('color: #2196F3; font-weight: bold; border-bottom: 1px solid #3D3D3D; padding-bottom: 3px;')
        layout.addWidget(vel_title)
        
        vel_grid = QGridLayout()
        vel_grid.setSpacing(3)
        
        vel_grid.addWidget(QLabel('vx:'), 0, 0)
        self.vx_label = QLabel('0.00 m/s')
        vel_grid.addWidget(self.vx_label, 0, 1)
        
        vel_grid.addWidget(QLabel('vy:'), 1, 0)
        self.vy_label = QLabel('0.00 m/s')
        vel_grid.addWidget(self.vy_label, 1, 1)
        
        vel_grid.addWidget(QLabel('vz:'), 2, 0)
        self.vz_label = QLabel('0.00 m/s')
        vel_grid.addWidget(self.vz_label, 2, 1)
        
        vel_grid.addWidget(QLabel('omega:'), 3, 0)
        self.omega_label = QLabel('0.00 rad/s')
        vel_grid.addWidget(self.omega_label, 3, 1)
        
        layout.addLayout(vel_grid)
        
        layout.addSpacing(5)
        
        # 滤波器健康
        health_title = QLabel('滤波器健康')
        health_title.setStyleSheet('color: #2196F3; font-weight: bold; border-bottom: 1px solid #3D3D3D; padding-bottom: 3px;')
        layout.addWidget(health_title)
        
        health_grid = QGridLayout()
        health_grid.setSpacing(3)
        
        health_grid.addWidget(QLabel('协方差范数:'), 0, 0)
        self.cov_label = QLabel('0.000')
        health_grid.addWidget(self.cov_label, 0, 1)
        
        health_grid.addWidget(QLabel('新息范数:'), 1, 0)
        self.innov_label = QLabel('0.000')
        health_grid.addWidget(self.innov_label, 1, 1)
        
        layout.addLayout(health_grid)
        
        # 打滑概率
        slip_row = QHBoxLayout()
        slip_row.addWidget(QLabel('打滑概率:'))
        self.slip_label = QLabel('0.0%')
        slip_row.addWidget(self.slip_label)
        slip_row.addStretch()
        layout.addLayout(slip_row)
        
        self.slip_progress = ColorProgressBar(show_text=False)
        layout.addWidget(self.slip_progress)
        
        # IMU 状态
        self.imu_drift_led = StatusLED('IMU 漂移')
        layout.addWidget(self.imu_drift_led)
        
        self.imu_avail_led = StatusLED('IMU 可用')
        layout.addWidget(self.imu_avail_led)
        
        layout.addSpacing(5)
        
        # IMU Bias 估计
        bias_title = QLabel('IMU Bias 估计')
        bias_title.setStyleSheet('color: #2196F3; font-weight: bold; border-bottom: 1px solid #3D3D3D; padding-bottom: 3px;')
        layout.addWidget(bias_title)
        
        bias_grid = QGridLayout()
        bias_grid.setSpacing(3)
        
        bias_grid.addWidget(QLabel('bias_ax:'), 0, 0)
        self.bias_ax_label = QLabel('0.00 m/s²')
        self.bias_ax_label.setStyleSheet('color: #B0B0B0;')
        bias_grid.addWidget(self.bias_ax_label, 0, 1)
        
        bias_grid.addWidget(QLabel('bias_ay:'), 1, 0)
        self.bias_ay_label = QLabel('0.00 m/s²')
        self.bias_ay_label.setStyleSheet('color: #B0B0B0;')
        bias_grid.addWidget(self.bias_ay_label, 1, 1)
        
        bias_grid.addWidget(QLabel('bias_az:'), 2, 0)
        self.bias_az_label = QLabel('0.00 m/s²')
        self.bias_az_label.setStyleSheet('color: #B0B0B0;')
        bias_grid.addWidget(self.bias_az_label, 2, 1)
        
        layout.addLayout(bias_grid)
        
        layout.addStretch()

    def update_display(self, data):
        """使用统一数据模型更新显示"""
        from ..models import DisplayData
        if not isinstance(data, DisplayData):
            return

        # 检查数据可用性
        if not data.availability.estimator_data_available:
            self._show_unavailable()
            return

        est = data.estimator
        traj = data.trajectory

        # 位置估计
        pos = traj.current_position
        self.x_label.setText(f'{pos[0]:.2f} m')
        self.y_label.setText(f'{pos[1]:.2f} m')
        self.z_label.setText(f'{pos[2]:.2f} m')

        # 航向估计
        import math
        heading = traj.current_heading
        self.theta_label.setText(f'{heading:.2f} rad ({math.degrees(heading):.1f}°)')

        # 航向来源
        if est.heading_fallback_enabled:
            self.source_label.setText('Odom 备选')
            self.fallback_label.setText('● 使用中')
            self.fallback_label.setStyleSheet(f'color: #FFC107;')
        else:
            self.source_label.setText('EKF 融合' if est.ekf_enabled else 'Odom')
            self.fallback_label.setText('○ 未使用')
            self.fallback_label.setStyleSheet('color: #B0B0B0;')

        # 速度估计
        vel = traj.current_velocity
        self.vx_label.setText(f'{vel[0]:.2f} m/s')
        self.vy_label.setText(f'{vel[1]:.2f} m/s')
        self.vz_label.setText('0.00 m/s')
        self.omega_label.setText('0.00 rad/s')

        # 滤波器健康
        self.cov_label.setText(f'{est.covariance_norm:.4f}')
        self.innov_label.setText(f'{est.innovation_norm:.4f}')

        # 打滑概率
        slip = est.slip_probability * 100
        self.slip_label.setText(f'{slip:.1f}%')
        self.slip_progress.set_value(slip, 100)

        # IMU 状态
        self.imu_drift_led.set_status(not est.imu_drift_detected, 
                                       '○ 未检测' if not est.imu_drift_detected else '⚠ 检测到')
        self.imu_avail_led.set_status(est.imu_available, 
                                       '✓ 是' if est.imu_available else '✗ 否')

        # IMU Bias
        bias = est.imu_bias
        self.bias_ax_label.setText(f'{bias[0]:.3f} m/s²')
        self.bias_ay_label.setText(f'{bias[1]:.3f} m/s²')
        self.bias_az_label.setText(f'{bias[2]:.3f} m/s²')

    def _show_unavailable(self):
        """显示数据不可用状态"""
        unavailable_style = f'color: {COLORS["unavailable"]};'
        
        # 位置显示不可用
        self.x_label.setText('无数据')
        self.x_label.setStyleSheet(unavailable_style)
        self.y_label.setText('无数据')
        self.y_label.setStyleSheet(unavailable_style)
        self.z_label.setText('无数据')
        self.z_label.setStyleSheet(unavailable_style)
        
        # 航向显示不可用
        self.theta_label.setText('无数据')
        self.theta_label.setStyleSheet(unavailable_style)
        self.source_label.setText('无数据')
        self.source_label.setStyleSheet(unavailable_style)
        self.fallback_label.setText('无数据')
        self.fallback_label.setStyleSheet(unavailable_style)
        
        # 速度显示不可用
        self.vx_label.setText('无数据')
        self.vx_label.setStyleSheet(unavailable_style)
        self.vy_label.setText('无数据')
        self.vy_label.setStyleSheet(unavailable_style)
        self.vz_label.setText('无数据')
        self.vz_label.setStyleSheet(unavailable_style)
        self.omega_label.setText('无数据')
        self.omega_label.setStyleSheet(unavailable_style)
        
        # 滤波器健康显示不可用
        self.cov_label.setText('无数据')
        self.cov_label.setStyleSheet(unavailable_style)
        self.innov_label.setText('无数据')
        self.innov_label.setStyleSheet(unavailable_style)
        
        # 打滑概率显示不可用
        self.slip_label.setText('无数据')
        self.slip_label.setStyleSheet(unavailable_style)
        self.slip_progress.set_value(0, 100)
        
        # IMU 状态显示不可用
        self.imu_drift_led.set_status(None, '无数据')
        self.imu_avail_led.set_status(None, '无数据')
        
        # IMU Bias 显示不可用
        self.bias_ax_label.setText('无数据')
        self.bias_ax_label.setStyleSheet(unavailable_style)
        self.bias_ay_label.setText('无数据')
        self.bias_ay_label.setStyleSheet(unavailable_style)
        self.bias_az_label.setText('无数据')
        self.bias_az_label.setStyleSheet(unavailable_style)
