"""
运行统计面板
"""

from PyQt5.QtWidgets import QGroupBox, QVBoxLayout, QHBoxLayout, QLabel, QGridLayout
from PyQt5.QtCore import Qt
from ..styles import COLORS, STATE_NAMES


class StatisticsPanel(QGroupBox):
    """运行统计面板"""
    
    def __init__(self, parent=None):
        super().__init__('运行统计', parent)
        self._setup_ui()
    
    def _setup_ui(self):
        layout = QHBoxLayout(self)
        layout.setSpacing(20)
        
        # 左侧: 运行时长
        left_widget = self._create_section('运行时长')
        left_layout = left_widget.layout()
        
        left_grid = QGridLayout()
        left_grid.setSpacing(3)
        
        left_grid.addWidget(QLabel('运行时长:'), 0, 0)
        self.elapsed_label = QLabel('00:00:00')
        self.elapsed_label.setStyleSheet('font-weight: bold;')
        left_grid.addWidget(self.elapsed_label, 0, 1)
        
        left_grid.addWidget(QLabel('总控制周期:'), 1, 0)
        self.total_cycles_label = QLabel('0')
        left_grid.addWidget(self.total_cycles_label, 1, 1)
        
        left_grid.addWidget(QLabel('实际频率:'), 2, 0)
        self.actual_freq_label = QLabel('0.0 Hz')
        left_grid.addWidget(self.actual_freq_label, 2, 1)
        
        left_layout.addLayout(left_grid)
        left_layout.addStretch()
        
        layout.addWidget(left_widget)
        
        # 中间: 控制周期统计
        mid_widget = self._create_section('控制周期统计')
        mid_layout = mid_widget.layout()
        
        mid_grid = QGridLayout()
        mid_grid.setSpacing(3)
        
        mid_grid.addWidget(QLabel('平均周期:'), 0, 0)
        self.avg_cycle_label = QLabel('0.0 ms')
        mid_grid.addWidget(self.avg_cycle_label, 0, 1)
        
        mid_grid.addWidget(QLabel('最大周期:'), 1, 0)
        self.max_cycle_label = QLabel('0.0 ms')
        mid_grid.addWidget(self.max_cycle_label, 1, 1)
        
        mid_grid.addWidget(QLabel('最小周期:'), 2, 0)
        self.min_cycle_label = QLabel('0.0 ms')
        mid_grid.addWidget(self.min_cycle_label, 2, 1)
        
        mid_layout.addLayout(mid_grid)
        mid_layout.addStretch()
        
        layout.addWidget(mid_widget)
        
        # 右侧: 状态统计
        right_widget = self._create_section('状态统计')
        right_layout = right_widget.layout()
        
        self.state_labels = {}
        for state_id, (name, _) in STATE_NAMES.items():
            row = QHBoxLayout()
            row.setSpacing(5)
            
            name_label = QLabel(f'{name}:')
            name_label.setFixedWidth(120)
            row.addWidget(name_label)
            
            value_label = QLabel('0.0% (0)')
            value_label.setStyleSheet('color: #B0B0B0;')
            row.addWidget(value_label)
            row.addStretch()
            
            right_layout.addLayout(row)
            self.state_labels[state_id] = value_label
        
        right_layout.addStretch()
        
        layout.addWidget(right_widget)
        
        # 最右侧: 性能指标和事件统计
        perf_widget = self._create_section('性能指标')
        perf_layout = perf_widget.layout()
        
        perf_grid = QGridLayout()
        perf_grid.setSpacing(3)
        
        perf_grid.addWidget(QLabel('MPC 成功率:'), 0, 0)
        self.mpc_success_label = QLabel('0.0%')
        perf_grid.addWidget(self.mpc_success_label, 0, 1)
        
        perf_grid.addWidget(QLabel('备用切换次数:'), 1, 0)
        self.backup_switch_label = QLabel('0')
        perf_grid.addWidget(self.backup_switch_label, 1, 1)
        
        perf_grid.addWidget(QLabel('安全限制次数:'), 2, 0)
        self.safety_limit_label = QLabel('0')
        perf_grid.addWidget(self.safety_limit_label, 2, 1)
        
        perf_grid.addWidget(QLabel('TF2 降级次数:'), 3, 0)
        self.tf2_fallback_label = QLabel('0')
        perf_grid.addWidget(self.tf2_fallback_label, 3, 1)
        
        perf_grid.addWidget(QLabel('Soft 禁用次数:'), 4, 0)
        self.soft_disable_label = QLabel('0')
        perf_grid.addWidget(self.soft_disable_label, 4, 1)
        
        perf_layout.addLayout(perf_grid)
        perf_layout.addStretch()
        
        layout.addWidget(perf_widget)
    
    def _create_section(self, title: str):
        """创建分区"""
        widget = QGroupBox()
        widget.setStyleSheet('QGroupBox { border: none; }')
        layout = QVBoxLayout(widget)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(5)
        
        title_label = QLabel(title)
        title_label.setStyleSheet('color: #2196F3; font-weight: bold; border-bottom: 1px solid #3D3D3D; padding-bottom: 3px;')
        layout.addWidget(title_label)
        
        return widget

    def update_display(self, data):
        """使用统一数据模型更新显示"""
        from ..models import DisplayData
        if not isinstance(data, DisplayData):
            return

        # 检查数据可用性 - 统计面板始终显示，因为它是本地计算的
        # 但如果没有诊断数据，显示部分不可用
        stats = data.statistics

        # 运行时长 - 始终显示
        self.elapsed_label.setText(stats.elapsed_time_str)

        # 总周期
        self.total_cycles_label.setText(f'{stats.total_cycles:,}')

        # 实际频率
        self.actual_freq_label.setText(f'{stats.actual_freq:.1f} Hz')

        # 周期统计
        self.avg_cycle_label.setText(f'{stats.avg_cycle_ms:.1f} ms')
        self.max_cycle_label.setText(f'{stats.max_cycle_ms:.1f} ms')
        self.min_cycle_label.setText(f'{stats.min_cycle_ms:.1f} ms')

        # 状态统计 - 如果没有诊断数据，显示不可用
        if not data.availability.diagnostics_available:
            unavailable_style = f'color: {COLORS["unavailable"]};'
            for state_id, label in self.state_labels.items():
                label.setText('--')
                label.setStyleSheet(unavailable_style)
            self.mpc_success_label.setText('--')
            self.mpc_success_label.setStyleSheet(unavailable_style)
        else:
            total_cycles = max(stats.total_cycles, 1)
            for state_id, label in self.state_labels.items():
                count = stats.state_counts.get(state_id, 0)
                percent = count / total_cycles * 100
                label.setText(f'{percent:.1f}% ({count:,})')

                # 根据状态设置颜色
                if state_id == 1 and percent > 90:  # NORMAL
                    label.setStyleSheet(f'color: {COLORS["success"]};')
                elif state_id >= 4:  # BACKUP_ACTIVE 及以上
                    if count > 0:
                        label.setStyleSheet(f'color: {COLORS["warning"]};')
                    else:
                        label.setStyleSheet('color: #B0B0B0;')
                else:
                    label.setStyleSheet('color: #B0B0B0;')

            # 性能指标
            self.mpc_success_label.setText(f'{stats.mpc_success_rate:.1f}%')
            self.mpc_success_label.setStyleSheet('')

        self.backup_switch_label.setText(str(stats.backup_switch_count))
        self.safety_limit_label.setText(str(stats.safety_limit_count))
        self.tf2_fallback_label.setText(str(stats.tf2_fallback_count))
        self.soft_disable_label.setText(str(stats.soft_disable_count))
