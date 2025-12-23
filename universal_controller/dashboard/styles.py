"""
样式定义 - 颜色方案和样式常量
"""

# 状态颜色
COLORS = {
    'INIT': '#9E9E9E',          # 灰色
    'NORMAL': '#4CAF50',        # 绿色
    'SOFT_DISABLED': '#FFC107', # 黄色
    'MPC_DEGRADED': '#FF9800',  # 橙色
    'BACKUP_ACTIVE': '#FF5722', # 橙红色
    'STOPPING': '#F44336',      # 红色
    'STOPPED': '#B71C1C',       # 深红色
    
    # 通用颜色
    'success': '#4CAF50',
    'warning': '#FFC107',
    'error': '#F44336',
    'info': '#2196F3',
    'disabled': '#9E9E9E',
    'unavailable': '#757575',   # 数据不可用颜色
    
    # 背景色
    'bg_dark': '#1E1E1E',
    'bg_panel': '#2D2D2D',
    'bg_header': '#3D3D3D',
    'bg_unavailable': '#252525',  # 数据不可用背景色
    'text': '#FFFFFF',
    'text_secondary': '#B0B0B0',
    'text_unavailable': '#606060',  # 数据不可用文字颜色
}

# 数据不可用显示文本
UNAVAILABLE_TEXT = '无数据'
UNAVAILABLE_VALUE = '--'

# 进度条颜色阈值
def get_progress_color(ratio: float) -> str:
    """根据占比返回颜色"""
    if ratio < 0.5:
        return COLORS['success']
    elif ratio < 0.8:
        return COLORS['warning']
    else:
        return COLORS['error']

# 状态名称映射
STATE_NAMES = {
    0: ('INIT', '初始化'),
    1: ('NORMAL', '正常运行'),
    2: ('SOFT_DISABLED', 'Soft禁用'),
    3: ('MPC_DEGRADED', 'MPC降级'),
    4: ('BACKUP_ACTIVE', '备用激活'),
    5: ('STOPPING', '停车中'),
    6: ('STOPPED', '已停车'),
}

# 状态描述
STATE_DESCRIPTIONS = {
    0: '系统启动中，等待数据',
    1: 'MPC 正常工作，Soft 启用',
    2: 'α < 0.1，仅使用 Hard 轨迹',
    3: 'Horizon 降低，性能下降',
    4: 'Pure Pursuit 接管控制',
    5: '正在平滑减速',
    6: '速度为零，等待恢复',
}

# 主窗口样式
MAIN_STYLE = """
QMainWindow {
    background-color: #1E1E1E;
}
QWidget {
    background-color: #1E1E1E;
    color: #FFFFFF;
    font-family: "Microsoft YaHei", "Segoe UI", sans-serif;
    font-size: 12px;
}
QGroupBox {
    background-color: #2D2D2D;
    border: 1px solid #3D3D3D;
    border-radius: 5px;
    margin-top: 10px;
    padding-top: 10px;
    font-weight: bold;
}
QGroupBox::title {
    subcontrol-origin: margin;
    left: 10px;
    padding: 0 5px;
    color: #2196F3;
}
QLabel {
    color: #FFFFFF;
}
QProgressBar {
    border: 1px solid #3D3D3D;
    border-radius: 3px;
    background-color: #1E1E1E;
    text-align: center;
    height: 16px;
}
QProgressBar::chunk {
    border-radius: 2px;
}
QScrollArea {
    border: none;
}
QTextEdit {
    background-color: #1E1E1E;
    border: 1px solid #3D3D3D;
    border-radius: 3px;
    color: #FFFFFF;
}
QPushButton {
    background-color: #3D3D3D;
    border: 1px solid #4D4D4D;
    border-radius: 3px;
    padding: 5px 15px;
    color: #FFFFFF;
}
QPushButton:hover {
    background-color: #4D4D4D;
}
QPushButton:pressed {
    background-color: #2D2D2D;
}
"""
