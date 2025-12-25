"""
轨迹可视化组件

在相机图像或俯视图上显示轨迹和机器人位置。
支持使用单应性变换将轨迹投影到相机图像上。
"""
from typing import Optional, List, Tuple
import math
import logging

from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel, QFrame
from PyQt5.QtCore import Qt, QTimer, QPointF
from PyQt5.QtGui import QPainter, QPen, QBrush, QColor, QFont, QPolygonF, QImage, QPixmap
import numpy as np

from ..models import TrajectoryData, RobotPose, Point3D
from ..homography import HomographyTransform

logger = logging.getLogger(__name__)


class TrajectoryView(QWidget):
    """
    轨迹可视化视图
    
    功能:
    - 显示相机图像 (如果有)
    - 在图像/俯视图上绘制轨迹点
    - 显示机器人当前位置和朝向
    - 显示轨迹元信息
    """
    
    # 颜色定义
    COLOR_TRAJECTORY = QColor(0, 255, 0)       # 绿色 - 轨迹点
    COLOR_ROBOT = QColor(255, 0, 0)            # 红色 - 机器人
    COLOR_TARGET = QColor(255, 255, 0)         # 黄色 - 目标点
    COLOR_BACKGROUND = QColor(40, 40, 40)      # 深灰 - 背景
    COLOR_GRID = QColor(60, 60, 60)            # 浅灰 - 网格
    COLOR_TEXT = QColor(200, 200, 200)         # 浅灰 - 文字
    
    def __init__(self, parent=None):
        super().__init__(parent)
        
        # 数据
        self._trajectory: Optional[TrajectoryData] = None
        self._robot_pose: Optional[RobotPose] = None
        self._camera_image: Optional[np.ndarray] = None
        
        # 视图参数
        self._scale = 100.0          # 像素/米
        self._view_range = 3.0       # 视野范围 (米)
        self._use_camera = False     # 是否使用相机图像
        
        # 单应性变换 (用于相机图像模式)
        self._homography = HomographyTransform()
        self._boundary_margin = 50   # 边界容差 (像素)
        
        # 设置最小尺寸
        self.setMinimumSize(400, 300)
        
        # 启用鼠标追踪 (用于显示坐标)
        self.setMouseTracking(True)
        
        self._init_ui()
    
    def _init_ui(self):
        """初始化 UI"""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        
        # 信息标签
        self._info_label = QLabel()
        self._info_label.setStyleSheet("""
            QLabel {
                color: #cccccc;
                background-color: rgba(0, 0, 0, 150);
                padding: 5px;
                font-size: 11px;
            }
        """)
        self._info_label.setAlignment(Qt.AlignLeft | Qt.AlignTop)
        layout.addWidget(self._info_label)
        layout.addStretch()
    
    def set_trajectory(self, trajectory: TrajectoryData):
        """设置轨迹数据"""
        self._trajectory = trajectory
        self._update_info_label()
        self.update()
    
    def set_robot_pose(self, pose: RobotPose):
        """设置机器人位姿"""
        self._robot_pose = pose
        self.update()
    
    def set_camera_image(self, image: np.ndarray):
        """设置相机图像"""
        self._camera_image = image
        self._use_camera = image is not None
        self.update()
    
    def set_view_range(self, range_m: float):
        """设置视野范围 (米)"""
        self._view_range = max(0.5, range_m)
        self.update()
    
    def load_homography_calibration(self, calib_file: str) -> bool:
        """
        加载单应性标定文件
        
        Args:
            calib_file: YAML 标定文件路径
            
        Returns:
            是否加载成功
        """
        success = self._homography.load_calibration(calib_file)
        if success:
            logger.info(f"Homography calibration loaded: {calib_file}")
            error = self._homography.compute_reprojection_error()
            logger.info(f"Reprojection error: {error:.2f} pixels")
        return success
    
    @property
    def is_homography_calibrated(self) -> bool:
        """是否已加载单应性标定"""
        return self._homography.is_calibrated
    
    def _update_info_label(self):
        """更新信息标签"""
        if self._trajectory is None:
            self._info_label.setText("等待轨迹数据...")
            return
        
        info = (
            f"轨迹点数: {self._trajectory.num_points}\n"
            f"置信度: {self._trajectory.confidence:.2f}\n"
            f"模式: {self._trajectory.mode_name}\n"
            f"dt: {self._trajectory.dt_sec:.2f}s"
        )
        self._info_label.setText(info)
    
    def paintEvent(self, event):
        """绘制事件"""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        # 绘制背景
        if self._use_camera and self._camera_image is not None:
            self._draw_camera_image(painter)
        else:
            self._draw_grid_background(painter)
        
        # 绘制轨迹
        if self._trajectory is not None:
            self._draw_trajectory(painter)
        
        # 绘制机器人
        self._draw_robot(painter)
        
        painter.end()
    
    def _draw_camera_image(self, painter: QPainter):
        """绘制相机图像"""
        if self._camera_image is None:
            return
        
        h, w = self._camera_image.shape[:2]
        if len(self._camera_image.shape) == 3:
            # BGR -> RGB
            rgb = self._camera_image[:, :, ::-1].copy()
            qimg = QImage(rgb.data, w, h, 3 * w, QImage.Format_RGB888)
        else:
            qimg = QImage(self._camera_image.data, w, h, w, QImage.Format_Grayscale8)
        
        # 缩放到窗口大小
        pixmap = QPixmap.fromImage(qimg)
        scaled = pixmap.scaled(self.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        
        # 居中绘制
        x = (self.width() - scaled.width()) // 2
        y = (self.height() - scaled.height()) // 2
        painter.drawPixmap(x, y, scaled)
        
        # 保存缩放信息 (用于轨迹投影)
        self._camera_scale = scaled.width() / w
        self._camera_offset = (x, y)
        self._camera_size = (w, h)
    
    def _draw_grid_background(self, painter: QPainter):
        """绘制网格背景 (俯视图模式)"""
        # 背景
        painter.fillRect(self.rect(), self.COLOR_BACKGROUND)
        
        # 计算缩放
        self._scale = min(self.width(), self.height()) / (2 * self._view_range)
        
        # 网格
        pen = QPen(self.COLOR_GRID)
        pen.setWidth(1)
        painter.setPen(pen)
        
        grid_spacing = 0.5  # 0.5 米网格
        grid_pixels = grid_spacing * self._scale
        
        cx, cy = self.width() // 2, self.height() // 2
        
        # 垂直线
        x = cx
        while x < self.width():
            painter.drawLine(int(x), 0, int(x), self.height())
            painter.drawLine(int(2 * cx - x), 0, int(2 * cx - x), self.height())
            x += grid_pixels
        
        # 水平线
        y = cy
        while y < self.height():
            painter.drawLine(0, int(y), self.width(), int(y))
            painter.drawLine(0, int(2 * cy - y), self.width(), int(2 * cy - y))
            y += grid_pixels
        
        # 坐标轴
        pen.setColor(QColor(100, 100, 100))
        pen.setWidth(2)
        painter.setPen(pen)
        painter.drawLine(cx, 0, cx, self.height())  # Y 轴
        painter.drawLine(0, cy, self.width(), cy)   # X 轴
        
        # 轴标签
        painter.setPen(self.COLOR_TEXT)
        font = QFont()
        font.setPointSize(10)
        painter.setFont(font)
        painter.drawText(cx + 5, 15, "X (前)")
        painter.drawText(5, cy - 5, "Y (左)")
    
    def _world_to_screen(self, x: float, y: float) -> Tuple[int, int]:
        """世界坐标转屏幕坐标 (俯视图模式)"""
        # base_link 坐标系: X 朝前, Y 朝左
        # 屏幕坐标: X 朝右, Y 朝下
        # 映射: 世界 X -> 屏幕 -Y, 世界 Y -> 屏幕 -X
        cx, cy = self.width() // 2, self.height() // 2
        sx = cx - int(y * self._scale)  # Y 朝左 -> 屏幕 X 朝右取反
        sy = cy - int(x * self._scale)  # X 朝前 -> 屏幕 Y 朝上取反
        return sx, sy
    
    def _draw_trajectory(self, painter: QPainter):
        """绘制轨迹"""
        if self._trajectory is None or self._trajectory.num_points == 0:
            return
        
        # 根据模式选择投影方式
        if self._use_camera and self._homography.is_calibrated:
            self._draw_trajectory_on_camera(painter)
        else:
            self._draw_trajectory_on_birdview(painter)
    
    def _draw_trajectory_on_birdview(self, painter: QPainter):
        """在俯视图上绘制轨迹"""
        points = self._trajectory.points
        
        # 绘制轨迹线
        pen = QPen(self.COLOR_TRAJECTORY)
        pen.setWidth(2)
        painter.setPen(pen)
        
        for i in range(len(points) - 1):
            x1, y1 = self._world_to_screen(points[i].x, points[i].y)
            x2, y2 = self._world_to_screen(points[i + 1].x, points[i + 1].y)
            painter.drawLine(x1, y1, x2, y2)
        
        # 绘制轨迹点
        for i, p in enumerate(points):
            sx, sy = self._world_to_screen(p.x, p.y)
            
            if i == 0:
                # 第一个点 (目标点) - 黄色
                painter.setBrush(QBrush(self.COLOR_TARGET))
                radius = 8
            else:
                # 其他点 - 绿色
                painter.setBrush(QBrush(self.COLOR_TRAJECTORY))
                radius = 5
            
            painter.setPen(Qt.NoPen)
            painter.drawEllipse(sx - radius, sy - radius, radius * 2, radius * 2)
    
    def _draw_trajectory_on_camera(self, painter: QPainter):
        """在相机图像上绘制轨迹 (使用单应性变换)"""
        if not hasattr(self, '_camera_scale'):
            return
        
        points = self._trajectory.points
        scale = self._camera_scale
        offset_x, offset_y = self._camera_offset
        img_w, img_h = self._camera_size
        margin = self._boundary_margin
        
        # 投影所有轨迹点
        projected_points = []
        for p in points:
            proj = self._homography.project(p.x, p.y)
            if proj:
                u, v = proj
                # 检查是否在图像范围内 (带边界容差)
                if -margin <= u < img_w + margin and -margin <= v < img_h + margin:
                    # 裁剪到图像边界
                    u = max(0, min(img_w - 1, u))
                    v = max(0, min(img_h - 1, v))
                    # 转换到窗口坐标
                    sx = int(offset_x + u * scale)
                    sy = int(offset_y + v * scale)
                    projected_points.append((sx, sy))
                else:
                    projected_points.append(None)
            else:
                projected_points.append(None)
        
        # 绘制轨迹线
        pen = QPen(self.COLOR_TRAJECTORY)
        pen.setWidth(3)
        painter.setPen(pen)
        
        num_points = len(projected_points)
        for i in range(num_points - 1):
            if projected_points[i] and projected_points[i + 1]:
                # 渐变颜色
                ratio = i / max(num_points - 1, 1)
                color = QColor(
                    int(255 * ratio),       # R: 0 -> 255
                    int(255 * (1 - ratio)), # G: 255 -> 0
                    0                        # B: 0
                )
                pen.setColor(color)
                painter.setPen(pen)
                painter.drawLine(
                    projected_points[i][0], projected_points[i][1],
                    projected_points[i + 1][0], projected_points[i + 1][1]
                )
        
        # 绘制轨迹点
        for i, pt in enumerate(projected_points):
            if pt is None:
                continue
            
            ratio = i / max(num_points - 1, 1)
            
            if i == 0:
                # 第一个点 - 绿色大圆
                color = self.COLOR_TRAJECTORY
                radius = 12
            elif i == num_points - 1:
                # 最后一个点 - 蓝色
                color = QColor(255, 100, 0)
                radius = 10
            else:
                color = QColor(
                    int(255 * ratio),
                    int(255 * (1 - ratio)),
                    0
                )
                radius = 8
            
            painter.setBrush(QBrush(color))
            painter.setPen(QPen(QColor(255, 255, 255), 2))
            painter.drawEllipse(pt[0] - radius, pt[1] - radius, radius * 2, radius * 2)
            
            # 显示点序号
            painter.setPen(QColor(255, 255, 255))
            font = QFont()
            font.setPointSize(9)
            font.setBold(True)
            painter.setFont(font)
            painter.drawText(pt[0] + radius + 3, pt[1] + 4, str(i + 1))
    
    def _draw_robot(self, painter: QPainter):
        """绘制机器人"""
        # 机器人始终在中心 (base_link 坐标系原点)
        cx, cy = self.width() // 2, self.height() // 2
        
        # 机器人大小
        robot_size = 20
        
        # 绘制机器人三角形 (指向前方)
        painter.setBrush(QBrush(self.COLOR_ROBOT))
        painter.setPen(Qt.NoPen)
        
        # 三角形顶点 (指向屏幕上方，即世界 X 正方向)
        triangle = QPolygonF([
            QPointF(cx, cy - robot_size),           # 前端
            QPointF(cx - robot_size * 0.6, cy + robot_size * 0.5),  # 左后
            QPointF(cx + robot_size * 0.6, cy + robot_size * 0.5),  # 右后
        ])
        painter.drawPolygon(triangle)
        
        # 绘制中心点
        painter.setBrush(QBrush(QColor(255, 255, 255)))
        painter.drawEllipse(cx - 3, cy - 3, 6, 6)
