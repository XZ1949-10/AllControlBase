#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
轨迹可视化节点 - 在相机图像上叠加显示网络输出的轨迹点

功能:
- 订阅相机图像和轨迹数据
- 使用单应性变换将地面轨迹点投影到图像平面
- 支持标定模式：通过点击图像标定地面-图像对应关系
- 实时显示叠加后的图像

使用方法:
    # 正常运行 (需要先标定)
    roslaunch controller_ros trajectory_visualizer.launch
    
    # 标定模式
    roslaunch controller_ros trajectory_visualizer.launch calibration_mode:=true
    
标定步骤:
    1. 在机器人前方地面放置4个标记点，测量其相对于 base_footprint 的坐标 (x, y)
    2. 启动标定模式
    3. 按顺序点击图像中的4个标记点
    4. 输入对应的地面坐标
    5. 按 's' 保存标定结果
"""

import rospy
import cv2
import numpy as np
import yaml
import os
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from controller_ros.msg import LocalTrajectoryV4


class TrajectoryVisualizer:
    """轨迹可视化器 - 使用单应性变换投影轨迹点"""
    
    def __init__(self):
        rospy.init_node('trajectory_visualizer', anonymous=False)
        
        # 参数
        self.image_topic = rospy.get_param('~image_topic', '/usb_cam/image_raw')
        self.trajectory_topic = rospy.get_param('~trajectory_topic', '/nn/local_trajectory')
        self.output_topic = rospy.get_param('~output_topic', '/trajectory_overlay/image')
        
        # 标定文件路径
        default_calib_file = os.path.join(
            os.path.dirname(os.path.dirname(__file__)), 
            'config', 'homography_calib.yaml'
        )
        self.calib_file = rospy.get_param('~calibration_file', default_calib_file)
        
        # 模式
        self.calibration_mode = rospy.get_param('~calibration_mode', False)
        self.show_window = rospy.get_param('~show_window', True)
        
        # 显示参数
        self.point_radius = rospy.get_param('~point_radius', 8)
        self.line_thickness = rospy.get_param('~line_thickness', 3)
        
        # 单应性矩阵
        self.H = None  # 地面坐标 -> 图像坐标
        
        # 标定数据
        self.calib_image_points = []  # 图像上点击的点
        self.calib_ground_points = []  # 对应的地面坐标
        self.current_image = None
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # 数据缓存
        self.latest_trajectory = None
        self.trajectory_stamp = None
        
        # 加载标定
        if not self.calibration_mode:
            self._load_calibration()
        
        # 订阅
        self.image_sub = rospy.Subscriber(
            self.image_topic, Image, self.image_callback, queue_size=1
        )
        if not self.calibration_mode:
            self.traj_sub = rospy.Subscriber(
                self.trajectory_topic, LocalTrajectoryV4, self.trajectory_callback, queue_size=1
            )
        
        # 发布
        self.image_pub = rospy.Publisher(self.output_topic, Image, queue_size=1)
        
        # 统计
        self.frame_count = 0
        
        # 设置鼠标回调
        if self.calibration_mode and self.show_window:
            cv2.namedWindow('Calibration')
            cv2.setMouseCallback('Calibration', self._mouse_callback)
        
        self._print_startup_info()
    
    def _print_startup_info(self):
        """打印启动信息"""
        rospy.loginfo("=" * 50)
        if self.calibration_mode:
            rospy.loginfo("TrajectoryVisualizer 标定模式")
            rospy.loginfo("-" * 50)
            rospy.loginfo("操作说明:")
            rospy.loginfo("  1. 在地面放置4个标记点")
            rospy.loginfo("  2. 点击图像中的标记点 (按顺序)")
            rospy.loginfo("  3. 在终端输入对应的地面坐标 (x y)")
            rospy.loginfo("  4. 完成4个点后按 's' 保存")
            rospy.loginfo("  5. 按 'r' 重置, 'q' 退出")
            rospy.loginfo("-" * 50)
            rospy.loginfo("Suggested calibration points:")
            rospy.loginfo("  Point 1: (1.5, 0.0)  - front 1.5m")
            rospy.loginfo("  Point 2: (2.5, 0.0)  - front 2.5m")
            rospy.loginfo("  Point 3: (2.0, -0.5) - front-right")
            rospy.loginfo("  Point 4: (2.0, 0.5)  - front-left")
        else:
            rospy.loginfo("TrajectoryVisualizer 运行模式")
            rospy.loginfo(f"  图像输入: {self.image_topic}")
            rospy.loginfo(f"  轨迹输入: {self.trajectory_topic}")
            rospy.loginfo(f"  标定文件: {self.calib_file}")
            if self.H is not None:
                rospy.loginfo("  单应性矩阵: 已加载 ✓")
            else:
                rospy.logwarn("  单应性矩阵: 未加载! 请先运行标定模式")
        rospy.loginfo("=" * 50)
    
    def _load_calibration(self):
        """加载标定文件"""
        if not os.path.exists(self.calib_file):
            rospy.logwarn(f"标定文件不存在: {self.calib_file}")
            rospy.logwarn("请先运行标定模式: calibration_mode:=true")
            return False
        
        try:
            with open(self.calib_file, 'r') as f:
                data = yaml.safe_load(f)
            
            self.H = np.array(data['homography_matrix'], dtype=np.float64)
            self.calib_ground_points = data.get('ground_points', [])
            self.calib_image_points = data.get('image_points', [])
            
            rospy.loginfo(f"已加载标定文件: {self.calib_file}")
            return True
        except Exception as e:
            rospy.logerr(f"加载标定文件失败: {e}")
            return False
    
    def _save_calibration(self):
        """保存标定结果"""
        if self.H is None:
            rospy.logerr("没有有效的单应性矩阵，无法保存")
            return False
        
        # 确保目录存在
        os.makedirs(os.path.dirname(self.calib_file), exist_ok=True)
        
        data = {
            'homography_matrix': self.H.tolist(),
            'ground_points': self.calib_ground_points,
            'image_points': self.calib_image_points,
            'description': '地面坐标(x,y) -> 图像坐标(u,v) 单应性变换'
        }
        
        try:
            with open(self.calib_file, 'w') as f:
                yaml.dump(data, f, default_flow_style=False)
            rospy.loginfo(f"标定结果已保存: {self.calib_file}")
            return True
        except Exception as e:
            rospy.logerr(f"保存标定文件失败: {e}")
            return False
    
    def _mouse_callback(self, event, x, y, flags, param):
        """鼠标点击回调 (标定模式)"""
        if event == cv2.EVENT_LBUTTONDOWN:
            # 检查是否还有未输入地面坐标的点
            if len(self.calib_image_points) > len(self.calib_ground_points):
                rospy.logwarn("Please enter ground coord first before clicking next point!")
                return
            
            if len(self.calib_image_points) >= 4:
                rospy.logwarn("4 points already selected. Press 'r' to reset or 's' to save")
                return
            
            self.calib_image_points.append([x, y])
            pt_num = len(self.calib_image_points)
            rospy.loginfo(f"Point {pt_num}: image coord ({x}, {y})")
            
            # 请求输入地面坐标
            print(f"\nEnter ground coord for point {pt_num} (x y): ", end='', flush=True)
    
    def _compute_homography(self):
        """计算单应性矩阵"""
        if len(self.calib_image_points) < 4 or len(self.calib_ground_points) < 4:
            rospy.logerr("需要至少4个对应点")
            return False
        
        # 地面点 (x, y) -> 图像点 (u, v)
        src_pts = np.array(self.calib_ground_points, dtype=np.float32)
        dst_pts = np.array(self.calib_image_points, dtype=np.float32)
        
        # 计算单应性矩阵
        self.H, status = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
        
        if self.H is None:
            rospy.logerr("计算单应性矩阵失败")
            return False
        
        rospy.loginfo("单应性矩阵计算成功!")
        rospy.loginfo(f"H = \n{self.H}")
        
        # 计算重投影误差
        reproj_error = self._compute_reprojection_error()
        rospy.loginfo(f"重投影误差: {reproj_error:.2f} 像素")
        
        return True
    
    def _compute_reprojection_error(self):
        """计算重投影误差"""
        if self.H is None:
            return float('inf')
        
        total_error = 0
        for i, (gp, ip) in enumerate(zip(self.calib_ground_points, self.calib_image_points)):
            projected = self.project_point(gp[0], gp[1])
            if projected:
                error = np.sqrt((projected[0] - ip[0])**2 + (projected[1] - ip[1])**2)
                total_error += error
        
        return total_error / len(self.calib_ground_points)
    
    def project_point(self, x, y):
        """
        使用单应性矩阵将地面点投影到图像
        
        参数:
            x, y: 地面坐标 (base_footprint 坐标系, 米)
        
        返回:
            (u, v): 图像像素坐标，或 None 如果投影失败
        """
        if self.H is None:
            return None
        
        # 齐次坐标
        pt = np.array([x, y, 1.0], dtype=np.float64)
        
        # 投影
        result = self.H @ pt
        
        # 归一化
        if abs(result[2]) < 1e-10:
            return None
        
        u = result[0] / result[2]
        v = result[1] / result[2]
        
        return (int(round(u)), int(round(v)))
    
    def trajectory_callback(self, msg):
        """轨迹回调"""
        self.latest_trajectory = msg
        self.trajectory_stamp = rospy.Time.now()
    
    def image_callback(self, msg):
        """图像回调"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge 错误: {e}")
            return
        
        self.current_image = cv_image.copy()
        
        if self.calibration_mode:
            self._handle_calibration_mode(cv_image)
        else:
            self._handle_normal_mode(cv_image)
        
        self.frame_count += 1
    
    def _handle_calibration_mode(self, image):
        """处理标定模式"""
        # 绘制已标定的点
        for i, pt in enumerate(self.calib_image_points):
            cv2.circle(image, tuple(pt), 10, (0, 255, 0), -1)
            cv2.circle(image, tuple(pt), 10, (255, 255, 255), 2)
            cv2.putText(image, str(i + 1), (pt[0] + 15, pt[1] + 5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # 显示地面坐标
            if i < len(self.calib_ground_points):
                gp = self.calib_ground_points[i]
                text = f"({gp[0]:.2f}, {gp[1]:.2f})"
                cv2.putText(image, text, (pt[0] + 15, pt[1] + 25),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        
        # 如果有单应性矩阵，绘制网格验证
        if self.H is not None:
            self._draw_ground_grid(image)
        
        # 状态信息
        status = f"Calibration - {len(self.calib_image_points)}/4 points"
        cv2.putText(image, status, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        if len(self.calib_image_points) >= 4 and self.H is not None:
            cv2.putText(image, "Press 's' to save, 'r' to reset", (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        # 显示
        if self.show_window:
            cv2.imshow('Calibration', image)
            key = cv2.waitKey(1) & 0xFF
            self._handle_key(key)
    
    def _handle_key(self, key):
        """处理键盘输入"""
        if key == ord('s'):
            # 保存
            if self._save_calibration():
                rospy.loginfo("标定完成! 可以退出标定模式了")
        elif key == ord('r'):
            # 重置
            self.calib_image_points = []
            self.calib_ground_points = []
            self.H = None
            rospy.loginfo("已重置标定数据")
        elif key == ord('q'):
            rospy.signal_shutdown("用户退出")
    
    def _handle_normal_mode(self, image):
        """处理正常运行模式"""
        # 叠加轨迹
        if self.latest_trajectory is not None and self.H is not None:
            image = self.overlay_trajectory(image, self.latest_trajectory)
        elif self.H is None:
            cv2.putText(image, "Not calibrated! Run calibration_mode:=true", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        # 状态信息
        image = self.draw_status(image)
        
        # 发布
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(image, 'bgr8'))
        except CvBridgeError as e:
            rospy.logerr(f"发布图像错误: {e}")
        
        # 显示
        if self.show_window:
            cv2.imshow('Trajectory Overlay', image)
            cv2.waitKey(1)
    
    def _draw_ground_grid(self, image):
        """绘制地面网格 (验证标定)"""
        # 绘制 x 方向线 (每 0.5m, 从 1.0m 到 3.0m)
        for x in np.arange(1.0, 3.5, 0.5):
            pts = []
            for y in np.arange(-1.0, 1.1, 0.1):
                pt = self.project_point(x, y)
                if pt:
                    pts.append(pt)
            if len(pts) >= 2:
                for i in range(len(pts) - 1):
                    cv2.line(image, pts[i], pts[i+1], (100, 100, 100), 1)
        
        # 绘制 y 方向线 (每 0.5m)
        for y in np.arange(-1.0, 1.1, 0.5):
            pts = []
            for x in np.arange(1.0, 3.5, 0.1):
                pt = self.project_point(x, y)
                if pt:
                    pts.append(pt)
            if len(pts) >= 2:
                for i in range(len(pts) - 1):
                    cv2.line(image, pts[i], pts[i+1], (100, 100, 100), 1)
    
    def overlay_trajectory(self, image, trajectory):
        """在图像上叠加轨迹点"""
        # 投影所有轨迹点
        points_2d = []
        for pt in trajectory.points:
            projected = self.project_point(pt.x, pt.y)
            if projected:
                h, w = image.shape[:2]
                # 允许稍微超出边界
                if -50 <= projected[0] < w + 50 and -50 <= projected[1] < h + 50:
                    u = max(0, min(w - 1, projected[0]))
                    v = max(0, min(h - 1, projected[1]))
                    points_2d.append((u, v))
        
        # 绘制轨迹
        if points_2d:
            self.draw_trajectory_on_image(image, points_2d)
        
        return image
    
    def draw_trajectory_on_image(self, image, points_2d):
        """在图像上绘制轨迹"""
        if not points_2d:
            return
        
        num_points = len(points_2d)
        
        # 绘制连线
        for i in range(len(points_2d) - 1):
            ratio = i / max(num_points - 1, 1)
            color = (
                int(255 * ratio),       # B: 0 -> 255
                int(255 * (1 - ratio)), # G: 255 -> 0
                0                        # R: 0
            )
            cv2.line(image, points_2d[i], points_2d[i + 1], color, self.line_thickness)
        
        # 绘制点
        for i, pt in enumerate(points_2d):
            ratio = i / max(num_points - 1, 1)
            
            if i == 0:
                # 第一个点 - 绿色大圆
                color = (0, 255, 0)
                radius = self.point_radius + 4
            elif i == num_points - 1:
                # 最后一个点 - 蓝色
                color = (255, 100, 0)
                radius = self.point_radius + 2
            else:
                color = (
                    int(255 * ratio),
                    int(255 * (1 - ratio)),
                    0
                )
                radius = self.point_radius
            
            cv2.circle(image, pt, radius, color, -1)
            cv2.circle(image, pt, radius, (255, 255, 255), 1)
            
            # 显示点序号
            cv2.putText(image, str(i + 1), (pt[0] + 10, pt[1] - 5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
    
    def draw_status(self, image):
        """绘制状态信息"""
        h, w = image.shape[:2]
        
        # 背景
        overlay = image.copy()
        cv2.rectangle(overlay, (5, 5), (180, 50), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.5, image, 0.5, 0, image)
        
        # 状态
        traj_count = len(self.latest_trajectory.points) if self.latest_trajectory else 0
        
        cv2.putText(image, f"Frame: {self.frame_count}", (10, 25),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.putText(image, f"Traj Points: {traj_count}", (10, 45),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        return image
    
    def run(self):
        """运行"""
        if self.calibration_mode:
            rospy.loginfo("进入标定模式...")
            self._run_calibration()
        else:
            rospy.loginfo("TrajectoryVisualizer 正在运行...")
            rospy.spin()
        
        if self.show_window:
            cv2.destroyAllWindows()
    
    def _run_calibration(self):
        """运行标定流程"""
        rate = rospy.Rate(30)
        
        while not rospy.is_shutdown():
            # 检查是否需要输入地面坐标
            if len(self.calib_image_points) > len(self.calib_ground_points):
                try:
                    # 非阻塞检查输入
                    import select
                    import sys
                    if select.select([sys.stdin], [], [], 0.0)[0]:
                        line = sys.stdin.readline().strip()
                        if line:
                            parts = line.split()
                            if len(parts) >= 2:
                                x, y = float(parts[0]), float(parts[1])
                                self.calib_ground_points.append([x, y])
                                rospy.loginfo(f"地面坐标: ({x}, {y})")
                                
                                # 如果有4个点，计算单应性矩阵
                                if len(self.calib_ground_points) >= 4:
                                    self._compute_homography()
                            else:
                                rospy.logwarn("请输入两个数字: x y")
                except Exception as e:
                    pass
            
            rate.sleep()


def main():
    try:
        visualizer = TrajectoryVisualizer()
        visualizer.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"TrajectoryVisualizer 异常: {e}")
        raise


if __name__ == '__main__':
    main()
