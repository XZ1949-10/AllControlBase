#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
轨迹可视化节点 - 在相机图像上叠加显示网络输出的轨迹点

功能:
- 订阅相机图像和轨迹数据
- 将轨迹点从 base_footprint 坐标系投影到图像平面
- 实时显示叠加后的图像

使用方法:
    roslaunch controller_ros trajectory_visualizer.launch
    或
    rosrun controller_ros trajectory_visualizer.py
"""

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from controller_ros.msg import LocalTrajectoryV4

class TrajectoryVisualizer:
    """轨迹可视化器 - 在图像上叠加轨迹点"""
    
    def __init__(self):
        rospy.init_node('trajectory_visualizer', anonymous=False)
        
        # 参数
        self.image_topic = rospy.get_param('~image_topic', '/usb_cam/image_raw')
        self.trajectory_topic = rospy.get_param('~trajectory_topic', '/nn/local_trajectory')
        self.output_topic = rospy.get_param('~output_topic', '/trajectory_overlay/image')
        
        # 相机内参
        self.fx = rospy.get_param('~fx', 525.0)
        self.fy = rospy.get_param('~fy', 525.0)
        self.cx = rospy.get_param('~cx', 319.5)
        self.cy = rospy.get_param('~cy', 239.5)
        
        # 畸变系数 (k1, k2, p1, p2, k3)
        self.dist_coeffs = np.array([
            rospy.get_param('~k1', 0.0),
            rospy.get_param('~k2', 0.0),
            rospy.get_param('~p1', 0.0),
            rospy.get_param('~p2', 0.0),
            rospy.get_param('~k3', 0.0)
        ], dtype=np.float64)
        
        # 相机外参 (相对于 base_footprint)
        self.cam_x = rospy.get_param('~cam_x', 0.08)    # 前方距离 (m)
        self.cam_y = rospy.get_param('~cam_y', 0.0)     # 左右偏移 (m)
        self.cam_z = rospy.get_param('~cam_z', 0.50)    # 高度 (m)
        self.cam_pitch = rospy.get_param('~cam_pitch', 0.52)  # 俯仰角 (rad, 正值向下看)
        
        # 构建相机内参矩阵
        self.camera_matrix = np.array([
            [self.fx, 0, self.cx],
            [0, self.fy, self.cy],
            [0, 0, 1]
        ], dtype=np.float64)
        
        # 预计算外参矩阵
        self._build_extrinsic_matrix()
        
        # 显示参数
        self.show_window = rospy.get_param('~show_window', True)
        self.point_radius = rospy.get_param('~point_radius', 8)
        self.line_thickness = rospy.get_param('~line_thickness', 3)
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # 数据缓存
        self.latest_trajectory = None
        self.trajectory_stamp = None
        
        # 订阅
        self.image_sub = rospy.Subscriber(
            self.image_topic, Image, self.image_callback, queue_size=1
        )
        self.traj_sub = rospy.Subscriber(
            self.trajectory_topic, LocalTrajectoryV4, self.trajectory_callback, queue_size=1
        )
        
        # 发布
        self.image_pub = rospy.Publisher(self.output_topic, Image, queue_size=1)
        
        # 统计
        self.frame_count = 0
        
        rospy.loginfo(f"TrajectoryVisualizer 已启动:")
        rospy.loginfo(f"  图像输入: {self.image_topic}")
        rospy.loginfo(f"  轨迹输入: {self.trajectory_topic}")
        rospy.loginfo(f"  图像输出: {self.output_topic}")
        rospy.loginfo(f"  显示窗口: {self.show_window}")
        rospy.loginfo(f"  相机内参: fx={self.fx}, fy={self.fy}, cx={self.cx}, cy={self.cy}")
        rospy.loginfo(f"  畸变系数: {self.dist_coeffs}")
        rospy.loginfo(f"  相机外参: x={self.cam_x}, y={self.cam_y}, z={self.cam_z}, pitch={self.cam_pitch}")
    
    def _build_extrinsic_matrix(self):
        """构建外参矩阵 (base_footprint → camera_optical)"""
        # 坐标系转换: base(x前,y左,z上) → cam_optical(x右,y下,z前)
        R_base_to_cam = np.array([
            [0, -1, 0],   # cam_x = -base_y
            [0, 0, -1],   # cam_y = -base_z
            [1, 0, 0]     # cam_z = base_x
        ], dtype=np.float64)
        
        # 俯仰旋转 (绕相机 x 轴)
        cos_p = np.cos(self.cam_pitch)
        sin_p = np.sin(self.cam_pitch)
        R_pitch = np.array([
            [1, 0, 0],
            [0, cos_p, -sin_p],
            [0, sin_p, cos_p]
        ], dtype=np.float64)
        
        # 组合旋转
        self.R_cam_base = R_pitch @ R_base_to_cam
        
        # 相机位置
        self.t_cam_in_base = np.array([self.cam_x, self.cam_y, self.cam_z], dtype=np.float64)
        
        # 计算 rvec (旋转向量) 用于 cv2.projectPoints
        self.rvec, _ = cv2.Rodrigues(self.R_cam_base)
        
        # 计算 tvec (平移向量): t = -R @ t_cam
        self.tvec = -self.R_cam_base @ self.t_cam_in_base
    
    def trajectory_callback(self, msg):
        """轨迹回调"""
        self.latest_trajectory = msg
        self.trajectory_stamp = rospy.Time.now()
    
    def image_callback(self, msg):
        """图像回调 - 主处理函数"""
        try:
            # 转换图像
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge 错误: {e}")
            return
        
        # 叠加轨迹
        if self.latest_trajectory is not None:
            cv_image = self.overlay_trajectory(cv_image, self.latest_trajectory)
        
        # 添加状态信息
        cv_image = self.draw_status(cv_image)
        
        # 发布
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, 'bgr8'))
        except CvBridgeError as e:
            rospy.logerr(f"发布图像错误: {e}")
        
        # 显示窗口
        if self.show_window:
            cv2.imshow('Trajectory Overlay', cv_image)
            cv2.waitKey(1)
        
        self.frame_count += 1
    
    def overlay_trajectory(self, image, trajectory):
        """在图像上叠加轨迹点 (真实 3D 投影)"""
        # 构建完整轨迹: 原点 (0,0,0) + 网络输出的 8 个点
        # 所有点都在地面 z=0 (地面机器人)
        points_3d = [(0.0, 0.0, 0.0)]  # 原点
        for pt in trajectory.points:
            points_3d.append((pt.x, pt.y, 0.0))  # z=0 地面
        
        # 统一投影所有点
        all_points_2d = self.project_points_opencv(points_3d, image.shape)
        
        # 绘制轨迹
        if all_points_2d:
            self.draw_trajectory_on_image(image, all_points_2d)
        
        return image
    
    def project_points_opencv(self, points_3d, image_shape):
        """
        使用 OpenCV projectPoints 进行精准投影
        
        参数:
            points_3d: list of (x, y, z) tuples，在 base_footprint 坐标系中
            image_shape: 图像尺寸
        
        返回:
            list of (u, v) 像素坐标
        """
        h, w = image_shape[:2]
        
        if not points_3d:
            return []
        
        # 构建 3D 点数组 (N x 3)
        points_array = np.array(points_3d, dtype=np.float64)
        
        # 使用 OpenCV projectPoints (自动处理畸变)
        points_2d, _ = cv2.projectPoints(
            points_array,
            self.rvec,
            self.tvec,
            self.camera_matrix,
            self.dist_coeffs
        )
        
        # 转换格式并过滤
        result = []
        for i, pt_2d in enumerate(points_2d):
            u, v = pt_2d[0]
            
            # 检查点是否在相机前方
            P_cam = self.R_cam_base @ (points_array[i] - self.t_cam_in_base)
            if P_cam[2] <= 0.01:  # 点在相机后方
                continue
            
            u_int = int(round(u))
            v_int = int(round(v))
            
            # 允许稍微超出边界，但裁剪到图像范围
            if -100 <= u_int < w + 100 and -100 <= v_int < h + 100:
                u_int = max(0, min(w - 1, u_int))
                v_int = max(0, min(h - 1, v_int))
                result.append((u_int, v_int))
        
        return result
    
    def draw_trajectory_on_image(self, image, points_2d, depths=None):
        """在图像上绘制轨迹"""
        if not points_2d:
            return
        
        # 颜色渐变: 近处绿色 -> 远处红色
        num_points = len(points_2d)
        
        # 绘制连线
        for i in range(len(points_2d) - 1):
            # 颜色渐变
            ratio = i / max(num_points - 1, 1)
            color = (
                int(255 * ratio),      # B: 0 -> 255
                int(255 * (1 - ratio)), # G: 255 -> 0
                0                       # R: 0
            )
            cv2.line(image, points_2d[i], points_2d[i + 1], color, self.line_thickness)
        
        # 绘制点
        for i, pt in enumerate(points_2d):
            ratio = i / max(num_points - 1, 1)
            
            if i == 0:
                # 第一个点 (当前位置) - 红色大圆
                color = (0, 0, 255)
                radius = self.point_radius + 4
            else:
                # 其他点 - 渐变色
                color = (
                    int(255 * ratio),
                    int(255 * (1 - ratio)),
                    0
                )
                radius = self.point_radius
            
            cv2.circle(image, pt, radius, color, -1)
            cv2.circle(image, pt, radius, (255, 255, 255), 1)  # 白色边框
            
            # 显示点序号
            cv2.putText(image, str(i), (pt[0] + 10, pt[1] - 5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
    
    def draw_status(self, image):
        """绘制状态信息"""
        h, w = image.shape[:2]
        
        # 背景半透明矩形
        overlay = image.copy()
        cv2.rectangle(overlay, (5, 5), (250, 80), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.5, image, 0.5, 0, image)
        
        # 状态文字
        traj_count = len(self.latest_trajectory.points) if self.latest_trajectory else 0
        status_lines = [
            f"Frame: {self.frame_count}",
            f"Traj points: {traj_count + 1} (origin + {traj_count})",  # 原点 + 网络输出
            f"Mode: {'TRACK' if self.latest_trajectory and self.latest_trajectory.mode == 1 else 'STOP' if self.latest_trajectory else 'N/A'}"
        ]
        
        y_offset = 25
        for line in status_lines:
            cv2.putText(image, line, (10, y_offset),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            y_offset += 20
        
        return image
    
    def run(self):
        """运行"""
        rospy.loginfo("TrajectoryVisualizer 正在运行...")
        rospy.spin()
        
        # 清理
        if self.show_window:
            cv2.destroyAllWindows()


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
