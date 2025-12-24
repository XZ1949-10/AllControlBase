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
import tf2_ros
import tf2_geometry_msgs
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from controller_ros.msg import LocalTrajectoryV4

class TrajectoryVisualizer:
    """轨迹可视化器 - 在图像上叠加轨迹点"""
    
    def __init__(self):
        rospy.init_node('trajectory_visualizer', anonymous=False)
        
        # 参数
        self.image_topic = rospy.get_param('~image_topic', '/usb_cam/image_raw')
        self.trajectory_topic = rospy.get_param('~trajectory_topic', '/nn/local_trajectory')
        self.output_topic = rospy.get_param('~output_topic', '/trajectory_overlay/image')
        self.camera_frame = rospy.get_param('~camera_frame', 'usb_cam')
        self.base_frame = rospy.get_param('~base_frame', 'base_footprint')
        
        # 相机内参 (估算值，640x480 USB 摄像头)
        self.fx = rospy.get_param('~fx', 500.0)
        self.fy = rospy.get_param('~fy', 500.0)
        self.cx = rospy.get_param('~cx', 320.0)
        self.cy = rospy.get_param('~cy', 240.0)
        
        # 显示参数
        self.show_window = rospy.get_param('~show_window', True)
        self.point_radius = rospy.get_param('~point_radius', 8)
        self.line_thickness = rospy.get_param('~line_thickness', 3)
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
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
        rospy.loginfo(f"  相机坐标系: {self.camera_frame}")
        rospy.loginfo(f"  基准坐标系: {self.base_frame}")
        rospy.loginfo(f"  显示窗口: {self.show_window}")
    
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
        """在图像上叠加轨迹点 - 简化版，直接使用2D投影"""
        # 直接使用简化的2D投影，起始点固定在底部中间
        points_2d = self.project_points_manual(trajectory.points, image.shape)
        
        # 绘制轨迹
        if points_2d:
            self.draw_trajectory_on_image(image, points_2d)
        
        return image
    
    def project_points_manual(self, points, image_shape):
        """手动投影 - 简化版，起始点固定在底部中间"""
        h, w = image_shape[:2]
        points_2d = []
        
        # 图像坐标系参数
        # 起始点 (0,0) 固定在底部中间
        origin_u = w // 2  # 水平中心
        origin_v = h - 30  # 底部留一点边距
        
        # 缩放因子 (米 -> 像素)
        # 调整这个值来改变轨迹在图像中的大小
        scale_x = 300  # 前方距离缩放 (越大轨迹越往上)
        scale_y = 300  # 左右距离缩放 (越大轨迹越宽)
        
        for pt in points:
            # 轨迹坐标系: x前方, y左方
            # 图像坐标系: u右方, v下方
            # 转换: x前 -> v上 (减小), y左 -> u左 (减小)
            
            u = int(origin_u - pt.y * scale_y)  # y左为正 -> u减小
            v = int(origin_v - pt.x * scale_x)  # x前为正 -> v减小 (往上)
            
            # 限制在图像范围内
            u = max(0, min(w - 1, u))
            v = max(0, min(h - 1, v))
            
            points_2d.append((u, v))
        
        return points_2d
    
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
        status_lines = [
            f"Frame: {self.frame_count}",
            f"Traj points: {len(self.latest_trajectory.points) if self.latest_trajectory else 0}",
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
