#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
轨迹发布器 - 订阅 /waypoint 话题并转换为 LocalTrajectoryV4 消息

功能:
- 订阅 /waypoint (std_msgs/Float32MultiArray)，包含 8 个点的 [x, y] 坐标
- 转换为 controller_ros/LocalTrajectoryV4 格式
- 发布到 /nn/local_trajectory 话题

使用方法:
    rosrun controller_ros trajectory_publisher.py
    rosrun controller_ros trajectory_publisher.py --stop  # 发布停止轨迹

话题:
    订阅: /waypoint (std_msgs/Float32MultiArray)
           data 格式: [x0, y0, x1, y1, x2, y2, ..., x7, y7] (16 个 float)
    发布: /nn/local_trajectory (controller_ros/LocalTrajectoryV4)

重要:
    - 输入轨迹坐标系假设为 base_link (机器人当前位置为原点，X轴朝前)
    - 差速车只使用 x, y 坐标
"""

# 注意：不要在这里修改 sys.path！
# PYTHONPATH 已经由 source devel/setup.bash 正确设置
# 手动修改 sys.path 会导致 controller_ros.msg 找不到

import rospy
import numpy as np
from geometry_msgs.msg import Point
from std_msgs.msg import Header, Float32MultiArray

try:
    from controller_ros.msg import LocalTrajectoryV4
except ImportError:
    rospy.logerr("无法导入 controller_ros.msg，请确保 controller_ros 已编译")
    raise

# 导入轨迹模式枚举，保持与 universal_controller 一致
try:
    from universal_controller.core.enums import TrajectoryMode
    MODE_TRACK = TrajectoryMode.MODE_TRACK.value      # 跟踪模式 (正常跟踪轨迹)
    MODE_STOP = TrajectoryMode.MODE_STOP.value        # 停止模式 (减速停车)
    MODE_HOVER = TrajectoryMode.MODE_HOVER.value      # 悬停模式 (仅四旋翼)
    MODE_EMERGENCY = TrajectoryMode.MODE_EMERGENCY.value  # 紧急模式 (立即停止)
except ImportError:
    # 如果 universal_controller 不可用，使用本地定义
    MODE_TRACK = 0
    MODE_STOP = 1
    MODE_HOVER = 2
    MODE_EMERGENCY = 3


# =============================================================================
# 轨迹发布器
# =============================================================================
class TrajectoryPublisher:
    """
    轨迹发布器类
    
    订阅 /waypoint 话题，转换为 LocalTrajectoryV4 格式发布
    """
    
    def __init__(self):
        rospy.init_node('trajectory_publisher', anonymous=False)
        
        # 参数
        self.dt = rospy.get_param('~dt', 0.1)  # 轨迹点时间间隔 (秒)
        self.confidence = rospy.get_param('~confidence', 1.0)  # 默认置信度
        self.soft_enabled = rospy.get_param('~soft_enabled', False)  # 是否启用 Soft 约束
        
        input_topic = rospy.get_param('~input_topic', '/waypoint')
        output_topic = rospy.get_param('~output_topic', '/nn/local_trajectory')
        
        # 发布器
        self.pub = rospy.Publisher(output_topic, LocalTrajectoryV4, queue_size=1)
        
        # 订阅器
        self.sub = rospy.Subscriber(input_topic, Float32MultiArray, self.waypoint_callback)
        
        # 状态
        self.last_waypoint = None
        self.publish_count = 0
        self.receive_count = 0
        
        rospy.loginfo("=" * 50)
        rospy.loginfo("TrajectoryPublisher 已启动")
        rospy.loginfo(f"  订阅话题: {input_topic} (Float32MultiArray)")
        rospy.loginfo(f"  发布话题: {output_topic} (LocalTrajectoryV4)")
        rospy.loginfo(f"  轨迹时间间隔: {self.dt} s")
        rospy.loginfo(f"  Soft 约束: {'启用' if self.soft_enabled else '禁用'}")
        rospy.loginfo("=" * 50)

    def waypoint_callback(self, msg: Float32MultiArray):
        """
        /waypoint 话题回调
        
        Args:
            msg: Float32MultiArray，data 格式为 [x0, y0, x1, y1, ..., xN, yN]
        """
        self.receive_count += 1
        
        # 解析数据
        data = msg.data
        if len(data) < 2:
            rospy.logwarn(f"收到空轨迹数据，忽略")
            return
        
        # 检查数据长度是否为偶数 (每个点有 x, y)
        if len(data) % 2 != 0:
            rospy.logwarn(f"轨迹数据长度 {len(data)} 不是偶数，忽略最后一个值")
            data = data[:-1]
        
        # 解析为点列表
        num_points = len(data) // 2
        positions = np.array(data).reshape(num_points, 2)
        
        # 创建并发布消息
        traj_msg = self.create_trajectory_msg(positions)
        self.pub.publish(traj_msg)
        
        self.publish_count += 1
        self.last_waypoint = positions
        
        # 日志 (每 50 次打印一次)
        if self.publish_count % 50 == 0:
            rospy.loginfo(f"已转发 {self.publish_count} 条轨迹 (当前 {num_points} 个点)")

    def create_trajectory_msg(self, positions: np.ndarray, 
                               velocities: np.ndarray = None,
                               mode: int = MODE_TRACK) -> LocalTrajectoryV4:
        """
        创建 LocalTrajectoryV4 消息
        
        Args:
            positions: numpy array, shape (N, 2), [x, y] 坐标 (base_link 坐标系)
            velocities: numpy array, shape (N, 2) 或 (N, 4), 可选
            mode: 轨迹模式
        
        Returns:
            LocalTrajectoryV4 消息
        """
        msg = LocalTrajectoryV4()
        
        # Header
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_link"  # 差速车使用 base_link 坐标系
        
        # 轨迹模式
        msg.mode = mode
        
        # 轨迹点 (差速车只用 x, y，z 设为 0)
        msg.points = []
        for i in range(len(positions)):
            p = Point()
            p.x = float(positions[i, 0])
            p.y = float(positions[i, 1])
            p.z = 0.0
            msg.points.append(p)
        
        # 速度 (可选，差速车通常不需要)
        if velocities is not None and len(velocities) > 0 and self.soft_enabled:
            msg.velocities_flat = []
            for i in range(len(velocities)):
                if velocities.shape[1] >= 4:
                    msg.velocities_flat.extend([
                        float(velocities[i, 0]),  # vx
                        float(velocities[i, 1]),  # vy
                        float(velocities[i, 2]),  # vz
                        float(velocities[i, 3]),  # wz
                    ])
                else:
                    # 只有 [vx, vy]，补零
                    msg.velocities_flat.extend([
                        float(velocities[i, 0]),
                        float(velocities[i, 1]),
                        0.0, 0.0,
                    ])
            msg.soft_enabled = True
        else:
            msg.velocities_flat = []
            msg.soft_enabled = False
        
        # 时间间隔
        msg.dt_sec = self.dt
        
        # 置信度
        msg.confidence = self.confidence
        
        return msg

    def run(self):
        """主循环 (spin)"""
        rospy.loginfo("TrajectoryPublisher 正在运行，等待 /waypoint 数据...")
        rospy.spin()


# =============================================================================
# 停止轨迹发布器 (用于紧急停止)
# =============================================================================
class StopTrajectoryPublisher:
    """发布停止轨迹"""
    
    def __init__(self):
        rospy.init_node('stop_trajectory_publisher', anonymous=True)
        output_topic = rospy.get_param('~output_topic', '/nn/local_trajectory')
        self.pub = rospy.Publisher(output_topic, LocalTrajectoryV4, queue_size=1)
        rospy.sleep(0.5)  # 等待连接
    
    def publish_stop(self):
        """发布停止轨迹"""
        msg = LocalTrajectoryV4()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_link"
        msg.mode = MODE_STOP
        msg.points = [Point(x=0.0, y=0.0, z=0.0)]
        msg.velocities_flat = []
        msg.soft_enabled = False
        msg.dt_sec = 0.1
        msg.confidence = 1.0
        
        self.pub.publish(msg)
        rospy.loginfo("已发布停止轨迹")


# =============================================================================
# 主函数
# =============================================================================
def main():
    try:
        publisher = TrajectoryPublisher()
        publisher.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"TrajectoryPublisher 异常: {e}")
        raise


def main_stop():
    """发布停止轨迹的入口函数"""
    try:
        publisher = StopTrajectoryPublisher()
        publisher.publish_stop()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"StopTrajectoryPublisher 异常: {e}")
        raise


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='轨迹发布器')
    parser.add_argument('--stop', action='store_true', help='发布停止轨迹')
    args = parser.parse_args()
    
    if args.stop:
        main_stop()
    else:
        main()
