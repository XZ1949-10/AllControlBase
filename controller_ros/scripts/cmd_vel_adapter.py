#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
cmd_vel 适配器 - 将 UnifiedCmd 转换为 TurtleBot 的 cmd_vel

功能:
- 订阅 /cmd_unified (controller_ros/UnifiedCmd)
- 发布 /cmd_vel (geometry_msgs/Twist)
- 应用速度限制保护机器人

使用方法:
    rosrun controller_ros cmd_vel_adapter.py
    
    或通过 launch 文件启动:
    roslaunch controller_ros turtlebot1.launch

参数:
    ~max_linear: 最大线速度 (m/s), 默认 0.5
    ~max_angular: 最大角速度 (rad/s), 默认 1.0
    ~input_topic: 输入话题, 默认 /cmd_unified
    ~output_topic: 输出话题, 默认 /cmd_vel
"""

# 注意：不要在这里修改 sys.path！
# PYTHONPATH 已经由 source devel/setup.bash 正确设置

import rospy
from geometry_msgs.msg import Twist

try:
    from controller_ros.msg import UnifiedCmd
except ImportError:
    rospy.logerr("无法导入 controller_ros.msg，请确保 controller_ros 已编译")
    raise


class CmdVelAdapter:
    """cmd_vel 适配器类"""
    
    def __init__(self):
        rospy.init_node('cmd_vel_adapter', anonymous=False)
        
        # 获取参数
        self.max_linear = rospy.get_param('~max_linear', 0.5)
        self.max_angular = rospy.get_param('~max_angular', 1.0)
        input_topic = rospy.get_param('~input_topic', '/cmd_unified')
        output_topic = rospy.get_param('~output_topic', '/cmd_vel')
        
        # 订阅控制器输出
        self.sub = rospy.Subscriber(input_topic, UnifiedCmd, self.callback, queue_size=1)
        
        # 发布 cmd_vel
        self.pub = rospy.Publisher(output_topic, Twist, queue_size=1)
        
        # 统计
        self.msg_count = 0
        self.clamp_count = 0
        
        rospy.loginfo(f"CmdVelAdapter 已启动:")
        rospy.loginfo(f"  输入话题: {input_topic}")
        rospy.loginfo(f"  输出话题: {output_topic}")
        rospy.loginfo(f"  最大线速度: {self.max_linear} m/s")
        rospy.loginfo(f"  最大角速度: {self.max_angular} rad/s")

    def callback(self, msg: UnifiedCmd):
        """
        处理 UnifiedCmd 消息，转换为 Twist 并发布
        
        Args:
            msg: UnifiedCmd 消息，包含 vx, vy, vz, omega 等字段
        """
        twist = Twist()
        
        # 差速车只使用 vx 和 omega
        # 应用速度限制
        linear_x = self._clamp(msg.vx, -self.max_linear, self.max_linear)
        angular_z = self._clamp(msg.omega, -self.max_angular, self.max_angular)
        
        # 检查是否被限制
        if abs(linear_x - msg.vx) > 0.001 or abs(angular_z - msg.omega) > 0.001:
            self.clamp_count += 1
            if self.clamp_count % 100 == 1:  # 每 100 次打印一次警告
                rospy.logwarn(f"速度被限制: vx {msg.vx:.2f}->{linear_x:.2f}, "
                             f"omega {msg.omega:.2f}->{angular_z:.2f}")
        
        # 设置 Twist 消息
        twist.linear.x = linear_x
        twist.linear.y = 0.0  # 差速车不支持横向移动
        twist.linear.z = 0.0
        
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = angular_z
        
        # 发布
        self.pub.publish(twist)
        
        self.msg_count += 1
        if self.msg_count % 200 == 0:  # 每 200 条消息打印一次状态
            rospy.logdebug(f"已处理 {self.msg_count} 条消息, "
                          f"限制次数: {self.clamp_count}")
    
    def _clamp(self, value: float, min_val: float, max_val: float) -> float:
        """限制值在指定范围内"""
        return max(min_val, min(max_val, value))
    
    def run(self):
        """运行适配器"""
        rospy.loginfo("CmdVelAdapter 正在运行...")
        rospy.spin()


def main():
    try:
        adapter = CmdVelAdapter()
        adapter.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"CmdVelAdapter 异常: {e}")
        raise


if __name__ == '__main__':
    main()
