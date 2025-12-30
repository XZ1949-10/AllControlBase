#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
cmd_vel 适配器 - 将 UnifiedCmd 转换为 TurtleBot 的 cmd_vel

功能:
- 订阅 /cmd_unified (controller_ros/UnifiedCmd) - 控制器输出
- 订阅 /joy_cmd_vel (geometry_msgs/Twist) - 手柄控制输出
- 订阅 /visualizer/control_mode (std_msgs/Bool) - 控制模式切换
- 发布 /cmd_vel (geometry_msgs/Twist)
- 应用速度限制保护机器人
- 以固定频率发布命令，确保底盘不会因超时停止

架构说明:
- 本节点是平台适配器，负责命令的最终处理和发布
- 以固定频率 (publish_rate) 发布命令，解决底盘 cmd_vel 超时问题
- 手柄命令超时检测在本节点完成，确保手柄断开时机器人停止

控制模式:
- control_mode = False: 使用控制器输出 (/cmd_unified)
- control_mode = True:  使用手柄输出 (/joy_cmd_vel)

参数:
    ~max_linear: 最大线速度 (m/s), 默认 0.5
    ~max_angular: 最大角速度 (rad/s), 默认 1.0
    ~max_linear_accel: 最大线加速度 (m/s²), 默认 0 (禁用)
    ~max_angular_accel: 最大角加速度 (rad/s²), 默认 0 (禁用)
    ~publish_rate: 发布频率 (Hz), 默认 20
    ~joy_timeout: 手柄命令超时 (秒), 默认 0.5
"""

import rospy
import math
from typing import Optional
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

try:
    from controller_ros.msg import UnifiedCmd
except ImportError:
    rospy.logerr("无法导入 controller_ros.msg，请确保 controller_ros 已编译")
    raise


class CmdVelAdapter:
    """
    cmd_vel 适配器类
    
    职责:
    - 根据控制模式选择命令源 (控制器/手柄)
    - 应用速度限制保护机器人
    - 以固定频率发布 cmd_vel 到底盘
    - 处理手柄命令超时
    """
    
    def __init__(self):
        rospy.init_node('cmd_vel_adapter', anonymous=False)
        
        # 关闭标志
        self._shutting_down = False
        
        # 从配置文件读取参数
        # 优先级: 私有参数 > 命名空间参数 > 默认值
        def get_param(namespace, name, default):
            """获取参数，支持多级命名空间回退"""
            # 先尝试私有参数
            value = rospy.get_param(f'~{name}', None)
            if value is not None:
                return value
            # 再尝试指定命名空间
            value = rospy.get_param(f'{namespace}/{name}', None)
            if value is not None:
                return value
            return default
        
        # 速度限制统一从 constraints 读取，确保与控制器一致
        self.max_linear = get_param('constraints', 'v_max', 0.5)
        self.max_angular = get_param('constraints', 'omega_max', 1.0)
        
        # cmd_vel_adapter 特有参数
        self.max_linear_accel = get_param('cmd_vel_adapter', 'max_linear_accel', 0.0)
        self.max_angular_accel = get_param('cmd_vel_adapter', 'max_angular_accel', 0.0)
        self.publish_rate = get_param('cmd_vel_adapter', 'publish_rate', 20.0)
        self.joy_timeout = get_param('cmd_vel_adapter', 'joy_timeout', 0.5)
        
        input_topic = get_param('cmd_vel_adapter', 'input_topic', '/cmd_unified')
        joy_topic = get_param('cmd_vel_adapter', 'joy_topic', '/joy_cmd_vel')
        mode_topic = get_param('cmd_vel_adapter', 'mode_topic', '/visualizer/control_mode')
        output_topic = get_param('cmd_vel_adapter', 'output_topic', '/cmd_vel')
        
        # 控制模式: False=控制器, True=手柄
        self.joystick_mode = False
        
        # 最新的命令
        self.latest_controller_cmd = Twist()
        self.latest_joy_cmd = Twist()
        self.controller_cmd_time: Optional[rospy.Time] = None
        self.joy_cmd_time: Optional[rospy.Time] = None
        
        # 订阅控制器输出
        self.sub = rospy.Subscriber(input_topic, UnifiedCmd, self._controller_callback, queue_size=1)
        
        # 订阅手柄输出
        self.joy_sub = rospy.Subscriber(joy_topic, Twist, self._joy_callback, queue_size=1)
        
        # 订阅模式切换
        self.mode_sub = rospy.Subscriber(mode_topic, Bool, self._mode_callback, queue_size=1)
        
        # 发布 cmd_vel
        self.pub = rospy.Publisher(output_topic, Twist, queue_size=1)
        
        # 状态（用于速度变化率限制）
        self.last_linear_x = 0.0
        self.last_angular_z = 0.0
        self.last_time: Optional[rospy.Time] = None
        
        # 统计
        self.msg_count = 0
        self.clamp_count = 0
        self.rate_limit_count = 0
        
        # 注册关闭回调
        rospy.on_shutdown(self.shutdown)
        
        # 定时器：以固定频率发布命令
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self._timer_callback)
        
        # 重置服务
        from std_srvs.srv import Empty, EmptyResponse
        self.reset_srv = rospy.Service('~reset', Empty, self._reset_service_callback)
        
        rospy.loginfo(f"CmdVelAdapter 已启动:")
        rospy.loginfo(f"  控制器输入话题: {input_topic}")
        rospy.loginfo(f"  手柄输入话题: {joy_topic}")
        rospy.loginfo(f"  模式切换话题: {mode_topic}")
        rospy.loginfo(f"  输出话题: {output_topic}")
        rospy.loginfo(f"  发布频率: {self.publish_rate} Hz")
        rospy.loginfo(f"  手柄超时: {self.joy_timeout} s")
        rospy.loginfo(f"  最大线速度: {self.max_linear} m/s")
        rospy.loginfo(f"  最大角速度: {self.max_angular} rad/s")
        rospy.loginfo(f"  重置服务: ~reset")
        if self.max_linear_accel > 0:
            rospy.loginfo(f"  最大线加速度: {self.max_linear_accel} m/s² (启用)")
        if self.max_angular_accel > 0:
            rospy.loginfo(f"  最大角加速度: {self.max_angular_accel} rad/s² (启用)")
    
    def shutdown(self):
        """关闭适配器，发布零速度"""
        if self._shutting_down:
            return
        self._shutting_down = True
        
        rospy.loginfo("CmdVelAdapter 正在关闭，发布零速度...")
        
        # 停止定时器
        if hasattr(self, 'timer'):
            self.timer.shutdown()
        
        # 发布零速度
        try:
            stop_cmd = Twist()
            self.pub.publish(stop_cmd)
        except Exception as e:
            rospy.logwarn(f"关闭时发布零速度失败: {e}")
    
    def reset(self):
        """重置内部状态"""
        self.last_linear_x = 0.0
        self.last_angular_z = 0.0
        self.last_time = None
        self.joystick_mode = False
        self.latest_controller_cmd = Twist()
        self.latest_joy_cmd = Twist()
        self.controller_cmd_time = None
        self.joy_cmd_time = None
        self.msg_count = 0
        self.clamp_count = 0
        self.rate_limit_count = 0
        rospy.loginfo("CmdVelAdapter 状态已重置")
    
    def _reset_service_callback(self, req):
        """重置服务回调"""
        from std_srvs.srv import EmptyResponse
        self.reset()
        return EmptyResponse()
    
    def _mode_callback(self, msg: Bool):
        """模式切换回调"""
        if msg.data != self.joystick_mode:
            self.joystick_mode = msg.data
            mode_name = "手柄控制" if self.joystick_mode else "网络轨迹控制"
            rospy.loginfo(f"控制模式切换: {mode_name}")
    
    def _joy_callback(self, msg: Twist):
        """手柄命令回调"""
        self.latest_joy_cmd = msg
        self.joy_cmd_time = rospy.Time.now()
    
    def _controller_callback(self, msg: UnifiedCmd):
        """控制器命令回调"""
        twist = Twist()
        twist.linear.x = msg.vx
        twist.angular.z = msg.omega
        self.latest_controller_cmd = twist
        self.controller_cmd_time = rospy.Time.now()
    
    def _timer_callback(self, event):
        """
        定时器回调：以固定频率发布命令
        
        解决问题：TurtleBot 底盘有 cmd_vel 超时保护，
        如果一段时间没收到命令就会停止。
        """
        if self._shutting_down:
            return
        
        linear_x = 0.0
        angular_z = 0.0
        
        if self.joystick_mode:
            # 手柄模式：检查手柄命令是否超时
            if self.joy_cmd_time is not None:
                time_since_joy = (rospy.Time.now() - self.joy_cmd_time).to_sec()
                if time_since_joy <= self.joy_timeout:
                    linear_x = self.latest_joy_cmd.linear.x
                    angular_z = self.latest_joy_cmd.angular.z
                # 超时则保持零速度
        else:
            # 控制器模式：使用最新的控制器命令
            linear_x = self.latest_controller_cmd.linear.x
            angular_z = self.latest_controller_cmd.angular.z
        
        self._publish_twist(linear_x, angular_z)
    
    def _publish_twist(self, linear_x: float, angular_z: float):
        """发布 Twist 消息"""
        if self._shutting_down:
            return
        
        twist = Twist()
        original_linear_x = linear_x
        original_angular_z = angular_z
        
        # 数据有效性检查
        if not (math.isfinite(linear_x) and math.isfinite(angular_z)):
            rospy.logwarn_throttle(1.0, 
                f"收到无效速度命令: vx={linear_x}, omega={angular_z}，发布零速度")
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.pub.publish(twist)
            self.last_linear_x = 0.0
            self.last_angular_z = 0.0
            return
        
        # 应用速度变化率限制
        if self.max_linear_accel > 0 or self.max_angular_accel > 0:
            linear_x, angular_z = self._apply_rate_limit(linear_x, angular_z)
        
        # 应用速度限制
        linear_x_clamped = self._clamp(linear_x, -self.max_linear, self.max_linear)
        angular_z_clamped = self._clamp(angular_z, -self.max_angular, self.max_angular)
        
        # 检查是否被限制
        if abs(linear_x_clamped - original_linear_x) > 0.001 or abs(angular_z_clamped - original_angular_z) > 0.001:
            self.clamp_count += 1
            if self.clamp_count % 100 == 1:
                rospy.logwarn(f"速度被限制: vx {original_linear_x:.2f}->{linear_x_clamped:.2f}, "
                             f"omega {original_angular_z:.2f}->{angular_z_clamped:.2f}")
        
        # 更新状态
        self.last_linear_x = linear_x_clamped
        self.last_angular_z = angular_z_clamped
        
        # 设置并发布
        twist.linear.x = linear_x_clamped
        twist.angular.z = angular_z_clamped
        self.pub.publish(twist)
        
        self.msg_count += 1
        if self.msg_count % 200 == 0:
            rospy.logdebug(f"已处理 {self.msg_count} 条消息")
    
    def _apply_rate_limit(self, linear_x: float, angular_z: float) -> tuple:
        """应用速度变化率限制"""
        now = rospy.Time.now()
        
        if self.last_time is None:
            self.last_time = now
            return linear_x, angular_z
        
        dt = (now - self.last_time).to_sec()
        self.last_time = now
        
        if dt <= 0:
            return linear_x, angular_z
        
        rate_limited = False
        
        if self.max_linear_accel > 0:
            max_delta_linear = self.max_linear_accel * dt
            delta_linear = linear_x - self.last_linear_x
            if abs(delta_linear) > max_delta_linear:
                linear_x = self.last_linear_x + max_delta_linear * (1 if delta_linear > 0 else -1)
                rate_limited = True
        
        if self.max_angular_accel > 0:
            max_delta_angular = self.max_angular_accel * dt
            delta_angular = angular_z - self.last_angular_z
            if abs(delta_angular) > max_delta_angular:
                angular_z = self.last_angular_z + max_delta_angular * (1 if delta_angular > 0 else -1)
                rate_limited = True
        
        if rate_limited:
            self.rate_limit_count += 1
        
        return linear_x, angular_z
    
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
