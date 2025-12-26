#!/usr/bin/env python3
"""
控制器诊断工具

统一的诊断脚本，用于排查控制器和 Dashboard 的问题。

用法:
    rosrun controller_ros diagnose.py              # 基础诊断
    rosrun controller_ros diagnose.py --verbose    # 详细诊断
    rosrun controller_ros diagnose.py --test-pub   # 测试发布功能
    rosrun controller_ros diagnose.py --watch      # 持续监控模式
"""

import sys
import time
import argparse


class ControllerDiagnostics:
    """控制器诊断工具类"""
    
    def __init__(self, verbose: bool = False):
        self.verbose = verbose
        self.rospy = None
        self.DiagnosticsV2 = None
        self.received_data = {'diag': None, 'state': None, 'count': 0}
        
    def run_all_checks(self) -> int:
        """运行所有诊断检查"""
        print("=" * 60)
        print("  Universal Controller 诊断工具")
        print("=" * 60)
        print()
        
        # 1. 检查 ROS 环境
        if not self._check_ros_environment():
            return 1
        
        # 2. 检查消息类型
        if not self._check_message_types():
            return 1
        
        # 3. 检查话题
        self._check_topics()
        
        # 4. 检查诊断数据
        self._check_diagnostics_data()
        
        # 5. 检查 universal_controller
        self._check_universal_controller()
        
        # 6. 打印总结
        self._print_summary()
        
        return 0
    
    def _check_ros_environment(self) -> bool:
        """检查 ROS 环境"""
        print("[1/5] 检查 ROS 环境...")
        try:
            import rospy
            self.rospy = rospy
            rospy.init_node('controller_diagnostics', anonymous=True)
            print("  ✓ ROS 环境正常")
            
            if self.verbose:
                print(f"    ROS Master URI: {rospy.get_master().target}")
            return True
        except ImportError:
            print("  ✗ 无法导入 rospy")
            print("    请确保已 source ROS 环境: source /opt/ros/noetic/setup.bash")
            return False
        except Exception as e:
            print(f"  ✗ ROS 初始化失败: {e}")
            return False
    
    def _check_message_types(self) -> bool:
        """检查消息类型"""
        print()
        print("[2/5] 检查消息类型...")
        
        try:
            from controller_ros.msg import DiagnosticsV2, UnifiedCmd, LocalTrajectoryV4
            self.DiagnosticsV2 = DiagnosticsV2
            print("  ✓ DiagnosticsV2 消息类型可用")
            print("  ✓ UnifiedCmd 消息类型可用")
            print("  ✓ LocalTrajectoryV4 消息类型可用")
            
            if self.verbose:
                print(f"    DiagnosticsV2 字段: {DiagnosticsV2.__slots__[:5]}...")
            return True
        except ImportError as e:
            print(f"  ✗ 无法导入消息类型: {e}")
            print("    请确保已编译 controller_ros 包:")
            print("      cd ~/catkin_ws && catkin_make")
            print("      source devel/setup.bash")
            return False
    
    def _check_topics(self):
        """检查话题"""
        print()
        print("[3/5] 检查话题...")
        
        topics = self.rospy.get_published_topics()
        topic_dict = {t[0]: t[1] for t in topics}
        
        required_topics = {
            '/controller/diagnostics': 'controller_ros/DiagnosticsV2',
            '/controller/state': 'std_msgs/Int32',
            '/cmd_unified': 'controller_ros/UnifiedCmd',
        }
        
        optional_topics = {
            '/odom': 'nav_msgs/Odometry',
            '/nn/local_trajectory': 'controller_ros/LocalTrajectoryV4',
        }
        
        # 检查必需话题
        for topic, expected_type in required_topics.items():
            if topic in topic_dict:
                actual_type = topic_dict[topic]
                if expected_type in actual_type:
                    print(f"  ✓ {topic} (类型: {actual_type})")
                else:
                    print(f"  ⚠ {topic} 类型不匹配 (期望: {expected_type}, 实际: {actual_type})")
            else:
                print(f"  ✗ {topic} 不存在")
                print(f"    控制器节点可能未运行")
        
        # 检查可选话题
        if self.verbose:
            print()
            print("  可选话题:")
            for topic, expected_type in optional_topics.items():
                if topic in topic_dict:
                    print(f"    ✓ {topic}")
                else:
                    print(f"    - {topic} (未发布)")
    
    def _check_diagnostics_data(self):
        """检查诊断数据"""
        print()
        print("[4/5] 等待诊断数据 (5秒)...")
        
        def diag_callback(msg):
            self.received_data['diag'] = msg
            self.received_data['count'] += 1
        
        def state_callback(msg):
            self.received_data['state'] = msg
        
        try:
            from std_msgs.msg import Int32
            
            diag_sub = self.rospy.Subscriber(
                '/controller/diagnostics', self.DiagnosticsV2, diag_callback
            )
            state_sub = self.rospy.Subscriber(
                '/controller/state', Int32, state_callback
            )
            
            start_time = time.time()
            while time.time() - start_time < 5.0 and not self.rospy.is_shutdown():
                if self.received_data['diag'] is not None:
                    break
                self.rospy.sleep(0.1)
            
            if self.received_data['diag'] is not None:
                msg = self.received_data['diag']
                print(f"  ✓ 收到诊断数据!")
                print(f"    状态: {msg.state}")
                print(f"    MPC 成功: {msg.mpc_success}")
                print(f"    MPC 求解时间: {msg.mpc_solve_time_ms:.2f} ms")
                print(f"    备用控制器: {msg.backup_active}")
                
                if self.verbose:
                    print(f"    里程计超时: {msg.timeout_odom}")
                    print(f"    轨迹超时: {msg.timeout_traj}")
                    print(f"    紧急停止: {msg.emergency_stop}")
                    print(f"    Alpha: {msg.consistency_alpha_soft:.3f}")
            else:
                print("  ✗ 未收到诊断数据")
                self._print_no_data_suggestions()
            
            if self.received_data['state'] is not None:
                print(f"  ✓ 收到状态数据: {self.received_data['state'].data}")
            else:
                print("  ✗ 未收到状态数据")
                
        except Exception as e:
            print(f"  ✗ 订阅失败: {e}")
    
    def _check_universal_controller(self):
        """检查 universal_controller"""
        print()
        print("[5/5] 检查 universal_controller...")
        
        # 优先从新位置导入
        try:
            from controller_ros.dashboard import ROSDashboardDataSource
            print("  ✓ ROSDashboardDataSource 可用 (controller_ros.dashboard)")
        except ImportError:
            # 回退到旧位置（兼容性）
            try:
                from universal_controller.dashboard.ros_data_source import ROSDashboardDataSource
                print("  ⚠ ROSDashboardDataSource 可用 (旧位置，建议更新导入路径)")
            except ImportError as e:
                print(f"  ✗ 无法导入 ROSDashboardDataSource: {e}")
                print("    请确保已安装 controller_ros 和 universal_controller")
                return
        
        try:
            from universal_controller.manager.controller_manager import ControllerManager
            print("  ✓ ControllerManager 可用")
        except ImportError as e:
            print(f"  ⚠ ControllerManager 导入失败: {e}")
    
    def _print_no_data_suggestions(self):
        """打印无数据时的建议"""
        print()
        print("    可能原因:")
        print("    1. 控制器节点未运行")
        print("    2. 诊断发布频率过低")
        print("    3. 消息类型不匹配")
    
    def _print_summary(self):
        """打印总结"""
        print()
        print("=" * 60)
        print("  诊断完成")
        print("=" * 60)
        
        if self.received_data['diag'] is None:
            print()
            print("建议操作:")
            print("  1. 确保控制器节点正在运行:")
            print("     roslaunch controller_ros turtlebot1.launch")
            print()
            print("  2. 检查话题列表:")
            print("     rostopic list | grep controller")
            print()
            print("  3. 手动查看诊断数据:")
            print("     rostopic echo /controller/diagnostics")
    
    def test_publish(self) -> int:
        """测试发布功能"""
        print("=" * 60)
        print("  测试诊断发布")
        print("=" * 60)
        print()
        
        if not self._check_ros_environment():
            return 1
        
        if not self._check_message_types():
            return 1
        
        print()
        print("测试 ROS1PublisherManager...")
        
        try:
            from controller_ros.io import ROS1PublisherManager
            print("  ✓ ROS1PublisherManager 可导入")
            
            topics = {
                'cmd_unified': '/cmd_unified_test',
                'diagnostics': '/controller/diagnostics_test',
                'state': '/controller/state_test',
            }
            pm = ROS1PublisherManager(topics=topics, diag_publish_rate=1)
            print("  ✓ ROS1PublisherManager 实例创建成功")
            print(f"    _diag_pub: {pm._diag_pub}")
            print(f"    _DiagnosticsV2: {pm._DiagnosticsV2}")
            
            # 测试发布
            test_diag = {
                'state': 1,
                'mpc_success': True,
                'mpc_solve_time_ms': 5.0,
            }
            pm.publish_diagnostics(test_diag, force=True)
            print("  ✓ 诊断已发布到 /controller/diagnostics_test")
            print()
            print("验证: rostopic echo /controller/diagnostics_test -n 1")
            
        except Exception as e:
            import traceback
            print(f"  ✗ 错误: {e}")
            traceback.print_exc()
            return 1
        
        return 0
    
    def watch_mode(self) -> int:
        """持续监控模式"""
        print("=" * 60)
        print("  持续监控模式 (Ctrl+C 退出)")
        print("=" * 60)
        print()
        
        if not self._check_ros_environment():
            return 1
        
        if not self._check_message_types():
            return 1
        
        def diag_callback(msg):
            self.received_data['diag'] = msg
            self.received_data['count'] += 1
        
        self.rospy.Subscriber(
            '/controller/diagnostics', self.DiagnosticsV2, diag_callback
        )
        
        print("监控 /controller/diagnostics ...")
        print()
        
        last_count = 0
        while not self.rospy.is_shutdown():
            if self.received_data['count'] > last_count:
                msg = self.received_data['diag']
                print(f"[{self.received_data['count']:4d}] "
                      f"state={msg.state} "
                      f"mpc={'OK' if msg.mpc_success else 'FAIL'} "
                      f"time={msg.mpc_solve_time_ms:.1f}ms "
                      f"backup={msg.backup_active}")
                last_count = self.received_data['count']
            self.rospy.sleep(0.1)
        
        return 0


def main():
    parser = argparse.ArgumentParser(description='控制器诊断工具')
    parser.add_argument('--verbose', '-v', action='store_true', 
                        help='详细输出')
    parser.add_argument('--test-pub', action='store_true',
                        help='测试发布功能')
    parser.add_argument('--watch', '-w', action='store_true',
                        help='持续监控模式')
    args = parser.parse_args()
    
    diag = ControllerDiagnostics(verbose=args.verbose)
    
    try:
        if args.test_pub:
            return diag.test_publish()
        elif args.watch:
            return diag.watch_mode()
        else:
            return diag.run_all_checks()
    except KeyboardInterrupt:
        print("\n诊断中断")
        return 0


if __name__ == '__main__':
    sys.exit(main())
