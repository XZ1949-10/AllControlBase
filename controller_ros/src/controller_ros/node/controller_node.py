"""
控制器 ROS 节点

主节点实现，组装所有组件，管理控制循环。
"""
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

from ..bridge import ControllerBridge, TFBridge
from ..io import SubscriberManager, PublisherManager, ServiceManager
from ..utils import ParamLoader, TimeSync


class ControllerNode(Node):
    """
    控制器主节点
    
    职责:
    - 初始化 ROS 节点
    - 加载配置参数
    - 组装各层组件
    - 管理控制循环
    - 处理生命周期
    
    输出:
    - /cmd_unified (UnifiedCmd)
    - /controller/diagnostics (DiagnosticsV2)
    """
    
    def __init__(self):
        super().__init__('universal_controller_node')
        
        # 1. 加载参数
        self._params = ParamLoader.load(self)
        self._topics = ParamLoader.get_topics(self)
        
        # 2. 创建回调组 (用于并发控制)
        self._sensor_cb_group = ReentrantCallbackGroup()
        self._control_cb_group = MutuallyExclusiveCallbackGroup()
        
        # 3. 获取平台配置
        platform_type = self._params.get('system', {}).get('platform', 'differential')
        from universal_controller.config.default_config import PLATFORM_CONFIG
        platform_config = PLATFORM_CONFIG.get(platform_type, PLATFORM_CONFIG['differential'])
        default_frame_id = platform_config.get('output_frame', 'base_link')
        
        # 4. 创建 TF 桥接
        self._tf_bridge = TFBridge(self)
        
        # 5. 创建控制器桥接
        self._controller_bridge = ControllerBridge(self._params)
        
        # 6. 创建订阅管理器
        self._subscribers = SubscriberManager(
            self, self._topics, self._sensor_cb_group
        )
        
        # 7. 创建发布管理器
        self._publishers = PublisherManager(
            self, self._topics, default_frame_id
        )
        
        # 8. 创建服务管理器
        self._services = ServiceManager(
            self,
            reset_callback=self._handle_reset,
            get_diagnostics_callback=self._handle_get_diagnostics
        )
        
        # 9. 创建时间同步
        time_sync_config = self._params.get('time_sync', {})
        self._time_sync = TimeSync(
            max_odom_age_ms=time_sync_config.get('max_odom_age_ms', 100),
            max_traj_age_ms=time_sync_config.get('max_traj_age_ms', 200),
            max_imu_age_ms=time_sync_config.get('max_imu_age_ms', 50)
        )
        
        # 10. 设置诊断回调
        self._controller_bridge.set_diagnostics_callback(self._on_diagnostics)
        
        # 11. 创建控制定时器
        control_rate = self._params.get('node', {}).get('control_rate', 50.0)
        control_period = 1.0 / control_rate
        self._control_timer = self.create_timer(
            control_period,
            self._control_callback,
            callback_group=self._control_cb_group
        )
        
        # 状态
        self._waiting_for_data_logged = False
        
        self.get_logger().info(
            f'Controller node initialized (platform={platform_type}, rate={control_rate}Hz)'
        )
    
    def _control_callback(self):
        """控制循环回调"""
        # 1. 获取最新数据
        odom = self._subscribers.get_latest_odom()
        imu = self._subscribers.get_latest_imu()
        trajectory = self._subscribers.get_latest_trajectory()
        
        # 2. 检查数据有效性
        if odom is None or trajectory is None:
            if not self._waiting_for_data_logged:
                self.get_logger().warn('Waiting for odom and trajectory data...')
                self._waiting_for_data_logged = True
            return
        
        if self._waiting_for_data_logged:
            self.get_logger().info('Data received, starting control')
            self._waiting_for_data_logged = False
        
        # 3. 检查数据新鲜度
        ages = self._subscribers.get_data_ages()
        timeouts = self._time_sync.check_freshness(ages)
        
        if timeouts.get('odom_timeout', False):
            self.get_logger().warn_throttle(
                1.0, f"Odom timeout: age={ages.get('odom', 0)*1000:.1f}ms"
            )
        
        # 4. 执行控制更新
        try:
            cmd = self._controller_bridge.update(odom, trajectory, imu)
        except Exception as e:
            self.get_logger().error(f'Controller update failed: {e}')
            self._publishers.publish_stop_cmd()
            return
        
        # 5. 发布控制命令
        self._publishers.publish_cmd(cmd)
        
        # 6. 发布调试路径
        self._publishers.publish_debug_path(trajectory)
    
    def _on_diagnostics(self, diag):
        """诊断回调"""
        self._publishers.publish_diagnostics(diag)
    
    def _handle_reset(self):
        """处理重置请求"""
        self._controller_bridge.reset()
        self.get_logger().info('Controller reset')
    
    def _handle_get_diagnostics(self):
        """处理获取诊断请求"""
        return self._controller_bridge.get_diagnostics()
    
    def shutdown(self):
        """清理资源"""
        self._controller_bridge.shutdown()
        self.get_logger().info('Controller node shutdown')


def main(args=None):
    """主入口"""
    rclpy.init(args=args)
    
    node = ControllerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
