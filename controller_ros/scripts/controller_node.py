#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
控制器 ROS1 节点

主节点实现，组装所有组件，管理控制循环。
"""
import rospy
import threading
import time
from typing import Dict, Any, Optional

from nav_msgs.msg import Odometry as RosOdometry
from sensor_msgs.msg import Imu as RosImu
from std_srvs.srv import Trigger, TriggerResponse

# 导入自定义消息
from controller_ros.msg import LocalTrajectoryV4, UnifiedCmd, DiagnosticsV2

# 导入 universal_controller
from universal_controller.config.default_config import DEFAULT_CONFIG, PLATFORM_CONFIG
from universal_controller.manager.controller_manager import ControllerManager
from universal_controller.core.data_types import (
    Odometry as UcOdometry, Imu as UcImu, Trajectory as UcTrajectory,
    Header, Point3D, ControlOutput
)
from universal_controller.core.enums import TrajectoryMode

import numpy as np


class ControllerNode:
    """
    控制器主节点 (ROS1 Noetic)
    
    职责:
    - 初始化 ROS 节点
    - 加载配置参数
    - 管理控制循环
    
    输出:
    - /cmd_unified (UnifiedCmd)
    - /controller/diagnostics (DiagnosticsV2)
    """
    
    def __init__(self):
        rospy.init_node('universal_controller_node')
        
        # 1. 加载参数
        self._load_parameters()
        
        # 2. 初始化控制器
        self._init_controller()
        
        # 3. 创建订阅器
        self._create_subscribers()
        
        # 4. 创建发布器
        self._create_publishers()
        
        # 5. 创建服务
        self._create_services()
        
        # 6. 数据缓存 (线程安全)
        self._lock = threading.Lock()
        self._latest_odom: Optional[UcOdometry] = None
        self._latest_imu: Optional[UcImu] = None
        self._latest_traj: Optional[UcTrajectory] = None
        self._data_timestamps: Dict[str, float] = {}
        
        # 7. 状态
        self._waiting_for_data = True
        
        # 8. 创建控制定时器
        control_period = 1.0 / self._control_rate
        self._timer = rospy.Timer(rospy.Duration(control_period), self._control_callback)
        
        rospy.loginfo(f"Controller node initialized (platform={self._platform_type}, rate={self._control_rate}Hz)")
    
    def _load_parameters(self):
        """加载 ROS 参数"""
        # 节点参数
        self._control_rate = rospy.get_param('~control_rate', 
                                             rospy.get_param('node/control_rate', 50.0))
        
        # 平台类型
        self._platform_type = rospy.get_param('~platform_type',
                                              rospy.get_param('platform/type', 'differential'))
        
        # 话题
        self._topics = {
            'odom': rospy.get_param('~odom', rospy.get_param('topics/odom', '/odom')),
            'imu': rospy.get_param('~imu', rospy.get_param('topics/imu', '/imu')),
            'trajectory': rospy.get_param('~trajectory', 
                                          rospy.get_param('topics/trajectory', '/nn/local_trajectory')),
            'cmd_unified': rospy.get_param('~cmd_unified',
                                           rospy.get_param('topics/cmd_unified', '/cmd_unified')),
            'diagnostics': rospy.get_param('~diagnostics',
                                           rospy.get_param('topics/diagnostics', '/controller/diagnostics')),
        }
        
        # TF 配置
        self._tf_source_frame = rospy.get_param('tf/source_frame', 'base_link')
        self._tf_target_frame = rospy.get_param('tf/target_frame', 'odom')
        
        # 时间同步
        self._max_odom_age = rospy.get_param('time_sync/max_odom_age_ms', 100) / 1000.0
        self._max_traj_age = rospy.get_param('time_sync/max_traj_age_ms', 200) / 1000.0
        self._max_imu_age = rospy.get_param('time_sync/max_imu_age_ms', 50) / 1000.0
        
        # 构建控制器配置
        self._config = DEFAULT_CONFIG.copy()
        self._config['system'] = DEFAULT_CONFIG['system'].copy()
        self._config['system']['platform'] = self._platform_type
        self._config['system']['ctrl_freq'] = int(self._control_rate)
        
        # 加载额外的控制器参数
        controller_params = rospy.get_param('controller', {})
        for key, value in controller_params.items():
            if key in self._config:
                if isinstance(value, dict):
                    self._config[key].update(value)
                else:
                    self._config[key] = value
    
    def _init_controller(self):
        """初始化控制器"""
        self._platform_config = PLATFORM_CONFIG.get(
            self._platform_type, PLATFORM_CONFIG['differential']
        )
        self._default_frame_id = self._platform_config.get('output_frame', 'base_link')
        
        self._manager = ControllerManager(self._config)
        self._manager.initialize_default_components()
        
        # 设置诊断回调
        self._manager.set_diagnostics_callback(self._on_diagnostics)
        
        rospy.loginfo("ControllerManager initialized")
    
    def _create_subscribers(self):
        """创建订阅器"""
        # 里程计
        self._odom_sub = rospy.Subscriber(
            self._topics['odom'],
            RosOdometry,
            self._odom_callback,
            queue_size=10
        )
        rospy.loginfo(f"Subscribed to odom: {self._topics['odom']}")
        
        # IMU
        self._imu_sub = rospy.Subscriber(
            self._topics['imu'],
            RosImu,
            self._imu_callback,
            queue_size=10
        )
        rospy.loginfo(f"Subscribed to imu: {self._topics['imu']}")
        
        # 轨迹
        self._traj_sub = rospy.Subscriber(
            self._topics['trajectory'],
            LocalTrajectoryV4,
            self._traj_callback,
            queue_size=10
        )
        rospy.loginfo(f"Subscribed to trajectory: {self._topics['trajectory']}")
    
    def _create_publishers(self):
        """创建发布器"""
        # 统一控制命令
        self._cmd_pub = rospy.Publisher(
            self._topics['cmd_unified'],
            UnifiedCmd,
            queue_size=1
        )
        rospy.loginfo(f"Publishing cmd to: {self._topics['cmd_unified']}")
        
        # 诊断
        self._diag_pub = rospy.Publisher(
            self._topics['diagnostics'],
            DiagnosticsV2,
            queue_size=10
        )
        rospy.loginfo(f"Publishing diagnostics to: {self._topics['diagnostics']}")
    
    def _create_services(self):
        """创建服务"""
        self._reset_srv = rospy.Service(
            '/controller/reset',
            Trigger,
            self._handle_reset
        )
        rospy.loginfo("Created reset service: /controller/reset")
    
    # ==================== 回调函数 ====================
    
    def _odom_callback(self, msg: RosOdometry):
        """里程计回调"""
        uc_odom = UcOdometry(
            header=Header(
                stamp=msg.header.stamp.to_sec(),
                frame_id=msg.header.frame_id
            ),
            pose_position=Point3D(
                x=msg.pose.pose.position.x,
                y=msg.pose.pose.position.y,
                z=msg.pose.pose.position.z
            ),
            pose_orientation=(
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w
            ),
            twist_linear=(
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.linear.z
            ),
            twist_angular=(
                msg.twist.twist.angular.x,
                msg.twist.twist.angular.y,
                msg.twist.twist.angular.z
            )
        )
        
        with self._lock:
            self._latest_odom = uc_odom
            self._data_timestamps['odom'] = time.time()
    
    def _imu_callback(self, msg: RosImu):
        """IMU 回调"""
        uc_imu = UcImu(
            header=Header(
                stamp=msg.header.stamp.to_sec(),
                frame_id=msg.header.frame_id
            ),
            orientation=(
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w
            ),
            angular_velocity=(
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z
            ),
            linear_acceleration=(
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z
            )
        )
        
        with self._lock:
            self._latest_imu = uc_imu
            self._data_timestamps['imu'] = time.time()
    
    def _traj_callback(self, msg: LocalTrajectoryV4):
        """轨迹回调"""
        # 转换轨迹点
        points = [
            Point3D(x=p.x, y=p.y, z=p.z)
            for p in msg.points
        ]
        
        # 转换速度数组
        velocities = None
        if msg.soft_enabled and len(msg.velocities_flat) > 0:
            velocities = np.array(msg.velocities_flat).reshape(-1, 4)
        
        # 转换轨迹模式
        try:
            mode = TrajectoryMode(msg.mode)
        except ValueError:
            mode = TrajectoryMode.MODE_TRACK
        
        uc_traj = UcTrajectory(
            header=Header(
                stamp=msg.header.stamp.to_sec(),
                frame_id=msg.header.frame_id
            ),
            points=points,
            velocities=velocities,
            dt_sec=msg.dt_sec,
            confidence=msg.confidence,
            mode=mode,
            soft_enabled=msg.soft_enabled
        )
        
        with self._lock:
            self._latest_traj = uc_traj
            self._data_timestamps['trajectory'] = time.time()
    
    def _control_callback(self, event):
        """控制循环回调"""
        # 1. 获取最新数据
        with self._lock:
            odom = self._latest_odom
            imu = self._latest_imu
            trajectory = self._latest_traj
        
        # 2. 检查数据有效性
        if odom is None or trajectory is None:
            if self._waiting_for_data:
                rospy.logwarn_throttle(5.0, "Waiting for odom and trajectory data...")
            return
        
        if self._waiting_for_data:
            rospy.loginfo("Data received, starting control")
            self._waiting_for_data = False
        
        # 3. 检查数据新鲜度
        now = time.time()
        with self._lock:
            odom_age = now - self._data_timestamps.get('odom', 0)
            traj_age = now - self._data_timestamps.get('trajectory', 0)
        
        if odom_age > self._max_odom_age:
            rospy.logwarn_throttle(1.0, f"Odom timeout: age={odom_age*1000:.1f}ms")
        
        # 4. 执行控制更新
        try:
            cmd = self._manager.update(odom, trajectory, imu)
        except Exception as e:
            rospy.logerr(f"Controller update failed: {e}")
            self._publish_stop_cmd()
            return
        
        # 5. 发布控制命令
        self._publish_cmd(cmd)
    
    def _on_diagnostics(self, diag: Dict[str, Any]):
        """诊断回调"""
        self._publish_diagnostics(diag)
    
    # ==================== 发布函数 ====================
    
    def _publish_cmd(self, cmd: ControlOutput):
        """发布控制命令"""
        msg = UnifiedCmd()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = cmd.frame_id or self._default_frame_id
        msg.vx = float(cmd.vx)
        msg.vy = float(cmd.vy)
        msg.vz = float(cmd.vz)
        msg.omega = float(cmd.omega)
        msg.success = cmd.success
        msg.solve_time_ms = float(cmd.solve_time_ms)
        self._cmd_pub.publish(msg)
    
    def _publish_stop_cmd(self):
        """发布停止命令"""
        msg = UnifiedCmd()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self._default_frame_id
        msg.vx = 0.0
        msg.vy = 0.0
        msg.vz = 0.0
        msg.omega = 0.0
        msg.success = True
        msg.solve_time_ms = 0.0
        self._cmd_pub.publish(msg)
    
    def _publish_diagnostics(self, diag: Dict[str, Any]):
        """发布诊断信息"""
        msg = DiagnosticsV2()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'controller'
        
        msg.state = diag.get('state', 0)
        msg.mpc_success = diag.get('mpc_success', False)
        msg.mpc_solve_time_ms = float(diag.get('mpc_solve_time_ms', 0.0))
        msg.backup_active = diag.get('backup_active', False)
        
        # MPC 健康状态
        mpc_health = diag.get('mpc_health', {})
        msg.mpc_health_kkt_residual = float(mpc_health.get('kkt_residual', 0.0))
        msg.mpc_health_condition_number = float(mpc_health.get('condition_number', 1.0))
        msg.mpc_health_consecutive_near_timeout = int(mpc_health.get('consecutive_near_timeout', 0))
        msg.mpc_health_degradation_warning = mpc_health.get('degradation_warning', False)
        msg.mpc_health_can_recover = mpc_health.get('can_recover', True)
        
        # 一致性指标
        consistency = diag.get('consistency', {})
        msg.consistency_curvature = float(consistency.get('curvature', 0.0))
        msg.consistency_velocity_dir = float(consistency.get('velocity_dir', 1.0))
        msg.consistency_temporal = float(consistency.get('temporal', 1.0))
        msg.consistency_alpha_soft = float(consistency.get('alpha_soft', 0.0))
        msg.consistency_data_valid = consistency.get('data_valid', True)
        
        # 超时状态
        timeout = diag.get('timeout', {})
        msg.timeout_odom = timeout.get('odom_timeout', False)
        msg.timeout_traj = timeout.get('traj_timeout', False)
        msg.timeout_traj_grace_exceeded = timeout.get('traj_grace_exceeded', False)
        msg.timeout_imu = timeout.get('imu_timeout', False)
        msg.timeout_last_odom_age_ms = float(timeout.get('last_odom_age_ms', 0.0))
        msg.timeout_last_traj_age_ms = float(timeout.get('last_traj_age_ms', 0.0))
        msg.timeout_last_imu_age_ms = float(timeout.get('last_imu_age_ms', 0.0))
        msg.timeout_in_startup_grace = timeout.get('in_startup_grace', False)
        
        # 控制命令
        cmd = diag.get('cmd', {})
        msg.cmd_vx = float(cmd.get('vx', 0.0))
        msg.cmd_vy = float(cmd.get('vy', 0.0))
        msg.cmd_vz = float(cmd.get('vz', 0.0))
        msg.cmd_omega = float(cmd.get('omega', 0.0))
        msg.cmd_frame_id = cmd.get('frame_id', '')
        
        msg.transition_progress = float(diag.get('transition_progress', 0.0))
        
        self._diag_pub.publish(msg)
    
    # ==================== 服务处理 ====================
    
    def _handle_reset(self, req):
        """处理重置请求"""
        try:
            self._manager.reset()
            self._waiting_for_data = True
            rospy.loginfo("Controller reset")
            return TriggerResponse(success=True, message="Controller reset successfully")
        except Exception as e:
            return TriggerResponse(success=False, message=f"Reset failed: {e}")
    
    def shutdown(self):
        """关闭节点"""
        self._manager.shutdown()
        rospy.loginfo("Controller node shutdown")


def main():
    try:
        node = ControllerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        if 'node' in locals():
            node.shutdown()


if __name__ == '__main__':
    main()
