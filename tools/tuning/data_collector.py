#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
数据收集器

从 ROS bag 文件或实时 ROS 话题收集诊断数据。

支持的数据源：
1. ROS bag 文件 (--bag)
2. 实时 ROS 话题 (--live)
3. JSON 文件 (--json)

诊断数据格式：
- ROS 消息格式: DiagnosticsV2.msg (扁平化字段)
- Python 字典格式: DiagnosticsV2.to_ros_msg() (嵌套字段)
"""

import json
import time
from pathlib import Path
from typing import Dict, List, Any, Optional, Callable
from dataclasses import dataclass


@dataclass
class CollectionConfig:
    """数据收集配置"""
    diagnostics_topic: str = "/controller/diagnostics"
    state_topic: str = "/controller/state"
    max_samples: int = 10000
    timeout_sec: float = 300.0  # 5 分钟超时


def ros_msg_to_dict(msg) -> Dict[str, Any]:
    """
    将 ROS DiagnosticsV2 消息转换为 Python 字典
    
    ROS 消息使用扁平化字段名 (如 timeout_odom)
    转换为嵌套字典格式 (如 timeout.odom_timeout)
    
    Args:
        msg: ROS DiagnosticsV2 消息
    
    Returns:
        嵌套格式的字典，与 DiagnosticsV2.to_ros_msg() 输出一致
    """
    return {
        'header': {
            'stamp': msg.header.stamp.to_sec() if hasattr(msg.header.stamp, 'to_sec') else msg.header.stamp,
            'frame_id': msg.header.frame_id
        },
        'state': msg.state,
        'mpc_success': msg.mpc_success,
        'mpc_solve_time_ms': msg.mpc_solve_time_ms,
        'backup_active': msg.backup_active,
        'mpc_health': {
            'kkt_residual': msg.mpc_health_kkt_residual,
            'condition_number': msg.mpc_health_condition_number,
            'consecutive_near_timeout': msg.mpc_health_consecutive_near_timeout,
            'degradation_warning': msg.mpc_health_degradation_warning,
            'can_recover': msg.mpc_health_can_recover
        },
        'consistency': {
            'curvature': msg.consistency_curvature,
            'velocity_dir': msg.consistency_velocity_dir,
            'temporal': msg.consistency_temporal,
            'alpha_soft': msg.consistency_alpha_soft,
            'data_valid': msg.consistency_data_valid
        },
        'estimator_health': {
            'covariance_norm': msg.estimator_covariance_norm,
            'innovation_norm': msg.estimator_innovation_norm,
            'slip_probability': msg.estimator_slip_probability,
            'imu_drift_detected': msg.estimator_imu_drift_detected,
            'imu_bias': list(msg.estimator_imu_bias),
            'imu_available': msg.estimator_imu_available
        },
        'tracking': {
            'lateral_error': msg.tracking_lateral_error,
            'longitudinal_error': msg.tracking_longitudinal_error,
            'heading_error': msg.tracking_heading_error,
            'prediction_error': msg.tracking_prediction_error
        },
        'transform': {
            'tf2_available': msg.transform_tf2_available,
            'tf2_injected': msg.transform_tf2_injected,
            'fallback_duration_ms': msg.transform_fallback_duration_ms,
            'accumulated_drift': msg.transform_accumulated_drift,
            'source_frame': getattr(msg, 'transform_source_frame', ''),
            'target_frame': getattr(msg, 'transform_target_frame', ''),
            'error_message': getattr(msg, 'transform_error_message', '')
        },
        'timeout': {
            'odom_timeout': msg.timeout_odom,
            'traj_timeout': msg.timeout_traj,
            'traj_grace_exceeded': msg.timeout_traj_grace_exceeded,
            'imu_timeout': msg.timeout_imu,
            'last_odom_age_ms': msg.timeout_last_odom_age_ms,
            'last_traj_age_ms': msg.timeout_last_traj_age_ms,
            'last_imu_age_ms': msg.timeout_last_imu_age_ms,
            'in_startup_grace': msg.timeout_in_startup_grace
        },
        'cmd': {
            'vx': msg.cmd_vx,
            'vy': msg.cmd_vy,
            'vz': msg.cmd_vz,
            'omega': msg.cmd_omega,
            'frame_id': msg.cmd_frame_id
        },
        'transition_progress': msg.transition_progress,
        # 安全状态 (顶层字段，与 DiagnosticsV2.to_ros_msg() 一致)
        'safety_check_passed': msg.safety_check_passed,
        'emergency_stop': msg.emergency_stop,
        # 错误信息
        'error_message': msg.error_message,
        'consecutive_errors': msg.consecutive_errors
    }


class DataCollector:
    """数据收集器基类"""
    
    def __init__(self, config: CollectionConfig = None):
        self.config = config or CollectionConfig()
        self.samples: List[Dict[str, Any]] = []
        self.collection_start_time: Optional[float] = None
        self.collection_end_time: Optional[float] = None
    
    def get_samples(self) -> List[Dict[str, Any]]:
        """获取收集的样本"""
        return self.samples
    
    def get_collection_duration(self) -> float:
        """获取收集持续时间（秒）"""
        if self.collection_start_time and self.collection_end_time:
            return self.collection_end_time - self.collection_start_time
        return 0.0


class RosbagCollector(DataCollector):
    """从 ROS bag 文件收集数据"""
    
    def __init__(self, bag_path: str, config: CollectionConfig = None):
        super().__init__(config)
        self.bag_path = bag_path
    
    def collect(self, progress_callback: Callable[[int, int], None] = None) -> int:
        """
        从 bag 文件收集数据
        
        Args:
            progress_callback: 进度回调函数 (current, total)
        
        Returns:
            收集的样本数
        """
        try:
            import rosbag
        except ImportError:
            print("错误: 需要安装 rosbag 包")
            print("请运行: pip install rosbag bagpy")
            return 0
        
        self.samples = []
        self.collection_start_time = time.time()
        
        try:
            bag = rosbag.Bag(self.bag_path, 'r')
            
            # 获取消息总数
            info = bag.get_type_and_topic_info()
            topic_info = info.topics.get(self.config.diagnostics_topic, None)
            total_messages = topic_info.message_count if topic_info else 0
            
            print(f"Bag 文件: {self.bag_path}")
            print(f"诊断话题: {self.config.diagnostics_topic}")
            print(f"消息总数: {total_messages}")
            
            count = 0
            for topic, msg, t in bag.read_messages(topics=[self.config.diagnostics_topic]):
                if count >= self.config.max_samples:
                    break
                
                # 解析消息
                sample = self._parse_diagnostics_msg(msg)
                if sample:
                    self.samples.append(sample)
                    count += 1
                
                if progress_callback and count % 100 == 0:
                    progress_callback(count, total_messages)
            
            bag.close()
            
        except Exception as e:
            print(f"读取 bag 文件失败: {e}")
            import traceback
            traceback.print_exc()
        
        self.collection_end_time = time.time()
        return len(self.samples)
    
    def _parse_diagnostics_msg(self, msg) -> Optional[Dict[str, Any]]:
        """解析诊断消息"""
        try:
            # 检查是否是 DiagnosticsV2 消息
            if hasattr(msg, 'mpc_success') and hasattr(msg, 'timeout_odom'):
                return ros_msg_to_dict(msg)
            
            # 尝试解析 JSON 格式的诊断消息 (std_msgs/String)
            if hasattr(msg, 'data') and isinstance(msg.data, str):
                return json.loads(msg.data)
            
            return None
            
        except Exception as e:
            print(f"解析消息失败: {e}")
            return None


class LiveCollector(DataCollector):
    """从实时 ROS 话题收集数据"""
    
    def __init__(self, config: CollectionConfig = None):
        super().__init__(config)
        self._running = False
        self._subscriber = None
    
    def collect(self, duration_sec: float = 60.0,
                progress_callback: Callable[[int, float], None] = None) -> int:
        """
        从实时话题收集数据
        
        Args:
            duration_sec: 收集持续时间（秒）
            progress_callback: 进度回调函数 (samples, elapsed_time)
        
        Returns:
            收集的样本数
        """
        try:
            import rospy
        except ImportError:
            print("错误: 需要 ROS 环境")
            print("请确保已 source ROS 工作空间")
            return 0
        
        # 尝试导入 DiagnosticsV2 消息
        DiagnosticsV2 = None
        try:
            from controller_ros.msg import DiagnosticsV2
            print(f"使用 DiagnosticsV2 消息类型")
        except ImportError:
            print("警告: DiagnosticsV2 消息不可用，尝试使用 String 消息")
        
        self.samples = []
        self._running = True
        self.collection_start_time = time.time()
        
        def callback(msg):
            if not self._running:
                return
            if len(self.samples) >= self.config.max_samples:
                self._running = False
                return
            
            try:
                # 根据消息类型解析
                if DiagnosticsV2 is not None and hasattr(msg, 'mpc_success'):
                    sample = ros_msg_to_dict(msg)
                elif hasattr(msg, 'data'):
                    sample = json.loads(msg.data)
                else:
                    return
                
                self.samples.append(sample)
            except Exception as e:
                rospy.logwarn_throttle(5.0, f"解析诊断消息失败: {e}")
        
        try:
            # 初始化 ROS 节点（如果尚未初始化）
            try:
                rospy.init_node('diagnostics_collector', anonymous=True)
            except rospy.exceptions.ROSException:
                pass  # 节点已初始化
            
            # 确定消息类型并订阅
            if DiagnosticsV2 is not None:
                self._subscriber = rospy.Subscriber(
                    self.config.diagnostics_topic,
                    DiagnosticsV2,
                    callback
                )
            else:
                from std_msgs.msg import String
                self._subscriber = rospy.Subscriber(
                    self.config.diagnostics_topic,
                    String,
                    callback
                )
            
            print(f"开始收集数据...")
            print(f"话题: {self.config.diagnostics_topic}")
            print(f"持续时间: {duration_sec}秒")
            print(f"最大样本数: {self.config.max_samples}")
            print("-" * 40)
            
            start_time = time.time()
            rate = rospy.Rate(10)  # 10Hz 检查频率
            
            while self._running and not rospy.is_shutdown():
                elapsed = time.time() - start_time
                if elapsed >= duration_sec:
                    break
                
                if progress_callback:
                    progress_callback(len(self.samples), elapsed)
                
                rate.sleep()
            
            self._running = False
            
        except Exception as e:
            print(f"收集数据失败: {e}")
            import traceback
            traceback.print_exc()
        finally:
            if self._subscriber:
                self._subscriber.unregister()
        
        self.collection_end_time = time.time()
        return len(self.samples)
    
    def stop(self):
        """停止收集"""
        self._running = False


class JsonFileCollector(DataCollector):
    """从 JSON 文件收集数据"""
    
    def __init__(self, json_path: str, config: CollectionConfig = None):
        super().__init__(config)
        self.json_path = json_path
    
    def collect(self, progress_callback: Callable[[int, int], None] = None) -> int:
        """
        从 JSON 文件收集数据
        
        Args:
            progress_callback: 进度回调函数 (current, total)
        
        Returns:
            收集的样本数
        """
        self.samples = []
        self.collection_start_time = time.time()
        
        try:
            with open(self.json_path, 'r', encoding='utf-8') as f:
                data = json.load(f)
            
            if isinstance(data, list):
                self.samples = data[:self.config.max_samples]
            elif isinstance(data, dict) and 'samples' in data:
                self.samples = data['samples'][:self.config.max_samples]
            else:
                print("警告: JSON 文件格式不正确，期望列表或包含 'samples' 键的字典")
            
            print(f"从 {self.json_path} 加载了 {len(self.samples)} 个样本")
            
        except Exception as e:
            print(f"读取 JSON 文件失败: {e}")
        
        self.collection_end_time = time.time()
        return len(self.samples)


def save_samples_to_json(samples: List[Dict[str, Any]], output_path: str):
    """保存样本到 JSON 文件"""
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    
    with open(output_path, 'w', encoding='utf-8') as f:
        json.dump({
            'timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
            'sample_count': len(samples),
            'samples': samples
        }, f, indent=2, ensure_ascii=False)
    
    print(f"样本已保存到: {output_path}")
