"""
ROS 模拟类

模拟 rospy, tf2_ros 等 ROS 模块，用于非 ROS 环境下的测试和运行。
"""
import time
import numpy as np
import logging
from collections import deque
from typing import Optional, Tuple, Any

from .data_mock import MockTransformStamped, MockHeader

logger = logging.getLogger(__name__)


class LookupException(Exception):
    """TF2 查找异常"""
    pass


class ExtrapolationException(Exception):
    """TF2 外推异常"""
    pass


class ConnectivityException(Exception):
    """TF2 连接异常"""
    pass


class MockRospy:
    """模拟 rospy 模块"""
    
    class Time:
        def __init__(self, secs: float = 0, nsecs: int = 0):
            self._secs = secs
            self._nsecs = nsecs
        
        @classmethod
        def now(cls) -> 'MockRospy.Time':
            t = time.time()
            secs = int(t)
            nsecs = int((t - secs) * 1e9)
            return cls(secs, nsecs)
        
        @classmethod
        def from_sec(cls, sec: float) -> 'MockRospy.Time':
            secs = int(sec)
            nsecs = int((sec - secs) * 1e9)
            return cls(secs, nsecs)
        
        def to_sec(self) -> float:
            return self._secs + self._nsecs * 1e-9
        
        def __sub__(self, other: 'MockRospy.Time') -> 'MockRospy.Duration':
            diff = self.to_sec() - other.to_sec()
            return MockRospy.Duration.from_sec(diff)
        
        def __add__(self, other: 'MockRospy.Duration') -> 'MockRospy.Time':
            return MockRospy.Time.from_sec(self.to_sec() + other.to_sec())
    
    class Duration:
        def __init__(self, secs: float = 0, nsecs: int = 0):
            self._secs = secs
            self._nsecs = nsecs
        
        @classmethod
        def from_sec(cls, sec: float) -> 'MockRospy.Duration':
            secs = int(sec)
            nsecs = int((sec - secs) * 1e9)
            return cls(secs, nsecs)
        
        def to_sec(self) -> float:
            return self._secs + self._nsecs * 1e-9
    
    @staticmethod
    def loginfo(msg: str):
        logger.info(msg)
    
    @staticmethod
    def logwarn(msg: str):
        logger.warning(msg)
    
    @staticmethod
    def logerr(msg: str):
        logger.error(msg)
    
    @staticmethod
    def logwarn_throttle(period: float, msg: str):
        # 简化实现：不做节流，直接输出
        logger.warning(msg)
    
    @staticmethod
    def loginfo_throttle(period: float, msg: str):
        # 简化实现：不做节流，直接输出
        logger.info(msg)
    
    @staticmethod
    def logwarn_once(msg: str):
        # 简化实现：不做去重，直接输出
        logger.warning(msg)


class MockTF2BufferCore:
    """
    模拟 tf2_ros.Buffer - 完整 SE(3) 实现
    
    支持:
    - 直接变换查找
    - 反向变换查找
    - 多跳链式变换查找 (BFS 算法，最多 10 跳)
    - 时间插值 (简化实现)
    """
    
    MAX_CHAIN_DEPTH = 10  # 最大链式查找深度
    
    def __init__(self):
        self._transforms = {}
        self._static_transforms = {}
        self._frame_graph = {}  # 帧连接图，用于 BFS
    
    def set_transform(self, transform: MockTransformStamped, authority: str = "default"):
        """设置变换"""
        key = (transform.header.frame_id, transform.child_frame_id)
        self._transforms[key] = transform
        self._update_frame_graph(transform.header.frame_id, transform.child_frame_id)
    
    def set_transform_static(self, transform: MockTransformStamped, authority: str = "default"):
        """设置静态变换"""
        key = (transform.header.frame_id, transform.child_frame_id)
        self._static_transforms[key] = transform
        self._update_frame_graph(transform.header.frame_id, transform.child_frame_id)
    
    def _update_frame_graph(self, parent: str, child: str) -> None:
        """更新帧连接图"""
        if parent not in self._frame_graph:
            self._frame_graph[parent] = set()
        if child not in self._frame_graph:
            self._frame_graph[child] = set()
        self._frame_graph[parent].add(child)
        self._frame_graph[child].add(parent)
    
    def lookup_transform(self, target_frame: str, source_frame: str, 
                        time: Any, timeout: Any = None) -> MockTransformStamped:
        """查找变换"""
        # 同一坐标系
        if target_frame == source_frame:
            return self._identity_transform(target_frame)
        
        # 直接查找
        key = (target_frame, source_frame)
        if key in self._transforms:
            return self._transforms[key]
        if key in self._static_transforms:
            return self._static_transforms[key]
        
        # 尝试反向查找
        reverse_key = (source_frame, target_frame)
        if reverse_key in self._transforms:
            return self._invert_transform(self._transforms[reverse_key])
        if reverse_key in self._static_transforms:
            return self._invert_transform(self._static_transforms[reverse_key])
        
        # 使用 BFS 进行多跳链式查找
        chain_result = self._bfs_chain_lookup(target_frame, source_frame)
        if chain_result is not None:
            return chain_result
        
        raise LookupException(f"Transform from {source_frame} to {target_frame} not found")
    
    def _identity_transform(self, frame_id: str) -> MockTransformStamped:
        """返回单位变换"""
        result = MockTransformStamped()
        result.header.frame_id = frame_id
        result.child_frame_id = frame_id
        result.transform.rotation.w = 1.0
        return result
    
    def _bfs_chain_lookup(self, target_frame: str, source_frame: str) -> Optional[MockTransformStamped]:
        """
        使用 BFS 算法进行多跳链式查找
        
        支持任意深度的变换链 (最多 MAX_CHAIN_DEPTH 跳)
        """
        if target_frame not in self._frame_graph or source_frame not in self._frame_graph:
            return None
        
        # BFS 查找路径
        queue = deque([(target_frame, [target_frame])])
        visited = {target_frame}
        
        while queue:
            current_frame, path = queue.popleft()
            
            if len(path) > self.MAX_CHAIN_DEPTH:
                continue
            
            if current_frame == source_frame:
                # 找到路径，计算组合变换
                return self._compute_chain_transform(path)
            
            # 扩展邻居
            if current_frame in self._frame_graph:
                for neighbor in self._frame_graph[current_frame]:
                    if neighbor not in visited:
                        visited.add(neighbor)
                        queue.append((neighbor, path + [neighbor]))
        
        return None
    
    def _compute_chain_transform(self, path: list) -> Optional[MockTransformStamped]:
        """
        计算路径上的组合变换
        
        path: [target_frame, ..., source_frame]
        返回: target_frame <- source_frame 的变换
        """
        if len(path) < 2:
            return None
        
        all_transforms = {**self._transforms, **self._static_transforms}
        
        # 从 source 到 target 累积变换
        result = self._identity_transform(path[-1])
        result.child_frame_id = path[-1]
        result.header.frame_id = path[-1]
        
        for i in range(len(path) - 1, 0, -1):
            from_frame = path[i]
            to_frame = path[i - 1]
            
            # 查找 from_frame -> to_frame 的变换
            key = (to_frame, from_frame)
            reverse_key = (from_frame, to_frame)
            
            if key in all_transforms:
                step_transform = all_transforms[key]
            elif reverse_key in all_transforms:
                step_transform = self._invert_transform(all_transforms[reverse_key])
            else:
                return None
            
            # 组合变换
            result = self._compose_transforms(step_transform, result)
            result.header.frame_id = to_frame
        
        result.header.frame_id = path[0]  # target_frame
        result.child_frame_id = path[-1]  # source_frame
        return result
    
    def _try_chain_lookup(self, target_frame: str, source_frame: str) -> Optional[MockTransformStamped]:
        """尝试通过中间帧进行链式查找 (保留兼容性，内部调用 BFS)"""
        return self._bfs_chain_lookup(target_frame, source_frame)
    
    def _compose_transforms(self, t1: MockTransformStamped, t2: MockTransformStamped) -> MockTransformStamped:
        """
        组合两个变换: T_result = T1 * T2
        
        对于 SE(3): (R1, p1) * (R2, p2) = (R1*R2, R1*p2 + p1)
        """
        # 提取四元数
        q1 = (t1.transform.rotation.x, t1.transform.rotation.y, 
              t1.transform.rotation.z, t1.transform.rotation.w)
        q2 = (t2.transform.rotation.x, t2.transform.rotation.y,
              t2.transform.rotation.z, t2.transform.rotation.w)
        
        # 四元数乘法: q1 * q2
        q_result = self._quaternion_multiply(q1, q2)
        
        # 旋转矩阵 R1
        R1 = self._quaternion_to_rotation_matrix(q1)
        
        # 平移组合: p_result = R1 * p2 + p1
        p2 = np.array([t2.transform.translation.x, t2.transform.translation.y, t2.transform.translation.z])
        p1 = np.array([t1.transform.translation.x, t1.transform.translation.y, t1.transform.translation.z])
        p_result = R1 @ p2 + p1
        
        result = MockTransformStamped()
        result.header.stamp = t1.header.stamp
        result.header.frame_id = t1.header.frame_id
        result.child_frame_id = t2.child_frame_id
        result.transform.translation.x = p_result[0]
        result.transform.translation.y = p_result[1]
        result.transform.translation.z = p_result[2]
        result.transform.rotation.x = q_result[0]
        result.transform.rotation.y = q_result[1]
        result.transform.rotation.z = q_result[2]
        result.transform.rotation.w = q_result[3]
        return result
    
    def _quaternion_multiply(self, q1: Tuple[float, float, float, float], 
                            q2: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
        """四元数乘法 (x, y, z, w) 格式"""
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        
        w = w1*w2 - x1*x2 - y1*y2 - z1*z2
        x = w1*x2 + x1*w2 + y1*z2 - z1*y2
        y = w1*y2 - x1*z2 + y1*w2 + z1*x2
        z = w1*z2 + x1*y2 - y1*x2 + z1*w2
        
        return (x, y, z, w)
    
    def _quaternion_to_rotation_matrix(self, q: Tuple[float, float, float, float]) -> np.ndarray:
        """四元数转旋转矩阵"""
        x, y, z, w = q
        return np.array([
            [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
            [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
            [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)]
        ])
    
    def can_transform(self, target_frame: str, source_frame: str, 
                     time: Any, timeout: Any = None) -> bool:
        """检查变换是否可用"""
        key = (target_frame, source_frame)
        reverse_key = (source_frame, target_frame)
        return (key in self._transforms or key in self._static_transforms or
                reverse_key in self._transforms or reverse_key in self._static_transforms)
    
    def _invert_transform(self, transform: MockTransformStamped) -> MockTransformStamped:
        """
        反转变换 - 完整 SE(3) 逆变换
        
        对于变换 T = (R, t)，其逆变换为 T^-1 = (R^T, -R^T * t)
        """
        t = transform.transform
        q = (t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w)
        
        # 四元数归一化和求逆
        norm_sq = q[0]**2 + q[1]**2 + q[2]**2 + q[3]**2
        
        if norm_sq < 1e-10:
            # 无效四元数，返回单位变换的逆（即单位变换本身）
            logger.warning("Invalid quaternion detected, using identity")
            inv_q = (0.0, 0.0, 0.0, 1.0)
            R = np.eye(3)
        else:
            # 归一化四元数
            if abs(norm_sq - 1.0) > 1e-6:
                norm = np.sqrt(norm_sq)
                q = (q[0]/norm, q[1]/norm, q[2]/norm, q[3]/norm)
                norm_sq = 1.0
            
            # 四元数求逆: q^-1 = q* / |q|^2 = (-x, -y, -z, w) / |q|^2
            # 对于单位四元数 |q|^2 = 1
            inv_q = (-q[0], -q[1], -q[2], q[3])
            
            # 旋转矩阵
            x, y, z, w = q
            R = np.array([
                [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
                [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
                [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)]
            ])
        
        # 逆平移
        translation = np.array([t.translation.x, t.translation.y, t.translation.z])
        inv_translation = -R.T @ translation
        
        result = MockTransformStamped()
        result.header.stamp = transform.header.stamp
        result.header.frame_id = transform.child_frame_id
        result.child_frame_id = transform.header.frame_id
        result.transform.translation.x = inv_translation[0]
        result.transform.translation.y = inv_translation[1]
        result.transform.translation.z = inv_translation[2]
        result.transform.rotation.x = inv_q[0]
        result.transform.rotation.y = inv_q[1]
        result.transform.rotation.z = inv_q[2]
        result.transform.rotation.w = inv_q[3]
        return result


class MockTF2Ros:
    """模拟 tf2_ros 模块"""
    Buffer = MockTF2BufferCore
    TransformListener = lambda self, buffer: None
    TransformBroadcaster = lambda: None
    StaticTransformBroadcaster = lambda: None
    LookupException = LookupException
    ExtrapolationException = ExtrapolationException
    ConnectivityException = ConnectivityException


class MockTFTransformations:
    """模拟 tf.transformations 模块"""
    
    @staticmethod
    def euler_from_quaternion(q):
        from ..core.ros_compat import euler_from_quaternion
        return euler_from_quaternion(q)
    
    @staticmethod
    def quaternion_from_euler(roll, pitch, yaw):
        from ..core.ros_compat import quaternion_from_euler
        return quaternion_from_euler(roll, pitch, yaw)
