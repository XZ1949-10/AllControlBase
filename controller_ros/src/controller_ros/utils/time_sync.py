"""
时间同步工具

处理数据时间戳和超时检测。
"""
from typing import Dict, Optional, List


class TimeSync:
    """
    时间同步工具
    
    职责:
    - 检查数据新鲜度
    - 处理超时
    
    键名约定:
    - 内部和外部统一使用 'odom', 'trajectory', 'imu'
    - 输出超时键为 'odom_timeout', 'traj_timeout', 'imu_timeout' (保持向后兼容)
    """
    
    def __init__(self, max_odom_age_ms: float = 100,
                 max_traj_age_ms: float = 200,
                 max_imu_age_ms: float = 50):
        """
        初始化时间同步
        
        Args:
            max_odom_age_ms: 里程计最大年龄 (毫秒)
            max_traj_age_ms: 轨迹最大年龄 (毫秒)
            max_imu_age_ms: IMU 最大年龄 (毫秒)
        """
        self._max_ages = {
            'odom': max_odom_age_ms / 1000.0,
            'trajectory': max_traj_age_ms / 1000.0,
            'imu': max_imu_age_ms / 1000.0,
        }
        
        # 输出键名映射 (保持向后兼容，输出使用短名)
        self._output_key_map = {
            'odom': 'odom',
            'trajectory': 'traj',
            'imu': 'imu',
        }
    
    def check_freshness(self, ages: Dict[str, float]) -> Dict[str, bool]:
        """
        检查数据新鲜度
        
        Args:
            ages: 各数据的年龄 (秒)，键为 'odom', 'trajectory', 'imu'
        
        Returns:
            各数据是否超时的字典，键为 'odom_timeout', 'traj_timeout', 'imu_timeout'
        """
        timeouts = {}
        for key, max_age in self._max_ages.items():
            age = ages.get(key, float('inf'))
            output_key = self._output_key_map.get(key, key)
            timeouts[f'{output_key}_timeout'] = age > max_age
        return timeouts
    
    def is_all_fresh(self, ages: Dict[str, float], 
                     required: Optional[List[str]] = None) -> bool:
        """
        检查所有必需数据是否新鲜
        
        Args:
            ages: 各数据的年龄 (秒)
            required: 必需的数据列表，默认 ['odom', 'trajectory']
        
        Returns:
            所有必需数据都新鲜返回 True
        """
        if required is None:
            required = ['odom', 'trajectory']
        
        for key in required:
            max_age = self._max_ages.get(key, 0.1)
            age = ages.get(key, float('inf'))
            if age > max_age:
                return False
        return True
    
    def get_max_ages(self) -> Dict[str, float]:
        """获取最大年龄配置"""
        return self._max_ages.copy()
