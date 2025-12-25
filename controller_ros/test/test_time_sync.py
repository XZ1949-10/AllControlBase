"""
TimeSync 单元测试
"""
import pytest
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))


class TestTimeSyncInit:
    """TimeSync 初始化测试"""
    
    def test_default_init(self):
        """测试默认初始化"""
        from controller_ros.utils import TimeSync
        
        sync = TimeSync()
        max_ages = sync.get_max_ages()
        
        assert max_ages['odom'] == 0.1  # 100ms
        assert max_ages['traj'] == 0.2  # 200ms
        assert max_ages['imu'] == 0.05  # 50ms
    
    def test_custom_init(self):
        """测试自定义初始化"""
        from controller_ros.utils import TimeSync
        
        sync = TimeSync(
            max_odom_age_ms=500,
            max_traj_age_ms=1000,
            max_imu_age_ms=200
        )
        max_ages = sync.get_max_ages()
        
        assert max_ages['odom'] == 0.5
        assert max_ages['traj'] == 1.0
        assert max_ages['imu'] == 0.2
    
    def test_disabled_timeout_zero(self):
        """测试禁用超时 (值为 0)"""
        from controller_ros.utils import TimeSync
        
        sync = TimeSync(
            max_odom_age_ms=100,
            max_traj_age_ms=200,
            max_imu_age_ms=0  # 禁用 IMU 超时
        )
        
        assert sync.is_disabled('imu') == True
        assert sync.is_disabled('odom') == False
        assert sync.is_disabled('trajectory') == False
        
        max_ages = sync.get_max_ages()
        assert max_ages['imu'] is None
    
    def test_disabled_timeout_negative(self):
        """测试禁用超时 (负值)"""
        from controller_ros.utils import TimeSync
        
        sync = TimeSync(
            max_odom_age_ms=100,
            max_traj_age_ms=-1,  # 禁用轨迹超时
            max_imu_age_ms=-100  # 禁用 IMU 超时
        )
        
        assert sync.is_disabled('trajectory') == True
        assert sync.is_disabled('imu') == True
        assert sync.is_disabled('odom') == False


class TestCheckFreshness:
    """check_freshness 测试"""
    
    def test_all_fresh(self):
        """测试所有数据新鲜"""
        from controller_ros.utils import TimeSync
        
        sync = TimeSync(max_odom_age_ms=100, max_traj_age_ms=200, max_imu_age_ms=50)
        ages = {'odom': 0.05, 'trajectory': 0.1, 'imu': 0.02}
        
        timeouts = sync.check_freshness(ages)
        
        assert timeouts['odom_timeout'] == False
        assert timeouts['traj_timeout'] == False
        assert timeouts['imu_timeout'] == False
    
    def test_odom_timeout(self):
        """测试里程计超时"""
        from controller_ros.utils import TimeSync
        
        sync = TimeSync(max_odom_age_ms=100, max_traj_age_ms=200, max_imu_age_ms=50)
        ages = {'odom': 0.15, 'trajectory': 0.1, 'imu': 0.02}
        
        timeouts = sync.check_freshness(ages)
        
        assert timeouts['odom_timeout'] == True
        assert timeouts['traj_timeout'] == False
        assert timeouts['imu_timeout'] == False
    
    def test_traj_timeout(self):
        """测试轨迹超时"""
        from controller_ros.utils import TimeSync
        
        sync = TimeSync(max_odom_age_ms=100, max_traj_age_ms=200, max_imu_age_ms=50)
        ages = {'odom': 0.05, 'trajectory': 0.25, 'imu': 0.02}
        
        timeouts = sync.check_freshness(ages)
        
        assert timeouts['odom_timeout'] == False
        assert timeouts['traj_timeout'] == True
        assert timeouts['imu_timeout'] == False
    
    def test_imu_timeout(self):
        """测试 IMU 超时"""
        from controller_ros.utils import TimeSync
        
        sync = TimeSync(max_odom_age_ms=100, max_traj_age_ms=200, max_imu_age_ms=50)
        ages = {'odom': 0.05, 'trajectory': 0.1, 'imu': 0.06}
        
        timeouts = sync.check_freshness(ages)
        
        assert timeouts['odom_timeout'] == False
        assert timeouts['traj_timeout'] == False
        assert timeouts['imu_timeout'] == True
    
    def test_all_timeout(self):
        """测试所有数据超时"""
        from controller_ros.utils import TimeSync
        
        sync = TimeSync(max_odom_age_ms=100, max_traj_age_ms=200, max_imu_age_ms=50)
        ages = {'odom': 1.0, 'trajectory': 1.0, 'imu': 1.0}
        
        timeouts = sync.check_freshness(ages)
        
        assert timeouts['odom_timeout'] == True
        assert timeouts['traj_timeout'] == True
        assert timeouts['imu_timeout'] == True
    
    def test_missing_data(self):
        """测试缺失数据"""
        from controller_ros.utils import TimeSync
        
        sync = TimeSync(max_odom_age_ms=100, max_traj_age_ms=200, max_imu_age_ms=50)
        ages = {'odom': 0.05}  # 缺少 trajectory 和 imu
        
        timeouts = sync.check_freshness(ages)
        
        assert timeouts['odom_timeout'] == False
        assert timeouts['traj_timeout'] == True  # inf > 200ms
        assert timeouts['imu_timeout'] == True  # inf > 50ms
    
    def test_inf_age(self):
        """测试无穷大年龄"""
        from controller_ros.utils import TimeSync
        
        sync = TimeSync(max_odom_age_ms=100, max_traj_age_ms=200, max_imu_age_ms=50)
        ages = {'odom': float('inf'), 'trajectory': 0.1, 'imu': 0.02}
        
        timeouts = sync.check_freshness(ages)
        
        assert timeouts['odom_timeout'] == True
        assert timeouts['traj_timeout'] == False
        assert timeouts['imu_timeout'] == False
    
    def test_disabled_never_timeout(self):
        """测试禁用的数据源永远不超时"""
        from controller_ros.utils import TimeSync
        
        sync = TimeSync(max_odom_age_ms=100, max_traj_age_ms=200, max_imu_age_ms=0)  # IMU 禁用
        ages = {'odom': 0.05, 'trajectory': 0.1, 'imu': 999.0}  # IMU 年龄很大
        
        timeouts = sync.check_freshness(ages)
        
        assert timeouts['odom_timeout'] == False
        assert timeouts['traj_timeout'] == False
        assert timeouts['imu_timeout'] == False  # 禁用的永远不超时
    
    def test_disabled_missing_data(self):
        """测试禁用的数据源缺失时不超时"""
        from controller_ros.utils import TimeSync
        
        sync = TimeSync(max_odom_age_ms=100, max_traj_age_ms=200, max_imu_age_ms=-1)  # IMU 禁用
        ages = {'odom': 0.05, 'trajectory': 0.1}  # 没有 IMU 数据
        
        timeouts = sync.check_freshness(ages)
        
        assert timeouts['imu_timeout'] == False  # 禁用的永远不超时


class TestIsAllFresh:
    """is_all_fresh 测试"""
    
    def test_default_required(self):
        """测试默认必需数据"""
        from controller_ros.utils import TimeSync
        
        sync = TimeSync(max_odom_age_ms=100, max_traj_age_ms=200)
        
        # odom 和 trajectory 都新鲜
        ages = {'odom': 0.05, 'trajectory': 0.1}
        assert sync.is_all_fresh(ages) == True
        
        # odom 超时
        ages = {'odom': 0.15, 'trajectory': 0.1}
        assert sync.is_all_fresh(ages) == False
        
        # trajectory 超时
        ages = {'odom': 0.05, 'trajectory': 0.25}
        assert sync.is_all_fresh(ages) == False
    
    def test_custom_required(self):
        """测试自定义必需数据"""
        from controller_ros.utils import TimeSync
        
        sync = TimeSync(max_odom_age_ms=100, max_traj_age_ms=200, max_imu_age_ms=50)
        
        # 只要求 odom
        ages = {'odom': 0.05, 'trajectory': 1.0, 'imu': 1.0}
        assert sync.is_all_fresh(ages, required=['odom']) == True
        
        # 要求 odom 和 imu
        ages = {'odom': 0.05, 'trajectory': 1.0, 'imu': 0.02}
        assert sync.is_all_fresh(ages, required=['odom', 'imu']) == True
        
        # imu 超时
        ages = {'odom': 0.05, 'trajectory': 1.0, 'imu': 0.06}
        assert sync.is_all_fresh(ages, required=['odom', 'imu']) == False
    
    def test_empty_required(self):
        """测试空必需列表"""
        from controller_ros.utils import TimeSync
        
        sync = TimeSync()
        ages = {'odom': 1.0, 'trajectory': 1.0, 'imu': 1.0}
        
        # 空列表，所有数据都不是必需的
        assert sync.is_all_fresh(ages, required=[]) == True
    
    def test_disabled_in_required(self):
        """测试禁用的数据源在必需列表中"""
        from controller_ros.utils import TimeSync
        
        sync = TimeSync(max_odom_age_ms=100, max_traj_age_ms=200, max_imu_age_ms=0)  # IMU 禁用
        ages = {'odom': 0.05, 'trajectory': 0.1}  # 没有 IMU 数据
        
        # 即使 IMU 在必需列表中，但因为被禁用，所以不影响结果
        assert sync.is_all_fresh(ages, required=['odom', 'imu']) == True


class TestEdgeCases:
    """边界情况测试"""
    
    def test_exact_threshold(self):
        """测试精确阈值"""
        from controller_ros.utils import TimeSync
        
        sync = TimeSync(max_odom_age_ms=100)
        
        # 刚好等于阈值，不超时
        ages = {'odom': 0.1}
        timeouts = sync.check_freshness(ages)
        assert timeouts['odom_timeout'] == False
        
        # 略大于阈值，超时
        ages = {'odom': 0.1001}
        timeouts = sync.check_freshness(ages)
        assert timeouts['odom_timeout'] == True
    
    def test_zero_age(self):
        """测试零年龄"""
        from controller_ros.utils import TimeSync
        
        sync = TimeSync(max_odom_age_ms=100)
        ages = {'odom': 0.0, 'trajectory': 0.0, 'imu': 0.0}
        
        timeouts = sync.check_freshness(ages)
        
        assert timeouts['odom_timeout'] == False
        assert timeouts['traj_timeout'] == False
        assert timeouts['imu_timeout'] == False
    
    def test_negative_age(self):
        """测试负年龄（时钟回退）"""
        from controller_ros.utils import TimeSync
        
        sync = TimeSync(max_odom_age_ms=100)
        ages = {'odom': -0.1}  # 负年龄
        
        timeouts = sync.check_freshness(ages)
        
        # 负年龄应该被视为新鲜
        assert timeouts['odom_timeout'] == False
    
    def test_very_small_threshold(self):
        """测试非常小的阈值"""
        from controller_ros.utils import TimeSync
        
        sync = TimeSync(max_odom_age_ms=1)  # 1ms
        ages = {'odom': 0.002}  # 2ms
        
        timeouts = sync.check_freshness(ages)
        
        assert timeouts['odom_timeout'] == True
    
    def test_very_large_threshold(self):
        """测试非常大的阈值"""
        from controller_ros.utils import TimeSync
        
        sync = TimeSync(max_odom_age_ms=60000)  # 60 秒
        ages = {'odom': 30.0}  # 30 秒
        
        timeouts = sync.check_freshness(ages)
        
        assert timeouts['odom_timeout'] == False


class TestGetMaxAges:
    """get_max_ages 测试"""
    
    def test_returns_copy(self):
        """测试返回副本"""
        from controller_ros.utils import TimeSync
        
        sync = TimeSync(max_odom_age_ms=100)
        
        ages1 = sync.get_max_ages()
        ages1['odom'] = 999.0
        
        ages2 = sync.get_max_ages()
        
        # 修改返回值不应影响内部状态
        assert ages2['odom'] == 0.1
    
    def test_disabled_returns_none(self):
        """测试禁用的数据源返回 None"""
        from controller_ros.utils import TimeSync
        
        sync = TimeSync(max_odom_age_ms=100, max_traj_age_ms=0, max_imu_age_ms=-1)
        
        max_ages = sync.get_max_ages()
        
        assert max_ages['odom'] == 0.1
        assert max_ages['traj'] is None
        assert max_ages['imu'] is None


class TestIsDisabled:
    """is_disabled 测试"""
    
    def test_enabled_sources(self):
        """测试启用的数据源"""
        from controller_ros.utils import TimeSync
        
        sync = TimeSync(max_odom_age_ms=100, max_traj_age_ms=200, max_imu_age_ms=50)
        
        assert sync.is_disabled('odom') == False
        assert sync.is_disabled('trajectory') == False
        assert sync.is_disabled('imu') == False
    
    def test_disabled_sources(self):
        """测试禁用的数据源"""
        from controller_ros.utils import TimeSync
        
        sync = TimeSync(max_odom_age_ms=0, max_traj_age_ms=-1, max_imu_age_ms=-100)
        
        assert sync.is_disabled('odom') == True
        assert sync.is_disabled('trajectory') == True
        assert sync.is_disabled('imu') == True
    
    def test_unknown_source(self):
        """测试未知数据源"""
        from controller_ros.utils import TimeSync
        
        sync = TimeSync()
        
        # 未知数据源默认返回 False
        assert sync.is_disabled('unknown') == False


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
