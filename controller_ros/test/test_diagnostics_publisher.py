"""
诊断发布工具单元测试
"""
import pytest
import sys
import os
import math
import numpy as np

# 添加 src 目录到路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from controller_ros.utils.diagnostics_publisher import (
    safe_float,
    safe_float_list,
    DiagnosticsPublishHelper,
    fill_diagnostics_msg,
)


class TestSafeFloat:
    """测试 safe_float 函数"""
    
    def test_normal_float(self):
        """测试正常 float 值"""
        assert safe_float(1.5) == 1.5
        assert safe_float(0.0) == 0.0
        assert safe_float(-3.14) == -3.14
    
    def test_int_conversion(self):
        """测试 int 转换"""
        assert safe_float(42) == 42.0
        assert safe_float(-10) == -10.0
    
    def test_none_value(self):
        """测试 None 值"""
        assert safe_float(None) == 0.0
        assert safe_float(None, default=1.0) == 1.0
    
    def test_nan_value(self):
        """测试 nan 值"""
        assert safe_float(float('nan')) == 0.0
        assert safe_float(float('nan'), default=-1.0) == -1.0
    
    def test_inf_value(self):
        """测试 inf 值"""
        assert safe_float(float('inf')) == 0.0
        assert safe_float(float('-inf')) == 0.0
        assert safe_float(float('inf'), default=999.0) == 999.0
    
    def test_invalid_type(self):
        """测试无效类型"""
        assert safe_float("not a number") == 0.0
        assert safe_float([1, 2, 3]) == 0.0
        assert safe_float({}) == 0.0
    
    def test_numpy_scalar(self):
        """测试 numpy 标量"""
        assert safe_float(np.float64(3.14)) == pytest.approx(3.14)
        assert safe_float(np.int32(42)) == 42.0
        assert safe_float(np.nan) == 0.0


class TestSafeFloatList:
    """测试 safe_float_list 函数"""
    
    def test_normal_list(self):
        """测试正常列表"""
        result = safe_float_list([1.0, 2.0, 3.0], length=3)
        assert result == [1.0, 2.0, 3.0]
    
    def test_tuple_input(self):
        """测试元组输入"""
        result = safe_float_list((1.0, 2.0, 3.0), length=3)
        assert result == [1.0, 2.0, 3.0]
    
    def test_numpy_array(self):
        """测试 numpy 数组"""
        arr = np.array([1.0, 2.0, 3.0])
        result = safe_float_list(arr, length=3)
        assert result == [1.0, 2.0, 3.0]
    
    def test_none_value(self):
        """测试 None 值"""
        result = safe_float_list(None, length=3)
        assert result == [0.0, 0.0, 0.0]
        
        result = safe_float_list(None, length=3, default=1.0)
        assert result == [1.0, 1.0, 1.0]
    
    def test_short_list(self):
        """测试短列表（需要填充）"""
        result = safe_float_list([1.0, 2.0], length=3)
        assert result == [1.0, 2.0, 0.0]
    
    def test_long_list(self):
        """测试长列表（需要截断）"""
        result = safe_float_list([1.0, 2.0, 3.0, 4.0, 5.0], length=3)
        assert result == [1.0, 2.0, 3.0]
    
    def test_list_with_nan(self):
        """测试包含 nan 的列表"""
        result = safe_float_list([1.0, float('nan'), 3.0], length=3)
        assert result == [1.0, 0.0, 3.0]
    
    def test_list_with_none(self):
        """测试包含 None 的列表"""
        result = safe_float_list([1.0, None, 3.0], length=3)
        assert result == [1.0, 0.0, 3.0]
    
    def test_numpy_array_with_nan(self):
        """测试包含 nan 的 numpy 数组"""
        arr = np.array([1.0, np.nan, 3.0])
        result = safe_float_list(arr, length=3)
        assert result == [1.0, 0.0, 3.0]
    
    def test_empty_list(self):
        """测试空列表"""
        result = safe_float_list([], length=3)
        assert result == [0.0, 0.0, 0.0]


class TestDiagnosticsPublishHelper:
    """测试 DiagnosticsPublishHelper 类"""
    
    def test_first_publish(self):
        """测试首次发布"""
        helper = DiagnosticsPublishHelper(publish_rate=5)
        diag = {'state': 1}
        
        # 首次应该发布
        assert helper.should_publish(diag) == True
    
    def test_rate_limiting(self):
        """测试降频"""
        helper = DiagnosticsPublishHelper(publish_rate=5)
        diag = {'state': 1}
        
        # 首次发布
        assert helper.should_publish(diag) == True
        
        # 接下来 4 次不发布
        for i in range(4):
            assert helper.should_publish(diag) == False, f"Should not publish at iteration {i+1}"
        
        # 第 5 次发布
        assert helper.should_publish(diag) == True
    
    def test_state_change_forces_publish(self):
        """测试状态变化强制发布"""
        helper = DiagnosticsPublishHelper(publish_rate=5)
        
        # 首次发布
        assert helper.should_publish({'state': 1}) == True
        
        # 状态变化，应该发布
        assert helper.should_publish({'state': 2}) == True
        
        # 状态不变，不发布
        assert helper.should_publish({'state': 2}) == False
    
    def test_force_publish(self):
        """测试强制发布"""
        helper = DiagnosticsPublishHelper(publish_rate=5)
        diag = {'state': 1}
        
        # 首次发布
        assert helper.should_publish(diag) == True
        
        # 强制发布
        assert helper.should_publish(diag, force=True) == True
        assert helper.should_publish(diag, force=True) == True
    
    def test_reset(self):
        """测试重置"""
        helper = DiagnosticsPublishHelper(publish_rate=5)
        diag = {'state': 1}
        
        # 首次发布
        helper.should_publish(diag)
        
        # 几次不发布
        helper.should_publish(diag)
        helper.should_publish(diag)
        
        # 重置
        helper.reset()
        
        # 重置后首次应该发布
        assert helper.should_publish(diag) == True
    
    def test_get_current_state(self):
        """测试获取当前状态"""
        helper = DiagnosticsPublishHelper(publish_rate=5)
        
        # 初始状态为 None
        assert helper.get_current_state() is None
        
        # 发布后状态更新
        helper.should_publish({'state': 3})
        assert helper.get_current_state() == 3
        
        helper.should_publish({'state': 5})
        assert helper.get_current_state() == 5


class MockDiagnosticsMsg:
    """模拟 DiagnosticsV2 消息"""
    def __init__(self):
        self.header = MockHeader()
        self.state = 0
        self.mpc_success = False
        self.mpc_solve_time_ms = 0.0
        self.backup_active = False
        
        self.mpc_health_kkt_residual = 0.0
        self.mpc_health_condition_number = 1.0
        self.mpc_health_consecutive_near_timeout = 0
        self.mpc_health_degradation_warning = False
        self.mpc_health_can_recover = True
        
        self.consistency_curvature = 0.0
        self.consistency_velocity_dir = 1.0
        self.consistency_temporal = 1.0
        self.consistency_alpha_soft = 0.0
        self.consistency_data_valid = True
        
        self.estimator_covariance_norm = 0.0
        self.estimator_innovation_norm = 0.0
        self.estimator_slip_probability = 0.0
        self.estimator_imu_drift_detected = False
        self.estimator_imu_available = True
        self.estimator_imu_bias = [0.0, 0.0, 0.0]
        
        self.tracking_lateral_error = 0.0
        self.tracking_longitudinal_error = 0.0
        self.tracking_heading_error = 0.0
        self.tracking_prediction_error = 0.0
        
        self.transform_tf2_available = False
        self.transform_tf2_injected = False
        self.transform_fallback_duration_ms = 0.0
        self.transform_accumulated_drift = 0.0
        
        self.timeout_odom = False
        self.timeout_traj = False
        self.timeout_traj_grace_exceeded = False
        self.timeout_imu = False
        self.timeout_last_odom_age_ms = 0.0
        self.timeout_last_traj_age_ms = 0.0
        self.timeout_last_imu_age_ms = 0.0
        self.timeout_in_startup_grace = False
        
        self.cmd_vx = 0.0
        self.cmd_vy = 0.0
        self.cmd_vz = 0.0
        self.cmd_omega = 0.0
        self.cmd_frame_id = ''
        
        self.transition_progress = 0.0
        
        # 错误信息字段
        self.error_message = ''
        self.consecutive_errors = 0


class MockHeader:
    """模拟 Header"""
    def __init__(self):
        self.stamp = None
        self.frame_id = ''


class TestFillDiagnosticsMsg:
    """测试 fill_diagnostics_msg 函数"""
    
    def test_basic_fill(self):
        """测试基本填充"""
        msg = MockDiagnosticsMsg()
        diag = {
            'state': 2,
            'mpc_success': True,
            'mpc_solve_time_ms': 5.5,
            'backup_active': False,
        }
        
        fill_diagnostics_msg(msg, diag)
        
        assert msg.state == 2
        assert msg.mpc_success == True
        assert msg.mpc_solve_time_ms == 5.5
        assert msg.backup_active == False
    
    def test_nested_dict(self):
        """测试嵌套字典"""
        msg = MockDiagnosticsMsg()
        diag = {
            'state': 1,
            'mpc_health': {
                'kkt_residual': 0.001,
                'condition_number': 100.0,
            },
            'consistency': {
                'curvature': 0.5,
                'alpha_soft': 0.8,
            },
        }
        
        fill_diagnostics_msg(msg, diag)
        
        assert msg.mpc_health_kkt_residual == 0.001
        assert msg.mpc_health_condition_number == 100.0
        assert msg.consistency_curvature == 0.5
        assert msg.consistency_alpha_soft == 0.8
    
    def test_imu_bias_numpy_array(self):
        """测试 IMU bias numpy 数组"""
        msg = MockDiagnosticsMsg()
        diag = {
            'state': 1,
            'estimator_health': {
                'imu_bias': np.array([0.1, 0.2, 0.3]),
            },
        }
        
        fill_diagnostics_msg(msg, diag)
        
        assert msg.estimator_imu_bias == [0.1, 0.2, 0.3]
    
    def test_imu_bias_with_nan(self):
        """测试 IMU bias 包含 nan"""
        msg = MockDiagnosticsMsg()
        diag = {
            'state': 1,
            'estimator_health': {
                'imu_bias': np.array([0.1, np.nan, 0.3]),
            },
        }
        
        fill_diagnostics_msg(msg, diag)
        
        assert msg.estimator_imu_bias == [0.1, 0.0, 0.3]
    
    def test_imu_bias_none(self):
        """测试 IMU bias 为 None"""
        msg = MockDiagnosticsMsg()
        diag = {
            'state': 1,
            'estimator_health': {
                'imu_bias': None,
            },
        }
        
        fill_diagnostics_msg(msg, diag)
        
        assert msg.estimator_imu_bias == [0.0, 0.0, 0.0]
    
    def test_missing_fields_use_defaults(self):
        """测试缺失字段使用默认值"""
        msg = MockDiagnosticsMsg()
        diag = {'state': 1}  # 只有 state
        
        fill_diagnostics_msg(msg, diag)
        
        assert msg.state == 1
        assert msg.mpc_success == False
        assert msg.mpc_solve_time_ms == 0.0
        assert msg.consistency_data_valid == True
    
    def test_time_func(self):
        """测试时间函数"""
        msg = MockDiagnosticsMsg()
        diag = {'state': 1}
        
        mock_time = object()
        fill_diagnostics_msg(msg, diag, get_time_func=lambda: mock_time)
        
        assert msg.header.stamp is mock_time
    
    def test_error_message_fields(self):
        """测试错误信息字段"""
        msg = MockDiagnosticsMsg()
        diag = {
            'state': 0,
            'error_message': 'Test error occurred',
            'consecutive_errors': 5,
        }
        
        fill_diagnostics_msg(msg, diag)
        
        assert msg.error_message == 'Test error occurred'
        assert msg.consecutive_errors == 5
    
    def test_error_message_defaults(self):
        """测试错误信息字段默认值"""
        msg = MockDiagnosticsMsg()
        diag = {'state': 1}  # 无错误信息
        
        fill_diagnostics_msg(msg, diag)
        
        assert msg.error_message == ''
        assert msg.consecutive_errors == 0
    
    def test_transform_tf2_injected(self):
        """测试 tf2_injected 字段"""
        msg = MockDiagnosticsMsg()
        diag = {
            'state': 1,
            'transform': {
                'tf2_available': True,
                'tf2_injected': True,
            },
        }
        
        fill_diagnostics_msg(msg, diag)
        
        assert msg.transform_tf2_available == True
        assert msg.transform_tf2_injected == True


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
