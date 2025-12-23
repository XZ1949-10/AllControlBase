#!/usr/bin/env python3
"""
测试 ROSDashboardDataSource

验证 ROS 数据源能正确转换诊断消息为 DisplayData。
"""

import pytest
import sys
import os

# 添加路径
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))


class MockDiagnosticsV2:
    """模拟 DiagnosticsV2 消息"""
    def __init__(self):
        self.state = 1
        self.mpc_success = True
        self.mpc_solve_time_ms = 5.5
        self.backup_active = False
        
        # MPC 健康
        self.mpc_health_kkt_residual = 0.001
        self.mpc_health_condition_number = 100.0
        self.mpc_health_consecutive_near_timeout = 0
        self.mpc_health_degradation_warning = False
        self.mpc_health_can_recover = True
        
        # 一致性
        self.consistency_curvature = 0.5
        self.consistency_velocity_dir = 0.9
        self.consistency_temporal = 0.95
        self.consistency_alpha_soft = 0.8
        self.consistency_data_valid = True
        
        # 状态估计器
        self.estimator_covariance_norm = 0.01
        self.estimator_innovation_norm = 0.02
        self.estimator_slip_probability = 0.1
        self.estimator_imu_drift_detected = False
        self.estimator_imu_bias = [0.001, 0.002, 0.003]
        self.estimator_imu_available = True
        
        # 跟踪
        self.tracking_lateral_error = 0.05
        self.tracking_longitudinal_error = 0.1
        self.tracking_heading_error = 0.02
        self.tracking_prediction_error = 0.03

        # 坐标变换
        self.transform_tf2_available = True
        self.transform_tf2_injected = True
        self.transform_fallback_duration_ms = 0.0
        self.transform_accumulated_drift = 0.0
        
        # 超时
        self.timeout_odom = False
        self.timeout_traj = False
        self.timeout_traj_grace_exceeded = False
        self.timeout_imu = False
        self.timeout_last_odom_age_ms = 10.0
        self.timeout_last_traj_age_ms = 50.0
        self.timeout_last_imu_age_ms = 5.0
        self.timeout_in_startup_grace = False
        
        # 控制命令
        self.cmd_vx = 1.0
        self.cmd_vy = 0.0
        self.cmd_vz = 0.0
        self.cmd_omega = 0.1
        self.cmd_frame_id = 'base_link'
        
        self.transition_progress = 1.0
        self.error_message = ''
        self.consecutive_errors = 0


class TestROSDashboardDataSource:
    """测试 ROSDashboardDataSource"""
    
    def test_convert_diagnostics_msg(self):
        """测试诊断消息转换"""
        # 由于 ROSDashboardDataSource 依赖 ROS，这里只测试转换逻辑
        from universal_controller.dashboard.ros_data_source import ROSDashboardDataSource
        
        # 创建数据源（不初始化 ROS 订阅）
        data_source = ROSDashboardDataSource.__new__(ROSDashboardDataSource)
        data_source._config = {'system': {'platform': 'differential'}}
        
        # 模拟消息
        msg = MockDiagnosticsV2()
        
        # 转换
        diag = data_source._convert_diagnostics_msg(msg)
        
        # 验证基本字段
        assert diag['state'] == 1
        assert diag['mpc_success'] == True
        assert diag['mpc_solve_time_ms'] == 5.5
        assert diag['backup_active'] == False

        # 验证 MPC 健康
        assert diag['mpc_health']['kkt_residual'] == 0.001
        assert diag['mpc_health']['condition_number'] == 100.0
        
        # 验证一致性
        assert diag['consistency']['alpha_soft'] == 0.8
        assert diag['consistency']['curvature'] == 0.5
        
        # 验证跟踪
        assert diag['tracking']['lateral_error'] == 0.05
        
        # 验证超时
        assert diag['timeout']['odom_timeout'] == False
        assert diag['timeout']['last_odom_age_ms'] == 10.0
        
        # 验证控制命令
        assert diag['cmd']['vx'] == 1.0
        assert diag['cmd']['omega'] == 0.1

    def test_build_controller_status(self):
        """测试控制器状态构建"""
        from universal_controller.dashboard.ros_data_source import ROSDashboardDataSource
        from universal_controller.dashboard.models import ControllerStateEnum
        
        data_source = ROSDashboardDataSource.__new__(ROSDashboardDataSource)
        data_source._config = {}
        
        diag = {
            'state': 1,
            'mpc_success': True,
            'backup_active': False,
            'consistency': {'alpha_soft': 0.8},
        }
        
        status = data_source._build_controller_status(diag)
        
        assert status.state == ControllerStateEnum.NORMAL
        assert status.state_name == 'NORMAL'
        assert status.mpc_success == True
        assert status.backup_active == False
        assert status.soft_head_enabled == True  # alpha > 0.1
        assert status.alpha_soft == 0.8

    def test_build_mpc_health(self):
        """测试 MPC 健康状态构建"""
        from universal_controller.dashboard.ros_data_source import ROSDashboardDataSource
        
        data_source = ROSDashboardDataSource.__new__(ROSDashboardDataSource)
        
        diag = {
            'mpc_success': True,
            'mpc_solve_time_ms': 5.5,
            'mpc_health': {
                'kkt_residual': 0.001,
                'condition_number': 100.0,
                'degradation_warning': False,
            },
        }
        
        health = data_source._build_mpc_health(diag)
        
        assert health.kkt_residual == 0.001
        assert health.condition_number == 100.0
        assert health.solve_time_ms == 5.5
        assert health.healthy == True


    def test_build_timeout_status(self):
        """测试超时状态构建"""
        from universal_controller.dashboard.ros_data_source import ROSDashboardDataSource
        
        data_source = ROSDashboardDataSource.__new__(ROSDashboardDataSource)
        
        diag = {
            'timeout': {
                'odom_timeout': True,
                'traj_timeout': False,
                'last_odom_age_ms': 250.0,
                'last_traj_age_ms': 100.0,
            },
        }
        
        timeout = data_source._build_timeout_status(diag)
        
        assert timeout.odom_timeout == True
        assert timeout.traj_timeout == False
        assert timeout.last_odom_age_ms == 250.0

    def test_build_platform_config(self):
        """测试平台配置构建"""
        from universal_controller.dashboard.ros_data_source import ROSDashboardDataSource
        
        data_source = ROSDashboardDataSource.__new__(ROSDashboardDataSource)
        data_source._config = {
            'system': {
                'platform': 'quadrotor',
                'ctrl_freq': 100,
            },
            'mpc': {
                'horizon': 30,
                'horizon_degraded': 15,
                'dt': 0.05,
            },
        }
        
        platform = data_source._build_platform_config()
        
        assert platform.platform == 'quadrotor'
        assert platform.platform_display == '四旋翼'
        assert platform.ctrl_freq == 100
        assert platform.mpc_horizon == 30

    def test_empty_diagnostics(self):
        """测试空诊断数据处理"""
        from universal_controller.dashboard.ros_data_source import ROSDashboardDataSource
        
        data_source = ROSDashboardDataSource.__new__(ROSDashboardDataSource)
        data_source._config = {}
        
        # 空字典
        status = data_source._build_controller_status({})
        assert status.state_name == 'INIT'
        
        health = data_source._build_mpc_health({})
        assert health.healthy == False
        
        timeout = data_source._build_timeout_status({})
        assert timeout.odom_timeout == False


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
