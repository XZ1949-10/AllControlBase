"""状态机"""
from typing import Dict, Any, Optional
import time

from ..core.enums import ControllerState


def _get_monotonic_time() -> float:
    """
    获取单调时钟时间（秒）
    
    使用 time.monotonic() 而非 time.time()，避免系统时间跳变
    （如 NTP 同步）导致的时间间隔计算错误。
    """
    return time.monotonic()


class StateMachine:
    """控制器状态机"""
    
    def __init__(self, config: Dict[str, Any]):
        self.state = ControllerState.INIT
        self.alpha_recovery_count = 0
        self.mpc_recovery_count = 0
        self.mpc_fail_count: float = 0.0  # MPC 失败计数器（使用浮点数支持衰减）
        
        safety_config = config.get('safety', {})
        sm_config = safety_config.get('state_machine', {})
        
        self.alpha_recovery_thresh = sm_config.get('alpha_recovery_thresh', 5)
        self.alpha_recovery_value = sm_config.get('alpha_recovery_value', 0.3)
        self.alpha_disable_thresh = sm_config.get('alpha_disable_thresh', 0.1)
        self.mpc_recovery_thresh = sm_config.get('mpc_recovery_thresh', 5)
        self.mpc_fail_thresh = sm_config.get('mpc_fail_thresh', 3)  # 失败阈值
        self.mpc_fail_decay = sm_config.get('mpc_fail_decay', 0.5)  # 成功时的衰减因子
        self.v_stop_thresh = safety_config.get('v_stop_thresh', 0.05)
        self.vz_stop_thresh = safety_config.get('vz_stop_thresh', 0.1)
        self.stopping_timeout = safety_config.get('stopping_timeout', 5.0)
        self._stopping_start_time: Optional[float] = None
    
    def _reset_all_counters(self) -> None:
        """重置所有计数器 - 在状态转换时调用以确保一致性"""
        self.alpha_recovery_count = 0
        self.mpc_recovery_count = 0
        self.mpc_fail_count = 0.0
    
    def update(self, diagnostics: Dict[str, Any]) -> ControllerState:
        alpha = diagnostics.get('alpha', 1.0)
        mpc_health = diagnostics.get('mpc_health')
        odom_timeout = diagnostics.get('odom_timeout', False)
        traj_timeout_exceeded = diagnostics.get('traj_timeout_exceeded', False)
        v_horizontal = diagnostics.get('v_horizontal', 0)
        vz = diagnostics.get('vz', 0)
        mpc_success = diagnostics.get('mpc_success', False)
        has_valid_data = diagnostics.get('has_valid_data', False)
        tf2_critical = diagnostics.get('tf2_critical', False)
        data_valid = diagnostics.get('data_valid', True)
        safety_failed = diagnostics.get('safety_failed', False)
        
        # 超时 -> STOPPING (最高优先级)
        if odom_timeout or traj_timeout_exceeded:
            if self.state != ControllerState.STOPPING:
                self._stopping_start_time = _get_monotonic_time()
                self.state = ControllerState.STOPPING
                self._reset_all_counters()
            return self.state
        
        # 状态转换逻辑
        if self.state == ControllerState.INIT:
            if has_valid_data:
                self.state = ControllerState.NORMAL
                self._reset_all_counters()
        
        elif self.state == ControllerState.NORMAL:
            if safety_failed:
                self.state = ControllerState.MPC_DEGRADED
                self._reset_all_counters()
            elif not mpc_success:
                # 使用失败计数器，避免单次失败就切换
                self.mpc_fail_count += 1
                if self.mpc_fail_count >= self.mpc_fail_thresh:
                    self.state = ControllerState.BACKUP_ACTIVE
                    self._reset_all_counters()
            else:
                # MPC 成功，使用衰减而非立即重置
                # 这样可以处理间歇性失败的情况
                # 例如：失败-成功-失败-成功-失败 不会触发切换
                # 但：失败-失败-成功-失败-失败-失败 会触发切换
                self.mpc_fail_count = max(0, self.mpc_fail_count - self.mpc_fail_decay)
                
                if not data_valid and alpha < self.alpha_recovery_value:
                    self.state = ControllerState.SOFT_DISABLED
                    self._reset_all_counters()
                elif data_valid and alpha < self.alpha_disable_thresh:
                    self.state = ControllerState.SOFT_DISABLED
                    self._reset_all_counters()
                elif (mpc_health and mpc_health.warning) or tf2_critical:
                    self.state = ControllerState.MPC_DEGRADED
                    self._reset_all_counters()
        
        elif self.state == ControllerState.SOFT_DISABLED:
            if safety_failed:
                self.state = ControllerState.MPC_DEGRADED
                self._reset_all_counters()
                return self.state
            
            if not mpc_success:
                self.state = ControllerState.BACKUP_ACTIVE
                self._reset_all_counters()
                return self.state
            
            if (mpc_health and mpc_health.warning) or tf2_critical:
                self.state = ControllerState.MPC_DEGRADED
                self._reset_all_counters()
                return self.state
            
            # 尝试恢复到 NORMAL
            if alpha > self.alpha_recovery_value and data_valid:
                self.alpha_recovery_count += 1
                # mpc_health 为 None 表示 MPC 未运行或未初始化
                # 在这种情况下，我们应该保守地认为 MPC 不健康
                # 只有当 mpc_health 存在且 healthy=True 时才认为健康
                mpc_is_healthy = mpc_health is not None and mpc_health.healthy
                if self.alpha_recovery_count >= self.alpha_recovery_thresh and mpc_is_healthy:
                    self.state = ControllerState.NORMAL
                    self._reset_all_counters()
            else:
                self.alpha_recovery_count = 0
        
        elif self.state == ControllerState.MPC_DEGRADED:
            if not mpc_success:
                self.state = ControllerState.BACKUP_ACTIVE
                self._reset_all_counters()
                return self.state
            
            # 尝试恢复
            if mpc_health and mpc_health.can_recover and not tf2_critical and not safety_failed:
                self.mpc_recovery_count += 1
                if self.mpc_recovery_count >= self.mpc_recovery_thresh:
                    if alpha >= self.alpha_disable_thresh and data_valid:
                        self.state = ControllerState.NORMAL
                    else:
                        self.state = ControllerState.SOFT_DISABLED
                    self._reset_all_counters()
            else:
                self.mpc_recovery_count = 0
        
        elif self.state == ControllerState.BACKUP_ACTIVE:
            # 尝试恢复到 MPC
            if mpc_success and not tf2_critical and not safety_failed:
                self.state = ControllerState.MPC_DEGRADED
                self._reset_all_counters()
        
        elif self.state == ControllerState.STOPPING:
            stopped = (abs(v_horizontal) < self.v_stop_thresh and 
                      abs(vz) < self.vz_stop_thresh)
            
            if stopped:
                self.state = ControllerState.STOPPED
                self._stopping_start_time = None
                self._reset_all_counters()
            elif self._stopping_start_time is not None:
                elapsed = _get_monotonic_time() - self._stopping_start_time
                if elapsed > self.stopping_timeout:
                    print(f"STOPPING timeout after {elapsed:.1f}s, forcing STOPPED")
                    self.state = ControllerState.STOPPED
                    self._stopping_start_time = None
                    self._reset_all_counters()
        
        elif self.state == ControllerState.STOPPED:
            # 从 STOPPED 恢复
            if has_valid_data and not odom_timeout and not traj_timeout_exceeded:
                # mpc_health 为 None 表示 MPC 未运行或未初始化
                # 从 STOPPED 恢复时，如果 MPC 状态未知，应该先进入 MPC_DEGRADED 观察
                mpc_is_healthy = mpc_health is not None and mpc_health.healthy
                if mpc_is_healthy and not safety_failed:
                    self.state = ControllerState.NORMAL
                else:
                    self.state = ControllerState.MPC_DEGRADED
                self._reset_all_counters()  # 确保从 STOPPED 恢复时重置所有计数器
        
        return self.state
    
    def reset(self) -> None:
        self.state = ControllerState.INIT
        self._reset_all_counters()
        self._stopping_start_time = None
