"""状态机"""
from typing import Dict, Any, Optional, Callable
from collections import deque
import logging

from ..core.enums import ControllerState
from ..core.diagnostics_input import DiagnosticsInput
from ..core.ros_compat import get_monotonic_time

logger = logging.getLogger(__name__)


class StateMachine:
    """
    控制器状态机
    
    使用状态处理器模式，每个状态有独立的处理方法，
    提高可读性和可维护性。
    """
    
    def __init__(self, config: Dict[str, Any]):
        self.state = ControllerState.INIT
        self.alpha_recovery_count = 0
        self.mpc_recovery_count = 0
        
        safety_config = config.get('safety', {})
        sm_config = safety_config.get('state_machine', {})
        
        self.alpha_recovery_thresh = sm_config.get('alpha_recovery_thresh', 5)
        self.alpha_recovery_value = sm_config.get('alpha_recovery_value', 0.3)
        self.alpha_disable_thresh = sm_config.get('alpha_disable_thresh', 0.1)
        self.mpc_recovery_thresh = sm_config.get('mpc_recovery_thresh', 5)
        
        # MPC 失败检测参数
        self.mpc_fail_window_size = sm_config.get('mpc_fail_window_size', 10)
        self.mpc_fail_thresh = sm_config.get('mpc_fail_thresh', 3)
        self.mpc_fail_ratio_thresh = sm_config.get('mpc_fail_ratio_thresh', 0.5)
        self._mpc_success_history: deque = deque(maxlen=self.mpc_fail_window_size)
        
        # MPC 恢复检测参数
        self.mpc_recovery_history_min = sm_config.get('mpc_recovery_history_min', 3)
        self.mpc_recovery_recent_count = sm_config.get('mpc_recovery_recent_count', 5)
        self.mpc_recovery_tolerance = sm_config.get('mpc_recovery_tolerance', 1)  # 恢复容错次数
        self.mpc_recovery_success_ratio = sm_config.get('mpc_recovery_success_ratio', 0.8)  # 恢复所需成功率
        
        self.v_stop_thresh = safety_config.get('v_stop_thresh', 0.05)
        self.vz_stop_thresh = safety_config.get('vz_stop_thresh', 0.1)
        self.stopping_timeout = safety_config.get('stopping_timeout', 5.0)
        self._stopping_start_time: Optional[float] = None
        
        # 状态处理器映射
        self._state_handlers: Dict[ControllerState, Callable[[DiagnosticsInput], Optional[ControllerState]]] = {
            ControllerState.INIT: self._handle_init,
            ControllerState.NORMAL: self._handle_normal,
            ControllerState.SOFT_DISABLED: self._handle_soft_disabled,
            ControllerState.MPC_DEGRADED: self._handle_mpc_degraded,
            ControllerState.BACKUP_ACTIVE: self._handle_backup_active,
            ControllerState.STOPPING: self._handle_stopping,
            ControllerState.STOPPED: self._handle_stopped,
        }
    
    def _reset_all_counters(self) -> None:
        """重置所有计数器"""
        self.alpha_recovery_count = 0
        self.mpc_recovery_count = 0
        self._mpc_success_history.clear()
    
    def _transition_to(self, new_state: ControllerState) -> ControllerState:
        """执行状态转换"""
        self.state = new_state
        self._reset_all_counters()
        return new_state
    
    def _check_mpc_should_switch_to_backup(self, mpc_success: bool) -> bool:
        """检查是否应该切换到备用控制器"""
        self._mpc_success_history.append(mpc_success)
        
        if len(self._mpc_success_history) < self.mpc_fail_window_size:
            consecutive_failures = 0
            for success in reversed(self._mpc_success_history):
                if not success:
                    consecutive_failures += 1
                else:
                    break
            return consecutive_failures >= self.mpc_fail_thresh
        
        fail_count = sum(1 for s in self._mpc_success_history if not s)
        fail_ratio = fail_count / len(self._mpc_success_history)
        
        return (fail_count >= self.mpc_fail_thresh or 
                fail_ratio >= self.mpc_fail_ratio_thresh)
    
    def _check_mpc_can_recover(self) -> bool:
        """
        检查 MPC 是否可以恢复
        
        使用两种策略的组合:
        1. 绝对容错: 最近 N 次中允许最多 tolerance 次失败
        2. 比例要求: 成功率必须达到 mpc_recovery_success_ratio
        
        两个条件都满足才允许恢复，确保恢复决策的稳健性
        """
        history_len = len(self._mpc_success_history)
        if history_len < self.mpc_recovery_history_min:
            return False
        
        recent_count = min(self.mpc_recovery_recent_count, history_len)
        
        # 直接从 deque 末尾迭代，避免 list() 转换的内存分配
        # deque 支持负索引和迭代
        recent_successes = 0
        for i in range(recent_count):
            # 从末尾开始访问: -1, -2, -3, ...
            if self._mpc_success_history[-(i + 1)]:
                recent_successes += 1
        
        # 条件1: 绝对容错 - 失败次数不超过 tolerance
        failures = recent_count - recent_successes
        absolute_condition = failures <= self.mpc_recovery_tolerance
        
        # 条件2: 比例要求 - 成功率达到阈值
        success_ratio = recent_successes / recent_count if recent_count > 0 else 0.0
        ratio_condition = success_ratio >= self.mpc_recovery_success_ratio
        
        # 两个条件都满足才允许恢复
        return absolute_condition and ratio_condition
    
    def update(self, diagnostics: DiagnosticsInput) -> ControllerState:
        """更新状态机"""
        # 超时检查 - 最高优先级
        if diagnostics.odom_timeout or diagnostics.traj_timeout_exceeded:
            if self.state != ControllerState.STOPPING:
                self._stopping_start_time = get_monotonic_time()
                return self._transition_to(ControllerState.STOPPING)
            return self.state
        
        # 调用当前状态的处理器
        handler = self._state_handlers.get(self.state)
        if handler:
            new_state = handler(diagnostics)
            if new_state is not None and new_state != self.state:
                return self._transition_to(new_state)
        
        return self.state
    
    # ==================== 状态处理器 ====================
    
    def _handle_init(self, diag: DiagnosticsInput) -> Optional[ControllerState]:
        """处理 INIT 状态"""
        if diag.has_valid_data:
            return ControllerState.NORMAL
        return None
    
    def _handle_normal(self, diag: DiagnosticsInput) -> Optional[ControllerState]:
        """处理 NORMAL 状态"""
        if diag.safety_failed:
            return ControllerState.MPC_DEGRADED
        
        if self._check_mpc_should_switch_to_backup(diag.mpc_success):
            return ControllerState.BACKUP_ACTIVE
        
        if not diag.mpc_success:
            return None
        
        # MPC 成功，检查其他转换条件
        if self._should_disable_soft(diag):
            return ControllerState.SOFT_DISABLED
        
        if self._should_degrade_mpc(diag):
            return ControllerState.MPC_DEGRADED
        
        return None
    
    def _handle_soft_disabled(self, diag: DiagnosticsInput) -> Optional[ControllerState]:
        """处理 SOFT_DISABLED 状态"""
        if diag.safety_failed:
            return ControllerState.MPC_DEGRADED
        
        if not diag.mpc_success:
            return ControllerState.BACKUP_ACTIVE
        
        if self._should_degrade_mpc(diag):
            return ControllerState.MPC_DEGRADED
        
        # 尝试恢复到 NORMAL
        if diag.alpha > self.alpha_recovery_value and diag.data_valid:
            self.alpha_recovery_count += 1
            mpc_is_healthy = diag.mpc_health is not None and diag.mpc_health.healthy
            if self.alpha_recovery_count >= self.alpha_recovery_thresh and mpc_is_healthy:
                return ControllerState.NORMAL
        else:
            self.alpha_recovery_count = 0
        
        return None
    
    def _handle_mpc_degraded(self, diag: DiagnosticsInput) -> Optional[ControllerState]:
        """处理 MPC_DEGRADED 状态"""
        if not diag.mpc_success:
            return ControllerState.BACKUP_ACTIVE
        
        # 尝试恢复
        can_recover = (
            diag.mpc_health is not None and 
            diag.mpc_health.can_recover and 
            not diag.tf2_critical and 
            not diag.safety_failed
        )
        
        if can_recover:
            self.mpc_recovery_count += 1
            if self.mpc_recovery_count >= self.mpc_recovery_thresh:
                if diag.alpha >= self.alpha_disable_thresh and diag.data_valid:
                    return ControllerState.NORMAL
                else:
                    return ControllerState.SOFT_DISABLED
        else:
            self.mpc_recovery_count = 0
        
        return None
    
    def _handle_backup_active(self, diag: DiagnosticsInput) -> Optional[ControllerState]:
        """处理 BACKUP_ACTIVE 状态"""
        self._mpc_success_history.append(diag.mpc_success)
        
        can_try_recover = (
            diag.mpc_success and 
            not diag.tf2_critical and 
            not diag.safety_failed
        )
        
        if can_try_recover and self._check_mpc_can_recover():
            return ControllerState.MPC_DEGRADED
        
        return None
    
    def _handle_stopping(self, diag: DiagnosticsInput) -> Optional[ControllerState]:
        """处理 STOPPING 状态"""
        stopped = (
            abs(diag.v_horizontal) < self.v_stop_thresh and 
            abs(diag.vz) < self.vz_stop_thresh
        )
        
        if stopped:
            self._stopping_start_time = None
            return ControllerState.STOPPED
        
        if self._stopping_start_time is not None:
            elapsed = get_monotonic_time() - self._stopping_start_time
            if elapsed > self.stopping_timeout:
                logger.warning(f"STOPPING timeout after {elapsed:.1f}s, forcing STOPPED")
                self._stopping_start_time = None
                return ControllerState.STOPPED
        
        return None
    
    def _handle_stopped(self, diag: DiagnosticsInput) -> Optional[ControllerState]:
        """处理 STOPPED 状态"""
        can_resume = (
            diag.has_valid_data and 
            not diag.odom_timeout and 
            not diag.traj_timeout_exceeded
        )
        
        if can_resume:
            mpc_is_healthy = diag.mpc_health is not None and diag.mpc_health.healthy
            if mpc_is_healthy and not diag.safety_failed:
                return ControllerState.NORMAL
            else:
                return ControllerState.MPC_DEGRADED
        
        return None
    
    # ==================== 辅助方法 ====================
    
    def _should_disable_soft(self, diag: DiagnosticsInput) -> bool:
        """检查是否应该禁用 soft 模式"""
        if not diag.data_valid and diag.alpha < self.alpha_recovery_value:
            return True
        if diag.data_valid and diag.alpha < self.alpha_disable_thresh:
            return True
        return False
    
    def _should_degrade_mpc(self, diag: DiagnosticsInput) -> bool:
        """检查是否应该降级 MPC"""
        if diag.mpc_health is not None and diag.mpc_health.warning:
            return True
        if diag.tf2_critical:
            return True
        return False
    
    def reset(self) -> None:
        """重置状态机"""
        self.state = ControllerState.INIT
        self._reset_all_counters()
        self._stopping_start_time = None
