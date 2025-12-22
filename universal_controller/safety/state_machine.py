"""状态机"""
from typing import Dict, Any, Optional
from collections import deque
import logging

from ..core.enums import ControllerState
from ..core.diagnostics_input import DiagnosticsInput
from ..core.ros_compat import get_monotonic_time

logger = logging.getLogger(__name__)


class StateMachine:
    """控制器状态机"""
    
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
        # 使用滑动窗口方法：在最近 N 次调用中，如果失败次数超过阈值则切换
        self.mpc_fail_window_size = sm_config.get('mpc_fail_window_size', 10)  # 窗口大小
        self.mpc_fail_thresh = sm_config.get('mpc_fail_thresh', 3)  # 窗口内失败次数阈值
        self.mpc_fail_ratio_thresh = sm_config.get('mpc_fail_ratio_thresh', 0.5)  # 失败率阈值
        self._mpc_success_history: deque = deque(maxlen=self.mpc_fail_window_size)
        
        # MPC 恢复检测参数 - 从配置读取
        self.mpc_recovery_history_min = sm_config.get('mpc_recovery_history_min', 3)
        self.mpc_recovery_recent_count = sm_config.get('mpc_recovery_recent_count', 5)
        
        self.v_stop_thresh = safety_config.get('v_stop_thresh', 0.05)
        self.vz_stop_thresh = safety_config.get('vz_stop_thresh', 0.1)
        self.stopping_timeout = safety_config.get('stopping_timeout', 5.0)
        self._stopping_start_time: Optional[float] = None
    
    def _reset_all_counters(self) -> None:
        """重置所有计数器 - 在状态转换时调用以确保一致性"""
        self.alpha_recovery_count = 0
        self.mpc_recovery_count = 0
        self._mpc_success_history.clear()
    
    def _check_mpc_should_switch_to_backup(self, mpc_success: bool) -> bool:
        """
        检查是否应该切换到备用控制器
        
        使用滑动窗口方法：
        1. 记录最近 N 次 MPC 调用的成功/失败状态
        2. 如果窗口内失败次数超过阈值，或失败率超过阈值，则切换
        
        这种方法的优点：
        - 单次失败不会触发切换
        - 连续失败会快速触发切换
        - 间歇性失败（如 70% 成功率）会在窗口填满后稳定判断
        - 恢复后历史记录自然滑出窗口
        
        Args:
            mpc_success: 本次 MPC 是否成功
        
        Returns:
            是否应该切换到备用控制器
        """
        # 记录本次结果
        self._mpc_success_history.append(mpc_success)
        
        # 窗口未填满时，使用更保守的策略
        # 只有连续失败达到阈值才切换
        if len(self._mpc_success_history) < self.mpc_fail_window_size:
            # 计算最近连续失败次数
            consecutive_failures = 0
            for success in reversed(self._mpc_success_history):
                if not success:
                    consecutive_failures += 1
                else:
                    break
            return consecutive_failures >= self.mpc_fail_thresh
        
        # 窗口已填满，使用完整的判断逻辑
        fail_count = sum(1 for s in self._mpc_success_history if not s)
        fail_ratio = fail_count / len(self._mpc_success_history)
        
        # 两个条件满足其一即切换：
        # 1. 失败次数超过阈值
        # 2. 失败率超过阈值
        return (fail_count >= self.mpc_fail_thresh or 
                fail_ratio >= self.mpc_fail_ratio_thresh)
    
    def _check_mpc_can_recover(self) -> bool:
        """
        检查 MPC 是否可以从备用控制器恢复
        
        恢复条件：最近的成功率足够高
        
        Returns:
            是否可以尝试恢复到 MPC
        """
        if len(self._mpc_success_history) < self.mpc_recovery_history_min:
            # 历史记录太少，不恢复
            return False
        
        # 检查最近几次是否都成功
        recent_count = min(self.mpc_recovery_recent_count, len(self._mpc_success_history))
        recent_successes = sum(1 for s in list(self._mpc_success_history)[-recent_count:] if s)
        
        # 最近 N 次中至少 N-1 次成功才考虑恢复
        return recent_successes >= recent_count - 1
    
    def update(self, diagnostics: DiagnosticsInput) -> ControllerState:
        """更新状态机"""
        diag = diagnostics
        
        alpha = diag.alpha
        mpc_health = diag.mpc_health
        odom_timeout = diag.odom_timeout
        traj_timeout_exceeded = diag.traj_timeout_exceeded
        v_horizontal = diag.v_horizontal
        vz = diag.vz
        mpc_success = diag.mpc_success
        has_valid_data = diag.has_valid_data
        tf2_critical = diag.tf2_critical
        data_valid = diag.data_valid
        safety_failed = diag.safety_failed
        
        # 超时 -> STOPPING (最高优先级)
        if odom_timeout or traj_timeout_exceeded:
            if self.state != ControllerState.STOPPING:
                self._stopping_start_time = get_monotonic_time()
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
            else:
                # 使用滑动窗口方法检测 MPC 失败
                should_switch_to_backup = self._check_mpc_should_switch_to_backup(mpc_success)
                
                if should_switch_to_backup:
                    self.state = ControllerState.BACKUP_ACTIVE
                    self._reset_all_counters()
                elif mpc_success:
                    # MPC 成功，检查其他状态转换条件
                    if not data_valid and alpha < self.alpha_recovery_value:
                        self.state = ControllerState.SOFT_DISABLED
                        self._reset_all_counters()
                    elif data_valid and alpha < self.alpha_disable_thresh:
                        self.state = ControllerState.SOFT_DISABLED
                        self._reset_all_counters()
                    elif (mpc_health is not None and mpc_health.warning) or tf2_critical:
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
            
            if (mpc_health is not None and mpc_health.warning) or tf2_critical:
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
            if mpc_health is not None and mpc_health.can_recover and not tf2_critical and not safety_failed:
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
            # 记录 MPC 成功/失败状态到滑动窗口
            self._mpc_success_history.append(mpc_success)
            
            # 尝试恢复到 MPC
            # 使用滑动窗口方法检查是否可以恢复
            if mpc_success and not tf2_critical and not safety_failed:
                if self._check_mpc_can_recover():
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
                elapsed = get_monotonic_time() - self._stopping_start_time
                if elapsed > self.stopping_timeout:
                    logger.warning(f"STOPPING timeout after {elapsed:.1f}s, forcing STOPPED")
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
