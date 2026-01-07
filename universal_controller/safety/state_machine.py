"""状态机"""
from typing import Dict, Any, Optional, Callable
from collections import deque
import logging
import threading
import numpy as np

from ..core.enums import ControllerState
from ..core.diagnostics_input import DiagnosticsInput
from ..core.ros_compat import get_monotonic_time

logger = logging.getLogger(__name__)


class StateMachine:
    """
    控制器状态机
    
    使用状态处理器模式，每个状态有独立的处理方法，
    提高可读性和可维护性。
    
    外部请求机制:
    - request_stop(): 请求紧急停止，这是唯一允许的外部状态干预
    - clear_stop_request(): 清除停止请求（用于恢复）
    - 其他状态转换由内部逻辑自动控制
    
    线程安全性说明:
    - update() 方法不是线程安全的，应该在单个线程（控制循环）中调用
    - request_stop() 是线程安全的，可以从任何线程调用
      - 使用 threading.Event 确保原子性和可见性
      - 实际状态转换在 update() 中统一处理
    - is_stop_requested() 是线程安全的
    - 如果需要完全的线程安全，调用者应在外部加锁
    """
    
    def __init__(self, config: Dict[str, Any]):
        self.state = ControllerState.INIT
        self.alpha_recovery_count = 0
        self.mpc_recovery_count = 0
        
        safety_config = config.get('safety', {})
        sm_config = safety_config.get('state_machine', {})
        
        # Alpha 恢复参数
        self.alpha_recovery_thresh = sm_config['alpha_recovery_thresh']
        self.alpha_recovery_value = sm_config['alpha_recovery_value']
        self.alpha_disable_thresh = sm_config['alpha_disable_thresh']
        
        # MPC 恢复参数
        self.mpc_recovery_thresh = sm_config['mpc_recovery_thresh']
        
        # MPC 失败检测参数
        self.mpc_fail_window_size = sm_config['mpc_fail_window_size']
        self.mpc_fail_thresh = sm_config['mpc_fail_thresh']
        self.mpc_fail_ratio_thresh = sm_config['mpc_fail_ratio_thresh']
        self._mpc_success_history: deque = deque(maxlen=self.mpc_fail_window_size)
        
        # MPC 恢复检测参数
        self.mpc_recovery_history_min = sm_config['mpc_recovery_history_min']
        self.mpc_recovery_recent_count = sm_config['mpc_recovery_recent_count']
        self.mpc_recovery_tolerance = sm_config['mpc_recovery_tolerance']
        self.mpc_recovery_success_ratio = sm_config['mpc_recovery_success_ratio']
        
        # 安全参数
        self.v_stop_thresh = safety_config.get('v_stop_thresh', 0.05)
        self.vz_stop_thresh = safety_config.get('vz_stop_thresh', 0.1)
        self.stopping_timeout = safety_config.get('stopping_timeout', 5.0)
        self._stopping_start_time: Optional[float] = None
        
        # 外部停止请求 - 使用 threading.Event 确保线程安全
        self._stop_event = threading.Event()
        
        # 状态持续时间监控参数
        self._state_entry_time: Optional[float] = get_monotonic_time()  # 初始化时设置进入时间
        self._degraded_state_timeout = sm_config['degraded_state_timeout']
        self._backup_state_timeout = sm_config['backup_state_timeout']
        self._enable_state_timeout_stop = sm_config['enable_state_timeout_stop']
        
        # 状态超时回调（可选，用于外部告警系统）
        self._state_timeout_callback: Optional[Callable[[ControllerState, float], None]] = None
        # 已触发超时告警的状态（避免重复告警）
        self._timeout_warned_states: set = set()

        # NaN 速度警告标志（避免日志泛滥）
        self._nan_velocity_warned: bool = False

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
    
    def request_stop(self) -> bool:
        """
        请求紧急停止
        
        这是唯一允许的外部状态干预。调用后，状态机会在下一次 update() 时
        转换到 STOPPING 状态。
        
        线程安全性:
        - 此方法是线程安全的，可以从任何线程调用
        - 使用 threading.Event 确保原子性和跨线程可见性
        - 实际状态转换在 update() 中进行，确保状态一致性
        
        设计说明:
        - 使用 Event 而非布尔标志，提供更强的线程安全保证
        - 状态转换在 update() 中统一处理，确保计数器正确重置
        - 这是安全关键功能，用于紧急停止场景
        
        Returns:
            True: 请求已接受，将在下次 update() 时转换到 STOPPING 状态
            False: 请求被忽略，因为系统已经在 STOPPING 或 STOPPED 状态
        """
        # 如果已经在停止过程中，返回 False 表示请求无效
        # 这让调用者知道系统已经在处理停止
        # 注意：这里读取 self.state 不是原子的，但这是可接受的：
        # - 最坏情况是在状态刚变为 STOPPING 时返回 True
        # - 这不会导致问题，因为 update() 会正确处理重复的停止请求
        if self.state in (ControllerState.STOPPING, ControllerState.STOPPED):
            logger.debug(f"Stop request ignored: already in {self.state.name} state")
            return False
        
        self._stop_event.set()
        logger.info("Stop requested via external interface")
        return True
    
    def clear_stop_request(self) -> None:
        """
        清除停止请求
        
        用于在系统恢复后清除未处理的停止请求。
        通常在 reset() 中自动调用。
        
        线程安全性:
        - 此方法是线程安全的
        """
        self._stop_event.clear()
    
    def is_stop_requested(self) -> bool:
        """
        检查是否有外部停止请求
        
        线程安全性:
        - 此方法是线程安全的
        """
        return self._stop_event.is_set()
    
    def _reset_all_counters(self) -> None:
        """
        重置所有恢复计数器和 MPC 历史
        
        在状态转换时调用，确保新状态从干净的监控周期开始。
        
        设计说明:
        - 状态转换意味着进入新的监控周期，旧历史不应影响新状态的判断
        - 这是保守的安全设计：恢复决策基于当前表现而非历史数据
        - 虽然可能导致恢复时间较长，但确保了恢复决策的可靠性
        """
        self.alpha_recovery_count = 0
        self.mpc_recovery_count = 0
        # 清空 MPC 历史：状态转换意味着进入新的监控周期
        # 旧的历史数据不应影响新状态的判断
        self._mpc_success_history.clear()
    
    def _transition_to(self, new_state: ControllerState) -> ControllerState:
        """执行状态转换"""
        old_state = self.state
        self.state = new_state
        self._reset_all_counters()
        # 注意：_reset_all_counters 已经清空了 MPC 历史
        # 不需要额外的条件判断
        
        # 更新状态进入时间
        self._state_entry_time = get_monotonic_time()
        # 清除该状态的超时告警标记（允许下次进入时重新告警）
        self._timeout_warned_states.discard(new_state)
        
        return new_state
    
    def set_state_timeout_callback(self, callback: Callable[[ControllerState, float], None]) -> None:
        """
        设置状态超时回调函数
        
        当状态持续时间超过阈值时，会调用此回调函数。
        回调函数签名: callback(state: ControllerState, duration: float)
        
        Args:
            callback: 回调函数，接收当前状态和持续时间（秒）
        """
        self._state_timeout_callback = callback
    
    def _check_state_duration(self) -> Optional[ControllerState]:
        """
        检查状态持续时间，返回是否需要转换状态
        
        Returns:
            如果需要因超时而转换状态，返回目标状态；否则返回 None
        """
        if self._state_entry_time is None:
            return None
        
        duration = get_monotonic_time() - self._state_entry_time
        timeout = None
        
        # 根据当前状态确定超时阈值
        if self.state == ControllerState.MPC_DEGRADED:
            timeout = self._degraded_state_timeout
        elif self.state == ControllerState.BACKUP_ACTIVE:
            timeout = self._backup_state_timeout
        
        if timeout is None or timeout <= 0:
            return None  # 该状态不需要超时检查或超时已禁用
        
        if duration > timeout:
            # 触发超时告警（每个状态周期只告警一次）
            if self.state not in self._timeout_warned_states:
                self._timeout_warned_states.add(self.state)
                logger.warning(
                    f"State {self.state.name} duration ({duration:.1f}s) exceeded timeout ({timeout:.1f}s)"
                )
                # 调用外部回调（如果设置）
                if self._state_timeout_callback is not None:
                    try:
                        self._state_timeout_callback(self.state, duration)
                    except Exception as e:
                        logger.error(f"State timeout callback error: {e}")
            
            # 如果启用了超时自动停止，返回 STOPPING 状态
            if self._enable_state_timeout_stop:
                logger.warning(f"Auto-stopping due to {self.state.name} timeout")
                self._stopping_start_time = get_monotonic_time()
                return ControllerState.STOPPING
        
        return None
    
    def get_state_duration(self) -> float:
        """
        获取当前状态的持续时间（秒）
        
        Returns:
            当前状态的持续时间，如果状态刚进入则返回 0
        """
        if self._state_entry_time is None:
            return 0.0
        return get_monotonic_time() - self._state_entry_time
    
    def _check_mpc_should_switch_to_backup(self) -> bool:
        """检查是否应该切换到备用控制器
        
        注意: MPC 成功历史已在 update() 中统一追加
        """
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
        
        使用分层恢复策略:
        
        1. 快速恢复路径: 如果最近 N 次全部成功，立即允许恢复
           - 目的: 短暂故障后快速恢复，减少不必要的降级时间
           - 条件: recent_successes == recent_count
        
        2. 标准恢复路径: 使用两种互补策略的组合
           a. 绝对容错 (tolerance): 最近 N 次中允许最多 tolerance 次失败
              - 目的: 确保最近的表现稳定，没有连续失败
              - 默认 tolerance=1 表示最近 N 次最多 1 次失败
           b. 比例要求 (success_ratio): 整体成功率必须达到阈值
              - 目的: 确保整体表现良好，不是偶然成功
              - 默认 0.8 表示需要 80% 的成功率
        
        设计说明:
        - 快速恢复路径优先，减少不必要的降级时间
        - 标准恢复路径作为后备，确保恢复决策的稳健性
        - tolerance 和 success_ratio 应该配置为互补关系
        """
        history_len = len(self._mpc_success_history)
        if history_len < self.mpc_recovery_history_min:
            return False
        
        recent_count = min(self.mpc_recovery_recent_count, history_len)
        
        # 直接从 deque 末尾迭代，避免 list() 转换的内存分配
        recent_successes = 0
        for i in range(recent_count):
            if self._mpc_success_history[-(i + 1)]:
                recent_successes += 1
        
        # 快速恢复路径: 最近 N 次全部成功，立即恢复
        if recent_successes == recent_count:
            return True
        
        # 标准恢复路径: 绝对容错 + 比例要求
        failures = recent_count - recent_successes
        absolute_condition = failures <= self.mpc_recovery_tolerance
        
        success_ratio = recent_successes / recent_count if recent_count > 0 else 0.0
        ratio_condition = success_ratio >= self.mpc_recovery_success_ratio
        
        return absolute_condition and ratio_condition
    
    def update(self, diagnostics: DiagnosticsInput) -> ControllerState:
        """更新状态机
        
        MPC 历史管理说明:
        - 在 update() 开头追加当前帧的 MPC 结果，确保每次只追加一次
        - 状态处理器使用包含当前帧的历史做决策
        - 状态转换时清空历史，新状态从干净的监控周期开始
        - 这意味着当前帧的 MPC 结果参与了转换决策，但不会影响新状态的历史
        - 只在 MPC 实际运行的状态下追加历史，避免 STOPPING/STOPPED 状态污染历史
        """
        # 只在 MPC 实际运行的状态下追加历史
        # STOPPING 和 STOPPED 状态下 MPC 不运行，不应追加 False 到历史
        # 这避免了从 STOPPED 恢复时历史被污染的问题
        if self.state not in (ControllerState.STOPPING, ControllerState.STOPPED):
            self._mpc_success_history.append(diagnostics.mpc_success)
        
        # 外部停止请求 - 最高优先级
        # 使用 is_set() 检查并在处理后 clear()
        if self._stop_event.is_set():
            self._stop_event.clear()  # 清除请求标志
            if self.state != ControllerState.STOPPING:
                self._stopping_start_time = get_monotonic_time()
                logger.info("Transitioning to STOPPING due to external request")
                return self._transition_to(ControllerState.STOPPING)
            return self.state
        
        # 超时检查 - 次高优先级
        if diagnostics.odom_timeout or diagnostics.traj_timeout_exceeded:
            if self.state != ControllerState.STOPPING:
                self._stopping_start_time = get_monotonic_time()
                return self._transition_to(ControllerState.STOPPING)
            return self.state
        
        # 状态持续时间检查 - 检测长时间处于降级状态
        timeout_transition = self._check_state_duration()
        if timeout_transition is not None:
            return self._transition_to(timeout_transition)
        
        # 调用当前状态的处理器
        handler = self._state_handlers.get(self.state)
        if handler:
            new_state = handler(diagnostics)
            if new_state is not None and new_state != self.state:
                return self._transition_to(new_state)
        
        return self.state
    
    # ==================== 状态处理器 ====================
    
    def _handle_init(self, diag: DiagnosticsInput) -> Optional[ControllerState]:
        """处理 INIT 状态
        
        启动策略说明:
        ===============
        INIT 状态直接进入 NORMAL，而非 MPC_DEGRADED。
        
        设计原因:
        1. 启动时 MPC 通常是健康的（刚初始化）
        2. 如果 MPC 不健康，_handle_normal 会在第一帧检测并降级
        3. 用户期望系统快速启动，不需要额外的观察期
        
        与 _handle_stopped 的区别:
        - INIT: 系统刚启动，MPC 状态未知但通常健康
        - STOPPED: 异常后恢复，MPC 可能不稳定，需要观察期
        """
        if diag.has_valid_data:
            return ControllerState.NORMAL
        return None
    
    def _check_soft_recovery_to_normal(self, diag: DiagnosticsInput) -> bool:
        """检查是否可以从 SOFT_DISABLED 恢复到 NORMAL"""
        if diag.alpha > self.alpha_recovery_value and diag.data_valid:
            self.alpha_recovery_count += 1
            mpc_is_healthy = diag.mpc_health is not None and diag.mpc_health.healthy
            if self.alpha_recovery_count >= self.alpha_recovery_thresh and mpc_is_healthy:
                return True
        else:
            self.alpha_recovery_count = 0
        return False

    def _check_degraded_recovery(self, diag: DiagnosticsInput) -> Optional[ControllerState]:
        """检查是否可以从 MPC_DEGRADED 恢复"""
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

    def _handle_normal(self, diag: DiagnosticsInput) -> Optional[ControllerState]:
        """处理 NORMAL 状态"""
        if diag.safety_failed:
            return ControllerState.MPC_DEGRADED
        
        if self._check_mpc_should_switch_to_backup():
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

        # 使用与 NORMAL 状态相同的窗口检测逻辑，保持一致性
        if self._check_mpc_should_switch_to_backup():
            return ControllerState.BACKUP_ACTIVE

        if self._should_degrade_mpc(diag):
            return ControllerState.MPC_DEGRADED

        # 尝试恢复到 NORMAL
        if self._check_soft_recovery_to_normal(diag):
            return ControllerState.NORMAL

        return None
    
    def _handle_mpc_degraded(self, diag: DiagnosticsInput) -> Optional[ControllerState]:
        """处理 MPC_DEGRADED 状态"""
        if not diag.mpc_success:
            return ControllerState.BACKUP_ACTIVE
        
        # 尝试恢复
        return self._check_degraded_recovery(diag)
    
    def _handle_backup_active(self, diag: DiagnosticsInput) -> Optional[ControllerState]:
        """处理 BACKUP_ACTIVE 状态
        
        注意: MPC 成功历史已在 update() 中统一追加
        """
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
        # 安全关键: 检查速度值是否为 NaN/Inf
        # NaN 在比较时总是返回 False，会导致 stopped 永远为 False
        # 如果速度是 NaN，视为未停止，依赖超时机制强制停止
        v_horizontal_valid = np.isfinite(diag.v_horizontal)
        vz_valid = np.isfinite(diag.vz)

        if not v_horizontal_valid or not vz_valid:
            # 使用节流日志避免泛滥（每5秒最多记录一次）
            if not self._nan_velocity_warned:
                logger.warning(
                    f"NaN/Inf detected in velocity during STOPPING: "
                    f"v_horizontal={diag.v_horizontal}, vz={diag.vz}. "
                    f"Relying on timeout for safety."
                )
                self._nan_velocity_warned = True
            # 不认为已停止，让超时机制处理
            stopped = False
        else:
            # 速度有效时重置警告标志
            self._nan_velocity_warned = False
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
        """处理 STOPPED 状态
        
        恢复策略说明:
        ===============
        从 STOPPED 恢复时，始终先进入 MPC_DEGRADED 状态进行观察。
        
        设计原因:
        1. STOPPED 状态下 MPC 不运行，没有历史数据可供判断稳定性
        2. 直接进入 NORMAL 过于激进，MPC 可能不稳定
        3. 先进入 MPC_DEGRADED 可以:
           - 使用降级的 horizon 减少计算负担
           - 积累 MPC 成功历史
           - 通过 mpc_recovery_thresh 验证稳定性后再恢复到 NORMAL
        
        这与 _handle_backup_active 的恢复策略一致:
        BACKUP_ACTIVE -> MPC_DEGRADED -> NORMAL/SOFT_DISABLED
        """
        can_resume = (
            diag.has_valid_data and 
            not diag.odom_timeout and 
            not diag.traj_timeout_exceeded
        )
        
        if can_resume:
            # 始终先进入 MPC_DEGRADED 进行观察
            # 在 MPC_DEGRADED 中会通过 mpc_recovery_thresh 验证稳定性
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
        if diag.mpc_health is not None and diag.mpc_health.degradation_warning:
            return True
        if diag.tf2_critical:
            return True
        return False
    
    def reset_counters_only(self) -> None:
        """
        仅重置内部计数器和历史，不改变状态
        
        用于暂停恢复场景。与 reset() 不同：
        - 保留当前状态 (self.state)
        - 保留停止请求 (_stop_event)
        - 仅清除过时的历史数据和计数器
        
        这确保了：
        - 如果暂停前在 BACKUP_ACTIVE，恢复后仍在 BACKUP_ACTIVE（安全）
        - 如果暂停前在 STOPPING，恢复后继续停止流程（安全关键）
        - MPC 历史和恢复计数器被清空，避免基于过时数据做决策
        """
        self._reset_all_counters()  # 清空 MPC 历史和恢复计数器
        self._stopping_start_time = None  # 重置停止计时器（暂停时间不应计入）
        # 重新开始状态持续时间监控（暂停时间不应计入状态超时）
        self._state_entry_time = get_monotonic_time()
        self._timeout_warned_states.clear()
        self._nan_velocity_warned = False
        # 注意：不清除 _stop_event，保持停止请求有效
    
    def reset(self) -> None:
        """重置状态机

        重置后状态机进入 INIT 状态，并开始新的状态持续时间监控周期。
        """
        self.state = ControllerState.INIT
        self._reset_all_counters()  # 已包含 MPC 历史清空
        self._stopping_start_time = None
        self._stop_event.clear()  # 清除停止请求
        # 重置状态持续时间监控，设置进入时间以启用超时检查
        self._state_entry_time = get_monotonic_time()
        self._timeout_warned_states.clear()
        # 重置 NaN 速度警告标志
        self._nan_velocity_warned = False
