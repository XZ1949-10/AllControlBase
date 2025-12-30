"""MPC 健康监控器"""
from typing import Dict, Any

from ..core.data_types import MPCHealthStatus


class MPCHealthMonitor:
    """MPC 求解器健康监控，支持预测性降级
    
    配置路径: mpc.health_monitor.*
    
    配置示例:
        mpc:
          health_monitor:
            time_warning_thresh_ms: 8
            time_critical_thresh_ms: 15
            ...
    """
    
    def __init__(self, config: Dict[str, Any]):
        # 配置读取: mpc.health_monitor.* 或直接从顶层读取（向后兼容）
        mpc_config = config.get('mpc', {})
        health_config = mpc_config.get('health_monitor', {})
        
        self.time_warning_thresh = health_config.get('time_warning_thresh_ms', 8)
        self.time_critical_thresh = health_config.get('time_critical_thresh_ms', 15)
        self.time_recovery_thresh = health_config.get('time_recovery_thresh_ms', 6)
        self.condition_number_thresh = health_config.get('condition_number_thresh', 1e8)
        self.condition_number_recovery = health_config.get('condition_number_recovery', 1e5)
        self.kkt_residual_thresh = health_config.get('kkt_residual_thresh', 1e-3)
        self.consecutive_warning_limit = health_config.get('consecutive_warning_limit', 3)
        self.consecutive_recovery_limit = health_config.get('consecutive_recovery_limit', 5)
        self.recovery_multiplier = health_config.get('recovery_multiplier', 2.0)
        # 超时计数衰减参数
        self.consecutive_good_for_decay = health_config.get('consecutive_good_for_decay', 2)
        self.timeout_decay_rate = health_config.get('timeout_decay_rate', 2)
        # 中间区衰减率：默认为 1，比恢复区慢，提供滞后效应
        self.middle_zone_decay_rate = health_config.get('middle_zone_decay_rate', 1)
        
        self.consecutive_near_timeout = 0
        self.consecutive_good = 0
        self.degradation_warning = False
        self._can_recover = False
    
    def update(self, solve_time_ms: float, kkt_residual: float, 
               condition_number: float) -> MPCHealthStatus:
        """
        更新 MPC 健康状态
        
        超时计数器衰减策略（三区间设计）:
        - 警告区 (> warning_thresh): 累积超时计数，重置良好计数
        - 恢复区 (< recovery_thresh): 累积良好计数，达到阈值后快速衰减超时计数
        - 中间区 (recovery <= time <= warning): 重置良好计数，缓慢衰减超时计数
        
        这种设计提供了滞后效应，防止在阈值边界频繁切换状态。
        """
        # 更新连续超时计数
        if solve_time_ms > self.time_warning_thresh:
            # 警告区：累积超时，重置良好计数
            self.consecutive_near_timeout += 1
            self.consecutive_good = 0
        elif solve_time_ms < self.time_recovery_thresh:
            # 恢复区：累积良好计数，达到阈值后快速衰减
            self.consecutive_good += 1
            if self.consecutive_good >= self.consecutive_good_for_decay:
                self.consecutive_near_timeout = max(0, self.consecutive_near_timeout - self.timeout_decay_rate)
        else:
            # 中间区：重置良好计数，缓慢衰减超时计数
            # 使用配置的中间区衰减率，提供滞后效应防止边界震荡
            self.consecutive_good = 0
            self.consecutive_near_timeout = max(0, self.consecutive_near_timeout - self.middle_zone_decay_rate)
        
        # 判断是否需要警告
        should_warn = (
            self.consecutive_near_timeout >= self.consecutive_warning_limit or
            condition_number > self.condition_number_thresh or
            kkt_residual > self.kkt_residual_thresh
        )
        self.degradation_warning = should_warn
        
        # 判断是否可以恢复
        # 主要恢复条件: 连续良好次数足够且条件数正常
        # 备选恢复条件: 连续良好次数非常多 (即使条件数稍高)
        primary_recovery = (
            self.consecutive_good >= self.consecutive_recovery_limit and
            condition_number < self.condition_number_recovery
        )
        # 如果连续良好次数是恢复阈值的 recovery_multiplier 倍，放宽条件数要求
        fallback_recovery = (
            self.consecutive_good >= self.consecutive_recovery_limit * self.recovery_multiplier and
            condition_number < self.condition_number_thresh  # 使用警告阈值而非恢复阈值
        )
        self._can_recover = primary_recovery or fallback_recovery
        
        return MPCHealthStatus(
            healthy=not should_warn and solve_time_ms < self.time_critical_thresh,
            degradation_warning=should_warn, can_recover=self._can_recover,
            consecutive_near_timeout=self.consecutive_near_timeout,
            kkt_residual=kkt_residual, condition_number=condition_number
        )
    
    def should_preemptive_switch(self) -> bool:
        return self.degradation_warning
    
    def reset(self) -> None:
        self.consecutive_near_timeout = 0
        self.consecutive_good = 0
        self.degradation_warning = False
        self._can_recover = False
