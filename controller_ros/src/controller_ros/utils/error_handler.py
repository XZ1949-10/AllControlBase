from typing import Any, Callable, Dict, Optional
import logging

from universal_controller.core.data_types import TimeoutStatus
from universal_controller.core.enums import ControllerState

logger = logging.getLogger(__name__)


class ErrorHandler:
    """
    Controller error handling logic encapsulation.
    
    Responsible for:
    - Tracking consecutive error counts
    - Deciding logging strategy (detail vs summary)
    - Creating error diagnostic reports
    - Implementing "Crash Early" policies for structural errors
    """
    
    def __init__(
        self,
        log_error_func: Callable[[str], None],
        log_warn_func: Callable[[str], None],
        config: Dict[str, Any] = None
    ):
        """
        Initialize ErrorHandler.
        
        Args:
            log_error_func: Callback for logging errors
            log_warn_func: Callback for logging warnings
            config: Diagnostics configuration dictionary
        """
        self._log_error = log_error_func
        self._log_warn = log_warn_func
        
        config = config or {}
        self._max_consecutive_errors_detail = config.get('max_consecutive_errors_detail', 10)
        self._error_summary_interval = config.get('error_summary_interval', 50)
        self._max_error_count = config.get('max_error_count', 1000)
        
        self._consecutive_errors = 0
        
    def reset(self):
        """Reset error counters."""
        self._consecutive_errors = 0

    @property
    def consecutive_errors(self) -> int:
        return self._consecutive_errors

    def handle_control_error(self, error: Exception, 
                           tf2_reinjection_callback: Optional[Callable[[], None]] = None) -> Dict[str, Any]:
        """
        Handle an exception during the control loop.
        
        Args:
            error: The captured exception
            tf2_reinjection_callback: Optional callback to trigger TF2 reinjection on repeated failures
            
        Returns:
            Dictionary containing error diagnostics
            
        Raises:
            Exception: If the error is a critical structural error (TypeError, AttributeError, etc.)
        """
        # Critical structural errors: Do not catch, let it crash!
        if isinstance(error, (TypeError, AttributeError, NameError, SystemError)):
            logger.critical(f"Critical configuration or logic error detected: {error}. Crashing immediately.")
            raise error

        # Runtime errors: Handle gracefully
        self._consecutive_errors += 1
        
        # Cap the counter
        if self._consecutive_errors > self._max_error_count:
            self._consecutive_errors = self._max_error_count
        
        # Logging strategy
        if self._consecutive_errors <= self._max_consecutive_errors_detail:
            self._log_error(f'Controller update failed ({self._consecutive_errors}): {error}')
        elif self._consecutive_errors % self._error_summary_interval == 0:
            self._log_error(
                f'Controller update still failing ({self._consecutive_errors} consecutive errors): {error}'
            )
        
        # Attempt auto-recovery
        if self._consecutive_errors == 10 and tf2_reinjection_callback:
             self._log_warn("Detected repeated failures. Attempting to update TF2 injection...")
             tf2_reinjection_callback()
        
        return self._create_error_diagnostics(error)

    def _create_error_diagnostics(self, error: Exception) -> Dict[str, Any]:
        """Create a diagnostics dictionary for the error state."""
        return {
            'state': int(ControllerState.INIT),  # 错误状态使用 INIT (控制器未正常运行)
            'mpc_success': False,
            'backup_active': False,
            'error_message': str(error),
            'consecutive_errors': self._consecutive_errors,
            # Basic timeout skeleton, caller should populate real status if available
            'timeout': {
                'odom_timeout': False,
                'traj_timeout': False,
                'traj_grace_exceeded': False,
                'imu_timeout': False,
                'last_odom_age_ms': 0.0,
                'last_traj_age_ms': 0.0,
                'last_imu_age_ms': 0.0,
                'in_startup_grace': False,
            },
            'cmd': {'vx': 0.0, 'vy': 0.0, 'vz': 0.0, 'omega': 0.0},
            'emergency_stop': False, 
        }

    def enrich_diagnostics(self, diag: Dict[str, Any], 
                          timeout_status: Optional[TimeoutStatus] = None,
                          tf2_status: Optional[Dict[str, Any]] = None,
                          estop_status: bool = False) -> Dict[str, Any]:
        """
        Enrich the basic error diagnostics with context from the controller.
        """
        if timeout_status:
            diag['timeout'] = {
                'odom_timeout': timeout_status.odom_timeout,
                'traj_timeout': timeout_status.traj_timeout,
                'traj_grace_exceeded': timeout_status.traj_grace_exceeded,
                'imu_timeout': timeout_status.imu_timeout,
                'last_odom_age_ms': timeout_status.last_odom_age_ms,
                'last_traj_age_ms': timeout_status.last_traj_age_ms,
                'last_imu_age_ms': timeout_status.last_imu_age_ms,
                'in_startup_grace': timeout_status.in_startup_grace,
            }
            
        if tf2_status:
            diag['transform'] = tf2_status
            
        diag['emergency_stop'] = estop_status
        return diag
