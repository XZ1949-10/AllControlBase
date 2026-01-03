
import sys
import os
import logging
import numpy as np

# Add project root to path
sys.path.append(r"d:\AllControlBase")

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("Verification")

def verify_mpc_structure():
    logger.info("Verifying MPCController structure...")
    from universal_controller.tracker.mpc_controller import MPCController
    
    # Check if _solve_fallback exists (should be gone)
    if hasattr(MPCController, '_solve_fallback'):
        logger.error("FAIL: MPCController still has _solve_fallback method")
        return False
    
    logger.info("PASS: MPCController structure verified")
    return True

def verify_manager_logic():
    logger.info("Verifying ControllerManager logic...")
    from universal_controller.manager.controller_manager import ControllerManager
    from universal_controller.core.data_types import ControlOutput, Trajectory, ConsistencyResult
    
    # Mock classes
    class MockMPC:
        def compute(self, state, traj, cons):
            return ControlOutput(vx=0, vy=0, vz=0, omega=0, frame_id='base', success=False, 
                               health_metrics={'error_type': 'mock_failure'})
        def reset(self): pass
        def set_horizon(self, h): return True
        def get_health_metrics(self): return {'acados_available': True}
        def get_predicted_next_state(self): return None
        def shutdown(self): pass

    class MockBackup:
        def compute(self, state, traj, cons):
            return ControlOutput(vx=0.5, vy=0, vz=0, omega=0.1, frame_id='base', success=True,
                               health_metrics={'source': 'backup'})
        def reset(self): pass
        def shutdown(self): pass

    # Instantiate Manager with minimal config
    config = {
        'system': {'ctrl_freq': 50},
        'mpc': {'horizon': 20},
        'backup': {},
        'transform': {'target_frame': 'odom'}
    }
    
    manager = ControllerManager(config, validate_config=False)
    
    # Inject mocks
    manager.mpc_tracker = MockMPC()
    manager.backup_tracker = MockBackup()
    
    # Test _select_controller_output
    state = np.zeros(8)
    # Initialize Trajectory with empty lists for points and velocities, and None for header if allowed or a mock object
    # Assuming standard signature based on error: header, points, velocities
    # Let's try passing empty list for points/velocities and None for header
    traj = Trajectory(None, [], [])
    # Correct Init: alpha, kappa, v_dir, temporal, disable_soft
    consistency = ConsistencyResult(0.5, 1.0, 1.0, 1.0, False)
    mpc_cmd = manager.mpc_tracker.compute(state, traj, consistency)
    
    # Direct call to internal method for verification
    cmd = manager._select_controller_output(state, traj, consistency, mpc_cmd)
    
    if cmd.health_metrics.get('source') == 'backup_fallback':
        logger.info("PASS: Manager correctly fell back to backup controller")
    else:
        logger.error(f"FAIL: Manager did not fallback. Source: {cmd.health_metrics.get('source')}")
        return False
        
    return True

if __name__ == "__main__":
    try:
        if verify_mpc_structure() and verify_manager_logic():
            logger.info("ALL TESTS PASSED")
            sys.exit(0)
        else:
            logger.error("TESTS FAILED")
            sys.exit(1)
    except Exception as e:
        logger.error(f"Verification crashed: {e}")
        sys.exit(1)
