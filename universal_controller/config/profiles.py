"""
Configuration Profiles

Provides standard configuration presets for the Universal Controller.
"""
from typing import Dict, Any, Optional
import copy
from .default_config import DEFAULT_CONFIG

def create_default_config(profile: str = "balanced") -> Dict[str, Any]:
    """
    Create a default configuration based on a profile.
    
    Args:
        profile: Profile name ("balanced", "aggressive", "safe")
        
    Returns:
        Complete configuration dictionary
    """
    config = copy.deepcopy(DEFAULT_CONFIG)
    
    # Base settings ensuring safety
    config['safety']['state_machine']['enable_state_timeout_stop'] = True
    
    if profile == "aggressive":
        # Aggressive: Higher limits, faster response, less conservative safety
        config['constraints']['v_max'] = 2.0
        config['constraints']['a_max'] = 2.0
        config['mpc']['weight_q_pos'] = 30.0  # Tighter tracking
        config['mpc']['weight_r_accel'] = 0.5 # Allow more aggressive accel
        config['trajectory']['default_confidence'] = 0.8 # Trust network more
        config['backup']['lookahead_ratio'] = 0.8 # Shorter lookahead for tighter turns
        
    elif profile == "balanced":
        # Balanced: Standard defaults (already in DEFAULT_CONFIG)
        pass
        
    elif profile == "safe":
        # Safe: Conservative limits, smooth motion, high safety margins
        config['constraints']['v_max'] = 0.8
        config['constraints']['a_max'] = 0.8
        config['constraints']['omega_max'] = 1.0
        config['mpc']['weight_q_pos'] = 10.0   # Looser tracking for smoothness
        config['mpc']['weight_r_accel'] = 5.0  # Penalize accel heavily
        config['trajectory']['default_confidence'] = 0.95 # Require high confidence
        config['safety']['state_machine']['alpha_disable_thresh'] = 0.7 # Strict alpha 
        config['backup']['lookahead_ratio'] = 1.5 # Look further ahead
        
    else:
        raise ValueError(f"Unknown profile: {profile}. Available: aggressive, balanced, safe")
        
    return config
