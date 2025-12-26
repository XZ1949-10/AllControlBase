"""核心模块"""
from .enums import ControllerState, TrajectoryMode, TransformStatus, HeadingMode, PlatformType
from .data_types import (
    Header, Point3D, Twist3D, Trajectory, EstimatorOutput, ControlOutput,
    ConsistencyResult, SafetyDecision, MPCHealthStatus, TimeoutStatus,
    AttitudeCommand, DiagnosticsV2, Odometry, Imu, TrajectoryDefaults,
    # TF2 相关数据类型
    Vector3, Quaternion, Transform, TransformStamped,
    # ROS geometry_msgs 兼容类型
    Pose, Twist, PoseWithCovariance, TwistWithCovariance,
)
from .diagnostics_input import DiagnosticsInput
from .interfaces import (
    ILifecycleComponent, LifecycleState,
    IStateEstimator, ITrajectoryTracker, IConsistencyChecker,
    ISafetyMonitor, ISmoothTransition, ICoordinateTransformer, IAttitudeController
)
from .ros_compat import (
    ROS_AVAILABLE, TF2_AVAILABLE,
    EPSILON, EPSILON_SMALL, EPSILON_ANGLE,
    euler_from_quaternion, quaternion_from_euler,
    normalize_angle, angle_difference,
    get_current_time, create_time, create_duration
)
from .velocity_smoother import VelocitySmoother, clip_velocity
from .constants import (
    EPSILON, EPSILON_SMALL, EPSILON_ANGLE, EPSILON_VELOCITY,
    MIN_DENOMINATOR, MIN_SEGMENT_LENGTH, MIN_RELATIVE_CROSS,
    DEFAULT_GRAVITY, NEVER_RECEIVED_TIME_MS,
    QUATERNION_NORM_SQ_MIN, QUATERNION_NORM_SQ_MAX,
    normalize_angle, angle_difference,
)
from .exceptions import (
    ControllerError, ConfigurationError, ConfigValidationError,
    ComponentError, InitializationError, ShutdownError,
    ControllerRuntimeError, SolverError, TransformError,
)
