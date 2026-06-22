"""Default configuration parameters for PyChronoRover simulations."""

# ============================================================================
# PHYSICS PARAMETERS
# ============================================================================

# Martian gravity (m/s²)
MARS_GRAVITY = -3.71

# Earth gravity (m/s²) for comparison
EARTH_GRAVITY = -9.81

# Default simulation time step (seconds)
DEFAULT_TIME_STEP = 0.01

# ============================================================================
# CHASSIS PARAMETERS
# ============================================================================

# Default chassis dimensions (meters)
DEFAULT_CHASSIS_LENGTH = 1.4  # X
DEFAULT_CHASSIS_WIDTH = 0.8  # Y
DEFAULT_CHASSIS_HEIGHT = 0.4  # Z

# Default chassis mass (kg)
DEFAULT_CHASSIS_MASS = 50.0

# Chassis friction coefficient
DEFAULT_CHASSIS_FRICTION = 0.5

# Initial chassis height above ground (m)
DEFAULT_CHASSIS_INIT_HEIGHT = 0.6

# ============================================================================
# WHEEL PARAMETERS
# ============================================================================

# Default wheel radius (m)
DEFAULT_WHEEL_RADIUS = 0.3

# Default wheel width/thickness (m)
DEFAULT_WHEEL_WIDTH = 0.25

# Default wheel mass (kg)
DEFAULT_WHEEL_MASS = 15.0

# Wheel friction coefficient
DEFAULT_WHEEL_FRICTION = 0.7

# ============================================================================
# MULTI-WHEEL ROVER PARAMETERS
# ============================================================================

# Wheelbase: distance between front and rear axles (m)
DEFAULT_WHEELBASE = 0.8

# Track width: distance between left and right wheels (m)
DEFAULT_TRACK_WIDTH = 0.6

# ============================================================================
# TERRAIN PARAMETERS (MARS REGOLITH)
# ============================================================================

# Bekker terramechanics soil parameters for Mars regolith
MARS_SOIL_PARAMS = {
    "kc": 0.02e6,  # Cohesion stiffness modulus (Pa/m)
    "kphi": 0.25e6,  # Friction stiffness modulus (Pa/m)
    "n": 1.1,  # Compaction exponent
    "c": 500.0,  # Cohesion pressure (Pa)
    "phi": 35.0,  # Internal friction angle (degrees)
    "j": 0.015,  # Shear deformation limit (m)
    "elastic_k": 2.0e7,  # Elastic stiffness (Pa/m)
    "damping_r": 3.0e3,  # Viscous damping (Pa·s/m)
}

# Default terrain dimensions (meters)
DEFAULT_TERRAIN_WIDTH = 15.0  # Y direction
DEFAULT_TERRAIN_LENGTH = 5.0  # X direction

# Terrain grid resolution (meters) - trade-off between accuracy and speed
# Coarse: 0.05 m (faster, less accurate)
# Medium: 0.025 m (balanced)
# Fine: 0.015 m (slower, more accurate)
DEFAULT_TERRAIN_GRID_RESOLUTION = 0.05

# ============================================================================
# SIMULATION PARAMETERS
# ============================================================================

# Metrics collection frequency (seconds)
# Set to 0 to collect every step (expensive)
DEFAULT_METRICS_OUTPUT_FREQ = 0.5

# Default simulation duration (seconds)
DEFAULT_SIMULATION_DURATION = 10.0
