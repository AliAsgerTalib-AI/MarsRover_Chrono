# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in the PyChronoRover repository.

## Quick Commands

```bash
# Development
python -m venv venv              # Create virtual environment
source venv/bin/activate         # Activate (Linux/Mac)
pip install pychrono             # Install pychrono (may require compilation)

# Run simulations
python examples/scenario_0_static_drop.py         # Terrain deformation test
python examples/scenario_1_single_wheel_slip.py   # Slip-sinkage characterization
python examples/scenario_2_kinematic_control.py   # Kinematic chassis control
python examples/scenario_3_four_wheel_rover.py    # Multi-wheel rover (template)

# Testing
pytest tests/ -v                 # Run all tests
pytest tests/test_system.py -v   # Test system creation
pytest tests/test_metrics.py -v  # Test metrics collection

# Data analysis
python -m matplotlib examples/scenario_1_single_wheel_slip.py  # Plot slip curve
```

## Project Overview

**PyChronoRover** is a modular simulation framework for Mars rover locomotion using Chrono physics engine. It provides clean abstractions for terramechanics simulation, wheel-terrain interaction, and multi-wheel rover design.

**Goal**: Make it easy to iterate on rover designs (4-wheel, 6-wheel, steering, suspension) without duplicating system setup, terrain initialization, or data collection code.

**Key Capabilities**:
- Mars gravity and Bekker soil parameters (terramechanics)
- Single and multi-wheel rover configurations
- SCM (Soil Contact Model) deformable terrain
- Slip-sinkage characterization
- Kinematic constraint-based control
- Automated metrics collection (velocity, slip, sinkage, drawbar pull)

## Architecture

### Three-Layer Design

**1. Physics Engine Layer** (`src/rover/system.py`, `src/rover/materials.py`)
- Chrono physics system setup with Mars gravity (-3.71 m/s²)
- Contact material definitions (chassis, wheel friction)
- Collision backend configuration (Bullet NSC)

**2. Mechanical Design Layer** (`src/rover/bodies.py`, `src/rover/motors.py`, `src/rover/constraints.py`)
- Rigid body builders (chassis, wheels, carriage)
- Motor/actuator factories (constant speed, ramp, sine profiles)
- Kinematic constraints (prismatic joints, revolute joints)

**3. Terrain & Measurement Layer** (`src/rover/terrain.py`, `src/rover/metrics.py`)
- SCM deformable soil with Bekker parameters
- Terrain height queries and sinkage calculation
- Metrics collection (slip, velocity, forces, sinkage)

### Module Breakdown

| Module | Purpose | Key Classes |
|--------|---------|------------|
| `system.py` | Physics system factory | `SystemFactory` |
| `materials.py` | Contact material properties | `MaterialFactory` |
| `bodies.py` | Rigid body assembly builders | `ChassisBuilder`, `WheelBuilder`, `MultiWheelBuilder` |
| `terrain.py` | SCM terrain management | `TerrainManager`, `SoilParameterSet`, `SoilType` |
| `motors.py` | Actuators and drive systems | `RotationMotor`, `LinearMotor`, `MotorProfile` |
| `constraints.py` | Kinematic constraints | `PrismaticConstraint`, `RevoluteJoint`, `CarriageConstraint` |
| `metrics.py` | Data collection & analysis | `MetricsCollector`, `MetricFrame` |

## Common Patterns in PyChronoRover

### 1. Creating a Simple Simulation

Most simulations follow this structure:

```python
import pychrono as chrono
from rover import SystemFactory, ChassisBuilder, WheelBuilder, TerrainManager

# 1. Create physics system
system = SystemFactory.create_system(gravity_mars=True)

# 2. Build bodies
chassis = ChassisBuilder.create(system, pos_z=0.6)
wheel = WheelBuilder.create(system, pos_y=0.5, pos_z=0.3)

# 3. Setup terrain
terrain_mgr = TerrainManager(system)
terrain_mgr.initialize_scm(width=15.0, length=5.0, grid_resolution=0.05)

# 4. Add actuators
linear_motor = LinearMotor.create_constant_speed(...)
rotation_motor = RotationMotor.create_constant_speed(...)

# 5. Run simulation
time_step = 0.01
while sim_time < max_time:
    system.DoStepDynamics(time_step)
    # collect metrics...
    sim_time += time_step
```

### 2. Working with Terrain (SCM Model)

```python
# Initialize with Mars regolith (default)
terrain_mgr = TerrainManager(system)
terrain = terrain_mgr.initialize_scm(
    width=15.0,
    length=5.0,
    grid_resolution=0.025  # 25mm nodes = high fidelity
)

# Query terrain height
height = terrain_mgr.get_height(x=0.0, y=0.0)

# Measure sinkage for a wheel
sinkage = terrain_mgr.get_sinkage(wheel, wheel_radius=0.3)

# Switch to different soil type
from rover import SoilParameterSet, SoilType
sandy_params = SoilParameterSet.get_preset(SoilType.SANDY_SOIL)
# Then reinitialize terrain with: terrain_mgr.initialize_scm(..., soil_params=sandy_params)
```

**Important**: Always call `initialize_scm()` BEFORE `add_active_domain()`. The terrain must be initialized to create the deformation grid.

### 3. Creating Motors with Different Profiles

```python
# Constant speed (e.g., wheel rotation)
motor = RotationMotor.create_constant_speed(
    system, chassis, wheel, wheel_pos,
    chrono.ChVector3d(0, 1, 0),  # Y-axis
    2.0  # rad/s
)

# Ramp speed (e.g., slip sweep: 0.6 → 0.0 m/s over 10s)
motor = LinearMotor.create_sweep_speed(
    system, ground, chassis, chassis_pos,
    initial_speed=0.6,
    final_speed=0.0,
    duration=10.0
)

# Sine profile (e.g., oscillatory control)
motor = LinearMotor.create_sine_speed(
    system, ground, chassis, chassis_pos,
    amplitude=0.3,  # m/s
    frequency=1.0   # Hz
)
```

### 4. Collecting and Visualizing Metrics

```python
from rover import MetricsCollector

metrics = MetricsCollector(output_freq=0.5)  # Collect every 0.5 seconds
metrics.print_header()  # Print column titles

# In simulation loop:
if metrics.should_collect(sim_time):
    frame = metrics.collect_frame(
        sim_time, chassis, wheel, wheel_radius=0.3,
        v_ideal=0.6, linear_motor=linear_motor,
        terrain_height_func=terrain_mgr.get_height
    )
    metrics.print_frame(frame)

# After simulation:
metrics.print_summary()  # Print statistics
metrics.save_csv("output.csv")  # Save for post-processing
```

### 5. Building Multi-Wheel Rovers

```python
from rover import MultiWheelBuilder

rover = MultiWheelBuilder.create_four_wheel_rover(
    system,
    chassis_mass=50.0,
    wheel_mass=15.0,
    wheel_radius=0.3,
    wheelbase=0.8,
    track_width=0.6
)

chassis = rover["chassis"]
wheels = rover["wheels"]  # List of 4 ChBody objects

# Drive all wheels
for wheel in wheels:
    motor = RotationMotor.create_constant_speed(
        system, chassis, wheel, wheel.GetPos(),
        chrono.ChVector3d(0, 1, 0), 2.0
    )
```

## Physics Background

### Bekker Terramechanics Parameters

The SCM terrain uses Bekker-Wong empirical model with 8 parameters:

| Parameter | Symbol | Units | Mars Regolith | Description |
|-----------|--------|-------|-----------------|-------------|
| Cohesion stiffness | Kc | Pa/m | 0.02e6 | Soil resistance to shear (cohesive term) |
| Friction stiffness | Kphi | Pa/m | 0.25e6 | Soil resistance to shear (friction term) |
| Compaction exponent | n | — | 1.1 | How soil stiffens with depth |
| Mohr cohesion | c | Pa | 500 | Cohesive strength intercept |
| Friction angle | φ | ° | 35 | Internal friction angle |
| Shear displacement | j | m | 0.015 | Slip displacement at max shear stress |
| Elastic recovery | K | Pa/m | 2.0e7 | How quickly soil rebounds |
| Damping | R | Pa·s/m | 3.0e3 | Viscous resistance to deformation |

**Use cases**:
- Mars regolith (default): Typical loose soil
- Sandy soil: Higher friction, lower cohesion
- Loose dust: Very low strength, high sinkage

### Slip Ratio Definition

Slip is the relative motion between wheel and soil:

```
slip_ratio (%) = (1 - v_actual / v_ideal) × 100

where:
  v_ideal = ω × r  (wheel angular velocity × radius)
  v_actual = dx/dt (chassis forward velocity)

Interpretation:
  0% slip   = perfect grip (wheel speed matches ground speed)
  50% slip  = wheel rolling half as fast as ideal
  100% slip = wheel spinning in place (no traction)
```

### Sinkage Calculation

Sinkage measures how deep a wheel pushes into deformable terrain:

```
sinkage = |terrain_height - (wheel_center_z - wheel_radius)|

For a wheel at height z with radius r:
  wheel_bottom = z - r
  sinkage = terrain_height - wheel_bottom
```

Larger sinkage = more energy loss + higher soil resistance.

## Development Workflow

### Adding a New Simulation Scenario

1. **Copy template from scenario_1**:
   ```bash
   cp examples/scenario_1_single_wheel_slip.py examples/scenario_4_my_test.py
   ```

2. **Modify the key sections**:
   - System initialization (gravity, collision)
   - Body creation (chassis, wheels, carriage if needed)
   - Terrain setup (width, length, grid resolution, soil type)
   - Motor profiles (wheel drive, chassis drive)
   - Metrics collection (what to measure)

3. **Run and validate**:
   ```bash
   python examples/scenario_4_my_test.py
   ```

### Adding a New Component (e.g., Suspension)

1. **Create builder class** in appropriate module:
   ```python
   # In src/rover/suspension.py (new file)
   class SpringDamperBuilder:
       @staticmethod
       def create(system, body1, body2, ...):
           # Create ChLinkSpringDamper or similar
           pass
   ```

2. **Export in `__init__.py`**:
   ```python
   # In src/rover/__init__.py
   from .suspension import SpringDamperBuilder
   __all__ = [..., "SpringDamperBuilder"]
   ```

3. **Use in scenarios**:
   ```python
   from rover import SpringDamperBuilder
   suspension = SpringDamperBuilder.create(system, ...)
   ```

### Adding Sensor Simulation

1. **Extend `MetricFrame`** dataclass to include sensor readings:
   ```python
   @dataclass
   class MetricFrame:
       # ... existing fields ...
       wheel_load: float = 0.0  # New: normal force on wheel
       imu_accel_x: float = 0.0  # New: longitudinal acceleration
   ```

2. **Update `MetricsCollector.collect_frame()`**:
   ```python
   def collect_frame(...):
       # ... existing code ...
       wheel_load = chassis.GetContactForce().z  # Query contact force
       imu_accel = chassis.GetPosDt2().x  # Query acceleration
       frame = MetricFrame(..., wheel_load=wheel_load, imu_accel_x=imu_accel)
   ```

## Configuration & Defaults

All simulation defaults are in `config/default_config.py`:

```python
MARS_GRAVITY = -3.71  # m/s²
DEFAULT_TIME_STEP = 0.01  # seconds
DEFAULT_CHASSIS_MASS = 50.0  # kg
DEFAULT_WHEEL_RADIUS = 0.3  # m
DEFAULT_TERRAIN_GRID_RESOLUTION = 0.05  # m
MARS_SOIL_PARAMS = { ... }  # 8-parameter Bekker set
```

Modify these to tune rover specifications or terrain properties globally.

## Testing Strategy

### Unit Tests

**`tests/test_system.py`**: Verify system creation
```python
def test_system_mars_gravity():
    system = SystemFactory.create_system(gravity_mars=True)
    g = system.GetGravitationalAcceleration().z
    assert abs(g - (-3.71)) < 1e-6  # Mars gravity

def test_system_bullet_collision():
    system = SystemFactory.create_system()
    assert system.GetCollisionSystemType() == chrono.ChCollisionSystem.Type_BULLET
```

**`tests/test_metrics.py`**: Verify metrics calculations
```python
def test_slip_calculation():
    v_ideal = 0.6  # m/s
    v_actual = 0.3  # m/s
    expected_slip = 50.0  # %
    # Verify MetricsCollector calculates correctly
```

### Integration Tests

**`tests/integration_test.py`**: Run full scenario and validate output
```python
def test_scenario_1_runs_without_error():
    # Run scenario_1 for 1 second (quick validation)
    # Check metrics output is well-formed (no NaN, correct keys)
    # Verify slip increases as wheel speed is ramped down
```

### Manual Validation

1. **Run scenario, inspect CSV output**:
   ```bash
   python examples/scenario_1_single_wheel_slip.py
   # Check: data/logs/scenario_1_slip_sweep.csv
   ```

2. **Visualize with matplotlib**:
   ```python
   import pandas as pd
   import matplotlib.pyplot as plt
   
   df = pd.read_csv("data/logs/scenario_1_slip_sweep.csv")
   plt.plot(df["time"], df["slip_percent"])
   plt.xlabel("Time (s)")
   plt.ylabel("Slip Ratio (%)")
   plt.show()
   ```

## Troubleshooting

### Problem: "ModuleNotFoundError: No module named 'pychrono'"
**Solution**: Install pychrono:
```bash
pip install pychrono
# If pip doesn't have wheels, compile from source (takes time)
```

### Problem: Simulation runs very slowly
**Diagnosis**: Check terrain grid resolution
```python
# Coarse (faster):
terrain.initialize_scm(width=15, length=5, grid_resolution=0.05)

# Fine (slower, more accurate):
terrain.initialize_scm(width=15, length=5, grid_resolution=0.015)
```
**Solution**: Use coarser grid for quick tests, fine grid for final validation.

### Problem: Wheel sinks unrealistically far
**Check**: Bekker parameters for soil type
- Mars regolith is relatively stiff (high Kc, Kphi)
- Sandy soil allows more sinkage (lower moduli)
- Loose dust has extreme sinkage

**Solution**: Verify soil parameters match your intended terrain type.

### Problem: Motor doesn't exert force
**Diagnosis**: Check motor is connected correctly
```python
# Verify body ordering:
motor.Initialize(body1, body2, joint_frame)
# body1 = anchor (e.g., ground or chassis)
# body2 = driven body (e.g., wheel or chassis)
```

### Problem: Constraint violation or instability
**Solution**: Reduce time step or increase solver iterations
```python
SystemFactory.configure_solver(
    system,
    time_step=0.005,  # Smaller = more stable but slower
    max_iterations=30  # More iterations for tighter accuracy
)
```

## File Organization Best Practices

```
ChronoRover/
├── src/rover/                    # Core modules (reusable components)
│   ├── __init__.py              # Public API
│   ├── system.py                # Physics system factory
│   ├── materials.py             # Contact materials
│   ├── bodies.py                # Rigid body builders
│   ├── terrain.py               # Terrain management
│   ├── motors.py                # Actuator/motor factories
│   ├── constraints.py           # Kinematic constraints
│   └── metrics.py               # Data collection
│
├── src/utils/                    # Utilities
│   ├── __init__.py
│   └── logger.py                # Logging
│
├── examples/                     # Simulation scenarios
│   ├── scenario_0_static_drop.py
│   ├── scenario_1_single_wheel_slip.py
│   ├── scenario_2_kinematic_control.py
│   └── scenario_3_four_wheel_rover.py
│
├── tests/                        # Unit and integration tests
│   ├── test_system.py
│   ├── test_metrics.py
│   └── integration_test.py
│
├── config/                       # Configuration & presets
│   ├── __init__.py
│   └── default_config.py
│
├── data/logs/                    # Output data (CSV files)
├── docs/                         # Documentation
│   ├── terramechanics.md        # Physics background
│   └── pychrono_patterns.md     # Chrono-specific patterns
│
├── CLAUDE.md                     # This file
├── README.md                     # Project overview
└── requirements.txt              # Dependencies
```

**Key Rule**: Put reusable code in `src/rover/` modules. Keep scenario-specific logic in `examples/`. Tests go in `tests/`.

## Next Steps for Development

### Immediate (Iteration 1-2)
- [ ] Validate scenario_1 matches original rover_sandbox1.py output
- [ ] Implement scenario_2 full metrics collection
- [ ] Write unit tests for all core modules

### Short-term (Iteration 3-4)
- [ ] Implement scenario_3 (4-wheel rover with metrics)
- [ ] Add steering/skid-steer control
- [ ] Add simple suspension (spring-damper between chassis and wheels)

### Medium-term (Iteration 5-6)
- [ ] Sensor simulation (wheel load cells, IMU)
- [ ] Traction control algorithm (closed-loop wheel speed modulation)
- [ ] Terrain variation (slopes, obstacles)

### Long-term (Iteration 7+)
- [ ] 6-wheel rover configurations
- [ ] Visualization (Irrlicht 3D rendering)
- [ ] Parameter sensitivity analysis
- [ ] Optimization (find best wheel radius/mass for Mars mission)

---

For physics background, see [docs/terramechanics.md](docs/terramechanics.md).
For Chrono-specific patterns, see [docs/pychrono_patterns.md](docs/pychrono_patterns.md).
