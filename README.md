# PyChronoRover: Modular Mars Rover Simulation Framework

A clean, modular Python framework for simulating Mars rover locomotion using the Chrono physics engine. Supports single-wheel and multi-wheel rovers with Bekker terramechanics.

## Features

- **Physics-Accurate Simulation**: Chrono physics engine with Mars gravity (-3.71 m/s²)
- **Deformable Terrain**: SCM (Soil Contact Model) with Bekker parameters for Mars regolith
- **Modular Design**: Reusable components for rapid rover prototyping
- **Multi-Wheel Support**: Single-wheel to N-wheel configurations
- **Metrics Collection**: Automated slip, sinkage, drawbar pull, and velocity logging
- **Extensible**: Easy to add sensors, suspension, steering, and control algorithms

## Quick Start

### Installation

```bash
# Create virtual environment
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install pychrono

# Clone or download PyChronoRover
cd ChronoRover
```

### Run a Simulation

```bash
# Static chassis drop test (terrain validation)
python examples/scenario_0_static_drop.py

# Single-wheel slip characterization
python examples/scenario_1_single_wheel_slip.py

# Kinematic control with vertical settlement
python examples/scenario_2_kinematic_control.py
```

### Output

Simulations produce CSV files in `data/logs/` with columns:
- `time`: Simulation time (s)
- `velocity_x`: Forward velocity (m/s)
- `slip_percent`: Wheel slip ratio (%)
- `drawbar_force`: Traction force (N)
- `sinkage_mm`: Wheel sinkage depth (mm)

## Architecture

**Three-layer design**:

1. **Physics Layer** (`src/rover/system.py`, `materials.py`)
   - Chrono system setup, gravity, collision backends

2. **Mechanics Layer** (`bodies.py`, `motors.py`, `constraints.py`)
   - Rigid body builders (chassis, wheels)
   - Motor/actuator factories (constant speed, ramps, sine waves)
   - Kinematic constraints (prismatic, revolute joints)

3. **Terrain & Measurement** (`terrain.py`, `metrics.py`)
   - SCM deformable soil with Bekker parameters
   - Terrain height queries, sinkage calculation
   - Automated metrics collection

## Core Modules

| Module | Purpose |
|--------|---------|
| `SystemFactory` | Create physics system with Mars gravity |
| `ChassisBuilder` | Build rover chassis (box body) |
| `WheelBuilder` | Build wheel (cylinder body) |
| `TerrainManager` | SCM terrain with Bekker parameters |
| `RotationMotor` | Wheel drive with speed profiles |
| `LinearMotor` | Chassis/carriage drive |
| `MetricsCollector` | Slip, sinkage, force measurement |

## Example: Single-Wheel Rover

```python
from rover import (
    SystemFactory, ChassisBuilder, WheelBuilder,
    TerrainManager, RotationMotor, LinearMotor,
    MetricsCollector, GroundBuilder
)
import pychrono as chrono

# Setup
system = SystemFactory.create_system(gravity_mars=True)
ground = GroundBuilder.create(system)
chassis = ChassisBuilder.create(system, pos_z=0.6)
wheel = WheelBuilder.create(system, pos_y=0.5, pos_z=0.3)

# Terrain
terrain_mgr = TerrainManager(system)
terrain_mgr.initialize_scm(width=15.0, length=5.0)

# Motors: wheel rotates at 2.0 rad/s, chassis sweeps from 0.6 to 0.0 m/s
rotation_motor = RotationMotor.create_constant_speed(
    system, chassis, wheel, wheel.GetPos(), chrono.ChVector3d(0,1,0), 2.0
)
linear_motor = LinearMotor.create_sweep_speed(
    system, ground, chassis, chassis.GetPos(), 0.6, 0.0, 10.0
)

# Metrics
metrics = MetricsCollector(output_freq=0.5)

# Run
time_step = 0.01
sim_time = 0.0
while sim_time <= 10.0:
    system.DoStepDynamics(time_step)
    if metrics.should_collect(sim_time):
        frame = metrics.collect_frame(
            sim_time, chassis, wheel, 0.3, 0.6,
            linear_motor, terrain_mgr.get_height
        )
        metrics.print_frame(frame)
    sim_time += time_step

metrics.save_csv("output.csv")
```

## Development

### Run Tests
```bash
pytest tests/ -v
```

### Add a Scenario
1. Copy `examples/scenario_1_single_wheel_slip.py`
2. Modify bodies, motors, terrain setup
3. Run: `python examples/scenario_4_my_test.py`

### Add a Component
1. Create builder class in `src/rover/` (e.g., `suspension.py`)
2. Export in `src/rover/__init__.py`
3. Use in scenarios

## Physics Background

### Bekker Terramechanics
The terrain model uses 8 Bekker parameters:
- `Kc`, `Kphi`: Soil stiffness moduli (Pa/m)
- `n`: Compaction exponent
- `c`, `φ`: Mohr-Coulomb cohesion and friction
- `j`: Shear displacement limit
- `K`, `R`: Elastic recovery and damping

**Mars regolith** (default): Relatively stiff loose soil
**Sandy soil**: Higher friction, lower cohesion  
**Loose dust**: Very low strength, extreme sinkage

### Slip Ratio
```
slip (%) = (1 - v_actual / v_ideal) × 100

v_ideal = ω × r  (wheel angular velocity × radius)
v_actual = dx/dt (chassis forward velocity)
```
- 0% = perfect grip
- 50% = wheel speed is half ideal
- 100% = wheel spinning in place

## Directory Structure

```
src/rover/                  # Core modules
├── system.py, materials.py, bodies.py, terrain.py, motors.py,
├── constraints.py, metrics.py, __init__.py

examples/                   # Simulation scenarios
├── scenario_0_static_drop.py
├── scenario_1_single_wheel_slip.py
├── scenario_2_kinematic_control.py
├── scenario_3_four_wheel_rover.py

tests/                      # Unit & integration tests
config/                     # Configuration & presets
data/logs/                  # Output CSV files
docs/                       # Documentation
```

## Next Steps

- Validate scenario outputs match original sandboxes
- Implement 4-wheel rover control
- Add suspension system
- Sensor simulation (wheel load cells, IMU)
- Closed-loop traction control

## Documentation

See [CLAUDE.md](CLAUDE.md) for comprehensive development guide including:
- Quick commands and common patterns
- Physics background and Bekker parameters
- Development workflow and testing
- Troubleshooting and best practices

## License

MIT

## References

- [Chrono Documentation](https://projectchrono.org/docs/)
- Bekker, M. G. (1969). "Introduction to Terrain-Vehicle Systems"
- Mars soil properties from rover field studies (NASA/JPL)
