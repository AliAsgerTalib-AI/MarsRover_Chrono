# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Quick Commands

```bash
# Run simulations
python examples/scenario_0_static_drop.py         # Terrain deformation test
python examples/scenario_1_single_wheel_slip.py   # Slip-sinkage characterization
python examples/scenario_2_kinematic_control.py   # Kinematic chassis control
python examples/scenario_3_four_wheel_rover.py    # Multi-wheel rover

# Run with 3D visualization (requires Irrlicht support in pychrono)
python examples/scenario_1_single_wheel_slip.py --visualize
python examples/example_visualization.py

# Testing
pytest tests/ -v                    # Run all tests
pytest tests/test_system.py -v      # System factory tests only
pytest tests/test_validation.py -v  # Validation tests only
```

**Note**: All test files require `sys.path.insert(0, str(Path(__file__).parent.parent / "src"))` at the top to resolve the `rover` and `utils` packages.

## Architecture

### Three-Layer Design

**1. Physics Engine Layer** (`src/rover/system.py`, `src/rover/materials.py`)
- `SystemFactory.create_system(gravity_mars=True)` → `ChSystemNSC` with Mars gravity (-3.71 m/s²) and Bullet collision
- `SystemFactory.configure_solver()` → sets EULER_IMPLICIT_LINEARIZED timestepper and iteration limits
- `MaterialFactory` → creates friction/restitution contact materials for chassis and wheels

**2. Mechanical Design Layer** (`src/rover/bodies.py`, `src/rover/motors.py`, `src/rover/constraints.py`, `src/rover/wheels.py`)
- `GroundBuilder`, `ChassisBuilder`, `WheelBuilder`, `MultiWheelBuilder` — basic rigid body construction
- `AdvancedWheelBuilder` + `WheelSpecLibrary` + `WheelArray` — preset wheel specs (STANDARD, LIGHTWEIGHT, HIGH_GRIP, LOW_GRAVITY, FLEXIBLE) and convenience builders for 1/2/4/6-wheel arrays
- `RotationMotor` — wheel drive (constant speed, profiled)
- `LinearMotor` — chassis drive (constant, sweep/ramp, sine profiles)
- `PrismaticConstraint`, `RevoluteJoint`, `CarriageConstraint` — kinematic constraints

**3. Terrain & Measurement Layer** (`src/rover/terrain.py`, `src/rover/terrain_config.py`, `src/rover/metrics.py`)
- `TerrainManager` — direct SCM terrain setup with `SoilParameterSet` presets (MARS_REGOLITH, SANDY_SOIL, LOOSE_DUST)
- `TerrainManager2` — enhanced manager with preset/config/JSON support; use `initialize_from_preset(TerrainPreset.MARS_FLAT)` for one-liner setup. Configs can be saved/loaded as JSON.
- `TerrainPresetLibrary` — named configs: MARS_FLAT, MARS_VALLEY, SANDY_PLAIN, ROCKY_TERRAIN, TEST_TRACK
- `TerrainConfigBuilder` — fluent builder for custom terrain configs
- `MetricsCollector` — collects slip, velocity, drawbar pull, sinkage at a configurable frequency; outputs CSV

### Supporting Modules

- `src/rover/visualizer.py` — Irrlicht real-time 3D viewer; gracefully degrades if Irrlicht unavailable (`create_visualizer_for_scenario` returns `None`)
- `src/utils/data_logger.py` — `DataLogger` with CSV/JSON/both output formats; more flexible than `MetricsCollector.save_csv()`
- `config/default_config.py` — all numeric constants (chassis/wheel sizes, soil params, time steps) in one place
- `src/sandbox/` — original pre-refactor sandbox scripts kept for reference/numerical comparison

### Public API

All modules are re-exported from `src/rover/__init__.py`. Import everything from `rover`:

```python
from rover import (
    SystemFactory, ChassisBuilder, WheelBuilder, GroundBuilder, MultiWheelBuilder,
    AdvancedWheelBuilder, WheelArray, WheelSpec, WheelSpecLibrary, WheelType,
    TerrainManager, TerrainManager2, TerrainConfig, TerrainConfigBuilder,
    TerrainPreset, TerrainPresetLibrary, SoilParameterSet, SoilType,
    RotationMotor, LinearMotor, MotorProfile,
    PrismaticConstraint, RevoluteJoint, CarriageConstraint,
    MetricsCollector, MetricFrame,
    Visualizer, VisualizationConfig, create_visualizer_for_scenario,
)
```

## Common Patterns

### Standard Simulation Loop

```python
system = SystemFactory.create_system(gravity_mars=True)
ground = GroundBuilder.create(system)
chassis = ChassisBuilder.create(system, pos_z=0.6)
wheel = WheelBuilder.create(system, pos_y=0.5, pos_z=0.3)

terrain_mgr = TerrainManager(system)
terrain_mgr.initialize_scm(width=15.0, length=5.0)  # BEFORE add_active_domain()
terrain_mgr.add_active_domain(wheel)

rotation_motor = RotationMotor.create_constant_speed(
    system, chassis, wheel, wheel.GetPos(), chrono.ChVector3d(0, 1, 0), 2.0
)
linear_motor = LinearMotor.create_sweep_speed(
    system, ground, chassis, chassis.GetPos(), 0.6, 0.0, 10.0
)

metrics = MetricsCollector(output_freq=0.5)
sim_time = 0.0
while sim_time <= 10.0:
    system.DoStepDynamics(0.01)
    if metrics.should_collect(sim_time):
        frame = metrics.collect_frame(
            sim_time, chassis, wheel, 0.3, 0.6, linear_motor, terrain_mgr.get_height
        )
        metrics.print_frame(frame)
    sim_time += 0.01

metrics.save_csv("data/logs/output.csv")
```

**Critical ordering**: `initialize_scm()` must be called before `add_active_domain()`.

### Using Terrain Presets (TerrainManager2)

```python
terrain_mgr = TerrainManager2(system)
terrain = terrain_mgr.initialize_from_preset(TerrainPreset.MARS_FLAT)
terrain_mgr.save_config("config/my_terrain.json")  # Reproducible configs
```

### Advanced Wheel Configurations

```python
# Preset wheel type
wheel = AdvancedWheelBuilder.create_by_type(system, WheelType.HIGH_GRIP, pos_y=0.5)

# 4-wheel array
wheels = WheelArray.create_four_wheels(system, WheelType.STANDARD, wheelbase=0.8, track_width=0.6)
# Returns [FR, FL, RR, RL]
```

## Physics Reference

### Slip Ratio
```
slip (%) = (1 - v_actual / v_ideal) × 100
v_ideal = ω × r   (angular velocity × wheel radius)
v_actual = chassis dx/dt
```
0% = perfect grip, 100% = spinning in place.

### Sinkage
```
sinkage = |terrain_height - (wheel_center_z - wheel_radius)|
```

### Bekker Soil Presets (`SoilParameterSet`)

| Preset | kc (Pa/m) | kphi (Pa/m) | n | c (Pa) | φ (°) | Expected sinkage |
|--------|-----------|-------------|---|--------|--------|-----------------|
| MARS_REGOLITH | 0.02e6 | 0.25e6 | 1.1 | 500 | 35 | Moderate |
| SANDY_SOIL | 0.05e6 | 0.4e6 | 0.9 | 200 | 32 | Higher |
| LOOSE_DUST | 0.01e6 | 0.1e6 | 1.2 | 100 | 25 | Extreme |

## Troubleshooting

**Simulation runs slowly**: Grid resolution is the main lever. `grid_resolution=0.05` (5 cm) is fast enough for testing; `0.015` is high-fidelity but ~10× slower.

**Motor has no effect**: Verify body ordering — `body1` is the anchor (ground or chassis), `body2` is the driven body.

**Instability/constraint violations**: Reduce `time_step` (try `0.005`) or increase solver iterations via `SystemFactory.configure_solver(system, max_iterations=30)`.

**Sinkage is unrealistic**: Check that soil preset matches the intended terrain type — LOOSE_DUST causes extreme sinkage.

**Visualization not available**: `pychrono.irrlicht` is optional; `create_visualizer_for_scenario` returns `None` if missing. Simulations work headlessly without it.
