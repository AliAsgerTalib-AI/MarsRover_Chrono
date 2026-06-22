# PyChronoRover Reusable Components Guide

This document explains the advanced reusable components that enable rapid prototyping of Mars rover simulations.

## Overview

PyChronoRover is organized around three key reusable component systems:

1. **Wheel Builders** - Different wheel types and arrays
2. **Terrain Configurators** - Preset and custom terrain setups
3. **Data Loggers** - Multiple output formats with real-time streaming

## Wheel Builders

### Basic Wheel Builder

The basic `WheelBuilder` (in `bodies.py`) creates standard rover wheels:

```python
from rover import WheelBuilder
import pychrono as chrono

system = chrono.ChSystemNSC()
wheel = WheelBuilder.create(
    system,
    radius=0.3,      # Standard wheel radius
    width=0.25,      # Standard wheel width
    mass=15.0,       # Standard wheel mass
    pos_y=0.5        # Position on Y axis
)
```

### Advanced Wheel Builder

The `AdvancedWheelBuilder` (in `wheels.py`) provides **preset wheel types** for different mission profiles:

```python
from rover import AdvancedWheelBuilder, WheelType

# Create standard wheel
wheel = AdvancedWheelBuilder.create_by_type(
    system,
    WheelType.STANDARD
)

# Create lightweight wheel (for extended range)
wheel = AdvancedWheelBuilder.create_by_type(
    system,
    WheelType.LIGHTWEIGHT
)

# Create high-grip wheel (for rocky terrain)
wheel = AdvancedWheelBuilder.create_by_type(
    system,
    WheelType.HIGH_GRIP
)
```

### Wheel Type Presets

| Wheel Type | Radius | Width | Mass | Friction | Use Case |
|------------|--------|-------|------|----------|----------|
| **STANDARD** | 0.3 m | 0.25 m | 15.0 kg | 0.70 | Default rover wheel |
| **LIGHTWEIGHT** | 0.3 m | 0.20 m | 10.0 kg | 0.65 | Extended range missions |
| **HIGH_GRIP** | 0.3 m | 0.28 m | 18.0 kg | 0.85 | Rocky/sandy terrain |
| **LOW_GRAVITY** | 0.25 m | 0.30 m | 8.0 kg | 0.60 | Moon/asteroid rovers |
| **FLEXIBLE** | 0.32 m | 0.22 m | 12.0 kg | 0.75 | Suspension-equipped |

### Custom Wheel Specifications

Create your own wheel spec:

```python
from rover import AdvancedWheelBuilder, WheelSpec, WheelType

custom_wheel = WheelSpec(
    name="My Custom Wheel",
    radius=0.35,
    width=0.24,
    mass=16.0,
    friction=0.72,
    wheel_type=WheelType.STANDARD
)

wheel = AdvancedWheelBuilder.create_from_spec(system, custom_wheel)
```

### Wheel Arrays

The `WheelArray` class (in `wheels.py`) builds common multi-wheel configurations:

```python
from rover import WheelArray, WheelType

# Single wheel (test configuration)
wheels = WheelArray.create_single_wheel(system)

# Two-wheel axle
wheels = WheelArray.create_two_wheel_axle(
    system,
    wheel_type=WheelType.HIGH_GRIP,
    track_width=0.6
)

# Four-wheel rover (2x2)
wheels = WheelArray.create_four_wheels(
    system,
    wheel_type=WheelType.STANDARD,
    wheelbase=0.8,
    track_width=0.6
)

# Six-wheel rover (3 axles)
wheels = WheelArray.create_six_wheels(
    system,
    wheel_type=WheelType.LIGHTWEIGHT,
    wheelbase=1.2,
    track_width=0.6,
    axle_spacing=0.4
)
```

**Example: Build 6-wheel rover with custom spacing**

```python
# Create 6-wheel array
wheels = WheelArray.create_six_wheels(
    system,
    wheel_type=WheelType.HIGH_GRIP,
    wheelbase=1.0,    # Shorter wheelbase
    track_width=0.7,  # Wider stance
    axle_spacing=0.33 # Tighter spacing for rough terrain
)

print(f"Created rover with {len(wheels)} wheels")

# Drive all wheels
for wheel in wheels:
    RotationMotor.create_constant_speed(
        system, chassis, wheel, wheel.GetPos(),
        chrono.ChVector3d(0, 1, 0), 2.0  # 2.0 rad/s
    )
```

## Terrain Configurators

### Terrain Presets

The `TerrainManager2` class provides **preset terrain configurations** for rapid testing:

```python
from rover import TerrainManager2, TerrainPreset

terrain_mgr = TerrainManager2(system)

# Initialize from preset
terrain_mgr.initialize_from_preset(TerrainPreset.MARS_FLAT)

# Query terrain
height = terrain_mgr.get_height(x=0.0, y=0.0)
sinkage = terrain_mgr.get_sinkage(wheel, wheel_radius=0.3)
```

### Available Terrain Presets

| Preset | Size | Resolution | Soil | Description |
|--------|------|------------|------|-------------|
| **MARS_FLAT** | 20×15 m | 0.05 m | Regolith | Large flat plain for testing |
| **MARS_VALLEY** | 15×5 m | 0.025 m | Regolith | Narrow valley, tests stability |
| **SANDY_PLAIN** | 10×15 m | 0.03 m | Sandy | Loose terrain, high sinkage |
| **ROCKY_TERRAIN** | 10×10 m | 0.02 m | Dust | Mixed rocks and dust |
| **TEST_TRACK** | 5×5 m | 0.05 m | Regolith | Small area for quick validation |

### Using Terrain Presets

```python
from rover import TerrainManager2, TerrainPreset

terrain_mgr = TerrainManager2(system)

# Quick setup for slip testing
terrain_mgr.initialize_from_preset(TerrainPreset.MARS_FLAT)

# Quick setup for validation
terrain_mgr.initialize_from_preset(TerrainPreset.TEST_TRACK)
```

### Custom Terrain Configuration

Use the `TerrainConfigBuilder` for custom terrain:

```python
from rover import TerrainConfigBuilder, SoilType, TerrainManager2

# Build custom configuration using fluent API
config = (
    TerrainConfigBuilder("My Custom Terrain")
    .with_dimensions(width=12.0, length=8.0)
    .with_grid_resolution(0.02)  # Fine resolution
    .with_soil_type(SoilType.SANDY_SOIL)
    .with_description("Sandy terrain with fine deformation")
    .build()
)

# Initialize terrain with custom config
terrain_mgr = TerrainManager2(system)
terrain_mgr.initialize_from_config(config)
```

### Save and Load Terrain Configurations

```python
# Save configuration for reproducibility
terrain_mgr.save_config("my_terrain_config.json")

# Load configuration later
terrain_mgr = TerrainManager2(system)
terrain_mgr.load_config_from_file("my_terrain_config.json")

# Get current configuration
config = terrain_mgr.get_config()
print(f"Terrain: {config.name}")
print(f"Size: {config.length}m x {config.width}m")
```

### Terrain Configuration Structure

```python
@dataclass
class TerrainConfig:
    name: str                      # Configuration name
    width: float                   # Y direction (m)
    length: float                  # X direction (m)
    grid_resolution: float         # Node spacing (m)
    soil_type: SoilType           # Soil type (mars_regolith, sandy, etc.)
    description: str              # Human-readable description
```

## Data Loggers

### Basic Data Logger

The `DataLogger` class provides flexible output formats:

```python
from utils import DataLogger, LogFormat

# Create logger (CSV only)
logger = DataLogger(
    output_dir="data/logs",
    name="my_simulation",
    format=LogFormat.CSV
)

# Log data during simulation
logger.log({
    "velocity_x": 0.3,
    "slip_percent": 25.5,
    "sinkage_mm": 15.2,
    "drawbar_force": 100.5,
})

# Flush and save
logger.save()
```

### Multiple Output Formats

```python
# CSV only
logger = DataLogger(format=LogFormat.CSV)

# JSON only
logger = DataLogger(format=LogFormat.JSON)

# Both CSV and JSON
logger = DataLogger(format=LogFormat.BOTH)
```

### Console Logger

The `ConsoleLogger` provides real-time simulation output:

```python
from utils import ConsoleLogger

console = ConsoleLogger(verbose=True)

# Log messages
console.info("Starting simulation")
console.warning("High slip detected")

# Print formatted tables
console.table_header(
    ["Time (s)", "Velocity (m/s)", "Slip (%)"],
    [10, 15, 12]
)

console.table_row([0.0, 0.30, 0.0], [10, 15, 12])
console.table_row([0.5, 0.29, 3.3], [10, 15, 12])
console.table_row([1.0, 0.27, 10.0], [10, 15, 12])

console.table_separator([10, 15, 12])
```

### Accessing and Analyzing Logged Data

```python
# Get all data
all_data = logger.get_data()
print(f"Logged {len(all_data)} entries")

# Get single column
velocities = logger.get_column("velocity_x")
print(f"Velocities: {velocities}")

# Get statistics
stats = logger.get_statistics("slip_percent")
print(f"Slip - Mean: {stats['mean']:.1f}%, Std: {stats['std']:.1f}%")

# Print summary
logger.print_summary(columns=["velocity_x", "slip_percent", "sinkage_mm"])
```

## Complete Example: Multi-Wheel Rover Test

Combining all components:

```python
import pychrono as chrono
from rover import (
    ChassisBuilder,
    GroundBuilder,
    WheelArray,
    WheelType,
    RotationMotor,
    LinearMotor,
    SystemFactory,
    TerrainManager2,
    TerrainPreset,
)
from utils import DataLogger, ConsoleLogger, LogFormat

# Create system
system = SystemFactory.create_system(gravity_mars=True)

# Create chassis
chassis = ChassisBuilder.create(system, pos_z=0.8)

# Create 4-wheel array with high-grip wheels
wheels = WheelArray.create_four_wheels(
    system,
    wheel_type=WheelType.HIGH_GRIP,
    wheelbase=0.8,
    track_width=0.6
)

# Initialize terrain from preset
terrain_mgr = TerrainManager2(system)
terrain_mgr.initialize_from_preset(TerrainPreset.SANDY_PLAIN)

# Setup motors
ground = GroundBuilder.create(system)
linear_motor = LinearMotor.create_constant_speed(
    system, ground, chassis, chassis.GetPos(), 0.4
)

for wheel in wheels:
    RotationMotor.create_constant_speed(
        system, chassis, wheel, wheel.GetPos(),
        chrono.ChVector3d(0, 1, 0), 2.0
    )

# Setup logging
logger = DataLogger(
    output_dir="data/logs",
    name="four_wheel_sandy_test",
    format=LogFormat.BOTH
)

console = ConsoleLogger(verbose=True)
console.table_header(["Time", "Velocity", "Avg Sinkage"], [10, 12, 15])

# Run simulation
time_step = 0.01
sim_time = 0.0

while sim_time <= 3.0:
    system.DoStepDynamics(time_step)

    if int(sim_time * 100) % 50 == 0:
        v = chassis.GetPosDt().x
        sinkages = [terrain_mgr.get_sinkage(w, 0.3) * 1000 for w in wheels]
        avg_sinkage = sum(sinkages) / len(sinkages)

        logger.log({
            "time": sim_time,
            "velocity_x": v,
            "avg_sinkage_mm": avg_sinkage,
        })

        console.table_row([f"{sim_time:.1f}", f"{v:.3f}", f"{avg_sinkage:.1f}"],
                         [10, 12, 15])

    sim_time += time_step

console.table_separator([10, 12, 15])
logger.save()
logger.print_summary()
```

## Customization Patterns

### Add Custom Wheel Type

```python
from rover import WheelSpec, WheelType, AdvancedWheelBuilder

# Define custom wheel
my_wheel = WheelSpec(
    name="Martian Sand Wheel",
    radius=0.35,
    width=0.30,
    mass=17.0,
    friction=0.80,
    wheel_type=WheelType.STANDARD  # or custom category
)

# Create in simulation
wheel = AdvancedWheelBuilder.create_from_spec(system, my_wheel)
```

### Add Custom Terrain Preset

```python
from rover import TerrainConfig, SoilType, TerrainManager2

# Define custom terrain config
my_terrain = TerrainConfig(
    name="Crater Rim Terrain",
    width=20.0,
    length=30.0,
    grid_resolution=0.015,
    soil_type=SoilType.LOOSE_DUST,
    description="High-fidelity crater rim simulation"
)

# Initialize terrain
terrain_mgr = TerrainManager2(system)
terrain_mgr.initialize_from_config(my_terrain)
```

### Custom Data Logger Output

```python
from utils import DataLogger

logger = DataLogger(
    output_dir="my_results",
    name="custom_test_v2",
    format=LogFormat.JSON
)

# Log custom fields
logger.log({
    "time": sim_time,
    "wheel_1_load": wheel_load_1,
    "wheel_2_load": wheel_load_2,
    "wheel_3_load": wheel_load_3,
    "wheel_4_load": wheel_load_4,
    "imu_accel_x": accel_x,
    "imu_accel_y": accel_y,
    "imu_accel_z": accel_z,
}, timestamp=sim_time)
```

## Benefits of This Design

✅ **Rapid Prototyping**
- Swap wheel types with one parameter
- Change terrain with one preset
- New rover configuration in <5 lines

✅ **Reproducibility**
- Save/load terrain configurations as JSON
- Consistent wheel specifications
- Documented parameter sets

✅ **Flexibility**
- Easy to add new wheel types or presets
- Custom terrain configurations
- Multiple output formats

✅ **Maintainability**
- Clear separation of concerns
- Reusable across projects
- Minimal code duplication

## Next Steps

- See `example_advanced_components.py` for complete working examples
- Customize wheel types for specific mission profiles
- Create terrain presets for common test scenarios
- Extend data logger with additional statistics
