# PyChronoRover 3D Visualization Guide

## Overview

PyChronoRover now includes real-time 3D visualization using pychrono's Irrlicht rendering engine. This allows you to **watch your rover simulations in real-time**, making it easier to understand physics behavior and validate designs.

## Quick Start

### Run a Scenario with 3D Viewer

```bash
# Scenario 0: Static chassis drop
python examples/scenario_0_static_drop.py --visualize

# Scenario 1: Single wheel slip characterization
python examples/scenario_1_single_wheel_slip.py --visualize

# Scenario 2: Kinematic control
python examples/scenario_2_kinematic_control.py --visualize
```

That's it! The 3D viewer opens automatically and shows the simulation in real-time.

### Camera Control

While the 3D viewer is running:

| Action | Effect |
|--------|--------|
| **Mouse drag** | Rotate camera |
| **Mouse scroll** | Zoom in/out |
| **Right-click drag** | Pan camera |
| **Close window** | Stop simulation and continue to data analysis |

The camera automatically follows the chassis (if enabled), or you can move it freely with mouse controls.

## Using the Visualizer in Your Code

### Basic Usage

```python
from rover import SystemFactory, ChassisBuilder, Visualizer

# Create system and bodies as normal
system = SystemFactory.create_system()
chassis = ChassisBuilder.create(system)

# Create visualizer that follows the chassis
viz = Visualizer(system, title="My Scenario", follow_body=chassis)

# Run simulation with visualization
viz.run(duration=10.0, time_step=0.01)
```

### With Metrics Collection

```python
from rover import MetricsCollector

metrics = MetricsCollector(output_freq=0.5)

def callback(time, step):
    if metrics.should_collect(time):
        frame = metrics.collect_frame(...)
        metrics.print_frame(frame)

viz.run(duration=10.0, time_step=0.01, callback=callback)
metrics.save_csv("output.csv")
```

### Static Camera View

```python
# Don't follow any body - static camera position
viz = Visualizer(system, title="My Scenario", follow_body=None)
viz.run(duration=10.0)
```

## Visualizer API Reference

### `Visualizer` Class

```python
class Visualizer:
    def __init__(self, system, title="PyChronoRover", follow_body=None):
        """Initialize visualizer.
        
        Args:
            system: ChSystemNSC physics system
            title: Window title string
            follow_body: ChBody to follow with camera (None = static)
        """

    def run(self, duration=10.0, time_step=0.01, callback=None):
        """Run simulation with real-time visualization.
        
        Args:
            duration: Total simulation time in seconds
            time_step: Physics time step in seconds
            callback: Optional function(time, step_count) called each frame
        """

    def close(self):
        """Manually close visualization window."""
```

### `VisualizationConfig` Class

Configure visualization settings:

```python
from rover import VisualizationConfig

config = VisualizationConfig()
config.window_width = 1600
config.window_height = 1200
config.camera_distance = 3.0
config.camera_height = 2.0
config.background_color = (135, 206, 235)  # RGB tuple
config.follow_camera = True
config.show_grid = True
config.show_aabb = False  # Bounding boxes
```

## Examples

### Example 1: Simple Drop with Visualization

```python
from rover import SystemFactory, ChassisBuilder, TerrainManager, Visualizer

system = SystemFactory.create_system(gravity_mars=True)
chassis = ChassisBuilder.create(system, pos_z=0.5)
terrain_mgr = TerrainManager(system)
terrain_mgr.initialize_scm(width=10.0, length=10.0)

# Create and run with visualization
viz = Visualizer(system, title="Chassis Drop", follow_body=chassis)
viz.run(duration=2.0, time_step=0.01)
```

### Example 2: Wheel Slip with Data Collection

```python
from rover import (
    SystemFactory, ChassisBuilder, WheelBuilder, 
    RotationMotor, LinearMotor, TerrainManager, 
    MetricsCollector, Visualizer
)
import pychrono as chrono

system = SystemFactory.create_system(gravity_mars=True)
chassis = ChassisBuilder.create(system, pos_z=0.6)
wheel = WheelBuilder.create(system)

terrain_mgr = TerrainManager(system)
terrain_mgr.initialize_scm(width=15.0, length=5.0)
terrain_mgr.add_active_domain(wheel)

# Set up motors
rotation_motor = RotationMotor.create_constant_speed(
    system, chassis, wheel, wheel.GetPos(), 
    chrono.ChVector3d(0, 1, 0), 2.0
)
linear_motor = LinearMotor.create_sweep_speed(
    system, ground, chassis, chassis.GetPos(), 0.6, 0.0, 10.0
)

# Collect metrics and visualize
metrics = MetricsCollector(output_freq=0.5)

def on_frame(time, step):
    if metrics.should_collect(time):
        frame = metrics.collect_frame(
            time, chassis, wheel, 0.3, 0.6, 
            linear_motor, terrain_mgr.get_height
        )
        metrics.print_frame(frame)

viz = Visualizer(system, title="Wheel Slip Test", follow_body=chassis)
viz.run(duration=10.0, time_step=0.01, callback=on_frame)
metrics.save_csv("wheel_slip_data.csv")
```

## Troubleshooting

### "Irrlicht visualization not available"

**Problem:** ImportError when creating Visualizer

**Cause:** pychrono installed without Irrlicht support

**Solution:**
```bash
# Reinstall pychrono with full support
conda install projectchrono::pychrono -c conda-forge --force-reinstall
```

### Window freezes or crashes

**Problem:** Application hangs when opening 3D viewer

**Cause:** GPU driver or display issues

**Solution:**
1. Try running without visualization first
2. Update GPU drivers
3. Run on a machine with dedicated GPU

### Performance is slow

**Problem:** Simulation runs slowly with visualization

**Cause:** High terrain grid resolution or expensive render settings

**Solution:**
1. Use coarser terrain grid: `grid_resolution=0.05` (not 0.01)
2. Reduce simulation duration for testing
3. Close other applications to free GPU resources

## Performance Tips

1. **Terrain Resolution:** Coarser grids (~0.05m) render faster
2. **Camera Following:** Disabling follow (`follow_body=None`) can be slightly faster
3. **Callback Functions:** Minimize work in callbacks to keep 60 FPS
4. **Headless Mode:** Run without visualization for batch simulations

## Next Steps

- [Add custom sensors](COMPONENTS.md#sensors) (wheel load cell, IMU)
- [Create suspension system](COMPONENTS.md#suspension) (springs, dampers)
- [Implement steering control](COMPONENTS.md#steering) (Ackermann, skid-steer)
- [Export visualization to video](#video-export) (coming soon)

## Video Export (Future)

Coming soon: Record 3D visualization to MP4 video for reports and presentations.
