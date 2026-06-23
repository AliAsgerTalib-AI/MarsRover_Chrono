"""Simple visualization example - watch a rover drop and settle.

This is the simplest possible example of using PyChronoRover's 3D visualizer.
Just run it and watch the chassis fall onto Mars terrain in real-time.

Usage:
    python example_visualization.py
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from rover import SystemFactory, ChassisBuilder, TerrainManager, Visualizer


def main():
    """Simple drop test with real-time visualization."""

    print("\n" + "=" * 70)
    print("VISUALIZATION EXAMPLE: Chassis Drop on Mars Terrain")
    print("=" * 70)
    print("\nCamera Controls:")
    print("  - Mouse drag: Rotate view")
    print("  - Mouse scroll: Zoom")
    print("  - Right-click drag: Pan")
    print("  - Close window: End simulation\n")

    # Setup physics
    print("Initializing physics system...")
    system = SystemFactory.create_system(gravity_mars=True)

    # Create rover parts
    print("Creating chassis and terrain...")
    chassis = ChassisBuilder.create(system, pos_z=0.5)

    terrain_mgr = TerrainManager(system)
    terrain = terrain_mgr.initialize_scm(width=10.0, length=10.0, grid_resolution=0.05)

    # Create visualizer (static camera to see full scene)
    print("Opening 3D viewer...\n")
    print("You should see a box (chassis) falling onto terrain below.")
    print("Use mouse to rotate view. Close window when done.\n")
    viz = Visualizer(system, title="PyChronoRover: Chassis Drop", follow_body=None)

    # Run simulation with visualization
    # The visualizer handles the simulation loop internally
    viz.run(duration=2.0, time_step=0.01)

    print("\nSimulation complete!")
    print(f"Final chassis height: {chassis.GetPos().z:.4f} m")
    print(f"Total sinkage: {terrain_mgr.get_sinkage(chassis):.4f} m")


if __name__ == "__main__":
    main()
