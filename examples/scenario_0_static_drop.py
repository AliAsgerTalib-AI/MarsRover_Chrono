"""Scenario 0: Static Chassis Drop on Deformable Terrain.

A simple test of terrain deformation with a free-falling chassis.
This validates the SCM terrain model and sinkage calculation.

Original: rover_sandbox.py (79 lines)
Refactored: 25 lines of application code

Usage:
    python scenario_0_static_drop.py              # No visualization
    python scenario_0_static_drop.py --visualize  # With 3D viewer
"""

import sys
from pathlib import Path

# Add src directory to path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from rover import ChassisBuilder, SystemFactory, TerrainManager, Visualizer


def main(visualize=False):
    """Run scenario 0: Static drop test.

    Args:
        visualize: If True, open 3D viewer for real-time visualization
    """
    print("\n" + "=" * 70)
    print("SCENARIO 0: STATIC CHASSIS DROP ON MARS TERRAIN")
    print("=" * 70)

    # Create physics system
    system = SystemFactory.create_system(gravity_mars=True)

    # Create chassis
    chassis = ChassisBuilder.create(system, pos_z=0.5)

    # Initialize terrain
    terrain_mgr = TerrainManager(system)
    terrain = terrain_mgr.initialize_scm(width=10.0, length=10.0, grid_resolution=0.05)

    # Setup visualization if requested
    viz = None
    if visualize:
        try:
            viz = Visualizer(system, title="Scenario 0: Static Drop", follow_body=chassis)
            print("3D Viewer opened - close window to continue\n")
        except ImportError as e:
            print(f"Warning: Visualization not available ({e})\n")

    # Simulation loop
    time_step = 0.01
    sim_time = 0.0
    max_time = 2.0

    print(f"\n{'Time (s)':<10} | {'Chassis Z (m)':<20} | {'Max Sinkage (m)':<20}")
    print("-" * 60)

    while sim_time < max_time:
        system.DoStepDynamics(time_step)

        chassis_z = chassis.GetPos().z
        sinkage = terrain_mgr.get_sinkage(chassis)

        # Output every 0.2 seconds
        if int(sim_time * 100) % 20 == 0:
            print(f"{sim_time:<10.2f} | {chassis_z:<20.4f} | {sinkage:<20.4f}")

        # Render visualization if active
        if viz:
            try:
                viz.vis.Render()
                if not viz.vis.Run():
                    break  # Window closed
            except:
                viz = None

        sim_time += time_step

    print("-" * 60)
    print("[STATUS]: Simulation complete. Terrain deformation validated.")


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Scenario 0: Static Drop")
    parser.add_argument(
        "--visualize",
        action="store_true",
        help="Open 3D viewer for real-time visualization"
    )
    args = parser.parse_args()
    main(visualize=args.visualize)
