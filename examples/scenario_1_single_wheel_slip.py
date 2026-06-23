"""Scenario 1: Single Wheel Slip-Sinkage Characterization.

A single wheel drives against the chassis while slip is swept from 0% to 100%.
This characterizes wheel-terrain interaction and traction performance.

Original: rover_sandbox1.py (150 lines)
Refactored: 35 lines of application code

Usage:
    python scenario_1_single_wheel_slip.py              # No visualization
    python scenario_1_single_wheel_slip.py --visualize  # With 3D viewer
"""

import sys
from pathlib import Path

# Add src directory to path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

import pychrono as chrono

from rover import (
    ChassisBuilder,
    GroundBuilder,
    LinearMotor,
    MetricsCollector,
    RotationMotor,
    SystemFactory,
    TerrainManager,
    WheelBuilder,
    Visualizer,
)


def main(visualize=False):
    """Run scenario 1: Single wheel slip sweep.

    Args:
        visualize: If True, open 3D viewer for real-time visualization
    """
    print("\n" + "=" * 85)
    print("SCENARIO 1: SINGLE WHEEL SLIP-SINKAGE CHARACTERIZATION")
    print("=" * 85)

    # Create physics system
    system = SystemFactory.create_system(gravity_mars=True)

    # Create bodies
    ground = GroundBuilder.create(system)
    chassis = ChassisBuilder.create(system, pos_z=0.6)
    wheel = WheelBuilder.create(system, pos_y=0.5, pos_z=0.3)

    # Initialize terrain
    terrain_mgr = TerrainManager(system)
    terrain_mgr.initialize_scm(width=15.0, length=5.0, grid_resolution=0.025)
    terrain_mgr.add_active_domain(wheel)

    # Create actuators
    # Wheel rotation: constant 2.0 rad/s
    wheel_pos = wheel.GetPos()
    rotation_motor = RotationMotor.create_constant_speed(
        system, chassis, wheel, wheel_pos, chrono.ChVector3d(0, 1, 0), 2.0
    )

    # Chassis sweep: linear speed ramp from 0.6 m/s to 0.0 m/s over 10 seconds
    # (This sweeps slip from 0% to 100%)
    v_ideal = 2.0 * 0.3  # omega * r = 0.6 m/s
    total_time = 10.0

    chassis_pos = chassis.GetPos()
    linear_motor = LinearMotor.create_sweep_speed(
        system, ground, chassis, chassis_pos, v_ideal, 0.0, total_time
    )

    # Setup metrics collection
    metrics = MetricsCollector(output_freq=0.5)
    metrics.print_header()

    # Setup visualization if requested
    viz = None
    if visualize:
        try:
            viz = Visualizer(system, title="Scenario 1: Wheel Slip", follow_body=chassis)
            print("3D Viewer opened - close window to continue\n")
        except ImportError as e:
            print(f"Warning: Visualization not available ({e})\n")

    # Simulation loop
    time_step = 0.01
    sim_time = 0.0

    while sim_time <= total_time:
        system.DoStepDynamics(time_step)

        if metrics.should_collect(sim_time):
            frame = metrics.collect_frame(
                sim_time,
                chassis,
                wheel,
                0.3,  # wheel radius
                v_ideal,
                linear_motor,
                terrain_mgr.get_height,
            )
            metrics.print_frame(frame)

        # Render visualization if active
        if viz:
            try:
                viz.vis.Render()
                if not viz.vis.Run():
                    break  # Window closed
            except:
                viz = None

        sim_time += time_step

    print("=" * 85)
    metrics.print_summary()
    metrics.save_csv("data/logs/scenario_1_slip_sweep.csv")


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Scenario 1: Single Wheel Slip")
    parser.add_argument(
        "--visualize",
        action="store_true",
        help="Open 3D viewer for real-time visualization"
    )
    args = parser.parse_args()
    main(visualize=args.visualize)
