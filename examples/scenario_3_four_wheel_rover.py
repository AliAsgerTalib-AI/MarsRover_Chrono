"""Scenario 3: Four-Wheel Rover Locomotion Test.

A 4-wheel rover (2x2 configuration) with independent wheel drive.
This demonstrates multi-wheel coordination and load distribution.

Status: TEMPLATE - Ready for implementation
"""

import sys
from pathlib import Path

# Add src directory to path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

import pychrono as chrono

from rover import (
    LinearMotor,
    MetricsCollector,
    MultiWheelBuilder,
    RotationMotor,
    SystemFactory,
    TerrainManager,
)


def main():
    """Run scenario 3: Four-wheel rover test."""
    print("\n" + "=" * 85)
    print("SCENARIO 3: FOUR-WHEEL ROVER LOCOMOTION TEST")
    print("=" * 85)

    # Create physics system
    system = SystemFactory.create_system(gravity_mars=True)

    # Create 4-wheel rover
    rover = MultiWheelBuilder.create_four_wheel_rover(
        system,
        chassis_mass=50.0,
        wheel_mass=15.0,
        wheel_radius=0.3,
        wheelbase=0.8,
        track_width=0.6,
    )

    chassis = rover["chassis"]
    wheels = rover["wheels"]

    # Initialize terrain
    terrain_mgr = TerrainManager(system)
    terrain_mgr.initialize_scm(width=20.0, length=10.0, grid_resolution=0.025)

    # Register all wheels in active domain
    for wheel in wheels:
        terrain_mgr.add_active_domain(wheel)

    # Create wheel motors: all wheels drive at constant 2.0 rad/s
    motors = []
    for i, wheel in enumerate(wheels):
        motor = RotationMotor.create_constant_speed(
            system,
            chassis,
            wheel,
            wheel.GetPos(),
            chrono.ChVector3d(0, 1, 0),  # Y-axis rotation
            2.0,  # rad/s
        )
        motors.append(motor)

    # TODO: Implement chassis drive motor for forward locomotion
    # chassis_motor = LinearMotor.create_constant_speed(...)

    # Setup metrics collection
    metrics = MetricsCollector(output_freq=0.5)
    metrics.print_header()

    # Simulation loop
    time_step = 0.01
    sim_time = 0.0
    total_time = 5.0  # Shorter test for 4-wheel rover

    while sim_time <= total_time:
        system.DoStepDynamics(time_step)

        # TODO: Implement metrics for 4-wheel configuration
        # - Average wheel loads
        # - Load distribution
        # - Mean sinkage
        # - Overall traction efficiency

        sim_time += time_step

    print("=" * 85)
    print("[STATUS]: Four-wheel rover simulation complete.")
    print("[TODO]: Implement metrics for multi-wheel analysis")


if __name__ == "__main__":
    main()
