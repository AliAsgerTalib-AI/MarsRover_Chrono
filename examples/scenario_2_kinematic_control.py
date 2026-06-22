"""Scenario 2: Kinematically Controlled Chassis with Vertical Settlement.

A carriage body is driven horizontally by a linear motor. The chassis is attached
to the carriage via a prismatic constraint that allows vertical settlement only.
This decouples horizontal motion from vertical loading for cleaner terramechanics analysis.

Original: rover_sandbox2.py (137 lines)
Refactored: 45 lines of application code
"""

import sys
from pathlib import Path

# Add src directory to path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

import pychrono as chrono

from rover import (
    CarriageConstraint,
    ChassisBuilder,
    GroundBuilder,
    LinearMotor,
    MetricsCollector,
    RotationMotor,
    SystemFactory,
    TerrainManager,
    WheelBuilder,
)


def main():
    """Run scenario 2: Kinematic carriage control."""
    print("\n" + "=" * 85)
    print("SCENARIO 2: KINEMATICALLY CONTROLLED CHASSIS WITH VERTICAL SETTLEMENT")
    print("=" * 85)

    # Create physics system
    system = SystemFactory.create_system(gravity_mars=True)

    # Create bodies
    ground = GroundBuilder.create(system)

    # Virtual carriage (moved by linear motor, allows vertical slip only)
    carriage = chrono.ChBody()
    carriage.SetPos(chrono.ChVector3d(0, 0, 0.6))
    carriage.SetMass(1.0)
    system.Add(carriage)

    # Chassis (settles vertically on carriage)
    chassis = ChassisBuilder.create(system, pos_z=0.6)

    # Wheel
    wheel = WheelBuilder.create(system, pos_y=0.5, pos_z=0.3)

    # Initialize terrain with active domain around wheel
    terrain_mgr = TerrainManager(system)
    terrain_mgr.initialize_scm(width=15.0, length=5.0, grid_resolution=0.015)
    terrain_mgr.add_active_domain(wheel)

    # Create actuators
    # Wheel rotation: constant 2.0 rad/s
    wheel_pos = wheel.GetPos()
    rotation_motor = RotationMotor.create_constant_speed(
        system, chassis, wheel, wheel_pos, chrono.ChVector3d(0, 1, 0), 2.0
    )

    # Carriage sweep: linear speed ramp from 0.6 m/s to 0.0 m/s
    v_ideal = 2.0 * 0.3  # 0.6 m/s
    total_time = 10.0

    carriage_pos = carriage.GetPos()
    linear_motor = LinearMotor.create_sweep_speed(
        system, ground, carriage, carriage_pos, v_ideal, 0.0, total_time
    )

    # Vertical prismatic constraint: carriage ↔ chassis
    CarriageConstraint.create_vertical_lock(system, carriage, chassis, chassis.GetPos())

    # Setup metrics collection
    metrics = MetricsCollector(output_freq=0.5)
    metrics.print_header()

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

        sim_time += time_step

    print("=" * 85)
    metrics.print_summary()
    metrics.save_csv("data/logs/scenario_2_kinematic_control.csv")


if __name__ == "__main__":
    main()
