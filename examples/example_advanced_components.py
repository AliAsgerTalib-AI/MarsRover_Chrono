"""Example: Using Advanced Reusable Components

Demonstrates:
- Advanced wheel builders (different wheel types)
- Terrain configurators (presets and custom configs)
- Advanced data loggers (multiple formats, statistics)

This shows how the modular design enables rapid prototyping of different
rover configurations and simulation setups.
"""

import sys
from pathlib import Path

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

import pychrono as chrono

from rover import (
    AdvancedWheelBuilder,
    GroundBuilder,
    LinearMotor,
    RotationMotor,
    SystemFactory,
    TerrainManager2,
    TerrainPreset,
    WheelArray,
    WheelType,
    ChassisBuilder,
)
from utils import DataLogger, ConsoleLogger, LogFormat


def example_1_different_wheel_types():
    """Example 1: Test different wheel types on same terrain."""
    print("\n" + "=" * 80)
    print("EXAMPLE 1: COMPARING DIFFERENT WHEEL TYPES ON MARS REGOLITH")
    print("=" * 80)

    # Test different wheel types
    wheel_types = [
        WheelType.STANDARD,
        WheelType.LIGHTWEIGHT,
        WheelType.HIGH_GRIP,
    ]

    results = {}

    for wheel_type in wheel_types:
        print(f"\nTesting {wheel_type.value} wheel...")

        # Create system
        system = SystemFactory.create_system(gravity_mars=True)

        # Create chassis
        chassis = ChassisBuilder.create(system, pos_z=0.6)

        # Create wheel using advanced builder
        wheel = AdvancedWheelBuilder.create_by_type(
            system, wheel_type, pos_y=0.5, pos_z=0.3
        )

        # Setup terrain using new configurator
        terrain_mgr = TerrainManager2(system)
        terrain_mgr.initialize_from_preset(TerrainPreset.TEST_TRACK)

        # Setup motors
        v_ideal = 2.0 * 0.3  # omega * r
        rotation_motor = RotationMotor.create_constant_speed(
            system, chassis, wheel, wheel.GetPos(), chrono.ChVector3d(0, 1, 0), 2.0
        )
        linear_motor = LinearMotor.create_sweep_speed(
            system, GroundBuilder.create(system), chassis, chassis.GetPos(),
            v_ideal, 0.0, 5.0
        )

        # Data logger for this wheel type
        logger = DataLogger(
            output_dir="data/logs",
            name=f"wheel_comparison_{wheel_type.value}",
            format=LogFormat.CSV,
        )

        # Run simulation
        time_step = 0.01
        sim_time = 0.0
        collection_freq = 0.5
        last_log_time = 0.0

        while sim_time <= 5.0:
            system.DoStepDynamics(time_step)

            if sim_time - last_log_time >= collection_freq:
                v_actual = chassis.GetPosDt().x
                slip = (1.0 - v_actual / v_ideal) * 100 if v_ideal > 0 else 100.0
                sinkage = terrain_mgr.get_sinkage(wheel, 0.3) * 1000

                logger.log({
                    "time": sim_time,
                    "velocity_x": v_actual,
                    "slip_percent": slip,
                    "sinkage_mm": sinkage,
                    "drawbar_force": -linear_motor.GetMotorForce(),
                })

                last_log_time = sim_time

            sim_time += time_step

        logger.save()
        results[wheel_type.value] = logger.get_statistics("slip_percent")

    # Print comparison
    print("\n" + "=" * 80)
    print("WHEEL TYPE COMPARISON (Slip Ratio Statistics)")
    print("=" * 80)
    for wheel_type, stats in results.items():
        print(f"\n{wheel_type}:")
        print(f"  Mean slip: {stats.get('mean', 0):.1f}%")
        print(f"  Min slip:  {stats.get('min', 0):.1f}%")
        print(f"  Max slip:  {stats.get('max', 0):.1f}%")


def example_2_custom_wheel_array():
    """Example 2: Custom 6-wheel rover with advanced components."""
    print("\n" + "=" * 80)
    print("EXAMPLE 2: 6-WHEEL ROVER WITH CUSTOM TERRAIN CONFIGURATION")
    print("=" * 80)

    # Create system
    system = SystemFactory.create_system(gravity_mars=True)

    # Create chassis
    chassis = ChassisBuilder.create(system, pos_z=0.8)

    # Create 6-wheel array using WheelArray
    wheels = WheelArray.create_six_wheels(
        system,
        wheel_type=WheelType.HIGH_GRIP,
        wheelbase=1.2,
        track_width=0.8,
        axle_spacing=0.4,
    )

    print(f"\nCreated 6-wheel rover with {len(wheels)} wheels")
    for i, wheel in enumerate(wheels):
        pos = wheel.GetPos()
        print(f"  Wheel {i+1}: pos=({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})")

    # Create custom terrain configuration using builder
    from rover import TerrainConfigBuilder, SoilType

    terrain_config = (
        TerrainConfigBuilder("Custom Mars Valley")
        .with_dimensions(width=8.0, length=12.0)
        .with_grid_resolution(0.02)
        .with_soil_type(SoilType.MARS_REGOLITH)
        .with_description("Narrow valley with fine resolution for 6-wheel rover")
        .build()
    )

    # Initialize terrain with custom config
    terrain_mgr = TerrainManager2(system)
    terrain_mgr.initialize_from_config(terrain_config)

    # Save the configuration for reproducibility
    terrain_mgr.save_config("data/logs/custom_terrain_config.json")
    print(f"\nTerrain configuration saved:")
    print(f"  {terrain_config.description}")
    print(f"  Size: {terrain_config.length}m x {terrain_config.width}m")
    print(f"  Grid resolution: {terrain_config.grid_resolution}m")

    # Multi-format data logger (CSV + JSON)
    logger = DataLogger(
        output_dir="data/logs",
        name="six_wheel_rover_test",
        format=LogFormat.BOTH,  # Save as both CSV and JSON
    )

    console = ConsoleLogger(verbose=True)
    console.info("Starting 6-wheel rover simulation...")
    console.table_header(
        ["Time (s)", "Wheels sinking (avg mm)", "Velocity (m/s)"],
        [12, 25, 15]
    )

    # Run simulation
    time_step = 0.01
    sim_time = 0.0
    collection_freq = 0.25

    # Add constant forward motion
    ground = GroundBuilder.create(system)
    linear_motor = LinearMotor.create_constant_speed(
        system, ground, chassis, chassis.GetPos(), 0.3
    )

    for wheel in wheels:
        RotationMotor.create_constant_speed(
            system, chassis, wheel, wheel.GetPos(), chrono.ChVector3d(0, 1, 0), 2.0
        )

    last_log_time = 0.0
    while sim_time <= 3.0:
        system.DoStepDynamics(time_step)

        if sim_time - last_log_time >= collection_freq:
            # Calculate average sinkage across all wheels
            sinkages = [terrain_mgr.get_sinkage(w, 0.3) * 1000 for w in wheels]
            avg_sinkage = sum(sinkages) / len(sinkages)

            v_actual = chassis.GetPosDt().x

            logger.log({
                "time": sim_time,
                "velocity_x": v_actual,
                "avg_sinkage_mm": avg_sinkage,
                "max_sinkage_mm": max(sinkages),
                "min_sinkage_mm": min(sinkages),
            })

            console.table_row(
                [f"{sim_time:.2f}", f"{avg_sinkage:.2f}", f"{v_actual:.3f}"],
                [12, 25, 15]
            )

            last_log_time = sim_time

        sim_time += time_step

    console.table_separator([12, 25, 15])
    logger.save()
    logger.print_summary()

    console.info(f"Logged {len(logger.all_data)} data points")
    console.info(f"Output formats: CSV and JSON")


def example_3_terrain_presets():
    """Example 3: Quick comparison of different terrain presets."""
    print("\n" + "=" * 80)
    print("EXAMPLE 3: TERRAIN PRESET COMPARISON")
    print("=" * 80)

    from rover import TerrainPresetLibrary

    print("\nAvailable terrain presets:")
    for name, desc in TerrainPresetLibrary.list_presets().items():
        print(f"  • {name}: {desc}")

    # Show details of each preset
    print("\n" + "-" * 80)
    print("TERRAIN PRESET DETAILS")
    print("-" * 80)

    presets = [
        TerrainPreset.MARS_FLAT,
        TerrainPreset.MARS_VALLEY,
        TerrainPreset.SANDY_PLAIN,
    ]

    for preset in presets:
        config = TerrainPresetLibrary.get_preset(preset)
        print(f"\n{config.name}:")
        print(f"  Description: {config.description}")
        print(f"  Dimensions: {config.length}m (length) x {config.width}m (width)")
        print(f"  Grid resolution: {config.grid_resolution}m")
        print(f"  Soil type: {config.soil_type.value}")
        print(f"  Est. nodes: {int((config.length/config.grid_resolution) * (config.width/config.grid_resolution))}")


def main():
    """Run all examples."""
    print("\n" + "=" * 80)
    print("DEMONSTRATING ADVANCED REUSABLE COMPONENTS")
    print("=" * 80)
    print("\nThis example shows how the modular design enables:")
    print("  1. Testing different wheel types easily")
    print("  2. Building custom rovers with predefined components")
    print("  3. Using terrain presets or custom configurations")
    print("  4. Advanced data logging with multiple formats")

    # Run examples
    example_3_terrain_presets()  # Start with quick overview
    # example_1_different_wheel_types()  # Uncomment to test wheel types (slower)
    # example_2_custom_wheel_array()     # Uncomment to test 6-wheel rover (slower)

    print("\n" + "=" * 80)
    print("✓ Examples complete")
    print("=" * 80)
    print("\nCheck data/logs/ for output files (CSV, JSON)")


if __name__ == "__main__":
    main()
