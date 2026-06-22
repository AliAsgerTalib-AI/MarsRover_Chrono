"""Comprehensive validation script for PyChronoRover.

This script:
1. Tests scenario execution
2. Verifies outputs are reasonable
3. Checks component functionality
4. Reports validation results
"""

import sys
from pathlib import Path

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

import pychrono as chrono
from rover import (
    SystemFactory,
    ChassisBuilder,
    WheelBuilder,
    AdvancedWheelBuilder,
    WheelType,
    WheelArray,
    TerrainManager2,
    TerrainPreset,
    GroundBuilder,
    LinearMotor,
    RotationMotor,
    MetricsCollector,
    TerrainConfigBuilder,
    SoilType,
)
from utils import DataLogger, LogFormat


def print_header(title):
    """Print formatted header."""
    print("\n" + "=" * 80)
    print(f"  {title}")
    print("=" * 80)


def print_test(name, passed, message=""):
    """Print test result."""
    status = "✓ PASS" if passed else "✗ FAIL"
    print(f"  {status}: {name}")
    if message:
        print(f"         {message}")


class ValidationSuite:
    """Comprehensive validation test suite."""

    def __init__(self):
        self.results = {}
        self.passed = 0
        self.failed = 0

    def test_system_creation(self):
        """Test 1: System creation with Mars gravity."""
        print_header("TEST 1: System Creation")

        try:
            system = SystemFactory.create_system(gravity_mars=True)
            g = system.GetGravitationalAcceleration()

            passed = abs(g.z - (-3.71)) < 0.01
            print_test("Mars gravity set correctly", passed, f"g_z = {g.z}")

            self.results["system_creation"] = passed
            if passed:
                self.passed += 1
            else:
                self.failed += 1

        except Exception as e:
            print_test("System creation", False, str(e))
            self.failed += 1

    def test_body_builders(self):
        """Test 2: Body builder creation."""
        print_header("TEST 2: Body Builders")

        try:
            system = SystemFactory.create_system()

            # Test chassis
            chassis = ChassisBuilder.create(system)
            chassis_ok = (
                chassis is not None
                and abs(chassis.GetMass() - 50.0) < 0.1
            )
            print_test("Chassis builder", chassis_ok, f"mass = {chassis.GetMass()} kg")

            # Test wheel
            wheel = WheelBuilder.create(system)
            wheel_ok = (
                wheel is not None
                and abs(wheel.GetMass() - 15.0) < 0.1
            )
            print_test("Wheel builder", wheel_ok, f"mass = {wheel.GetMass()} kg")

            self.results["body_builders"] = chassis_ok and wheel_ok
            if chassis_ok and wheel_ok:
                self.passed += 2
            else:
                self.failed += 2

        except Exception as e:
            print_test("Body builders", False, str(e))
            self.failed += 2

    def test_wheel_types(self):
        """Test 3: Advanced wheel types."""
        print_header("TEST 3: Advanced Wheel Types")

        try:
            system = SystemFactory.create_system()

            wheel_types = [
                (WheelType.STANDARD, 15.0),
                (WheelType.LIGHTWEIGHT, 10.0),
                (WheelType.HIGH_GRIP, 18.0),
            ]

            all_ok = True
            for wheel_type, expected_mass in wheel_types:
                wheel = AdvancedWheelBuilder.create_by_type(system, wheel_type)
                mass_ok = abs(wheel.GetMass() - expected_mass) < 0.1
                print_test(
                    f"{wheel_type.value} wheel",
                    mass_ok,
                    f"mass = {wheel.GetMass()} kg (expected {expected_mass})",
                )
                all_ok = all_ok and mass_ok

            self.results["wheel_types"] = all_ok
            self.passed += 3 if all_ok else 0
            self.failed += 0 if all_ok else 3

        except Exception as e:
            print_test("Wheel types", False, str(e))
            self.failed += 3

    def test_wheel_arrays(self):
        """Test 4: Wheel array configurations."""
        print_header("TEST 4: Wheel Array Configurations")

        try:
            system = SystemFactory.create_system()

            configs = [
                ("Single wheel", lambda s: WheelArray.create_single_wheel(s), 1),
                ("2-wheel axle", lambda s: WheelArray.create_two_wheel_axle(s), 2),
                ("4-wheel rover", lambda s: WheelArray.create_four_wheels(s), 4),
                ("6-wheel rover", lambda s: WheelArray.create_six_wheels(s), 6),
            ]

            all_ok = True
            for name, builder, expected_count in configs:
                wheels = builder(system)
                count_ok = len(wheels) == expected_count
                print_test(
                    name,
                    count_ok,
                    f"wheels = {len(wheels)} (expected {expected_count})",
                )
                all_ok = all_ok and count_ok

            self.results["wheel_arrays"] = all_ok
            self.passed += 4 if all_ok else 0
            self.failed += 0 if all_ok else 4

        except Exception as e:
            print_test("Wheel arrays", False, str(e))
            self.failed += 4

    def test_terrain_presets(self):
        """Test 5: Terrain preset loading."""
        print_header("TEST 5: Terrain Preset Loading")

        try:
            presets = [
                (TerrainPreset.MARS_FLAT, "Mars Flat Plain"),
                (TerrainPreset.TEST_TRACK, "Test Track"),
                (TerrainPreset.SANDY_PLAIN, "Sandy Plain"),
            ]

            all_ok = True
            for preset, expected_name in presets:
                system = SystemFactory.create_system()
                terrain_mgr = TerrainManager2(system)
                terrain = terrain_mgr.initialize_from_preset(preset)

                preset_ok = (
                    terrain is not None
                    and terrain_mgr.get_config() is not None
                    and terrain_mgr.get_config().name == expected_name
                )
                print_test(
                    f"{preset.value} preset",
                    preset_ok,
                    f"name = {terrain_mgr.get_config().name if preset_ok else 'ERROR'}",
                )
                all_ok = all_ok and preset_ok

            self.results["terrain_presets"] = all_ok
            self.passed += 3 if all_ok else 0
            self.failed += 0 if all_ok else 3

        except Exception as e:
            print_test("Terrain presets", False, str(e))
            self.failed += 3

    def test_data_logging(self):
        """Test 6: Data logging system."""
        print_header("TEST 6: Data Logging System")

        try:
            logger = DataLogger(
                output_dir="data/logs",
                name="validation_test",
                format=LogFormat.CSV,
            )

            # Log test data
            for i in range(5):
                logger.log({
                    "time": i * 0.1,
                    "velocity": 0.3 - i * 0.01,
                    "slip": i * 5.0,
                }, timestamp=i * 0.1)

            # Check data was collected
            data_ok = len(logger.all_data) == 5
            print_test("Data collection", data_ok, f"entries = {len(logger.all_data)}")

            # Check column extraction
            velocities = logger.get_column("velocity")
            column_ok = len(velocities) == 5 and velocities[0] == 0.3
            print_test("Column extraction", column_ok, f"velocities = {len(velocities)}")

            # Check statistics
            stats = logger.get_statistics("velocity")
            stats_ok = "mean" in stats and "min" in stats and "max" in stats
            print_test("Statistics generation", stats_ok, f"mean = {stats.get('mean', 'N/A'):.3f}")

            all_ok = data_ok and column_ok and stats_ok
            self.results["data_logging"] = all_ok
            self.passed += 3 if all_ok else 0
            self.failed += 0 if all_ok else 3

        except Exception as e:
            print_test("Data logging", False, str(e))
            self.failed += 3

    def test_scenario_0_basic(self):
        """Test 7: Scenario 0 - Static chassis drop."""
        print_header("TEST 7: Scenario 0 - Static Chassis Drop")

        try:
            system = SystemFactory.create_system(gravity_mars=True)
            chassis = ChassisBuilder.create(system, pos_z=0.5)
            terrain_mgr = TerrainManager2(system)
            terrain_mgr.initialize_from_preset(TerrainPreset.TEST_TRACK)

            # Run a few simulation steps
            time_step = 0.01
            for _ in range(50):
                system.DoStepDynamics(time_step)

            # Verify chassis settled
            pos = chassis.GetPos()
            sinkage = terrain_mgr.get_sinkage(chassis)

            settled_ok = (
                0 <= sinkage <= 0.5
            )  # Reasonable sinkage range
            print_test(
                "Chassis settling",
                settled_ok,
                f"sinkage = {sinkage:.4f} m, pos_z = {pos.z:.4f} m",
            )

            self.results["scenario_0"] = settled_ok
            self.passed += 1 if settled_ok else 0
            self.failed += 0 if settled_ok else 1

        except Exception as e:
            print_test("Scenario 0 execution", False, str(e))
            self.failed += 1

    def test_scenario_1_basic(self):
        """Test 8: Scenario 1 - Single wheel slip (short run)."""
        print_header("TEST 8: Scenario 1 - Single Wheel Slip (Short Run)")

        try:
            system = SystemFactory.create_system(gravity_mars=True)
            ground = GroundBuilder.create(system)
            chassis = ChassisBuilder.create(system)
            wheel = WheelBuilder.create(system)

            terrain_mgr = TerrainManager2(system)
            terrain_mgr.initialize_from_preset(TerrainPreset.TEST_TRACK)

            # Create motors
            wheel_pos = wheel.GetPos()
            rotation_motor = RotationMotor.create_constant_speed(
                system, chassis, wheel, wheel_pos, chrono.ChVector3d(0, 1, 0), 2.0
            )

            v_ideal = 2.0 * 0.3
            linear_motor = LinearMotor.create_constant_speed(
                system, ground, chassis, chassis.GetPos(), v_ideal
            )

            # Run short simulation
            metrics = MetricsCollector(output_freq=0.5)
            time_step = 0.01
            sim_time = 0.0

            while sim_time <= 1.0:
                system.DoStepDynamics(time_step)

                if metrics.should_collect(sim_time):
                    frame = metrics.collect_frame(
                        sim_time, chassis, wheel, 0.3, v_ideal,
                        linear_motor, terrain_mgr.get_height
                    )

                sim_time += time_step

            # Verify metrics were collected
            metrics_ok = len(metrics.data) > 0
            print_test("Metrics collection", metrics_ok, f"frames = {len(metrics.data)}")

            # Verify slip is in reasonable range (0-100%)
            slip_values = [f.slip_percent for f in metrics.data]
            slip_ok = all(0 <= s <= 100 for s in slip_values)
            print_test(
                "Slip values valid",
                slip_ok,
                f"slip range = {min(slip_values):.1f}% to {max(slip_values):.1f}%",
            )

            # Verify velocity decreases (from ramp profile)
            velocity_values = [f.velocity_x for f in metrics.data]
            velocity_decreases = all(
                velocity_values[i] >= velocity_values[i + 1]
                for i in range(len(velocity_values) - 1)
            )
            print_test(
                "Velocity ramp working",
                velocity_decreases,
                f"v range = {max(velocity_values):.3f} to {min(velocity_values):.3f} m/s",
            )

            all_ok = metrics_ok and slip_ok and velocity_decreases
            self.results["scenario_1"] = all_ok
            self.passed += 3 if all_ok else 0
            self.failed += 0 if all_ok else 3

        except Exception as e:
            print_test("Scenario 1 execution", False, str(e))
            self.failed += 3

    def run_all(self):
        """Run all validation tests."""
        print("\n" + "=" * 80)
        print("  PYCHRONOROVER VALIDATION SUITE")
        print("=" * 80)

        self.test_system_creation()
        self.test_body_builders()
        self.test_wheel_types()
        self.test_wheel_arrays()
        self.test_terrain_presets()
        self.test_data_logging()
        self.test_scenario_0_basic()
        self.test_scenario_1_basic()

        self.print_summary()

    def print_summary(self):
        """Print validation summary."""
        print_header("VALIDATION SUMMARY")

        total = self.passed + self.failed
        percentage = (self.passed / total * 100) if total > 0 else 0

        print(f"\n  Total Tests: {total}")
        print(f"  Passed:      {self.passed} ✓")
        print(f"  Failed:      {self.failed} ✗")
        print(f"  Success Rate: {percentage:.1f}%")

        if self.failed == 0:
            print("\n  🎉 ALL VALIDATIONS PASSED!")
        else:
            print(f"\n  ⚠️  {self.failed} test(s) failed - see details above")

        print("\n" + "=" * 80)


if __name__ == "__main__":
    suite = ValidationSuite()
    suite.run_all()
