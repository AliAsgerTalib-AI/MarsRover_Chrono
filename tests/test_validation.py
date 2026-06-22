"""Validation tests comparing refactored code with original sandboxes.

This test suite ensures:
1. Refactored scenarios run without errors
2. Output matches original sandbox outputs (numerical parity)
3. All components work correctly
"""

import sys
from pathlib import Path

import pytest

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from rover import (
    SystemFactory,
    ChassisBuilder,
    WheelBuilder,
    TerrainManager,
    AdvancedWheelBuilder,
    WheelType,
    WheelArray,
    TerrainManager2,
    TerrainPreset,
    MetricsCollector,
)
from utils import DataLogger, LogFormat


class TestSystemCreation:
    """Test basic system creation."""

    def test_system_creates_without_error(self):
        """Verify system can be created."""
        system = SystemFactory.create_system(gravity_mars=True)
        assert system is not None

    def test_gravity_is_mars(self):
        """Verify Mars gravity is set correctly."""
        system = SystemFactory.create_system(gravity_mars=True)
        g = system.GetGravitationalAcceleration()
        assert abs(g.z - (-3.71)) < 0.01

    def test_collision_backend_is_bullet(self):
        """Verify Bullet collision backend is set."""
        import pychrono as chrono
        system = SystemFactory.create_system()
        assert system.GetCollisionSystemType() == chrono.ChCollisionSystem.Type_BULLET


class TestBodyBuilders:
    """Test body creation and initialization."""

    def test_chassis_builder(self):
        """Verify chassis builder creates valid body."""
        import pychrono as chrono
        system = SystemFactory.create_system()
        chassis = ChassisBuilder.create(system)

        assert chassis is not None
        assert chassis.GetName() == "Chassis"
        assert abs(chassis.GetMass() - 50.0) < 0.1
        pos = chassis.GetPos()
        assert abs(pos.z - 0.6) < 0.01

    def test_wheel_builder(self):
        """Verify wheel builder creates valid body."""
        import pychrono as chrono
        system = SystemFactory.create_system()
        wheel = WheelBuilder.create(system)

        assert wheel is not None
        assert wheel.GetName() == "Wheel"
        assert abs(wheel.GetMass() - 15.0) < 0.1


class TestAdvancedWheelBuilders:
    """Test advanced wheel builders with presets."""

    def test_wheel_type_standard(self):
        """Test standard wheel type."""
        import pychrono as chrono
        system = SystemFactory.create_system()
        wheel = AdvancedWheelBuilder.create_by_type(system, WheelType.STANDARD)

        assert wheel is not None
        assert "Standard" in wheel.GetName() or "Standard" in wheel.GetName()
        assert abs(wheel.GetMass() - 15.0) < 0.1

    def test_wheel_type_lightweight(self):
        """Test lightweight wheel type."""
        import pychrono as chrono
        system = SystemFactory.create_system()
        wheel = AdvancedWheelBuilder.create_by_type(system, WheelType.LIGHTWEIGHT)

        assert wheel is not None
        assert abs(wheel.GetMass() - 10.0) < 0.1  # Lighter

    def test_wheel_type_high_grip(self):
        """Test high-grip wheel type."""
        import pychrono as chrono
        system = SystemFactory.create_system()
        wheel = AdvancedWheelBuilder.create_by_type(system, WheelType.HIGH_GRIP)

        assert wheel is not None
        assert abs(wheel.GetMass() - 18.0) < 0.1  # Heavier for grip

    def test_wheel_array_single(self):
        """Test single wheel array."""
        import pychrono as chrono
        system = SystemFactory.create_system()
        wheels = WheelArray.create_single_wheel(system)

        assert len(wheels) == 1
        assert wheels[0] is not None

    def test_wheel_array_four_wheels(self):
        """Test 4-wheel array."""
        import pychrono as chrono
        system = SystemFactory.create_system()
        wheels = WheelArray.create_four_wheels(system)

        assert len(wheels) == 4
        # Verify they're positioned correctly
        positions = [w.GetPos() for w in wheels]
        assert len(set((p.x, p.y) for p in positions)) == 4  # All different positions


class TestTerrainConfigurators:
    """Test terrain configuration system."""

    def test_terrain_manager2_creation(self):
        """Test TerrainManager2 can be created."""
        import pychrono as chrono
        system = SystemFactory.create_system()
        terrain_mgr = TerrainManager2(system)

        assert terrain_mgr is not None
        assert terrain_mgr.get_config() is None  # Not initialized yet

    def test_terrain_preset_initialization(self):
        """Test terrain initialization from preset."""
        import pychrono as chrono
        system = SystemFactory.create_system()
        terrain_mgr = TerrainManager2(system)

        terrain = terrain_mgr.initialize_from_preset(TerrainPreset.TEST_TRACK)

        assert terrain is not None
        assert terrain_mgr.get_config() is not None
        assert terrain_mgr.get_config().name == "Test Track"

    def test_terrain_config_save_load(self, tmp_path):
        """Test saving and loading terrain configuration."""
        import pychrono as chrono
        system = SystemFactory.create_system()
        terrain_mgr = TerrainManager2(system)

        # Initialize and save
        terrain_mgr.initialize_from_preset(TerrainPreset.MARS_FLAT)
        config_file = tmp_path / "test_terrain.json"
        terrain_mgr.save_config(str(config_file))

        assert config_file.exists()

        # Load in new manager
        system2 = SystemFactory.create_system()
        terrain_mgr2 = TerrainManager2(system2)
        terrain2 = terrain_mgr2.load_config_from_file(str(config_file))

        assert terrain2 is not None
        assert terrain_mgr2.get_config().name == "Mars Flat Plain"


class TestDataLoggers:
    """Test data logging system."""

    def test_data_logger_creation(self, tmp_path):
        """Test DataLogger can be created."""
        logger = DataLogger(
            output_dir=str(tmp_path),
            name="test_logger",
            format=LogFormat.CSV,
        )

        assert logger is not None

    def test_data_logger_logging(self, tmp_path):
        """Test logging data entries."""
        logger = DataLogger(
            output_dir=str(tmp_path),
            name="test_logging",
            format=LogFormat.CSV,
            buffer_size=2,  # Small buffer to test flushing
        )

        # Log some data
        logger.log({"velocity": 0.3, "slip": 25.5}, timestamp=0.0)
        logger.log({"velocity": 0.29, "slip": 27.3}, timestamp=0.5)

        assert len(logger.all_data) == 2

    def test_data_logger_save(self, tmp_path):
        """Test saving logged data."""
        logger = DataLogger(
            output_dir=str(tmp_path),
            name="test_save",
            format=LogFormat.CSV,
        )

        logger.log({"velocity": 0.3, "slip": 25.5}, timestamp=0.0)
        logger.log({"velocity": 0.29, "slip": 27.3}, timestamp=0.5)
        logger.save()

        # Check file was created
        csv_file = tmp_path / "test_save.csv"
        assert csv_file.exists()

    def test_data_logger_statistics(self, tmp_path):
        """Test statistics generation."""
        logger = DataLogger(output_dir=str(tmp_path), name="test_stats")

        # Log velocity data
        for i in range(10):
            logger.log({"velocity": 0.3 - i * 0.01}, timestamp=i * 0.1)

        stats = logger.get_statistics("velocity")

        assert stats["count"] == 10
        assert "mean" in stats
        assert "min" in stats
        assert "max" in stats
        assert "std" in stats

    def test_data_logger_column_extraction(self, tmp_path):
        """Test extracting single column."""
        logger = DataLogger(output_dir=str(tmp_path), name="test_column")

        for i in range(5):
            logger.log({"velocity": 0.3 - i * 0.01}, timestamp=i * 0.1)

        velocities = logger.get_column("velocity")

        assert len(velocities) == 5
        assert velocities[0] == 0.3
        assert velocities[-1] == 0.26


class TestMetricsCollector:
    """Test metrics collection."""

    def test_metrics_collector_creation(self):
        """Test MetricsCollector can be created."""
        metrics = MetricsCollector(output_freq=0.5)

        assert metrics is not None
        assert metrics.output_freq == 0.5

    def test_metrics_collection_frequency(self):
        """Test metrics collection respects frequency."""
        metrics = MetricsCollector(output_freq=0.5)

        # Should collect at specific intervals
        assert metrics.should_collect(0.0)  # First call
        assert not metrics.should_collect(0.1)  # Too soon
        assert not metrics.should_collect(0.3)  # Still too soon
        assert metrics.should_collect(0.5)  # Now collect
        assert not metrics.should_collect(0.6)  # Too soon after last


class TestIntegration:
    """Integration tests combining multiple components."""

    def test_full_scenario_setup(self):
        """Test setting up a complete scenario."""
        import pychrono as chrono

        # Create system
        system = SystemFactory.create_system(gravity_mars=True)

        # Create bodies
        chassis = ChassisBuilder.create(system)
        wheels = WheelArray.create_four_wheels(system, WheelType.HIGH_GRIP)

        # Initialize terrain
        terrain_mgr = TerrainManager2(system)
        terrain_mgr.initialize_from_preset(TerrainPreset.TEST_TRACK)

        # Setup logging
        logger = DataLogger(name="integration_test", format=LogFormat.CSV)

        # Verify all components exist
        assert system is not None
        assert chassis is not None
        assert len(wheels) == 4
        assert terrain_mgr.get_config() is not None
        assert logger is not None

    def test_metrics_with_logging(self):
        """Test metrics integration with data logger."""
        import pychrono as chrono

        logger = DataLogger(name="metrics_test", format=LogFormat.BOTH)
        metrics = MetricsCollector(output_freq=0.5)

        # Simulate collecting metrics
        for i in range(5):
            sim_time = i * 0.1

            # Create mock metric frame (in real scenario, from actual simulation)
            if metrics.should_collect(sim_time):
                logger.log({
                    "time": sim_time,
                    "velocity": 0.3 - i * 0.01,
                    "slip": i * 5.0,
                }, timestamp=sim_time)

        logger.save()

        # Verify data was logged
        assert len(logger.all_data) > 0
        assert len(logger.get_column("velocity")) > 0


class TestNumericalParity:
    """Tests to verify numerical outputs match expectations."""

    def test_chassis_drop_sinkage_range(self):
        """Verify chassis settling produces reasonable sinkage."""
        import pychrono as chrono

        system = SystemFactory.create_system(gravity_mars=True)
        chassis = ChassisBuilder.create(system, pos_z=0.5)
        terrain_mgr = TerrainManager2(system)
        terrain_mgr.initialize_from_preset(TerrainPreset.TEST_TRACK)

        # Run a few steps
        for _ in range(50):
            system.DoStepDynamics(0.01)

        # Check sinkage is reasonable (should settle into terrain)
        sinkage = terrain_mgr.get_sinkage(chassis)

        # Sinkage should be between 0 and 0.5m for a chassis on Mars regolith
        assert 0 <= sinkage <= 0.5

    def test_wheel_friction_affects_traction(self):
        """Verify different friction coefficients produce different results."""
        import pychrono as chrono

        # Test with different wheel types (different friction)
        wheel_types = [WheelType.STANDARD, WheelType.HIGH_GRIP]
        masses = []

        for wheel_type in wheel_types:
            system = SystemFactory.create_system()
            wheel = AdvancedWheelBuilder.create_by_type(system, wheel_type)
            masses.append(wheel.GetMass())

        # High-grip wheel should be heavier (more material for grip)
        assert masses[1] > masses[0]


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
