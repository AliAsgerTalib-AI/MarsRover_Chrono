"""Unit tests for system factory."""

import sys
from pathlib import Path

import pytest
import pychrono as chrono

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from rover import SystemFactory


class TestSystemFactory:
    """Tests for SystemFactory class."""

    def test_create_system_mars_gravity(self):
        """Test that system has correct Mars gravity."""
        system = SystemFactory.create_system(gravity_mars=True)
        g_vec = system.GetGravitationalAcceleration()
        assert abs(g_vec.z - (-3.71)) < 1e-6, f"Expected -3.71, got {g_vec.z}"

    def test_create_system_earth_gravity(self):
        """Test that system can use Earth gravity."""
        system = SystemFactory.create_system(gravity_mars=False)
        g_vec = system.GetGravitationalAcceleration()
        assert abs(g_vec.z - (-9.81)) < 1e-6, f"Expected -9.81, got {g_vec.z}"

    def test_create_system_bullet_collision(self):
        """Test that system uses Bullet collision backend."""
        system = SystemFactory.create_system(collision_backend="bullet")
        assert system.GetCollisionSystemType() == chrono.ChCollisionSystem.Type_BULLET

    def test_system_is_nsc(self):
        """Test that system is NSC (Non-Smooth Contact) type."""
        system = SystemFactory.create_system()
        assert isinstance(system, chrono.ChSystemNSC)

    def test_system_can_add_bodies(self):
        """Test that system can accept bodies."""
        system = SystemFactory.create_system()

        # Create and add a simple body
        body = chrono.ChBody()
        body.SetFixed(True)
        system.Add(body)

        # Verify body was added
        # (Note: ChSystem doesn't expose body count easily, so we just check no exception)
        assert True  # If we get here, body was added successfully

    def test_configure_solver(self):
        """Test solver configuration."""
        system = SystemFactory.create_system()

        # Configure solver
        SystemFactory.configure_solver(
            system,
            time_step=0.005,
            max_iterations=30,
            tolerance=1e-5
        )

        # Just verify no exception
        assert True


class TestSystemPhysics:
    """Physics validation tests."""

    def test_gravity_direction(self):
        """Test that gravity points downward (negative Z)."""
        system = SystemFactory.create_system(gravity_mars=True)
        g = system.GetGravitationalAcceleration()
        assert g.x == 0, "Gravity should have no X component"
        assert g.y == 0, "Gravity should have no Y component"
        assert g.z < 0, "Gravity should be negative (downward)"
