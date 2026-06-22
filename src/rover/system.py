"""Physics system factory for Mars rover simulations.

Creates and configures Chrono physics systems with Mars-appropriate defaults.
"""

import pychrono as chrono


class SystemFactory:
    """Factory for creating and configuring physics systems."""

    @staticmethod
    def create_system(gravity_mars: bool = True, collision_backend: str = "bullet") -> chrono.ChSystemNSC:
        """Create a physics system with Mars-appropriate defaults.

        Args:
            gravity_mars: If True, use Mars gravity (-3.71 m/s²); else Earth (-9.81 m/s²)
            collision_backend: "bullet" for Bullet physics engine

        Returns:
            Configured ChSystemNSC instance ready for adding bodies
        """
        system = chrono.ChSystemNSC()

        # Set gravity (Mars = -3.71 m/s², Earth = -9.81 m/s²)
        g = -3.71 if gravity_mars else -9.81
        system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, g))

        # Use Bullet collision engine
        if collision_backend == "bullet":
            system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
        else:
            raise ValueError(f"Unknown collision backend: {collision_backend}")

        return system

    @staticmethod
    def configure_solver(
        system: chrono.ChSystemNSC,
        time_step: float = 0.01,
        max_iterations: int = 20,
        tolerance: float = 1e-4,
    ) -> None:
        """Configure solver parameters for stability and accuracy.

        Args:
            system: Physics system to configure
            time_step: Integration time step in seconds
            max_iterations: Max iterations for constraint solver
            tolerance: Convergence tolerance for solver
        """
        system.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_LINEARIZED)
        system.GetSolver().AsIterative().SetMaxIterations(max_iterations)
        system.GetSolver().AsIterative().SetTolerance(tolerance)
