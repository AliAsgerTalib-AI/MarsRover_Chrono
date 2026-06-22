"""Contact material definitions for rover components."""

import pychrono as chrono


class MaterialFactory:
    """Factory for creating contact materials with rover-specific properties."""

    @staticmethod
    def create_chassis_material(friction: float = 0.5) -> chrono.ChContactMaterialNSC:
        """Create material for chassis body.

        Args:
            friction: Coefficient of friction (default: 0.5)

        Returns:
            NSC contact material for chassis
        """
        material = chrono.ChContactMaterialNSC()
        material.SetFriction(friction)
        return material

    @staticmethod
    def create_wheel_material(friction: float = 0.7) -> chrono.ChContactMaterialNSC:
        """Create material for wheel body.

        Args:
            friction: Coefficient of friction (default: 0.7, typical for rover tires)

        Returns:
            NSC contact material for wheel with higher friction
        """
        material = chrono.ChContactMaterialNSC()
        material.SetFriction(friction)
        return material

    @staticmethod
    def create_generic_material(friction: float = 0.5, restitution: float = 0.1) -> chrono.ChContactMaterialNSC:
        """Create generic contact material.

        Args:
            friction: Coefficient of friction
            restitution: Coefficient of restitution (bounciness)

        Returns:
            NSC contact material
        """
        material = chrono.ChContactMaterialNSC()
        material.SetFriction(friction)
        material.SetRestitution(restitution)
        return material
