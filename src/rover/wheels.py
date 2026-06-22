"""Advanced wheel builders and configurations for different rover designs.

Provides:
- Standard wheels (smooth, flexible, rigid)
- Specialized wheels (low-gravity, high-grip, lightweight)
- Wheel arrays for easy multi-wheel configuration
- Tread pattern support (for future enhancements)
"""

import math
from dataclasses import dataclass
from enum import Enum
from typing import List, Optional

import pychrono as chrono

from .materials import MaterialFactory


class WheelType(Enum):
    """Wheel configuration presets."""

    STANDARD = "standard"  # Default rover wheel
    LIGHTWEIGHT = "lightweight"  # Low mass for extended missions
    HIGH_GRIP = "high_grip"  # Increased friction for rocky terrain
    LOW_GRAVITY = "low_gravity"  # Optimized for low-g environments
    FLEXIBLE = "flexible"  # Spring-loaded for suspension


@dataclass
class WheelSpec:
    """Complete wheel specification."""

    name: str
    radius: float  # meters
    width: float  # meters
    mass: float  # kg
    friction: float  # coefficient
    wheel_type: WheelType = WheelType.STANDARD


class WheelSpecLibrary:
    """Library of predefined wheel specifications."""

    # Mars rover standard wheel (Curiosity/Perseverance class)
    STANDARD = WheelSpec(
        name="Standard Mars Wheel",
        radius=0.3,
        width=0.25,
        mass=15.0,
        friction=0.7,
        wheel_type=WheelType.STANDARD,
    )

    # Lightweight wheel for extended range
    LIGHTWEIGHT = WheelSpec(
        name="Lightweight Wheel",
        radius=0.3,
        width=0.20,
        mass=10.0,
        friction=0.65,
        wheel_type=WheelType.LIGHTWEIGHT,
    )

    # High-grip wheel for rocky/sandy terrain
    HIGH_GRIP = WheelSpec(
        name="High-Grip Wheel",
        radius=0.3,
        width=0.28,
        mass=18.0,
        friction=0.85,
        wheel_type=WheelType.HIGH_GRIP,
    )

    # Low-gravity wheel (Moon/Asteroid optimized)
    LOW_GRAVITY = WheelSpec(
        name="Low-Gravity Wheel",
        radius=0.25,  # Smaller for reduced sinkage
        width=0.30,  # Wider for load distribution
        mass=8.0,  # Very light
        friction=0.6,
        wheel_type=WheelType.LOW_GRAVITY,
    )

    # Flexible/spring-loaded wheel
    FLEXIBLE = WheelSpec(
        name="Flexible Wheel",
        radius=0.32,  # Slightly larger
        width=0.22,
        mass=12.0,
        friction=0.75,
        wheel_type=WheelType.FLEXIBLE,
    )

    @staticmethod
    def get_spec(wheel_type: WheelType) -> WheelSpec:
        """Get wheel specification by type.

        Args:
            wheel_type: Type of wheel to retrieve

        Returns:
            WheelSpec instance
        """
        specs = {
            WheelType.STANDARD: WheelSpecLibrary.STANDARD,
            WheelType.LIGHTWEIGHT: WheelSpecLibrary.LIGHTWEIGHT,
            WheelType.HIGH_GRIP: WheelSpecLibrary.HIGH_GRIP,
            WheelType.LOW_GRAVITY: WheelSpecLibrary.LOW_GRAVITY,
            WheelType.FLEXIBLE: WheelSpecLibrary.FLEXIBLE,
        }
        return specs[wheel_type]


class AdvancedWheelBuilder:
    """Advanced wheel builder with preset specifications."""

    DEFAULT_DENSITY = 1000  # kg/m³

    @staticmethod
    def create_from_spec(
        system: chrono.ChSystemNSC,
        spec: WheelSpec,
        pos_x: float = 0.0,
        pos_y: float = 0.5,
        pos_z: float = 0.3,
    ) -> chrono.ChBody:
        """Create wheel from specification.

        Args:
            system: Physics system
            spec: Wheel specification
            pos_x: X position (m)
            pos_y: Y position (m)
            pos_z: Z position (m)

        Returns:
            Configured wheel ChBody
        """
        material = MaterialFactory.create_wheel_material(friction=spec.friction)

        wheel = chrono.ChBodyEasyCylinder(
            chrono.ChAxis_Z,
            spec.radius,
            spec.width,
            AdvancedWheelBuilder.DEFAULT_DENSITY,
            True,
            True,
            material,
        )

        wheel.SetPos(chrono.ChVector3d(pos_x, pos_y, pos_z))
        wheel.SetMass(spec.mass)
        wheel.SetName(spec.name)
        wheel.EnableCollision(True)

        # Rotate 90° around X-axis for Y-rolling
        wheel_rot = chrono.QuatFromAngleX(math.pi / 2)
        wheel.SetRot(wheel_rot)

        system.Add(wheel)
        return wheel

    @staticmethod
    def create_by_type(
        system: chrono.ChSystemNSC,
        wheel_type: WheelType,
        pos_x: float = 0.0,
        pos_y: float = 0.5,
        pos_z: float = 0.3,
    ) -> chrono.ChBody:
        """Create wheel by type preset.

        Args:
            system: Physics system
            wheel_type: Wheel type (standard, lightweight, high-grip, etc.)
            pos_x: X position (m)
            pos_y: Y position (m)
            pos_z: Z position (m)

        Returns:
            Configured wheel ChBody
        """
        spec = WheelSpecLibrary.get_spec(wheel_type)
        return AdvancedWheelBuilder.create_from_spec(system, spec, pos_x, pos_y, pos_z)

    @staticmethod
    def create_custom(
        system: chrono.ChSystemNSC,
        name: str,
        radius: float,
        width: float,
        mass: float,
        friction: float,
        pos_x: float = 0.0,
        pos_y: float = 0.5,
        pos_z: float = 0.3,
    ) -> chrono.ChBody:
        """Create custom wheel with arbitrary parameters.

        Args:
            system: Physics system
            name: Wheel name/identifier
            radius: Wheel radius (m)
            width: Wheel width (m)
            mass: Wheel mass (kg)
            friction: Friction coefficient
            pos_x: X position (m)
            pos_y: Y position (m)
            pos_z: Z position (m)

        Returns:
            Configured wheel ChBody
        """
        spec = WheelSpec(
            name=name,
            radius=radius,
            width=width,
            mass=mass,
            friction=friction,
        )
        return AdvancedWheelBuilder.create_from_spec(system, spec, pos_x, pos_y, pos_z)


class WheelArray:
    """Builder for wheel arrays (multiple wheels in coordinated positions)."""

    @staticmethod
    def create_single_wheel(
        system: chrono.ChSystemNSC,
        wheel_type: WheelType = WheelType.STANDARD,
        position_y: float = 0.5,
    ) -> List[chrono.ChBody]:
        """Create single-wheel configuration.

        Args:
            system: Physics system
            wheel_type: Type of wheel
            position_y: Y-offset for wheel

        Returns:
            List with 1 wheel
        """
        wheel = AdvancedWheelBuilder.create_by_type(
            system, wheel_type, pos_x=0.0, pos_y=position_y, pos_z=0.3
        )
        return [wheel]

    @staticmethod
    def create_two_wheel_axle(
        system: chrono.ChSystemNSC,
        wheel_type: WheelType = WheelType.STANDARD,
        track_width: float = 0.6,
        x_offset: float = 0.0,
    ) -> List[chrono.ChBody]:
        """Create two-wheel axle (left and right).

        Args:
            system: Physics system
            wheel_type: Type of wheel
            track_width: Distance between wheels (m)
            x_offset: X-position of axle (m)

        Returns:
            List with 2 wheels [left, right]
        """
        wheels = []
        for y_offset in [-track_width / 2, track_width / 2]:
            wheel = AdvancedWheelBuilder.create_by_type(
                system, wheel_type, pos_x=x_offset, pos_y=y_offset, pos_z=0.3
            )
            wheels.append(wheel)
        return wheels

    @staticmethod
    def create_four_wheels(
        system: chrono.ChSystemNSC,
        wheel_type: WheelType = WheelType.STANDARD,
        wheelbase: float = 0.8,
        track_width: float = 0.6,
    ) -> List[chrono.ChBody]:
        """Create 4-wheel configuration (2x2).

        Args:
            system: Physics system
            wheel_type: Type of wheel
            wheelbase: Distance between front and rear axles (m)
            track_width: Distance between left and right wheels (m)

        Returns:
            List with 4 wheels [FR, FL, RR, RL]
        """
        wheels = []
        # Front and rear positions
        x_positions = [wheelbase / 2, -wheelbase / 2]
        # Left and right positions
        y_positions = [track_width / 2, -track_width / 2]

        for x_offset in x_positions:
            for y_offset in y_positions:
                wheel = AdvancedWheelBuilder.create_by_type(
                    system, wheel_type, pos_x=x_offset, pos_y=y_offset, pos_z=0.3
                )
                wheels.append(wheel)

        return wheels

    @staticmethod
    def create_six_wheels(
        system: chrono.ChSystemNSC,
        wheel_type: WheelType = WheelType.STANDARD,
        wheelbase: float = 1.2,  # Longer wheelbase for 6 wheels
        track_width: float = 0.6,
        axle_spacing: float = 0.4,
    ) -> List[chrono.ChBody]:
        """Create 6-wheel configuration (3 axles, 2 wheels each).

        Args:
            system: Physics system
            wheel_type: Type of wheel
            wheelbase: Total wheelbase (m)
            track_width: Distance between left and right wheels (m)
            axle_spacing: Spacing between axles (m)

        Returns:
            List with 6 wheels [F-L, F-R, M-L, M-R, R-L, R-R]
        """
        wheels = []
        # Front, middle, rear
        x_positions = [
            wheelbase / 2,
            wheelbase / 2 - axle_spacing,
            wheelbase / 2 - 2 * axle_spacing,
        ]
        # Left and right
        y_positions = [track_width / 2, -track_width / 2]

        for x_offset in x_positions:
            for y_offset in y_positions:
                wheel = AdvancedWheelBuilder.create_by_type(
                    system, wheel_type, pos_x=x_offset, pos_y=y_offset, pos_z=0.3
                )
                wheels.append(wheel)

        return wheels
