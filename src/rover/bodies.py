"""Rigid body builders for rover components."""

import math
from dataclasses import dataclass
from typing import Optional

import pychrono as chrono

from .materials import MaterialFactory


@dataclass
class BodyConfig:
    """Configuration for rigid body creation."""

    mass: float
    pos_x: float = 0.0
    pos_y: float = 0.0
    pos_z: float = 0.0
    enable_collision: bool = True
    visualize: bool = True


class GroundBuilder:
    """Builder for fixed ground reference body."""

    @staticmethod
    def create(system: chrono.ChSystemNSC) -> chrono.ChBody:
        """Create a fixed ground body for constraint anchoring.

        Args:
            system: Physics system to add ground to

        Returns:
            Fixed ChBody representing ground
        """
        ground = chrono.ChBody()
        ground.SetFixed(True)
        ground.SetName("Ground")
        system.Add(ground)
        return ground


class ChassisBuilder:
    """Builder for rover chassis body."""

    # Default dimensions and mass for single-wheel rover
    DEFAULT_LENGTH = 1.4  # m (X-axis)
    DEFAULT_WIDTH = 0.8  # m (Y-axis)
    DEFAULT_HEIGHT = 0.4  # m (Z-axis)
    DEFAULT_MASS = 50.0  # kg
    DEFAULT_DENSITY = 1000  # kg/m³

    @staticmethod
    def create(
        system: chrono.ChSystemNSC,
        length: float = DEFAULT_LENGTH,
        width: float = DEFAULT_WIDTH,
        height: float = DEFAULT_HEIGHT,
        mass: float = DEFAULT_MASS,
        pos_z: float = 0.6,
        friction: float = 0.5,
    ) -> chrono.ChBody:
        """Create chassis body.

        Args:
            system: Physics system to add chassis to
            length: Chassis length in X direction (m)
            width: Chassis width in Y direction (m)
            height: Chassis height in Z direction (m)
            mass: Chassis mass (kg)
            pos_z: Initial height above ground (m)
            friction: Friction coefficient

        Returns:
            Configured chassis ChBody
        """
        material = MaterialFactory.create_chassis_material(friction=friction)

        chassis = chrono.ChBodyEasyBox(
            length, width, height, ChassisBuilder.DEFAULT_DENSITY, True, True, material
        )
        chassis.SetPos(chrono.ChVector3d(0, 0, pos_z))
        chassis.SetMass(mass)
        chassis.SetName("Chassis")
        chassis.EnableCollision(True)

        system.Add(chassis)
        return chassis


class WheelBuilder:
    """Builder for rover wheel body."""

    # Default dimensions and mass for single-wheel rover
    DEFAULT_RADIUS = 0.3  # m
    DEFAULT_WIDTH = 0.25  # m
    DEFAULT_MASS = 15.0  # kg
    DEFAULT_DENSITY = 1000  # kg/m³

    @staticmethod
    def create(
        system: chrono.ChSystemNSC,
        radius: float = DEFAULT_RADIUS,
        width: float = DEFAULT_WIDTH,
        mass: float = DEFAULT_MASS,
        pos_x: float = 0.0,
        pos_y: float = 0.5,
        pos_z: float = 0.3,
        friction: float = 0.7,
    ) -> chrono.ChBody:
        """Create wheel body with rotation aligned for Y-axis rolling.

        Args:
            system: Physics system to add wheel to
            radius: Wheel radius (m)
            width: Wheel width (m)
            mass: Wheel mass (kg)
            pos_x: X position (m)
            pos_y: Y position (m)
            pos_z: Z position (m)
            friction: Friction coefficient

        Returns:
            Configured wheel ChBody with proper rotation
        """
        material = MaterialFactory.create_wheel_material(friction=friction)

        # Create cylinder: ChAxis_Z is the cylinder axis in local frame
        wheel = chrono.ChBodyEasyCylinder(
            chrono.ChAxis_Z, radius, width, WheelBuilder.DEFAULT_DENSITY, True, True, material
        )

        wheel.SetPos(chrono.ChVector3d(pos_x, pos_y, pos_z))

        # Rotate 90° around X-axis so local Z (cylinder axis) points along global Y
        # This allows the wheel to roll in the Y direction
        wheel_rot = chrono.QuatFromAngleX(math.pi / 2)
        wheel.SetRot(wheel_rot)

        wheel.SetMass(mass)
        wheel.SetName("Wheel")
        wheel.EnableCollision(True)

        system.Add(wheel)
        return wheel


class MultiWheelBuilder:
    """Builder for multi-wheel rover configurations."""

    @staticmethod
    def create_four_wheel_rover(
        system: chrono.ChSystemNSC,
        chassis_mass: float = 50.0,
        wheel_mass: float = 15.0,
        wheel_radius: float = 0.3,
        wheelbase: float = 0.8,  # Distance between front and rear axles
        track_width: float = 0.6,  # Distance between left and right wheels
        friction: float = 0.5,
    ) -> dict:
        """Create a 4-wheel rover (2x2 configuration).

        Args:
            system: Physics system
            chassis_mass: Chassis mass (kg)
            wheel_mass: Mass of each wheel (kg)
            wheel_radius: Wheel radius (m)
            wheelbase: Distance between front and rear axles (m)
            track_width: Distance between left and right wheels (m)
            friction: Chassis friction coefficient

        Returns:
            Dictionary with 'chassis' and 'wheels' (list of 4 ChBody objects)
        """
        # Create chassis
        chassis = ChassisBuilder.create(system, mass=chassis_mass, friction=friction)

        # Create 4 wheels: front-left, front-right, rear-left, rear-right
        wheels = []
        wheel_positions = [
            (wheelbase / 2, track_width / 2),  # front-right
            (wheelbase / 2, -track_width / 2),  # front-left
            (-wheelbase / 2, track_width / 2),  # rear-right
            (-wheelbase / 2, -track_width / 2),  # rear-left
        ]

        for x_offset, y_offset in wheel_positions:
            wheel = WheelBuilder.create(
                system,
                radius=wheel_radius,
                pos_x=x_offset,
                pos_y=y_offset,
                pos_z=wheel_radius + 0.1,  # Slightly above ground
                mass=wheel_mass,
            )
            wheels.append(wheel)

        return {"chassis": chassis, "wheels": wheels}
