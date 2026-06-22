"""Kinematic constraints and joint builders for rover mechanisms."""

import math

import pychrono as chrono


class PrismaticConstraint:
    """Builder for prismatic (sliding) joint between two bodies."""

    @staticmethod
    def create(
        system: chrono.ChSystemNSC,
        body1: chrono.ChBody,
        body2: chrono.ChBody,
        joint_pos: chrono.ChVector3d,
        slide_axis: int = 2,  # 0=X, 1=Y, 2=Z (vertical for most rovers)
    ) -> chrono.ChLinkLockPrismatic:
        """Create a prismatic constraint allowing sliding along one axis.

        Typical use: Constrain chassis to move vertically on a carriage
        (allows vertical settlement while preventing lateral motion).

        Args:
            system: Physics system
            body1: First body (anchor, e.g., carriage)
            body2: Second body (sliding, e.g., chassis)
            joint_pos: Position of joint
            slide_axis: Axis along which sliding is allowed
                - 0: X-axis
                - 1: Y-axis
                - 2: Z-axis (default, vertical)

        Returns:
            ChLinkLockPrismatic constraint
        """
        # Create joint frame aligned with the system
        joint_frame = chrono.ChFramed(joint_pos, chrono.QUNIT)

        # Create prismatic constraint
        constraint = chrono.ChLinkLockPrismatic()
        constraint.Initialize(body1, body2, joint_frame)

        system.Add(constraint)
        return constraint


class RevoluteJoint:
    """Builder for revolute (hinge) joint between two bodies."""

    @staticmethod
    def create(
        system: chrono.ChSystemNSC,
        body1: chrono.ChBody,
        body2: chrono.ChBody,
        joint_pos: chrono.ChVector3d,
        joint_axis: chrono.ChVector3d = chrono.ChVector3d(0, 1, 0),  # Y-axis default
    ) -> chrono.ChLinkLockRevolute:
        """Create a revolute (hinge) joint.

        Args:
            system: Physics system
            body1: First body
            body2: Second body
            joint_pos: Position of joint
            joint_axis: Rotation axis (unit vector)

        Returns:
            ChLinkLockRevolute joint
        """
        joint_frame = chrono.ChFramed(joint_pos, chrono.QUNIT)

        joint = chrono.ChLinkLockRevolute()
        joint.Initialize(body1, body2, joint_frame)

        system.Add(joint)
        return joint


class CarriageConstraint:
    """Constraint system for kinematically controlled carriage (used in scenario_2)."""

    @staticmethod
    def create_vertical_lock(
        system: chrono.ChSystemNSC,
        carriage: chrono.ChBody,
        chassis: chrono.ChBody,
        joint_pos: chrono.ChVector3d,
    ) -> chrono.ChLinkLockPrismatic:
        """Create vertical prismatic constraint between carriage and chassis.

        Allows vertical settlement but prevents lateral motion.

        Args:
            system: Physics system
            carriage: Carriage body (moved by linear motor)
            chassis: Chassis body (settles vertically)
            joint_pos: Position of constraint

        Returns:
            Configured prismatic constraint
        """
        return PrismaticConstraint.create(
            system,
            body1=carriage,
            body2=chassis,
            joint_pos=joint_pos,
            slide_axis=2,  # Z-axis (vertical)
        )


class JointBuilder:
    """Utility builder for common joint configurations."""

    @staticmethod
    def create_revolute_xyz(
        system: chrono.ChSystemNSC,
        body1: chrono.ChBody,
        body2: chrono.ChBody,
        joint_pos: chrono.ChVector3d,
        axis_name: str = "y",
    ) -> chrono.ChLinkLockRevolute:
        """Create revolute joint with standard axis names.

        Args:
            system: Physics system
            body1: First body
            body2: Second body
            joint_pos: Joint position
            axis_name: "x", "y", or "z" for axis selection

        Returns:
            Configured revolute joint
        """
        axis_map = {
            "x": chrono.ChVector3d(1, 0, 0),
            "y": chrono.ChVector3d(0, 1, 0),
            "z": chrono.ChVector3d(0, 0, 1),
        }

        if axis_name.lower() not in axis_map:
            raise ValueError(f"axis_name must be 'x', 'y', or 'z', got {axis_name}")

        return RevoluteJoint.create(
            system,
            body1=body1,
            body2=body2,
            joint_pos=joint_pos,
            joint_axis=axis_map[axis_name.lower()],
        )
