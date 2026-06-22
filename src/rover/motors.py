"""Motor and actuator definitions for rover control."""

import math
from enum import Enum
from typing import Optional

import pychrono as chrono


class MotorProfile(Enum):
    """Motor speed profile types."""

    CONSTANT = "constant"
    RAMP = "ramp"
    SINE = "sine"


class RotationMotor:
    """Builder for rotational motor between two bodies (e.g., wheel drive)."""

    @staticmethod
    def create_constant_speed(
        system: chrono.ChSystemNSC,
        body1: chrono.ChBody,
        body2: chrono.ChBody,
        joint_pos: chrono.ChVector3d,
        joint_axis: chrono.ChVector3d,
        angular_velocity: float,
    ) -> chrono.ChLinkMotorRotationSpeed:
        """Create a motor with constant angular velocity.

        Args:
            system: Physics system
            body1: First body (e.g., chassis)
            body2: Second body (e.g., wheel)
            joint_pos: Position of motor joint
            joint_axis: Rotation axis (typically along one of X, Y, Z axes)
            angular_velocity: Constant angular speed (rad/s)

        Returns:
            ChLinkMotorRotationSpeed configured and added to system
        """
        motor = chrono.ChLinkMotorRotationSpeed()

        # Create joint frame
        joint_frame = chrono.ChFramed(joint_pos, chrono.QUNIT)
        motor.Initialize(body1, body2, joint_frame)

        # Set constant speed profile
        motor.SetSpeedFunction(chrono.ChFunctionConst(angular_velocity))

        system.Add(motor)
        return motor

    @staticmethod
    def create_profiled_speed(
        system: chrono.ChSystemNSC,
        body1: chrono.ChBody,
        body2: chrono.ChBody,
        joint_pos: chrono.ChVector3d,
        angular_velocity_func: chrono.ChFunction,
    ) -> chrono.ChLinkMotorRotationSpeed:
        """Create a motor with profiled angular velocity (e.g., ramp, sine).

        Args:
            system: Physics system
            body1: First body
            body2: Second body
            joint_pos: Position of motor joint
            angular_velocity_func: ChFunction for speed profile over time

        Returns:
            ChLinkMotorRotationSpeed configured with custom speed function
        """
        motor = chrono.ChLinkMotorRotationSpeed()

        joint_frame = chrono.ChFramed(joint_pos, chrono.QUNIT)
        motor.Initialize(body1, body2, joint_frame)

        motor.SetSpeedFunction(angular_velocity_func)

        system.Add(motor)
        return motor


class LinearMotor:
    """Builder for linear motor between two bodies (e.g., chassis drive)."""

    @staticmethod
    def create_constant_speed(
        system: chrono.ChSystemNSC,
        body1: chrono.ChBody,
        body2: chrono.ChBody,
        joint_pos: chrono.ChVector3d,
        linear_velocity: float,
    ) -> chrono.ChLinkMotorLinearSpeed:
        """Create a linear motor with constant speed.

        Args:
            system: Physics system
            body1: First body (anchor, often ground)
            body2: Second body (driven body)
            joint_pos: Position of motor joint
            linear_velocity: Constant linear speed (m/s)

        Returns:
            ChLinkMotorLinearSpeed configured and added to system
        """
        motor = chrono.ChLinkMotorLinearSpeed()

        joint_frame = chrono.ChFramed(joint_pos, chrono.QUNIT)
        motor.Initialize(body1, body2, joint_frame)

        motor.SetSpeedFunction(chrono.ChFunctionConst(linear_velocity))

        system.Add(motor)
        return motor

    @staticmethod
    def create_sweep_speed(
        system: chrono.ChSystemNSC,
        body1: chrono.ChBody,
        body2: chrono.ChBody,
        joint_pos: chrono.ChVector3d,
        initial_speed: float,
        final_speed: float,
        duration: float,
    ) -> chrono.ChLinkMotorLinearSpeed:
        """Create a linear motor with ramped speed (sweep from initial to final).

        Useful for slip-sinkage studies where you vary wheel speed over time.

        Args:
            system: Physics system
            body1: First body (anchor)
            body2: Second body (driven)
            joint_pos: Position of motor joint
            initial_speed: Starting linear speed (m/s)
            final_speed: Ending linear speed (m/s)
            duration: Time to ramp from initial to final (s)

        Returns:
            ChLinkMotorLinearSpeed with ChFunctionRamp speed profile
        """
        motor = chrono.ChLinkMotorLinearSpeed()

        joint_frame = chrono.ChFramed(joint_pos, chrono.QUNIT)
        motor.Initialize(body1, body2, joint_frame)

        # Create ramp: v(t) = initial_speed + slope * t
        # slope = (final_speed - initial_speed) / duration
        slope = (final_speed - initial_speed) / duration
        ramp_func = chrono.ChFunctionRamp(initial_speed, slope)
        motor.SetSpeedFunction(ramp_func)

        system.Add(motor)
        return motor

    @staticmethod
    def create_sine_speed(
        system: chrono.ChSystemNSC,
        body1: chrono.ChBody,
        body2: chrono.ChBody,
        joint_pos: chrono.ChVector3d,
        amplitude: float,
        frequency: float,
        mean_speed: float = 0.0,
    ) -> chrono.ChLinkMotorLinearSpeed:
        """Create a linear motor with sinusoidal speed profile.

        Useful for analyzing resonance or oscillatory wheel control.

        Args:
            system: Physics system
            body1: First body (anchor)
            body2: Second body (driven)
            joint_pos: Position of motor joint
            amplitude: Amplitude of sine wave (m/s)
            frequency: Frequency of sine wave (Hz)
            mean_speed: DC offset of sine wave (m/s)

        Returns:
            ChLinkMotorLinearSpeed with sine wave speed profile
        """
        motor = chrono.ChLinkMotorLinearSpeed()

        joint_frame = chrono.ChFramed(joint_pos, chrono.QUNIT)
        motor.Initialize(body1, body2, joint_frame)

        # Create sine function: v(t) = mean + amplitude * sin(2*pi*freq*t)
        sine_func = chrono.ChFunctionSine(mean_speed, amplitude, frequency)
        motor.SetSpeedFunction(sine_func)

        system.Add(motor)
        return motor

    @staticmethod
    def get_force(motor: chrono.ChLinkMotorLinearSpeed) -> float:
        """Get the instantaneous force exerted by a linear motor.

        Args:
            motor: Motor to query

        Returns:
            Motor force in Newtons
        """
        return -motor.GetMotorForce()
