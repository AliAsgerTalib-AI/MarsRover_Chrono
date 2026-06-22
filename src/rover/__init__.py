"""PyChronoRover: Modular Mars rover simulation framework."""

# Core modules
from .bodies import ChassisBuilder, GroundBuilder, MultiWheelBuilder, WheelBuilder
from .constraints import CarriageConstraint, PrismaticConstraint, RevoluteJoint
from .materials import MaterialFactory
from .metrics import MetricFrame, MetricsCollector
from .motors import LinearMotor, MotorProfile, RotationMotor
from .system import SystemFactory
from .terrain import SoilParameterSet, SoilType, TerrainManager

__version__ = "0.1.0"
__author__ = "ChronoRover Contributors"

__all__ = [
    # System
    "SystemFactory",
    # Materials
    "MaterialFactory",
    # Bodies
    "GroundBuilder",
    "ChassisBuilder",
    "WheelBuilder",
    "MultiWheelBuilder",
    # Terrain
    "TerrainManager",
    "SoilParameterSet",
    "SoilType",
    # Motors
    "RotationMotor",
    "LinearMotor",
    "MotorProfile",
    # Constraints
    "PrismaticConstraint",
    "RevoluteJoint",
    "CarriageConstraint",
    # Metrics
    "MetricsCollector",
    "MetricFrame",
]
