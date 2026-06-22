"""PyChronoRover: Modular Mars rover simulation framework."""

# Core modules
from .bodies import ChassisBuilder, GroundBuilder, MultiWheelBuilder, WheelBuilder
from .constraints import CarriageConstraint, PrismaticConstraint, RevoluteJoint
from .materials import MaterialFactory
from .metrics import MetricFrame, MetricsCollector
from .motors import LinearMotor, MotorProfile, RotationMotor
from .system import SystemFactory
from .terrain import SoilParameterSet, SoilType, TerrainManager
from .wheels import (
    AdvancedWheelBuilder,
    WheelArray,
    WheelSpec,
    WheelSpecLibrary,
    WheelType,
)
from .terrain_config import (
    TerrainConfig,
    TerrainConfigBuilder,
    TerrainManager2,
    TerrainPreset,
    TerrainPresetLibrary,
)

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
    # Advanced wheels
    "AdvancedWheelBuilder",
    "WheelArray",
    "WheelSpec",
    "WheelSpecLibrary",
    "WheelType",
    # Terrain
    "TerrainManager",
    "TerrainManager2",
    "TerrainConfig",
    "TerrainConfigBuilder",
    "TerrainPreset",
    "TerrainPresetLibrary",
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
