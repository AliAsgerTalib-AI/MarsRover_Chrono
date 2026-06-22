"""Terrain configuration system for Mars rover simulations.

Provides:
- Terrain preset configurations
- Parameter validation and documentation
- Configuration saving/loading
- Terrain builder with fluent API
"""

from dataclasses import dataclass, asdict
from enum import Enum
from typing import Dict, Optional
import json

import pychrono as chrono
import pychrono.vehicle as veh

from .terrain import SoilParameterSet, SoilType, TerrainManager


class TerrainPreset(Enum):
    """Predefined terrain configurations."""

    MARS_FLAT = "mars_flat"  # Large flat regolith plain
    MARS_VALLEY = "mars_valley"  # Narrow valley with high walls
    SANDY_PLAIN = "sandy_plain"  # Loose sandy terrain
    ROCKY_TERRAIN = "rocky_terrain"  # Mixed rock and regolith
    TEST_TRACK = "test_track"  # Small test area for validation


@dataclass
class TerrainConfig:
    """Terrain configuration parameters."""

    name: str
    width: float  # Y direction (m)
    length: float  # X direction (m)
    grid_resolution: float  # Node spacing (m)
    soil_type: SoilType
    description: str = ""

    def to_dict(self) -> dict:
        """Convert to dictionary."""
        return {
            "name": self.name,
            "width": self.width,
            "length": self.length,
            "grid_resolution": self.grid_resolution,
            "soil_type": self.soil_type.value,
            "description": self.description,
        }

    def to_json(self) -> str:
        """Convert to JSON string."""
        return json.dumps(self.to_dict(), indent=2)

    @staticmethod
    def from_dict(data: dict) -> "TerrainConfig":
        """Create from dictionary."""
        return TerrainConfig(
            name=data.get("name", "Custom"),
            width=data.get("width", 15.0),
            length=data.get("length", 5.0),
            grid_resolution=data.get("grid_resolution", 0.05),
            soil_type=SoilType(data.get("soil_type", "mars_regolith")),
            description=data.get("description", ""),
        )

    @staticmethod
    def from_json(json_str: str) -> "TerrainConfig":
        """Create from JSON string."""
        return TerrainConfig.from_dict(json.loads(json_str))


class TerrainPresetLibrary:
    """Library of predefined terrain configurations."""

    # Large flat Mars regolith plain (default test environment)
    MARS_FLAT = TerrainConfig(
        name="Mars Flat Plain",
        width=15.0,  # Y
        length=20.0,  # X (longer for slip sweep tests)
        grid_resolution=0.05,
        soil_type=SoilType.MARS_REGOLITH,
        description="Large flat regolith surface, typical for rover testing",
    )

    # Narrow valley (terrain interaction test)
    MARS_VALLEY = TerrainConfig(
        name="Mars Valley",
        width=5.0,  # Narrow
        length=15.0,  # Long
        grid_resolution=0.025,  # Fine resolution for accuracy
        soil_type=SoilType.MARS_REGOLITH,
        description="Narrow valley environment, tests lateral stability",
    )

    # Sandy terrain (higher sinkage expected)
    SANDY_PLAIN = TerrainConfig(
        name="Sandy Plain",
        width=15.0,
        length=10.0,
        grid_resolution=0.03,
        soil_type=SoilType.SANDY_SOIL,
        description="Loose sandy terrain, expect high sinkage",
    )

    # Rocky mixed terrain
    ROCKY_TERRAIN = TerrainConfig(
        name="Rocky Terrain",
        width=10.0,
        length=10.0,
        grid_resolution=0.02,  # Fine for rocky features
        soil_type=SoilType.LOOSE_DUST,
        description="Mixed rock and dust, challenging traction",
    )

    # Small test track for quick validation
    TEST_TRACK = TerrainConfig(
        name="Test Track",
        width=5.0,
        length=5.0,
        grid_resolution=0.05,
        soil_type=SoilType.MARS_REGOLITH,
        description="Small test area for quick validation runs",
    )

    @staticmethod
    def get_preset(preset: TerrainPreset) -> TerrainConfig:
        """Get terrain configuration by preset.

        Args:
            preset: Terrain preset type

        Returns:
            TerrainConfig instance
        """
        presets = {
            TerrainPreset.MARS_FLAT: TerrainPresetLibrary.MARS_FLAT,
            TerrainPreset.MARS_VALLEY: TerrainPresetLibrary.MARS_VALLEY,
            TerrainPreset.SANDY_PLAIN: TerrainPresetLibrary.SANDY_PLAIN,
            TerrainPreset.ROCKY_TERRAIN: TerrainPresetLibrary.ROCKY_TERRAIN,
            TerrainPreset.TEST_TRACK: TerrainPresetLibrary.TEST_TRACK,
        }
        return presets[preset]

    @staticmethod
    def list_presets() -> Dict[str, str]:
        """List all available presets.

        Returns:
            Dictionary of {preset_name: description}
        """
        return {
            "mars_flat": "Large flat regolith plain",
            "mars_valley": "Narrow valley environment",
            "sandy_plain": "Loose sandy terrain",
            "rocky_terrain": "Mixed rock and dust",
            "test_track": "Small test area",
        }


class TerrainConfigBuilder:
    """Fluent builder for terrain configurations."""

    def __init__(self, name: str = "Custom Terrain"):
        """Initialize builder.

        Args:
            name: Terrain configuration name
        """
        self.config = TerrainConfig(
            name=name,
            width=15.0,
            length=5.0,
            grid_resolution=0.05,
            soil_type=SoilType.MARS_REGOLITH,
        )

    def with_dimensions(self, width: float, length: float) -> "TerrainConfigBuilder":
        """Set terrain dimensions.

        Args:
            width: Terrain width in Y direction (m)
            length: Terrain length in X direction (m)

        Returns:
            Self for chaining
        """
        self.config.width = width
        self.config.length = length
        return self

    def with_grid_resolution(self, resolution: float) -> "TerrainConfigBuilder":
        """Set terrain grid resolution.

        Finer = more accurate but slower. Typical: 0.015-0.05 m

        Args:
            resolution: Grid node spacing (m)

        Returns:
            Self for chaining
        """
        if resolution < 0.005:
            raise ValueError("Grid resolution too fine (<5mm), will be very slow")
        if resolution > 0.1:
            print("Warning: Grid resolution >100mm may be too coarse")
        self.config.grid_resolution = resolution
        return self

    def with_soil_type(self, soil_type: SoilType) -> "TerrainConfigBuilder":
        """Set soil type.

        Args:
            soil_type: Type of soil (mars_regolith, sandy, loose_dust)

        Returns:
            Self for chaining
        """
        self.config.soil_type = soil_type
        return self

    def with_description(self, description: str) -> "TerrainConfigBuilder":
        """Set terrain description.

        Args:
            description: Human-readable description

        Returns:
            Self for chaining
        """
        self.config.description = description
        return self

    def build(self) -> TerrainConfig:
        """Build configuration.

        Returns:
            TerrainConfig instance
        """
        return self.config


class TerrainManager2:
    """Enhanced terrain manager with configuration support."""

    def __init__(self, system: chrono.ChSystemNSC):
        """Initialize terrain manager.

        Args:
            system: Physics system
        """
        self.system = system
        self.terrain: Optional[veh.SCMTerrain] = None
        self.config: Optional[TerrainConfig] = None
        self.terrain_init = False

    def initialize_from_preset(
        self, preset: TerrainPreset
    ) -> veh.SCMTerrain:
        """Initialize terrain from preset.

        Args:
            preset: Terrain preset (mars_flat, sandy_plain, etc.)

        Returns:
            Configured SCMTerrain object
        """
        config = TerrainPresetLibrary.get_preset(preset)
        return self.initialize_from_config(config)

    def initialize_from_config(
        self, config: TerrainConfig
    ) -> veh.SCMTerrain:
        """Initialize terrain from configuration object.

        Args:
            config: TerrainConfig instance

        Returns:
            Configured SCMTerrain object
        """
        self.config = config
        soil_params = SoilParameterSet.get_preset(config.soil_type)

        # Create terrain
        self.terrain = veh.SCMTerrain(self.system)

        # Apply soil parameters
        self.terrain.SetSoilParameters(*soil_params.to_tuple())

        # Initialize grid
        self.terrain.Initialize(config.length, config.width, config.grid_resolution)

        self.terrain_init = True
        return self.terrain

    def initialize_from_json(self, json_str: str) -> veh.SCMTerrain:
        """Initialize terrain from JSON configuration.

        Args:
            json_str: JSON string with terrain configuration

        Returns:
            Configured SCMTerrain object
        """
        config = TerrainConfig.from_json(json_str)
        return self.initialize_from_config(config)

    def save_config(self, filepath: str) -> None:
        """Save current terrain configuration to file.

        Args:
            filepath: Path to save JSON file
        """
        if not self.config:
            raise RuntimeError("No terrain configuration to save")

        with open(filepath, "w") as f:
            f.write(self.config.to_json())

        print(f"Terrain configuration saved to {filepath}")

    def load_config_from_file(self, filepath: str) -> veh.SCMTerrain:
        """Load terrain configuration from file.

        Args:
            filepath: Path to JSON configuration file

        Returns:
            Configured SCMTerrain object
        """
        with open(filepath, "r") as f:
            config = TerrainConfig.from_json(f.read())

        return self.initialize_from_config(config)

    def get_config(self) -> Optional[TerrainConfig]:
        """Get current terrain configuration.

        Returns:
            Current TerrainConfig or None if not initialized
        """
        return self.config

    def get_height(self, x: float, y: float) -> float:
        """Query terrain height at position.

        Args:
            x: X coordinate (m)
            y: Y coordinate (m)

        Returns:
            Terrain height (Z coordinate) in meters
        """
        if not self.terrain:
            raise RuntimeError("Terrain not initialized")

        return self.terrain.GetHeight(chrono.ChVector3d(x, y, 0))

    def get_sinkage(
        self, body: chrono.ChBody, wheel_radius: Optional[float] = None
    ) -> float:
        """Calculate sinkage depth of body into terrain.

        Args:
            body: Body whose sinkage to measure
            wheel_radius: Wheel radius (for wheel sinkage calculation)

        Returns:
            Sinkage depth in meters
        """
        if not self.terrain:
            raise RuntimeError("Terrain not initialized")

        pos = body.GetPos()
        terrain_height = self.get_height(pos.x, pos.y)

        if wheel_radius is not None:
            wheel_center_height = pos.z
            wheel_bottom = wheel_center_height - wheel_radius
            sinkage = abs(terrain_height - wheel_bottom)
        else:
            sinkage = abs(pos.z - terrain_height)

        return sinkage

    def add_active_domain(
        self,
        body: chrono.ChBody,
        domain_center: Optional[chrono.ChVector3d] = None,
        domain_size: Optional[chrono.ChVector3d] = None,
    ) -> None:
        """Register active domain for terrain deformation.

        Args:
            body: Body to track
            domain_center: Center of active domain (default: origin)
            domain_size: Size of active domain (default: 1.2x0.6x0.6)
        """
        if not self.terrain:
            raise RuntimeError("Terrain not initialized")

        if domain_center is None:
            domain_center = chrono.ChVector3d(0, 0, 0)
        if domain_size is None:
            domain_size = chrono.ChVector3d(1.2, 0.6, 0.6)

        self.terrain.AddActiveDomain(body, domain_center, domain_size)
