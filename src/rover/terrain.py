"""Terrain management for Mars rover simulations.

Handles SCM (Soil Contact Model) deformable terrain with Bekker parameters.
"""

from dataclasses import dataclass
from enum import Enum
from typing import Dict, Optional

import pychrono as chrono
import pychrono.vehicle as veh


class SoilType(Enum):
    """Predefined soil types with Bekker parameters."""

    MARS_REGOLITH = "mars_regolith"
    SANDY_SOIL = "sandy"
    LOOSE_DUST = "loose_dust"


@dataclass
class SoilParameters:
    """Bekker terramechanics soil parameters."""

    kc: float  # Bekker cohesion stiffness modulus (Pa/m)
    kphi: float  # Bekker friction stiffness modulus (Pa/m)
    n: float  # Soil compaction scaling exponent (unitless)
    c: float  # Mohr-Coulomb cohesion pressure (Pa)
    phi: float  # Mohr-Coulomb friction angle (degrees)
    j: float  # Janoski shear deformation displacement limit (m)
    elastic_k: float  # Elastic stiffness recovery rate (Pa/m)
    damping_r: float  # Viscous damping coefficient (Pa·s/m)

    def to_tuple(self) -> tuple:
        """Convert to tuple for SCMTerrain.SetSoilParameters()."""
        return (self.kc, self.kphi, self.n, self.c, self.phi, self.j, self.elastic_k, self.damping_r)


class SoilParameterSet:
    """Library of preset soil parameters for different terrain types."""

    # Mars regolith based on rover field studies and Bekker-Wong model
    MARS_REGOLITH = SoilParameters(
        kc=0.02e6,  # Bekker Kc
        kphi=0.25e6,  # Bekker Kphi
        n=1.1,  # compaction exponent
        c=500.0,  # Mohr cohesion (Pa)
        phi=35.0,  # friction angle (degrees)
        j=0.015,  # Janosi shear (m)
        elastic_k=2.0e7,  # recovery stiffness
        damping_r=3.0e3,  # viscous damping
    )

    # Sandy soil (terrestrial analog)
    SANDY_SOIL = SoilParameters(
        kc=0.05e6,
        kphi=0.4e6,
        n=0.9,
        c=200.0,
        phi=32.0,
        j=0.02,
        elastic_k=2.5e7,
        damping_r=2.0e3,
    )

    # Loose dust (lower strength)
    LOOSE_DUST = SoilParameters(
        kc=0.01e6,
        kphi=0.1e6,
        n=1.2,
        c=100.0,
        phi=25.0,
        j=0.025,
        elastic_k=1.0e7,
        damping_r=1.0e3,
    )

    @staticmethod
    def get_preset(soil_type: SoilType) -> SoilParameters:
        """Get preset soil parameters.

        Args:
            soil_type: Type of soil (enum or string)

        Returns:
            SoilParameters instance
        """
        if isinstance(soil_type, str):
            soil_type = SoilType(soil_type)

        if soil_type == SoilType.MARS_REGOLITH:
            return SoilParameterSet.MARS_REGOLITH
        elif soil_type == SoilType.SANDY_SOIL:
            return SoilParameterSet.SANDY_SOIL
        elif soil_type == SoilType.LOOSE_DUST:
            return SoilParameterSet.LOOSE_DUST
        else:
            raise ValueError(f"Unknown soil type: {soil_type}")


class TerrainManager:
    """Manager for SCM deformable terrain."""

    def __init__(self, system: chrono.ChSystemNSC):
        """Initialize terrain manager.

        Args:
            system: Physics system that terrain will be added to
        """
        self.system = system
        self.terrain: Optional[veh.SCMTerrain] = None
        self.terrain_init = False

    def initialize_scm(
        self,
        width: float = 15.0,
        length: float = 5.0,
        grid_resolution: float = 0.05,
        soil_params: Optional[SoilParameters] = None,
    ) -> veh.SCMTerrain:
        """Initialize SCM (Soil Contact Model) deformable terrain.

        Args:
            width: Terrain width in Y direction (m)
            length: Terrain length in X direction (m)
            grid_resolution: Node spacing in the deformation grid (m)
                - Smaller = higher fidelity but slower (typical: 0.015-0.05 m)
            soil_params: Bekker soil parameters; if None, uses Mars regolith

        Returns:
            Configured SCMTerrain object
        """
        if soil_params is None:
            soil_params = SoilParameterSet.MARS_REGOLITH

        # Create terrain
        self.terrain = veh.SCMTerrain(self.system)

        # Register soil parameters BEFORE initialization
        self.terrain.SetSoilParameters(*soil_params.to_tuple())

        # Initialize terrain grid
        self.terrain.Initialize(length, width, grid_resolution)

        self.terrain_init = True
        return self.terrain

    def add_active_domain(
        self,
        body: chrono.ChBody,
        domain_center: Optional[chrono.ChVector3d] = None,
        domain_size: Optional[chrono.ChVector3d] = None,
    ) -> None:
        """Register an active domain for terrain deformation around a body.

        Use this when you want tighter terrain resolution around specific wheels/components.

        Args:
            body: Body to track (e.g., wheel)
            domain_center: Center of active domain relative to body (default: body origin)
            domain_size: Size of active domain bounding box (default: 1.2 × 0.6 × 0.6 m)
        """
        if not self.terrain:
            raise RuntimeError("Terrain not initialized. Call initialize_scm() first.")

        if domain_center is None:
            domain_center = chrono.ChVector3d(0, 0, 0)
        if domain_size is None:
            domain_size = chrono.ChVector3d(1.2, 0.6, 0.6)

        self.terrain.AddActiveDomain(body, domain_center, domain_size)

    def get_height(self, x: float, y: float) -> float:
        """Query terrain height at a given (x, y) location.

        Args:
            x: X coordinate (m)
            y: Y coordinate (m)

        Returns:
            Terrain height (Z coordinate) in meters
        """
        if not self.terrain:
            raise RuntimeError("Terrain not initialized. Call initialize_scm() first.")

        return self.terrain.GetHeight(chrono.ChVector3d(x, y, 0))

    def get_sinkage(self, body: chrono.ChBody, wheel_radius: Optional[float] = None) -> float:
        """Calculate sinkage depth of a body into terrain.

        Args:
            body: Body whose sinkage to measure
            wheel_radius: Radius of wheel (for wheel sinkage calculation)
                If None, measures from body position

        Returns:
            Sinkage depth in meters (positive = body sinks into terrain)
        """
        if not self.terrain:
            raise RuntimeError("Terrain not initialized. Call initialize_scm() first.")

        pos = body.GetPos()
        terrain_height = self.get_height(pos.x, pos.y)

        if wheel_radius is not None:
            # Measure sinkage relative to wheel radius (for wheels)
            wheel_center_height = pos.z
            wheel_bottom = wheel_center_height - wheel_radius
            sinkage = abs(terrain_height - wheel_bottom)
        else:
            # Measure sinkage relative to body center
            sinkage = abs(pos.z - terrain_height)

        return sinkage
