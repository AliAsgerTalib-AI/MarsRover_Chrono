"""Metrics collection and analysis for rover simulations."""

import csv
from dataclasses import dataclass, field
from typing import List, Optional

import pychrono as chrono


@dataclass
class MetricFrame:
    """Data for a single simulation time step."""

    sim_time: float
    velocity_x: float
    slip_percent: float
    drawbar_force: float
    sinkage_mm: float
    wheel_height: float = 0.0
    terrain_height: float = 0.0

    def to_dict(self):
        """Convert to dictionary."""
        return {
            "time": self.sim_time,
            "velocity_x": self.velocity_x,
            "slip_percent": self.slip_percent,
            "drawbar_force": self.drawbar_force,
            "sinkage_mm": self.sinkage_mm,
            "wheel_height": self.wheel_height,
            "terrain_height": self.terrain_height,
        }


class MetricsCollector:
    """Collect and store metrics from rover simulation."""

    def __init__(self, output_freq: float = 0.5):
        """Initialize metrics collector.

        Args:
            output_freq: Frequency (in seconds) at which to collect data.
                Set to 0 to collect every step (expensive), or e.g. 0.5 to collect every 0.5s.
        """
        self.output_freq = output_freq
        self.data: List[MetricFrame] = []
        self.last_output_time = 0.0

    def should_collect(self, sim_time: float) -> bool:
        """Check if data should be collected at this time step.

        Args:
            sim_time: Current simulation time

        Returns:
            True if output_freq has elapsed since last collection
        """
        if sim_time - self.last_output_time >= self.output_freq - 1e-6:
            self.last_output_time = sim_time
            return True
        return False

    def collect_frame(
        self,
        sim_time: float,
        chassis: chrono.ChBody,
        wheel: chrono.ChBody,
        wheel_radius: float,
        v_ideal: float,
        linear_motor: chrono.ChLinkMotorLinearSpeed,
        terrain_height_func,  # Function that returns terrain height at (x, y)
    ) -> MetricFrame:
        """Collect metrics for a single time step.

        Args:
            sim_time: Current simulation time
            chassis: Chassis body
            wheel: Wheel body
            wheel_radius: Wheel radius (m)
            v_ideal: Ideal velocity for slip calculation (m/s)
            linear_motor: Linear motor for drawbar force measurement
            terrain_height_func: Function to get terrain height at position

        Returns:
            MetricFrame with collected data
        """
        # Velocity
        v_actual = chassis.GetPosDt().x

        # Slip ratio
        if v_ideal > 0:
            slip_percent = (1.0 - (v_actual / v_ideal)) * 100.0
        else:
            slip_percent = 100.0

        # Drawbar pull (traction force)
        drawbar_force = -linear_motor.GetMotorForce()

        # Sinkage
        wheel_pos = wheel.GetPos()
        terrain_height = terrain_height_func(wheel_pos.x, wheel_pos.y)
        wheel_center_height = wheel_pos.z
        wheel_bottom = wheel_center_height - wheel_radius
        sinkage_m = abs(terrain_height - wheel_bottom)
        sinkage_mm = sinkage_m * 1000.0

        frame = MetricFrame(
            sim_time=sim_time,
            velocity_x=v_actual,
            slip_percent=slip_percent,
            drawbar_force=drawbar_force,
            sinkage_mm=sinkage_mm,
            wheel_height=wheel_center_height,
            terrain_height=terrain_height,
        )

        self.data.append(frame)
        return frame

    def save_csv(self, filename: str) -> None:
        """Save collected metrics to CSV file.

        Args:
            filename: Output CSV file path
        """
        if not self.data:
            print(f"Warning: No data to save to {filename}")
            return

        with open(filename, "w", newline="") as f:
            fieldnames = list(self.data[0].to_dict().keys())
            writer = csv.DictWriter(f, fieldnames=fieldnames)

            writer.writeheader()
            for frame in self.data:
                writer.writerow(frame.to_dict())

        print(f"Metrics saved to {filename}")

    def print_header(self) -> None:
        """Print column headers for metrics table."""
        header = f"{'Time (s)':<10}{'Slip %':<12}{'Fwd Speed (m/s)':<18}{'Drawbar Pull (N)':<20}{'Sinkage (mm)':<15}"
        print("\n" + "=" * len(header))
        print(header)
        print("=" * len(header))

    def print_frame(self, frame: MetricFrame) -> None:
        """Print a single metrics frame as a table row.

        Args:
            frame: Metrics frame to print
        """
        print(
            f"{frame.sim_time:<10.1f}{frame.slip_percent:<12.1f}{frame.velocity_x:<18.3f}{frame.drawbar_force:<20.2f}{frame.sinkage_mm:<15.2f}"
        )

    def print_summary(self) -> None:
        """Print summary statistics of collected data."""
        if not self.data:
            print("No data collected")
            return

        velocities = [f.velocity_x for f in self.data]
        slips = [f.slip_percent for f in self.data]
        sinkages = [f.sinkage_mm for f in self.data]
        forces = [f.drawbar_force for f in self.data]

        print("\n" + "=" * 60)
        print("SUMMARY STATISTICS")
        print("=" * 60)
        print(f"Simulation time: {self.data[-1].sim_time:.2f} s")
        print(f"Data points collected: {len(self.data)}")
        print(f"\nVelocity (m/s):")
        print(f"  Mean: {sum(velocities) / len(velocities):.3f}")
        print(f"  Min:  {min(velocities):.3f}")
        print(f"  Max:  {max(velocities):.3f}")
        print(f"\nSlip ratio (%):")
        print(f"  Mean: {sum(slips) / len(slips):.1f}")
        print(f"  Min:  {min(slips):.1f}")
        print(f"  Max:  {max(slips):.1f}")
        print(f"\nSinkage (mm):")
        print(f"  Mean: {sum(sinkages) / len(sinkages):.2f}")
        print(f"  Min:  {min(sinkages):.2f}")
        print(f"  Max:  {max(sinkages):.2f}")
        print(f"\nDrawbar pull (N):")
        print(f"  Mean: {sum(forces) / len(forces):.1f}")
        print(f"  Min:  {min(forces):.1f}")
        print(f"  Max:  {max(forces):.1f}")
        print("=" * 60)
