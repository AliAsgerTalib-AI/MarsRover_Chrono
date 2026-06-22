"""Advanced data logging system for rover simulations.

Provides:
- Multiple output formats (CSV, JSON, HDF5)
- Real-time streaming and filtering
- Statistics and summary generation
- Data visualization hooks
"""

import csv
import json
from dataclasses import dataclass, asdict, field
from enum import Enum
from pathlib import Path
from typing import Any, Dict, List, Optional
from datetime import datetime


class LogFormat(Enum):
    """Output file formats."""

    CSV = "csv"
    JSON = "json"
    BOTH = "both"


class LogLevel(Enum):
    """Data logging levels."""

    DEBUG = "DEBUG"
    INFO = "INFO"
    WARNING = "WARNING"
    ERROR = "ERROR"


@dataclass
class LogEntry:
    """Single data logging entry."""

    timestamp: float
    data: Dict[str, Any]
    metadata: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary."""
        return {
            "timestamp": self.timestamp,
            **self.data,
            **{f"meta_{k}": v for k, v in self.metadata.items()},
        }


class DataLogger:
    """Advanced data logging system with multiple output formats."""

    def __init__(
        self,
        output_dir: str = "data/logs",
        name: str = "simulation",
        format: LogFormat = LogFormat.CSV,
        buffer_size: int = 100,
    ):
        """Initialize data logger.

        Args:
            output_dir: Directory for output files
            name: Base name for output files
            format: Output format (CSV, JSON, or both)
            buffer_size: Number of entries to buffer before flushing
        """
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)

        self.name = name
        self.format = format
        self.buffer_size = buffer_size
        self.buffer: List[LogEntry] = []
        self.all_data: List[LogEntry] = []
        self.fieldnames: Optional[List[str]] = None
        self.start_time = datetime.now()

    def log(
        self,
        data: Dict[str, Any],
        timestamp: Optional[float] = None,
        metadata: Optional[Dict[str, Any]] = None,
    ) -> None:
        """Log data entry.

        Args:
            data: Dictionary of data to log
            timestamp: Timestamp (auto-generated if not provided)
            metadata: Optional metadata dictionary
        """
        if timestamp is None:
            timestamp = (datetime.now() - self.start_time).total_seconds()

        entry = LogEntry(timestamp=timestamp, data=data, metadata=metadata or {})
        self.buffer.append(entry)
        self.all_data.append(entry)

        # Update fieldnames
        if self.fieldnames is None:
            self.fieldnames = list(entry.to_dict().keys())
        else:
            # Add any new fields
            new_fields = set(entry.to_dict().keys()) - set(self.fieldnames)
            if new_fields:
                self.fieldnames.extend(sorted(new_fields))

        # Auto-flush if buffer is full
        if len(self.buffer) >= self.buffer_size:
            self.flush()

    def flush(self) -> None:
        """Write buffered data to file."""
        if not self.buffer:
            return

        if self.format in (LogFormat.CSV, LogFormat.BOTH):
            self._flush_csv()

        if self.format in (LogFormat.JSON, LogFormat.BOTH):
            self._flush_json()

        self.buffer.clear()

    def _flush_csv(self) -> None:
        """Write buffered data to CSV file."""
        csv_file = self.output_dir / f"{self.name}.csv"

        # Determine if file exists (for header writing)
        file_exists = csv_file.exists()

        with open(csv_file, "a", newline="") as f:
            if self.fieldnames is None:
                return

            writer = csv.DictWriter(f, fieldnames=self.fieldnames)

            # Write header if file is new
            if not file_exists:
                writer.writeheader()

            # Write all buffered entries
            for entry in self.buffer:
                row = {k: "" for k in self.fieldnames}
                row.update(entry.to_dict())
                writer.writerow(row)

    def _flush_json(self) -> None:
        """Write buffered data to JSON file."""
        json_file = self.output_dir / f"{self.name}.json"

        # Read existing data if file exists
        existing_data = []
        if json_file.exists():
            with open(json_file, "r") as f:
                existing_data = json.load(f)

        # Append new entries
        for entry in self.buffer:
            existing_data.append(entry.to_dict())

        # Write all data
        with open(json_file, "w") as f:
            json.dump(existing_data, f, indent=2)

    def save(self) -> None:
        """Flush all data and close logger."""
        self.flush()
        print(f"✓ Logged {len(self.all_data)} entries to {self.output_dir}")

    def get_data(self) -> List[Dict[str, Any]]:
        """Get all logged data as list of dictionaries.

        Returns:
            List of all logged entries
        """
        return [entry.to_dict() for entry in self.all_data]

    def get_column(self, column_name: str) -> List[Any]:
        """Get single column of data.

        Args:
            column_name: Name of column

        Returns:
            List of values for that column
        """
        return [entry.to_dict().get(column_name) for entry in self.all_data]

    def get_statistics(self, column_name: str) -> Dict[str, float]:
        """Get statistics for a column.

        Args:
            column_name: Name of column

        Returns:
            Dictionary with mean, min, max, std
        """
        values = self.get_column(column_name)
        # Filter out None values
        values = [v for v in values if v is not None and isinstance(v, (int, float))]

        if not values:
            return {}

        mean = sum(values) / len(values)
        min_val = min(values)
        max_val = max(values)
        variance = sum((x - mean) ** 2 for x in values) / len(values)
        std = variance ** 0.5

        return {
            "count": len(values),
            "mean": mean,
            "min": min_val,
            "max": max_val,
            "std": std,
        }

    def print_summary(self, columns: Optional[List[str]] = None) -> None:
        """Print summary statistics.

        Args:
            columns: Columns to summarize (all if None)
        """
        if not self.all_data:
            print("No data logged")
            return

        if columns is None:
            # Auto-detect numeric columns
            columns = []
            for key in self.all_data[0].to_dict().keys():
                values = self.get_column(key)
                if any(isinstance(v, (int, float)) for v in values):
                    columns.append(key)

        print("\n" + "=" * 70)
        print(f"LOGGING SUMMARY: {self.name}")
        print("=" * 70)
        print(f"Total entries: {len(self.all_data)}")
        print(f"Time span: {self.all_data[0].timestamp:.2f}s to {self.all_data[-1].timestamp:.2f}s")
        print()

        for col in columns:
            stats = self.get_statistics(col)
            if stats:
                print(f"{col}:")
                print(f"  Mean: {stats['mean']:.4f}")
                print(f"  Min:  {stats['min']:.4f}")
                print(f"  Max:  {stats['max']:.4f}")
                print(f"  Std:  {stats['std']:.4f}")
        print("=" * 70)


class ConsoleLogger:
    """Simple console logger for real-time simulation output."""

    def __init__(self, verbose: bool = True):
        """Initialize console logger.

        Args:
            verbose: If False, suppress all output
        """
        self.verbose = verbose
        self.level = LogLevel.INFO

    def info(self, message: str) -> None:
        """Log info message."""
        if self.verbose:
            print(f"[INFO] {message}")

    def warning(self, message: str) -> None:
        """Log warning message."""
        if self.verbose:
            print(f"[WARNING] {message}")

    def error(self, message: str) -> None:
        """Log error message."""
        if self.verbose:
            print(f"[ERROR] {message}")

    def debug(self, message: str) -> None:
        """Log debug message."""
        if self.verbose and self.level == LogLevel.DEBUG:
            print(f"[DEBUG] {message}")

    def table_header(self, columns: List[str], widths: Optional[List[int]] = None) -> None:
        """Print table header.

        Args:
            columns: Column names
            widths: Optional column widths
        """
        if not self.verbose:
            return

        if widths is None:
            widths = [len(c) + 2 for c in columns]

        header = " | ".join(c.ljust(w) for c, w in zip(columns, widths))
        separator = "-" * len(header)

        print()
        print(separator)
        print(header)
        print(separator)

    def table_row(self, values: List[Any], widths: Optional[List[int]] = None) -> None:
        """Print table row.

        Args:
            values: Row values
            widths: Optional column widths
        """
        if not self.verbose:
            return

        if widths is None:
            widths = [10] * len(values)

        row = " | ".join(
            str(v).ljust(w)
            for v, w in zip(values, widths)
        )
        print(row)

    def table_separator(self, widths: Optional[List[int]] = None) -> None:
        """Print table separator.

        Args:
            widths: Column widths
        """
        if not self.verbose:
            return

        if widths is None:
            widths = [10] * 5

        separator = "-" * (sum(widths) + 3 * (len(widths) - 1))
        print(separator)
