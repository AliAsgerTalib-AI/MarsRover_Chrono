"""Logging utilities for PyChronoRover simulations."""

from enum import Enum
from typing import Optional


class LogLevel(Enum):
    """Log level enumeration."""

    DEBUG = "DEBUG"
    INFO = "INFO"
    WARNING = "WARNING"
    ERROR = "ERROR"


class SimulationLogger:
    """Simple logger for simulation output."""

    def __init__(self, level: LogLevel = LogLevel.INFO, verbose: bool = True):
        """Initialize logger.

        Args:
            level: Minimum log level to display
            verbose: If False, suppress all output
        """
        self.level = level
        self.verbose = verbose
        self._level_order = {
            LogLevel.DEBUG: 0,
            LogLevel.INFO: 1,
            LogLevel.WARNING: 2,
            LogLevel.ERROR: 3,
        }

    def debug(self, message: str) -> None:
        """Log debug message."""
        self._log(LogLevel.DEBUG, message)

    def info(self, message: str) -> None:
        """Log info message."""
        self._log(LogLevel.INFO, message)

    def warning(self, message: str) -> None:
        """Log warning message."""
        self._log(LogLevel.WARNING, message)

    def error(self, message: str) -> None:
        """Log error message."""
        self._log(LogLevel.ERROR, message)

    def _log(self, level: LogLevel, message: str) -> None:
        """Internal logging method."""
        if not self.verbose:
            return

        if self._level_order[level] >= self._level_order[self.level]:
            print(f"[{level.value}] {message}")
