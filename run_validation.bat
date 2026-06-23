@echo off
REM PyChronoRover Validation Script
REM This script activates the chrono_clean environment and runs the validation suite

setlocal enabledelayedexpansion

echo.
echo =====================================================================
echo    PyChronoRover Validation Suite
echo =====================================================================
echo.

REM Activate miniconda
call "C:\Users\alias\miniconda3\Scripts\activate.bat" chrono_clean

REM Verify we're in the right environment
echo Current environment: %CONDA_DEFAULT_ENV%
echo.

REM Run validation script
echo Starting validation tests...
echo.

cd C:\ChronoRover
"C:\Users\alias\miniconda3\envs\chrono_clean\python.exe" tests\run_validation.py

echo.
echo =====================================================================
echo    Validation Complete
echo =====================================================================
pause
