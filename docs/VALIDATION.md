# PyChronoRover Validation & Testing Guide

This document provides a complete validation checklist and testing procedures for PyChronoRover.

## Prerequisites for Full Validation

To run the complete validation suite, you need:

```bash
pip install pychrono pytest matplotlib pandas
```

**Note**: PyChronoRover requires pychrono, which is a compiled physics simulation library. Installation varies by platform:
- **Linux**: Usually available via package manager or pip
- **macOS**: Available via Homebrew or from source
- **Windows**: May require compilation from source; pre-built wheels available on some distributions

See [PyChronoRover README](../README.md) for detailed installation instructions.

---

## Phase 1: Static Code Validation (No pychrono required)

### ✅ Code Structure Validation

**Objective**: Verify the project structure is correct and all files are in place.

```
src/rover/
├── __init__.py                 ✓ Exports all public API
├── system.py                   ✓ SystemFactory class
├── materials.py                ✓ MaterialFactory class
├── bodies.py                   ✓ ChassisBuilder, WheelBuilder
├── wheels.py                   ✓ AdvancedWheelBuilder, WheelArray
├── terrain.py                  ✓ TerrainManager class
├── terrain_config.py           ✓ TerrainManager2, presets, builder
├── motors.py                   ✓ RotationMotor, LinearMotor
├── constraints.py              ✓ PrismaticConstraint, RevoluteJoint
└── metrics.py                  ✓ MetricsCollector, MetricFrame

src/utils/
├── __init__.py                 ✓ Exports utilities
├── logger.py                   ✓ SimulationLogger class
└── data_logger.py              ✓ DataLogger, ConsoleLogger

config/
├── __init__.py                 ✓ Config exports
└── default_config.py           ✓ Default parameters

examples/
├── scenario_0_static_drop.py           ✓ Refactored
├── scenario_1_single_wheel_slip.py     ✓ Refactored
├── scenario_2_kinematic_control.py     ✓ Refactored
├── scenario_3_four_wheel_rover.py      ✓ Template
└── example_advanced_components.py      ✓ New example

tests/
├── test_system.py              ✓ System factory tests
├── test_validation.py          ✓ Comprehensive validation tests
└── run_validation.py           ✓ Validation script

docs/
├── CLAUDE.md                   ✓ Main developer guide
├── COMPONENTS.md               ✓ Component usage guide
├── VALIDATION.md               ✓ This file
├── terramechanics.md           ✓ Physics reference
└── pychrono_patterns.md        ✓ Chrono patterns
```

### ✅ Import Validation

**Objective**: Verify all modules can be imported.

```bash
# Test core modules
python -c "from rover import SystemFactory; print('✓ SystemFactory imports')"
python -c "from rover import ChassisBuilder, WheelBuilder; print('✓ Body builders import')"
python -c "from rover import AdvancedWheelBuilder, WheelArray; print('✓ Advanced wheels import')"
python -c "from rover import TerrainManager2, TerrainPreset; print('✓ Terrain config imports')"
python -c "from rover import RotationMotor, LinearMotor; print('✓ Motors import')"
python -c "from utils import DataLogger, ConsoleLogger; print('✓ Data loggers import')"
```

### ✅ API Documentation Validation

**Objective**: Verify all public APIs are documented.

- [x] `SystemFactory` documented in CLAUDE.md
- [x] `ChassisBuilder`, `WheelBuilder` documented in COMPONENTS.md
- [x] `AdvancedWheelBuilder`, `WheelArray` documented in COMPONENTS.md
- [x] `TerrainManager2`, `TerrainConfigBuilder` documented in COMPONENTS.md
- [x] `DataLogger`, `ConsoleLogger` documented in COMPONENTS.md
- [x] All examples have docstrings

### ✅ Example Code Syntax Validation

**Objective**: Verify example code is syntactically correct.

```bash
python -m py_compile examples/scenario_0_static_drop.py
python -m py_compile examples/scenario_1_single_wheel_slip.py
python -m py_compile examples/scenario_2_kinematic_control.py
python -m py_compile examples/scenario_3_four_wheel_rover.py
python -m py_compile examples/example_advanced_components.py
```

Expected: All files compile without syntax errors.

---

## Phase 2: Unit Tests (Requires pychrono)

### Installation

```bash
pip install pytest pychrono
cd C:\ChronoRover
pytest tests/test_system.py -v
```

### Test Coverage

#### `test_system.py` - 12 tests
- [x] System creation without error
- [x] Mars gravity set correctly (-3.71 m/s²)
- [x] Bullet collision backend configured
- [x] System is NSC type
- [x] System can add bodies
- [x] Solver configuration works
- [x] Gravity direction is correct (negative Z)
- [x] Gravity magnitude for Mars is correct
- [x] Gravity x/y components are zero
- [x] Multiple systems can be created independently
- [x] Configuration persists after creation
- [x] Solver parameters applied correctly

Run: `pytest tests/test_system.py -v`

#### `test_validation.py` - 20+ tests
- [x] System creation with Mars gravity
- [x] Chassis builder creates valid body
- [x] Wheel builder creates valid body
- [x] Standard wheel type (15kg)
- [x] Lightweight wheel type (10kg)
- [x] High-grip wheel type (18kg)
- [x] Single wheel array (1 wheel)
- [x] Two-wheel array (2 wheels)
- [x] Four-wheel array (4 wheels)
- [x] Six-wheel array (6 wheels)
- [x] Terrain preset: Mars Flat
- [x] Terrain preset: Test Track
- [x] Terrain preset: Sandy Plain
- [x] Data logger creation
- [x] Data logging and collection
- [x] Data saving to CSV
- [x] Statistics generation (mean, min, max, std)
- [x] Column extraction
- [x] Scenario 0: Chassis settling (reasonable sinkage)
- [x] Scenario 1: Wheel slip characterization
- [x] Scenario 1: Metrics collection
- [x] Scenario 1: Slip values valid (0-100%)
- [x] Scenario 1: Velocity ramp working

Run: `pytest tests/test_validation.py -v`

Or run the validation script directly:
```bash
python tests/run_validation.py
```

---

## Phase 3: Integration Tests (Requires pychrono)

### Scenario Execution Tests

#### Scenario 0: Static Chassis Drop
```bash
python examples/scenario_0_static_drop.py
```

**Expected Output**:
- No errors during execution
- Chassis settles into terrain
- Sinkage values between 0-0.5m
- Output shows time progression and sinkage

**Numerical Parity Check**:
- Compare with original `rover_sandbox.py` output
- Chassis Z position should decrease over time
- Sinkage values should match to 3 decimal places

#### Scenario 1: Single Wheel Slip
```bash
python examples/scenario_1_single_wheel_slip.py
```

**Expected Output**:
- No errors during execution
- Table with: Time | Slip % | Velocity | Drawbar | Sinkage
- Slip ratio increases from 0% to ~100% over 10 seconds
- Velocity decreases from 0.6 m/s to 0.0 m/s
- CSV output saved to `data/logs/scenario_1_slip_sweep.csv`

**Numerical Parity Check**:
- Compare output CSV with original `rover_sandbox1.py`
- Slip ratio should match to 0.1%
- Velocity should match to 0.001 m/s
- Drawbar force should match to 0.1 N

#### Scenario 2: Kinematic Control
```bash
python examples/scenario_2_kinematic_control.py
```

**Expected Output**:
- No errors during execution
- Smoother output than Scenario 1 (prismatic constraint)
- Slip and velocity trends similar to Scenario 1
- CSV output saved to `data/logs/scenario_2_kinematic_control.csv`

**Numerical Parity Check**:
- Slip ratio should be very similar to Scenario 1
- Sinkage might be slightly different due to carriage mechanism
- Overall trends should match original `rover_sandbox2.py`

### Advanced Component Tests

#### Test Advanced Wheels
```python
from rover import AdvancedWheelBuilder, WheelType
import pychrono as chrono

system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -3.71))

# Test each wheel type
for wheel_type in [WheelType.STANDARD, WheelType.LIGHTWEIGHT, 
                    WheelType.HIGH_GRIP, WheelType.LOW_GRAVITY, 
                    WheelType.FLEXIBLE]:
    wheel = AdvancedWheelBuilder.create_by_type(system, wheel_type)
    print(f"✓ {wheel_type.value}: {wheel.GetName()} (mass={wheel.GetMass()}kg)")
```

**Expected**:
- All 5 wheel types create successfully
- Masses match specifications

#### Test Terrain Configurators
```python
from rover import TerrainManager2, TerrainPreset, TerrainConfigBuilder, SoilType

system = chrono.ChSystemNSC()

# Test presets
for preset in [TerrainPreset.MARS_FLAT, TerrainPreset.TEST_TRACK, 
               TerrainPreset.SANDY_PLAIN, TerrainPreset.ROCKY_TERRAIN,
               TerrainPreset.MARS_VALLEY]:
    terrain_mgr = TerrainManager2(system)
    terrain = terrain_mgr.initialize_from_preset(preset)
    print(f"✓ {preset.value}: {terrain_mgr.get_config().name}")

# Test custom builder
config = (TerrainConfigBuilder("Custom")
          .with_dimensions(20.0, 15.0)
          .with_grid_resolution(0.02)
          .with_soil_type(SoilType.SANDY_SOIL)
          .build())
terrain_mgr = TerrainManager2(system)
terrain_mgr.initialize_from_config(config)
print(f"✓ Custom terrain: {config.name}")
```

**Expected**:
- All presets load without error
- Custom builder creates valid config
- Grid resolution respected

#### Test Data Loggers
```python
from utils import DataLogger, LogFormat

logger = DataLogger(format=LogFormat.BOTH)
for i in range(10):
    logger.log({"velocity": 0.3 - i*0.01, "slip": i*10}, timestamp=i*0.1)

logger.save()
stats = logger.get_statistics("slip")
print(f"✓ Logged {len(logger.all_data)} entries")
print(f"✓ Slip stats: mean={stats['mean']:.1f}%, std={stats['std']:.1f}%")
```

**Expected**:
- Data logged successfully
- CSV and JSON files created
- Statistics generated correctly

---

## Phase 4: Numerical Validation

### Slip Ratio Validation

**Formula**: `slip = (1 - v_actual / v_ideal) × 100`

**Test Case**: Scenario 1 with constant motor speeds
- Wheel rotation: 2.0 rad/s
- Wheel radius: 0.3 m
- v_ideal = ω × r = 2.0 × 0.3 = 0.6 m/s

**Expected Results**:
- At t=0s: slip ≈ 0% (full traction)
- At t=5s: slip ≈ 50% (half traction)
- At t=10s: slip ≈ 100% (no traction)

**Tolerance**: ±5% slip

### Sinkage Validation

**Formula**: `sinkage = |terrain_height - (wheel_center_z - wheel_radius)|`

**Test Case**: Single wheel on Mars regolith

**Expected Results**:
- Initial: sinkage = 0 (wheel on surface)
- After settling: sinkage between 0.01-0.05 m
- Never exceeds 0.5 m (reasonable for Mars regolith)

**Tolerance**: ±0.01 m

### Drawbar Pull Validation

**Definition**: Horizontal force exerted by linear motor on chassis

**Expected Behavior**:
- Proportional to wheel slip and traction
- Higher at low slip (good grip)
- Decreases as slip increases
- Should be positive (forward motion)

**Tolerance**: Qualitative (should decrease monotonically with time)

---

## Phase 5: Performance Validation

### Execution Time

**Scenario 0** (2.0 second simulation, 10ms timestep):
- Expected time: < 5 seconds
- Acceptable: < 10 seconds

**Scenario 1** (10.0 second simulation, 10ms timestep):
- Expected time: < 30 seconds
- Acceptable: < 60 seconds

**Scenario 2** (10.0 second simulation, 10ms timestep):
- Expected time: < 30 seconds
- Acceptable: < 60 seconds

### Memory Usage

**Expected**: < 500 MB RAM for any single scenario

### Output File Sizes

**CSV files** (1000 data points):
- Expected: 50-200 KB
- Acceptable: < 1 MB

---

## Validation Checklist

### Code Structure (No pychrono needed)
- [ ] All 8 core modules exist in `src/rover/`
- [ ] All utilities exist in `src/utils/`
- [ ] Configuration files exist in `config/`
- [ ] All 5 example scenarios exist in `examples/`
- [ ] Test files exist in `tests/`
- [ ] Documentation files exist in `docs/`

### Static Validation (No pychrono needed)
- [ ] All Python files compile without syntax errors
- [ ] All modules can be imported
- [ ] All public APIs are documented
- [ ] Example code is well-formatted and has docstrings

### Unit Tests (Requires pychrono)
- [ ] `pytest tests/test_system.py` passes all 12 tests
- [ ] `pytest tests/test_validation.py` passes all 20+ tests
- [ ] `python tests/run_validation.py` shows 100% pass rate

### Integration Tests (Requires pychrono)
- [ ] Scenario 0 runs without error (< 5 seconds)
- [ ] Scenario 0 output reasonable (sinkage 0-0.5m)
- [ ] Scenario 1 runs without error (< 30 seconds)
- [ ] Scenario 1 slip ratio 0-100%, monotonically increasing
- [ ] Scenario 1 velocity monotonically decreasing
- [ ] Scenario 1 CSV output saved successfully
- [ ] Scenario 2 runs without error (< 30 seconds)
- [ ] Scenario 2 output similar to Scenario 1

### Numerical Validation (Requires pychrono)
- [ ] Slip ratio formula correct (within ±5%)
- [ ] Sinkage in reasonable range (0-0.5m)
- [ ] Drawbar pull decreases with increasing slip
- [ ] Wheel types have correct masses
- [ ] Terrain presets load with correct names
- [ ] Data logger statistics accurate (spot check)

### Component Tests (Requires pychrono)
- [ ] All 5 wheel types create successfully
- [ ] All 4 wheel array configs create (1, 2, 4, 6 wheels)
- [ ] All 5 terrain presets load
- [ ] Terrain builder creates custom configs
- [ ] Terrain configs save/load from JSON
- [ ] Data logger produces both CSV and JSON
- [ ] Data logger statistics generate correctly

### Documentation Validation (No pychrono needed)
- [ ] CLAUDE.md has 12+ sections covering all major topics
- [ ] COMPONENTS.md has examples for all reusable components
- [ ] README.md has quick start and architecture overview
- [ ] All code examples are syntactically correct
- [ ] API references match actual implementations
- [ ] VALIDATION.md (this file) is complete and accurate

---

## Running Full Validation

### Quick Validation (< 1 minute, no pychrono)
```bash
# Check code structure
ls -R src/ config/ examples/ tests/ docs/

# Check syntax
python -m py_compile examples/*.py

# Check imports
python -c "from rover import *; from utils import *; print('✓ All imports OK')"
```

### Standard Validation (5-10 minutes, requires pychrono)
```bash
cd C:\ChronoRover

# Unit tests
pytest tests/test_system.py -v
pytest tests/test_validation.py -v

# Scenario execution
python examples/scenario_0_static_drop.py
python examples/scenario_1_single_wheel_slip.py
```

### Full Validation (30-60 minutes, requires pychrono)
```bash
cd C:\ChronoRover

# All tests
pytest tests/ -v --tb=short

# All scenarios
python tests/run_validation.py
python examples/scenario_0_static_drop.py
python examples/scenario_1_single_wheel_slip.py
python examples/scenario_2_kinematic_control.py

# Component tests
python examples/example_advanced_components.py

# Check outputs
ls -l data/logs/*.csv
head -5 data/logs/*.csv
```

---

## Success Criteria

✅ **All validations pass** when:
1. No syntax errors in any Python file
2. All modules import successfully
3. All unit tests pass (if pychrono installed)
4. All scenarios run without errors
5. Numerical outputs are within tolerance
6. Documentation is complete and accurate

⚠️ **Partial validation acceptable** if:
- pychrono not installed (skip runtime tests)
- System resource constraints (skip performance tests)
- Platform-specific issues (document limitations)

---

## Troubleshooting

### "ModuleNotFoundError: No module named 'pychrono'"
**Solution**: Install pychrono:
```bash
pip install pychrono
# If that fails, see https://projectchrono.org/ for platform-specific instructions
```

### Scenario runs very slowly (> 60 seconds)
**Possible causes**:
- Terrain grid resolution too fine (< 0.01m)
- System resource constraints
- Python not optimized build

**Solution**: Reduce grid resolution in TerrainPreset or use TEST_TRACK preset.

### Sinkage values seem unrealistic
**Check**:
- Are you using correct soil parameters for your terrain?
- Is wheel radius parameter correct (0.3m by default)?
- Is gravity set correctly (Mars = -3.71 m/s²)?

**Solution**: Verify terrain preset and wheel specifications.

### Data logger not creating files
**Check**:
- Does `data/logs/` directory exist?
- Do you have write permissions?
- Did you call `logger.save()` before checking?

**Solution**: Create directory or call `.save()` before checking for files.

---

## Next Steps After Validation

Once all validations pass:
1. ✅ Code is stable and refactoring is correct
2. ✅ Components are reusable and well-tested
3. ✅ Ready to add new features (suspension, steering, sensors)
4. ✅ Safe to share code with collaborators
5. ✅ Foundation for publications or deployment

See [COMPONENTS.md](COMPONENTS.md) for next development steps.
