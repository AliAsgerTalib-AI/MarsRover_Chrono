"""Quick validation without vehicle module dependency."""

import sys
from pathlib import Path

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

print("\n" + "=" * 80)
print("QUICK VALIDATION - No Vehicle Module")
print("=" * 80 + "\n")

# Test 1: Import pychrono
try:
    import pychrono as chrono
    print("[PASS] PyChronoRover core imported successfully")
except Exception as e:
    print(f"[FAIL] PyChronoRover import failed: {e}")
    sys.exit(1)

# Test 2: Create basic system
try:
    system = chrono.ChSystemNSC()
    g = system.GetGravitationalAcceleration()
    print(f"[PASS] Physics system created (gravity: {g.z} m/s²)")
except Exception as e:
    print(f"[FAIL] System creation failed: {e}")
    sys.exit(1)

# Test 3: Try to import rovers (will fail on vehicle module)
try:
    from rover import SystemFactory, MaterialFactory, ChassisBuilder, WheelBuilder
    print("[PASS] Core rover modules imported")
except ImportError as e:
    if "vehicle" in str(e):
        print(f"[WARN] Vehicle module import issue (expected): {e}")
        print("\nDiagnosis: PyChronoRover is installed but vehicle module has DLL dependency issue")
        print("This is a common Windows issue requiring:")
        print("  1. Verify Visual C++ Runtime is installed")
        print("  2. Reinstall pychrono: pip install --force-reinstall pychrono")
        print("  3. Or check your miniconda environment is complete")
        sys.exit(1)
    else:
        print(f"[FAIL] Import failed: {e}")
        sys.exit(1)

print("\n" + "=" * 80)
print("Status: PyChronoRover core is working, but vehicle module needs DLL fix")
print("=" * 80)
