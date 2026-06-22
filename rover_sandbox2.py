import math

import pychrono as chrono
import pychrono.vehicle as veh

# =============================================================================
# 1. SYSTEM INITIALIZATION & HARDWARE BOUNDS
# =============================================================================
system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -3.71))  # Martian Gravity
system.SetCollisionSystemType(
    chrono.ChCollisionSystem.Type_BULLET
)  # Mandated Bullet Backend

# =============================================================================
# 2. MATERIAL PROPERTIES & SURFACE WRAPPERS
# =============================================================================
chassis_material = chrono.ChContactMaterialNSC()
wheel_material = chrono.ChContactMaterialNSC()
wheel_material.SetFriction(0.7)

# =============================================================================
# 3. RIGID BODY ASSEMBLY
# =============================================================================
ground = chrono.ChBody()
ground.SetFixed(True)
system.Add(ground)

# Virtual Carriage (Kinematic horizontal slider)
carriage = chrono.ChBody()
carriage.SetPos(chrono.ChVector3d(0, 0, 0.6))
carriage.SetMass(1.0)
system.Add(carriage)

# Chassis Block
chassis = chrono.ChBodyEasyBox(1.4, 0.8, 0.4, 1000, True, True, chassis_material)
chassis.SetPos(chrono.ChVector3d(0, 0, 0.6))
chassis.SetMass(50.0)
chassis.EnableCollision(True)
system.Add(chassis)

# Cylindrical Wheel
wheel_radius = 0.3
wheel_width = 0.25
wheel = chrono.ChBodyEasyCylinder(
    chrono.ChAxis_Z, wheel_radius, wheel_width, 1000, True, True, wheel_material
)

wheel_pos = chrono.ChVector3d(0.0, 0.5, 0.3)
wheel.SetPos(wheel_pos)
wheel_rot = chrono.QuatFromAngleX(math.pi / 2)
wheel.SetRot(wheel_rot)
wheel.SetMass(15.0)
wheel.EnableCollision(True)
system.Add(wheel)

# =============================================================================
# 4. KINEMATIC CONSTRAINTS & MOTORS
# =============================================================================
# Actuator A: Rotational Drive
rot_joint_frame = chrono.ChFramed(wheel_pos, wheel_rot)
rot_motor = chrono.ChLinkMotorRotationSpeed()
rot_motor.Initialize(chassis, wheel, rot_joint_frame)
omega_const = 2.0
rot_motor.SetSpeedFunction(chrono.ChFunctionConst(omega_const))
system.Add(rot_motor)

# Actuator B: Horizontal Sweep Drive
v_ideal = omega_const * wheel_radius
total_sweep_time = 10.0
linear_speed_profile = chrono.ChFunctionRamp(v_ideal, -v_ideal / total_sweep_time)

linear_motor = chrono.ChLinkMotorLinearSpeed()
linear_motor_rot = chrono.QuatFromAngleY(-math.pi / 2)
linear_frame = chrono.ChFramed(carriage.GetPos(), linear_motor_rot)
linear_motor.Initialize(ground, carriage, linear_frame)
linear_motor.SetSpeedFunction(linear_speed_profile)
system.Add(linear_motor)

# Constraint C: Vertical Prismatic Guide
prismatic = chrono.ChLinkLockPrismatic()
prismatic_frame = chrono.ChFramed(chassis.GetPos(), chrono.QUNIT)
prismatic.Initialize(carriage, chassis, prismatic_frame)
system.Add(prismatic)

# =============================================================================
# 5. SCM BEHAVIORAL TERRAIN INTERFACE
# =============================================================================
terrain = veh.SCMTerrain(system)

# Soil parameter definition must precede structural initialization
terrain.SetSoilParameters(0.02e6, 0.25e6, 1.1, 500.0, 35.0, 0.015, 2.0e7, 3.0e3)

# FIXED: Replaced AddMovingPatch with modern AddActiveDomain and moved BEFORE Initialize()
# Bounding box dimensions: 1.2m long, 0.6m wide, 0.6m high centered on the wheel hub
patch_center = chrono.ChVector3d(0, 0, 0)
patch_dims = chrono.ChVector3d(1.2, 0.6, 0.6)
terrain.AddActiveDomain(wheel, patch_center, patch_dims)

# Tightened node resolution to 15mm to eliminate node-crossing chatter
grid_resolution = 0.015
terrain.Initialize(15.0, 5.0, grid_resolution)

# =============================================================================
# 6. DYNAMICS SWEEP EXECUTION & LOGGING
# =============================================================================
time_step = 0.01
sim_time = 0.0

print("\n" + "=" * 85)
print(
    f"{'Time (s)':<10}{'Slip %':<12}{'Fwd Speed (m/s)':<18}{'Drawbar Pull (N)':<20}{'Sinkage (mm)':<15}"
)
print("=" * 85)

while sim_time <= total_sweep_time:
    system.DoStepDynamics(time_step)
    sim_time += time_step

    v_actual = chassis.GetPosDt().x

    if v_ideal > 0:
        slip_ratio = (1.0 - (v_actual / v_ideal)) * 100.0
    else:
        slip_ratio = 100.0

    drawbar_pull = -linear_motor.GetMotorForce()

    # Corrected absolute sinkage tracking formula relative to the 0.0 datum
    current_wheel_z = wheel.GetPos().z
    sinkage_mm = (0.0 - (current_wheel_z - wheel_radius)) * 1000.0

    if int(sim_time * 100) % 50 == 0:
        print(
            f"{sim_time:<10.1f}{slip_ratio:<12.1f}{v_actual:<18.3f}{drawbar_pull:<20.2f}{sinkage_mm:<15.2f}"
        )
