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
wheel_material.SetFriction(0.7)  # Clean control group friction coefficient

# =============================================================================
# 3. RIGID BODY ASSEMBLY
# =============================================================================
# Ground reference body needed to anchor our linear kinematic controller
# =============================================================================
# 3. RIGID BODY ASSEMBLY
# =============================================================================
# Ground reference body needed to anchor our linear kinematic controller
ground = chrono.ChBody()
ground.SetFixed(True)
system.Add(ground)

# Chassis Block
chassis = chrono.ChBodyEasyBox(1.4, 0.8, 0.4, 1000, True, True, chassis_material)
chassis.SetPos(chrono.ChVector3d(0, 0, 0.6))
chassis.SetMass(50.0)
chassis.EnableCollision(True)
system.Add(chassis)

# Cylindrical Wheel
wheel_radius = 0.3
wheel_width = 0.25

# FIXED: Added chrono.ChAxis_Z as the mandated first argument
wheel = chrono.ChBodyEasyCylinder(
    chrono.ChAxis_Z, wheel_radius, wheel_width, 1000, True, True, wheel_material
)

wheel_pos = chrono.ChVector3d(0.0, 0.5, 0.3)
wheel.SetPos(wheel_pos)

# Align the rolling axis: Rotate 90 degrees around X-axis so local Z points down global Y
wheel_rot = chrono.QuatFromAngleX(math.pi / 2)
wheel.SetRot(wheel_rot)
wheel.SetMass(15.0)
wheel.EnableCollision(True)
system.Add(wheel)

# =============================================================================
# 4. PRIMARY ACTUATOR: CONSTANT ANGULAR SPEED MOTOR
# =============================================================================
# Standard modern architecture for active rotational velocity joint
joint_frame = chrono.ChFramed(wheel_pos, wheel_rot)
rot_motor = chrono.ChLinkMotorRotationSpeed()
rot_motor.Initialize(chassis, wheel, joint_frame)

omega_const = 2.0  # Constant angular speed in rad/s
motor_speed_profile = chrono.ChFunctionConst(omega_const)
rot_motor.SetSpeedFunction(motor_speed_profile)
system.Add(rot_motor)

# =============================================================================
# 5. SECONDARY ACTUATOR: FORCED KINEMATIC LINEAR SLIP-SWEEP MOTOR
# =============================================================================
# Computes ideal unconstrained forward translation velocity (v = omega * r)
v_ideal = omega_const * wheel_radius  # 2.0 * 0.3 = 0.6 m/s

# Custom ramp function: Drops forward speed linearly from v_ideal (0% slip) to 0.0 (100% slip)
# Over a duration of 10.0 seconds. Slope = -0.06 m/s^2
total_sweep_time = 10.0
linear_speed_profile = chrono.ChFunctionRamp(v_ideal, -v_ideal / total_sweep_time)

# Initialize the linear motor between the fixed ground and the translation chassis
linear_motor = chrono.ChLinkMotorLinearSpeed()
# The joint frame's X-axis defines the axis of linear translation (Global X)
linear_frame = chrono.ChFramed(chassis.GetPos(), chrono.QUNIT)
linear_motor.Initialize(ground, chassis, linear_frame)
linear_motor.SetSpeedFunction(linear_speed_profile)
system.Add(linear_motor)

# =============================================================================
# 6. SCM BEHAVIORAL TERRAIN INTERFACE
# =============================================================================
terrain = veh.SCMTerrain(system)
terrain.Initialize(15.0, 5.0, 0.0)  # Expanded length track for a 10s execution window
terrain.SetSoilParameters(
    0.02e6,  # Bekker Kc
    0.25e6,  # Bekker Kphi
    1.1,  # Exponent n
    500.0,  # Cohesion c (Pa)
    35.0,  # Internal friction angle phi (degrees)
    0.015,  # Shear displacement limit J (m)
    2.0e7,  # elastic_K (Elastic stiffness recovery)
    3.0e3,  # damping_R (Vertical viscous damping)
)

# =============================================================================
# 7. DYNAMICS SWEEP EXECUTION & LOGGING
# =============================================================================
# =============================================================================
# 7. DYNAMICS SWEEP EXECUTION & LOGGING
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

    # FIXED: Changed GetPos_dt() to GetPosDt() per modern SWIG naming rules
    v_actual = chassis.GetPosDt().x

    if v_ideal > 0:
        slip_ratio = (1.0 - (v_actual / v_ideal)) * 100.0
    else:
        slip_ratio = 100.0

    # Isolate Drawbar Pull (Net tractive force)
    drawbar_pull = -linear_motor.GetMotorForce()

    # Track dynamic slip-sinkage profiles
    current_wheel_z = wheel.GetPos().z
    ground_height = terrain.GetHeight(
        chrono.ChVector3d(wheel.GetPos().x, wheel.GetPos().y, 0)
    )
    sinkage_mm = (ground_height - (current_wheel_z - wheel_radius)) * 1000.0

    # Output metrics at 2Hz frequency
    if int(sim_time * 100) % 50 == 0:
        print(
            f"{sim_time:<10.1f}{slip_ratio:<12.1f}{v_actual:<18.3f}{drawbar_pull:<20.2f}{sinkage_mm:<15.2f}"
        )
