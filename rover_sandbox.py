import os

import pychrono as chrono
import pychrono.vehicle as veh

print("====================================================")
print("     INITIALIZING MARTIAN ROVER CORE SIMULATION     ")
print("====================================================")

# 1. Instantiate the physical coordination space (Non-Smooth Contact Physics)
system = chrono.ChSystemNSC()

# 2. Re-tune via the updated C++ method naming convention
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -3.71))

# 3. Explicitly assign a CPU-optimized collision engine backend to the system
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# 4. Instantiate a standard surface material for physical contacts
chassis_material = chrono.ChContactMaterialNSC()

# 5. Pass 'True' for collision and hand it the material definition
# Arguments: Length(X), Width(Y), Height(Z), Density, Visualize?, Collide?, Material
chassis = chrono.ChBodyEasyBox(1.4, 0.8, 0.4, 1000, True, True, chassis_material)

chassis.SetPos(chrono.ChVector3d(0, 0, 0.5))  # Drop it from 0.5 meters high
chassis.SetMass(50.0)  # Give it a realistic payload mass (50 kg)
chassis.SetName("Rover_Chassis_Core")

# 6. Use the modern API name to commit the shape to the pipeline
chassis.EnableCollision(True)
system.Add(chassis)

# 7. Generate the SCM Deformable Soil Patch (Empirical Bekker-Wong Framework)
terrain = veh.SCMTerrain(system)
# Arguments: Plane Width (X), Plane Length (Y), and the Node Grid Resolution (meters)
terrain.Initialize(10.0, 10.0, 0.05)

# 8. Injected updated 8-parameter array containing elasto-plastic bounds
terrain.SetSoilParameters(
    0.02e6,  # 1. Bekker_Kphi: Frictional soil stiffness modulus
    0.25e6,  # 2. Bekker_Kc:   Cohesive soil stiffness modulus
    1.1,  # 3. Bekker_n:    Soil compaction scaling exponent
    500.0,  # 4. Mohr_cohesion: Soil cohesion pressure threshold (Pa)
    35.0,  # 5. Mohr_friction: Internal friction angle (degrees)
    0.015,  # 6. Janosi_shear: Shear deformation displacement limit (m)
    2.0e7,  # 7. elastic_K: Elastic stiffness recovery rate per unit area (Pa/m)
    3.0e3,  # 8. damping_R: Vertical viscous damping coefficient (Pa.s/m)
)

print("[-] Success: Physical Sandbox Architecture and Collision Engines Compiled.")
print("[-] Running terramechanics settlement evaluation loop...\n")

# 9. Execute the Time-Step Stepping Loop
time_step = 0.01  # Calculate physics tracks in sharp 10ms intervals
sim_time = 0.0  # Track elapsed global simulation time

print(f"{'Time (s)':<10} | {'Chassis Z-Pos (m)':<20} | {'Max Soil Sinkage (m)':<20}")
print("-" * 60)

while sim_time < 2.0:  # Run the physics simulation forward for 2 entire seconds
    system.DoStepDynamics(time_step)

    # Extract current structural telemetry out of the dynamic state matrices
    chassis_z = chassis.GetPos().z

    # Interrogate the soil deformation grid to see how deep the mass has pressed into it
    # We query a location directly beneath our chassis
    soil_sinkage = terrain.GetHeight(chrono.ChVector3d(0, 0, 0))

    # Print out status snapshots every 0.2 simulated seconds
    if int(sim_time * 100) % 20 == 0:
        print(f"{sim_time:<10.2f} | {chassis_z:<20.4f} | {abs(soil_sinkage):<20.4f}")

    sim_time += time_step

print("-" * 60)
print("[STATUS]: Simulation Cycle Complete. Physics data generated successfully.")
