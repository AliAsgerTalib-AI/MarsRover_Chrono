import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chirr  # Import the Irrlicht visualization engine

print("====================================================")
print("     INITIALIZING VISUAL MARTIAN ROVER SANDBOX     ")
print("====================================================")

# 1. Instantiate the physical coordination space (Non-Smooth Contact Physics)
system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -3.71))
system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

# 2. Instantiate a standard surface material for physical contacts
chassis_material = chrono.ChContactMaterialNSC()

# 3. Assemble the central chassis block
# Arguments: Length(X), Width(Y), Height(Z), Density, Visualize?, Collide?, Material
# Setting 'Visualize' (5th argument) to True builds the 3D geometry mesh automatically
chassis = chrono.ChBodyEasyBox(1.4, 0.8, 0.4, 1000, True, True, chassis_material)
chassis.SetPos(chrono.ChVector3d(0, 0, 1.5))        # Spawn it higher (1.5m) so you can watch it fall
chassis.SetMass(50.0)
chassis.SetName("Rover_Chassis_Core")
chassis.EnableCollision(True)
system.Add(chassis)

# 4. Generate the SCM Deformable Soil Patch
terrain = veh.SCMTerrain(system)
terrain.Initialize(10.0, 10.0, 0.05)
terrain.SetSoilParameters(
    0.02e6,   # 1. Bekker_Kphi: Frictional soil stiffness modulus
    0.25e6,   # 2. Bekker_Kc:   Cohesive soil stiffness modulus
    1.1,      # 3. Bekker_n:    Soil compaction scaling exponent
    500.0,    # 4. Mohr_cohesion: Soil cohesion pressure threshold (Pa)
    35.0,     # 5. Mohr_friction: Internal friction angle (degrees)
    0.015,    # 6. Janosi_shear: Shear deformation displacement limit (m)
    2.0e7,    # 7. elastic_K: Elastic stiffness recovery rate per unit area (Pa/m)
    3.0e3     # 8. damping_R: Vertical viscous damping coefficient (Pa.s/m)
)

print("[-] Success: Physical structures mapped inside memory.")
print("[-] Building 3D Irrlicht Render Canvas...")

# 5. INITIALIZE THE RENDERER: Bind the visual canvas to our system
vis = chirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("Chrono Martian Rover Sandbox")
vis.Initialize()

# 6. CONFIGURE SCENE LIGHTING & CAMERA TRACKS
# Add a typical ambient lighting array so the 3D shapes cast clear visual depth shadows
vis.AddTypicalLights()

# Set camera location position (X, Y, Z) and the target point it focuses its lens toward
vis.AddCamera(chrono.ChVector3d(3.0, -3.0, 2.0), chrono.ChVector3d(0.0, 0.0, 0.0))

print("[+][STATUS]: Rendering environment active. Control window initialized.")
print("[!] Use your mouse to rotate/pan around the scene viewport layout.")

# 7. Interactive Physics and Render Loop
time_step = 0.01   # 10ms simulation resolution calculation slice

# Instead of a fixed timer, vis.Run() keeps the window alive until you manually close it
while vis.Run():
    # Signal the graphics pipeline to begin drawing a fresh visual frame canvas
    vis.BeginScene()

    # Process and rasterize the 3D assets linked to the system bodies onto the viewport
    vis.Render()

    # Complete frame layout rendering calculations
    vis.EndScene()

    # Execute the step calculations forward in time
    system.DoStepDynamics(time_step)

print("====================================================")
print("[STATUS]: Viewport context closed. Physics session safely halted.")
