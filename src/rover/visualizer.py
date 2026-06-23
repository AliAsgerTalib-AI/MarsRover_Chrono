"""Real-time 3D visualization for PyChronoRover simulations."""

try:
    import pychrono as chrono
    import pychrono.irrlicht as irrlicht
    IRRLICHT_AVAILABLE = True
except ImportError:
    IRRLICHT_AVAILABLE = False


class Visualizer:
    """Real-time 3D visualization using Irrlicht renderer.

    Integrates with PyChronoRover simulations to display:
    - Chassis and wheels in 3D
    - Terrain deformation (SCM)
    - Motion in real-time
    - Interactive camera control
    """

    def __init__(self, system, title="PyChronoRover Visualizer", follow_body=None):
        """Initialize visualizer.

        Args:
            system: ChSystemNSC physics system
            title: Window title
            follow_body: ChBody to follow with camera (None = static camera)
        """
        if not IRRLICHT_AVAILABLE:
            raise ImportError(
                "Irrlicht visualization not available. "
                "Ensure pychrono is installed with Irrlicht support."
            )

        self.system = system
        self.title = title
        self.follow_body = follow_body

        # Create visualization system
        self.vis = irrlicht.ChVisualSystemIrrlicht()
        self.vis.AttachSystem(system)
        self.vis.SetWindowTitle(title)
        self.vis.SetWindowSize(1200, 800)
        self.vis.Initialize()
        self.vis.AddLogo()
        self.vis.AddSkyBox()
        self.vis.AddTypicalLights()

        # Add camera and set position
        self.vis.AddCamera(
            chrono.ChVector3d(3, 3, 2),    # position
            chrono.ChVector3d(0, 0, 0)     # target
        )

    def run(self, duration=10.0, time_step=0.01, callback=None, keep_open=True):
        """Run simulation with real-time visualization.

        Args:
            duration: Total simulation time (seconds)
            time_step: Physics time step (seconds)
            callback: Optional function called each frame: callback(time, step_count)
            keep_open: If True, keep window open after simulation (press ESC or close to exit)
        """
        time = 0.0
        step = 0

        # Run simulation
        while self.vis.Run() and time < duration:
            # Begin rendering frame
            self.vis.BeginScene()

            # Render the scene
            self.vis.Render()

            # End rendering frame
            self.vis.EndScene()

            # Physics step
            self.system.DoStepDynamics(time_step)
            time += time_step
            step += 1

            # User callback
            if callback:
                callback(time, step)

            # Update camera to follow body if specified
            if self.follow_body:
                pos = self.follow_body.GetPos()
                camera = self.vis.GetActiveCamera()
                if camera:
                    # Camera positioned relative to body
                    camera.setPosition(
                        irrlicht.vector3df(float(pos.x + 2.0), float(pos.y + 2.0), float(pos.z + 1.5))
                    )
                    camera.setTarget(
                        irrlicht.vector3df(float(pos.x), float(pos.y), float(pos.z))
                    )

        # Keep window open after simulation if requested
        if keep_open:
            print("\nSimulation complete. Window will stay open - close it to exit.")
            while self.vis.Run():
                self.vis.BeginScene()
                self.vis.Render()
                self.vis.EndScene()

    def run_with_metrics(self, duration=10.0, time_step=0.01, metrics=None):
        """Run simulation with visualization and metrics collection.

        Args:
            duration: Total simulation time
            time_step: Physics time step
            metrics: MetricsCollector instance to collect data each frame
        """
        def callback(time, step):
            if metrics and metrics.should_collect(time):
                metrics.collect_frame(time)

        self.run(duration, time_step, callback)

    def close(self):
        """Close visualization window."""
        self.vis.Quit()


class VisualizationConfig:
    """Configuration for visualization settings."""

    def __init__(self):
        self.enabled = True
        self.window_width = 1200
        self.window_height = 800
        self.follow_camera = True
        self.camera_distance = 2.0
        self.camera_height = 1.5
        self.background_color = (135, 206, 235)  # sky blue
        self.show_grid = True
        self.show_aabb = False


def create_visualizer_for_scenario(system, title="PyChronoRover", follow_body=None):
    """Factory function to create visualizer with standard settings.

    Args:
        system: Physics system
        title: Window title
        follow_body: Body to follow

    Returns:
        Visualizer instance or None if Irrlicht not available
    """
    if not IRRLICHT_AVAILABLE:
        print("Warning: Irrlicht visualization not available")
        return None

    return Visualizer(system, title=title, follow_body=follow_body)
