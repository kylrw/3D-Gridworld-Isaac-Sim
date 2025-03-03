import argparse
from isaacsim import SimulationApp

# Parse command-line arguments
parser = argparse.ArgumentParser(description="Load different USD scenes in Isaac Sim.")
parser.add_argument(
    "--scene", 
    type=str, 
    default="omniverse://localhost/Projects/zero-to-slam/scene.usd", 
    help="Path to the USD scene file"
)
args = parser.parse_args()

# Configuration for simulation
CONFIG = {
    "width": 1280,
    "height": 720,
    "sync_loads": True,
    "headless": False,
    "renderer": "RaytracedLighting",
}

print("Starting Isaac Sim...")

# Start the Omniverse application
simulation = SimulationApp(launch_config=CONFIG)

import carb
import omni

print("Launching ROS2 Bridge...")

from isaacsim.core.utils.extensions import enable_extension
enable_extension("isaacsim.ros2.bridge")

# Wait two frames so that ROS2 bridge starts loading
simulation.update()
simulation.update()

# Load the specified USD scene
usd_path = args.scene
print(f"Loading scene: {usd_path}")
omni.usd.get_context().open_stage(usd_path)

# Wait two frames so that stage starts loading
simulation.update()
simulation.update()

print("Loading stage...")
from isaacsim.core.utils.stage import is_stage_loading

while is_stage_loading():
    simulation.update()
print("Loading Complete")
omni.timeline.get_timeline_interface().play()

while simulation.is_running():
    # Run in real-time mode
    simulation.update()

omni.timeline.get_timeline_interface().stop()
simulation.close()
