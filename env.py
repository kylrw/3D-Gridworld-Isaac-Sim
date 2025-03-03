import omni
import omni.kit
import numpy as np
import carb
import time
from pxr import Usd, UsdGeom, Gf, Sdf

import omni.isaac.core.utils.prims as prims
import omni.isaac.core.utils.stage as stage_utils
import omni.isaac.core.utils.transforms as tf_utils
import omni.isaac.core.utils.extensions as ext_utils

from omni.isaac.core import World
from omni.isaac.core.utils.nucleus import get_assets_root

# Number of rooms and robots
X_ROOMS = 5  # Number of rooms
Y_MAP_SIZE = 10  # Map size (YxY)
N_ROBOTS = 2  # Number of robots

# Isaac Sim Asset Paths
ROBOT_USD_PATH = "/Isaac/Robots/Carter/carter.usd"  # Change based on robot used

def create_room(x, y, room_size=2.0):
    """Generate a single room at (x, y) coordinates."""
    room_prim_path = f"/World/Room_{x}_{y}"
    prims.create_prim(
        room_prim_path,
        "Cube",
        position=(x * room_size, y * room_size, 0),
        scale=(room_size, room_size, 1),
        attributes={"color": (0.6, 0.6, 0.6)}
    )

def create_environment(num_rooms, map_size):
    """Create a procedurally generated environment with num_rooms in a YxY grid."""
    stage_utils.set_stage_up_axis("z")  # Ensure correct gravity direction
    stage_utils.set_stage_meters_per_unit(1.0)

    ground_plane = prims.create_prim("/World/Ground", "Plane", position=(0, 0, 0))
    
    # Create rooms
    for _ in range(num_rooms):
        x, y = np.random.randint(0, map_size, size=2)
        create_room(x, y)

def spawn_robot(robot_id, position):
    """Spawn a robot with a unique namespace."""
    robot_path = f"/World/Robot_{robot_id}"
    prims.create_prim(
        robot_path,
        "Xform",
        position=position
    )
    prims.create_prim(
        f"{robot_path}/Carter",
        "Reference",
        usd_path=ROBOT_USD_PATH,
        position=position
    )

def launch_isaac_sim():
    """Launch Isaac Sim and create the simulation world."""
    # Start Isaac Sim
    ext_utils.enable_extension("omni.isaac.ros2_bridge")
    
    world = World(stage_units_in_meters=1.0)
    
    create_environment(X_ROOMS, Y_MAP_SIZE)

    # Spawn multiple robots
    for i in range(N_ROBOTS):
        spawn_robot(i, position=(np.random.uniform(-5, 5), np.random.uniform(-5, 5), 0))

    world.reset()

    return world

if __name__ == "__main__":
    # Start the simulation
    world = launch_isaac_sim()
    
    while world.is_running():
        world.step(render=True)
