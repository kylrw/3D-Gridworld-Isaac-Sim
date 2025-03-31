import subprocess
import os
import argparse
import sys
import cv2
import numpy as np
import yaml
import subprocess
from typing import Optional

def run_ros_command(command, log_file):
    """Run a ROS 2 command in a separate process"""
    with open(log_file, "w") as f:
        return subprocess.Popen(command, shell=True, stdout=f, stderr=f, executable="/bin/bash")
    

def resize_image(img: np.ndarray, size: int) -> np.ndarray:
    """
    Resize an image to a square of size x size.
    """
    unknown_val = 205 # Grey

    # Isolate the explored area
    # Find the black pixel closest to the top left corner
    top_left = np.argwhere(img == 0).min(axis=0)
    # Find the black pixel closest to the bottom right corner
    bottom_right = np.argwhere(img == 0).max(axis=0)

    cropped = img[top_left[0]:bottom_right[0] + 1, top_left[1]:bottom_right[1] + 1]

    # # If the map is smaller than the desired size, add padding (unexplored cells)
    # if cropped.shape[0] < size or cropped.shape[1] < size:
    #     print(f"Warning: Map size ({cropped.shape}) is smaller than the desired size ({size}). Padding...")
    #     padded = np.full((size, size), unknown_val, dtype=np.uint8)
    #     if cropped.shape[0] < size:
    #         top_pad = (size - cropped.shape[0]) // 2
    #         bottom_pad = size - cropped.shape[0] - top_pad
    #         cropped = np.pad(cropped, ((top_pad, bottom_pad), (0, 0)), mode='constant', constant_values=unknown_val)
    #     if cropped.shape[1] < size:
    #         left_pad = (size - cropped.shape[1]) // 2
    #         right_pad = size - cropped.shape[1] - left_pad
    #         cropped = np.pad(cropped, ((0, 0), (left_pad, right_pad)), mode='constant', constant_values=unknown_val)

    # # If the map is larger, throw a warning and crop it
    # if cropped.shape[0] > size or cropped.shape[1] > size:
    #     #print(f"Warning: Map size ({cropped.shape}) is larger than the desired size ({size}). Cropping...")
    #     cropped = cropped[:size, :size]
    
    h, w = cropped.shape
    if h > size or w > size:
        print(f"Warning: Cropped map size ({h}, {w}) is larger than desired size ({size}). Cropping...")
        cropped = cropped[:size, :size]
        h, w = cropped.shape

    # Create empty grey canvas and place cropped image in top-left
    padded = np.full((size, size), unknown_val, dtype=np.uint8)
    padded[:h, :w] = cropped

    return padded
    
def to_occupancy_map(img: np.ndarray, size: int = 30) -> np.ndarray:
    """
    Converts a slam map image to an occupancy map.
    Occupied cells are black (0) and safe (free/unknown) are white (255).
    """
    
    # Resize the image
    cropped = resize_image(img, size)

    # Save the cropped image
    occupancy_binary = np.where(cropped == 0, 0, 255).astype(np.uint8) # Occupied cells are black (0) and safe (free/unknown) are white (255)

    return occupancy_binary

def to_exploration_map(img: np.ndarray, size: int = 30) -> np.ndarray:
    """
    Converts a map image to an exploration map.
    Explored areas (free or occupied) are white (255) and unexplored (unknown) are black (0).
    """
    
    # Resize the image
    cropped = resize_image(img, size)

    # Save the cropped image
    exploration_binary = np.where(cropped == 205, 0, 255).astype(np.uint8) # Explored areas (free or occupied) are white (255) and unexplored (unknown) are black (0)

    return exploration_binary

def to_single_pose_map(x: int, y: int, size: Optional[int] = 30) -> None:
    """
    Converts a map image to an single pose map.
    0 for every pixel except the given position, which is 255.
    """
    
    x += 4 # Offset based on starting position in Isaac Sim
    y += 4

    map = np.zeros((size, size), dtype=np.uint8) 
    map[y, x] = 255

    return map

def to_multi_pose_map(coordinates: list[tuple[int, int]], size) -> None:
    """
    Converts a map image to a rescuer pose map.
    0 for every pixel except the rescuer's position, which is 255.
    """
    map = np.zeros((size, size), dtype=np.uint8)
    for x, y in coordinates:
        x += 4 # Offset based on starting position in Isaac Sim
        y += 4
        map[y, x] = 255
    
    return map

def getPose(namespace: str = "robot1") -> tuple[int, int]:
    """
    Extracts the channels from the robot's map and saves them as images.
    """
    
    try:
        # Load the latest position of the robot
        with open(f"logs/{namespace}/pose.txt", "r") as f:
            lines = f.readlines()
            if not lines:
                raise FileNotFoundError(f"No position data found for {namespace}")
            # Get the latest position
            y, x = lines[-1].strip().split(",") # The coordinate system is flipped in nav2 map vs the image
            x, y = int(x), int(y)
    except FileNotFoundError:
        print(f"No position data found for {namespace}")
        x, y = 0, 0
    
    return x, y

def getMap(namespace: str = "robot1", size: int = 30) -> np.ndarray:
    """
    Extracts the channels from the robot's map and saves them as images.
    """
    # Check if the logs directory exists
    if not os.path.exists("maps"):
        os.makedirs("maps")
    if not os.path.exists("logs"):
        os.makedirs("logs")
    if not os.path.exists(f"maps/{namespace}"):
        os.makedirs(f"maps/{namespace}")

    # Extract the map
    extract_map_command = f"ros2 run nav2_map_server map_saver_cli -f maps/{namespace}/map --ros-args -r __ns:=/{namespace}"
    extract_map_process = run_ros_command(extract_map_command, f"logs/{namespace}/map_log.txt")
    extract_map_process.wait()

    # Load YAML file to get map metadata
    yaml_path = f"maps/{namespace}/map.yaml"
    with open(yaml_path, 'r') as f:
        map_info = yaml.safe_load(f)

    # Construct the full path to the pgm file
    pgm_path = os.path.join(os.path.dirname(yaml_path), map_info["image"])

    # Load the map image
    img = cv2.imread(pgm_path, cv2.IMREAD_UNCHANGED)

    # Rotate the image +270 degrees to match orientation of numpy array
    img = np.rot90(img, k=3)

    return img

"""
Isolates a local map of 7x7 centered around the robot's position.
"""
def isolateLocalMap(x: int, y: int, img: np.ndarray) -> np.ndarray:
    """
    Isolates a local map of 7x7 centered around the robot's position.
    """

    # Get the local map 
    x += 4 # Offset based on starting position in Isaac Sim
    y += 4

    # Find corners of the local map within bounds
    x1 = max(0, x - 3)
    x2 = min(img.shape[1], x + 4)
    y1 = max(0, y - 3)
    y2 = min(img.shape[0], y + 4)

    local_map = img[y1:y2, x1:x2]

    # Pad the local map if it is smaller than 7x7 ensuring the robot is centered
    if local_map.shape[0] < 7:
        top_pad = (7 - local_map.shape[0]) // 2
        bottom_pad = 7 - local_map.shape[0] - top_pad
        local_map = np.pad(local_map, ((top_pad, bottom_pad), (0, 0)), mode='constant', constant_values=205)
    if local_map.shape[1] < 7:
        left_pad = (7 - local_map.shape[1]) // 2
        right_pad = 7 - local_map.shape[1] - left_pad
        local_map = np.pad(local_map, ((0, 0), (left_pad, right_pad)), mode='constant', constant_values=205)

    return local_map

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run channel processing for a robot.")
    parser.add_argument("--size", type=int, default=30, help="The size of the map to process.")
    args = parser.parse_args()

    namespaces = ["robot1", "robot2", "robot3"]
    explorers = ["robot1", "robot2"]
    rescuers = ["robot3"]
    robot_positions = {}

    # Local maps
    for namespace in namespaces:

        print("Processing maps for", namespace)

        # Check if the directory exists
        if not os.path.exists(f"channels/{namespace}"):
            os.makedirs(f"channels/{namespace}")

        # Add the currnet directory to the python path
        sys.path.append(os.getcwd())

        # Get the robot's pose
        x, y = getPose(namespace)
        robot_positions[namespace] = (x, y)

        # Load the map image
        img = getMap(namespace, args.size)

        # Save the raw map image
        cv2.imwrite(f"channels/{namespace}/raw_map.png", img)

        # Convert the map image to an occupancy map
        occupancy_map = to_occupancy_map(img)
        cv2.imwrite(f"channels/{namespace}/occupancy_map.png", occupancy_map)

        local_occupancy_map = isolateLocalMap(x, y, occupancy_map)
        cv2.imwrite(f"channels/{namespace}/local_occupancy_map.png", local_occupancy_map)
        
        # Convert the map image to an exploration map
        exploration_map = to_exploration_map(img)
        cv2.imwrite(f"channels/{namespace}/exploration_map.png", exploration_map)

        local_exploration_map = isolateLocalMap(x, y, exploration_map)
        cv2.imwrite(f"channels/{namespace}/local_exploration_map.png", local_exploration_map)

        # To ego pose map
        ego_pose_map = to_single_pose_map(x, y, args.size)
        cv2.imwrite(f"channels/{namespace}/ego_pose_map.png", ego_pose_map)

        # Check if robot has found the target
        # Load the target map and get the target's position
        target_map = cv2.imread("channels/global/target_map.png", cv2.IMREAD_GRAYSCALE)
        target_y, target_x = np.argwhere(target_map == 255)[0]

        # Check if the target is in the robot's exploration map
        if exploration_map[target_y, target_x] == 255:
            print(f"{namespace} has found the target.")

        # Check if the robot is at the target
        if abs(x - target_x) <= 3 and abs(y - target_y) <= 3:
            print(f"{namespace} is at the target.")

        print("Processed maps for", namespace)
    print()


    #Global maps

    # Check if the directory exists
    if not os.path.exists(f"channels/global"):
        os.makedirs(f"channels/global")

    all_positions = robot_positions.values()
    explorer_positions = [robot_positions[namespace] for namespace in explorers]
    rescuer_positions = [robot_positions[namespace] for namespace in rescuers]

    all_robots_map = to_multi_pose_map(all_positions, args.size)
    cv2.imwrite(f"channels/global/all_robots_map.png", all_robots_map)

    explorer_map = to_multi_pose_map(explorer_positions, args.size)
    cv2.imwrite(f"channels/global/explorer_map.png", explorer_map)

    rescuer_map = to_multi_pose_map(rescuer_positions, args.size)
    cv2.imwrite(f"channels/global/rescuer_map.png", rescuer_map)

    print("Processed global maps")




    