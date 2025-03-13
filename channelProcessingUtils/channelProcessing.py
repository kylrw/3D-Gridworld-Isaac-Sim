import subprocess
import os
import argparse
import sys
import cv2
import numpy as np
import yaml
import subprocess

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

    # If the map is smaller than the desired size, add padding (unexplored cells)
    if cropped.shape[0] < size or cropped.shape[1] < size:
        print(f"Warning: Map size ({cropped.shape}) is smaller than the desired size ({size}). Padding...")
        padded = np.full((size, size), unknown_val, dtype=np.uint8)

        padded[0:cropped.shape[0], 0:cropped.shape[1]] = cropped
        cropped = padded

    # If the map is larger, throw a warning and crop it
    if cropped.shape[0] > size or cropped.shape[1] > size:
        print(f"Warning: Map size ({cropped.shape}) is larger than the desired size ({size}). Cropping...")
        cropped = cropped[:size, :size]

    return cropped
    
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


def to_target_pose_map(img: np.ndarray, x: int, y: int, dest: str) -> None:
    """
    Converts a map image to a target pose map.
    0 for every pixel except the target's position, which is 255.
    """
    target_pose = np.zeros_like(img)
    target_pose[y, x] = 255
    
    return target_pose


def to_ego_pose_map(x: int, y: int, size: int) -> None:
    """
    Converts a map image to an ego pose map.
    0 for every pixel except the ego's position, which is 255.
    """
    
    ego_pose = np.zeros((size, size), dtype=np.uint8)
    ego_pose[y, x] = 255

    return ego_pose

def to_rescuer_pose_map(img: np.ndarray, x: list[int], y: list[int], dest: str) -> None:
    """
    Converts a map image to a rescuer pose map.
    0 for every pixel except the rescuer's position, which is 255.
    """
    rescuer_pose = np.zeros_like(img)
    for x, y in zip(x, y):
        rescuer_pose[y, x] = 255
    
    return rescuer_pose

def to_explorer_pose_map(img: np.ndarray, x: list[int], y: list[int], dest: str) -> None:
    """
    Converts a map image to an explorer pose map.
    0 for every pixel except the explorer's position, which is 255.
    """
    explorer_pose = np.zeros_like(img)
    for x, y in zip(x, y):
        explorer_pose[y, x] = 255

    return explorer_pose

def to_goal_history_map(img: np.ndarray, x: int, y: int) -> np.ndarray:
    """
    Converts a map image to a goal history map.
    (what is this?)
    """
    pass

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
            x, y = lines[-1].strip().split(",")
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

    # Rotate the image +270 degrees
    img = np.rot90(img, k=3)

    return img


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run channel processing for a robot.")
    parser.add_argument("--namespace", type=str, default="robot1", help="The namespace of the robot to process.")
    parser.add_argument("--size", type=int, default=30, help="The size of the map to process.")
    args = parser.parse_args()

    namespace = args.namespace

    # Check if the directory exists
    if not os.path.exists(f"channels/{namespace}"):
        os.makedirs(f"channels/{namespace}")

    # Add the currnet directory to the python path
    sys.path.append(os.getcwd())

    x, y = getPose(namespace)

    # Load the map image
    img = getMap(namespace, args.size)

    # Save the raw map image
    cv2.imwrite(f"channels/{namespace}/raw_map.png", img)

    # Convert the map image to an occupancy map
    occupancy_map = to_occupancy_map(img)
    cv2.imwrite(f"channels/{namespace}/occupancy_map.png", occupancy_map)
    
    # Convert the map image to an exploration map
    exploration_map = to_exploration_map(img)
    cv2.imwrite(f"channels/{namespace}/exploration_map.png", exploration_map)

    # To ego pose map
    ego_pose_map = to_ego_pose_map(x, y, args.size)
    cv2.imwrite(f"channels/{namespace}/ego_pose_map.png", ego_pose_map)

    