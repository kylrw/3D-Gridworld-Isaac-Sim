import os
import cv2
import numpy as np
import yaml
import subprocess


def extract_channels(namespace: str = "robot1", size: int = 30) -> None:
    """
    Extracts the slam map and the robots position from the ROS2 map server. 
    Processes into 7 channels to be fed into the CATMiP model:
    1. Occupancy Binary: occupied cells are black (0) and safe (free/unknown) are white (255).
    2. Exploration Binary: explored areas (free or occupied) are white (255) and unexplored (unknown) are black (0).
    3. Target Pose: 0 for every pixel except the target's position, which is 255.
    4. Ego Pose: 0 for every pixel except the explorer robot's position, which is 255.
    5. Rescuer Pose: 0 for every pixel except the rescuer robot's position, which is 255.
    6. Explorer Pose: 0 for every pixel except the current robot's position, which is 255.
    7. Goal History: (what is this?)

    The function will each of the 7 channels as a 30x30 image stored in the /maps/namespace folder.
    """

    # --- Prepare directory for the map and logs ---
    if not os.path.exists("maps"):
        os.makedirs("maps")
    if not os.path.exists("logs"):
        os.makedirs("logs")
    if not os.path.exists(f"maps/{namespace}"):
        os.makedirs(f"maps/{namespace}")

    # --- Extract the robots' position ---
    # Pose subscriber is already running in the background, so we can just read the latest position from the file
    with open(f"{namespace}_path.txt", "r") as f:
        lines = f.readlines()
        if not lines:
            raise FileNotFoundError(f"No position data found for {namespace}")
        # Get the latest position
        x, y = lines[-1].strip().split(",")
        x, y = int(x), int(y)

    
    # to rescuer pose map: TODO
    # to explorer pose map: TODO
    # to goal history map: TODO

    # --- Extract the SLAM map ---
    # extract_map_command = f"ros2 run nav2_map_server map_saver_cli -f maps/{namespace}_map --ros-args -r __ns:=/{namespace}"

    # # Extract the map
    # extract_map_process = run_ros_command(extract_map_command, f"logs/{namespace}_map_log.txt")
    # extract_map_process.wait()

    # Load YAML file to get map metadata
    yaml_path = f"maps/{namespace}_map.yaml"
    with open(yaml_path, 'r') as f:
        map_info = yaml.safe_load(f)

    # Construct the full path to the pgm file
    pgm_path = os.path.join(os.path.dirname(yaml_path), map_info["image"])

    # Load the pgm image as a grayscale image
    img = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise FileNotFoundError(f"Could not load image from {pgm_path}")
    
    unknown_val = 205 # Grey value for unknown cells

    # --- Isolate the explored area ---
    # Define explored as any pixel that is not equal to the unknown value.
    if np.any(img != unknown_val):
        coords = np.argwhere(img != unknown_val)
        # Bounding box of the explored region
        y0, x0 = coords.min(axis=0)
        y1, x1 = coords.max(axis=0) + 1  # add 1 since slicing is exclusive at the end
        cropped = img[y0:y1, x0:x1]
    else:
        # If no pixel is explored, fall back to using the full image
        cropped = img.copy()

    # --- Scale the map to the proper size ---
    # If the map is smaller than the desired size, add padding (unexplored cells)
    if cropped.shape[0] < size or cropped.shape[1] < size:
        print(f"Warning: Map size ({cropped.shape}) is smaller than the desired size ({size}). Padding...")
        padded = np.full((size, size), unknown_val, dtype=np.uint8)

        start_row = size - cropped.shape[0] # Map starts at the bottom left
        padded[start_row:start_row + cropped.shape[0], :cropped.shape[1]] = cropped
        cropped = padded

    # If the map is larger, throw a warning and crop it
    if cropped.shape[0] > size or cropped.shape[1] > size:
        print(f"Warning: Map size ({cropped.shape}) is larger than the desired size ({size}). Cropping...")
        cropped = cropped[:size, :size]

    # Save the cropped image
    occupancy_binary = np.where(cropped == 0, 0, 255).astype(np.uint8)

    cv2.imwrite(f"maps/{namespace}_occupancy.png", occupancy_binary)

    to_ego_pose_map(img, x, y, f"maps/{namespace}/ego_pose.png")
    # to target pose map: TODO

extract_channels("robot1")