## Multi-robot SLAM in Isaac Sim
# Overview

This project implements a simulation environment for multi-robot SLAM (Simultaneous Localization and Mapping) system in NVIDIA's Isaac Sim, and sets up the connection bridges to a macro control algorithm CATMiP.

The system enables multiple robots to navigate and map an environment simultaneously, with each robot building its own occupancy map and sharing to the others. The project uses ROS2 for communication and coordination between robots, while Isaac Sim provides the simulation environment.

Prerequisites
- Ubuntu with ROS2 Humble installed
- NVIDIA Isaac Sim (installed in ~/isaacsim)
- Python 3.10+
- OpenCV (cv2)
- ROS2 packages:
    - slam_toolbox ()
    - turtle_navigation ()
    - nav2
    - rviz2

# File Structure

```
multiSLAM/
├── .gitignore
├── launch_multiSLAM.py          # Main script to launch multi-robot SLAM
├── launch_singleSLAM.py         # Script to launch single-robot SLAM
├── README.md
├── channels/                    # Communication channels
│   ├── global/                  # Global channel configurations for CATMiP
│   ├── robot1/                  # Robot 1 local channels
│   ├── robot2/                  # Robot 2 local channels
│   └── robot3/                  # Robot 3 local channels
├── channelUtils/                # Utilities for channel management
│   ├── channel_processing.py    # Process channel data
│   ├── pose_subscriber.py       # Subscribe to pose data
├── groundTruth/                 # Ground truth data
│   └── occupancy_map30x30.png   # Reference occupancy map for Isaac Sim scene
├── isaacsimUtils/               # Isaac Sim utilities
│   ├── load_isaacsim_stage.py   # Script to load the Isaac Sim stage
│   └── ros_utils.py             # ROS utilities for Isaac Sim
├── logs/                        # Log files
│   ├── isaacsim_log.txt         # Isaac Sim logs
│   ├── nav_log.txt              # Navigation logs
│   ├── robot1/                  # Robot 1 specific logs
│   ├── robot2/                  # Robot 2 specific logs
│   └── robot3/                  # Robot 3 specific logs
└── maps/                        # Generated maps
    ├── robot1/                  # Robot 1 specific logs
    ├── robot2/                  # Robot 2 specific logs
    └── robot3/                  # Robot 3 specific logs
```


# Setup

1. Make sure you have ROS2 Humble installed and sourced:
    ```
    source /opt/ros/humble/setup.bash
    source ~/ros2_ws/install/setup.bash
    ```
2. Ensure Isaac Sim is installed at ~/isaacsim
3. Clone this repository to your workspace

# Running the Multi-Robot SLAM

To launch the multi-robot SLAM system:

```
python3 launch_multiSLAM.py 
```

Command-line options:
- ```--debug``` Enable debug mode (runs additional visualization for each robot in rviz2)

# How It Works
1. The script first launches Isaac Sim with the specified scene
2. It selects a random target location for the robots
3. It launches SLAM components for each robot
4. It launches navigation components for the robots
5. It starts pose subscribers to monitor robot positions
6. You'll be prompted to enter navigation goals for each robot
7. The system processes channel data after robots reach their goals
8. To stop the simulation, press CTRL+C

# Shutting Down

To safely shut down all components, press CTRL+C in the terminal running the script. The script will:

1. Stop all ROS2 processes
2. Kill the Isaac Sim process
3. Stop the ROS2 daemon