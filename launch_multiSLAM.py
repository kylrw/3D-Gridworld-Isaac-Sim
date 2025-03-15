import subprocess
import time
import os
import signal
import argparse
import random
import cv2
import channelUtils.channel_processing as cproc
from isaacsimUtils.ros_utils import run_ros_command, send_nav_goal
import threading

"""
This is the main file for the multi-robot SLAM simulation for CATMiP.
It launches the Isaac Sim simulation, multi-robot SLAM, and navigation2.
It also closes the loop between Isaac Sim and CATMiP by:
- Extracting the channels from the robots and sending them to CATMiP
- Recieving navigation goals from CATMiP and sending them to the robots
"""

# Set paths
HOME = os.path.expanduser("~")
ISAAC_SIM_PATH = os.path.join(HOME, "isaacsim")

def launch_isaac_sim():
    """Launch Isaac Sim with ROS 2 bridge."""

    log_file = "logs/isaacsim_log.txt"
    with open(log_file, "w") as f:
        # Call load_isaacsim_stage python file
        return subprocess.Popen(
            [
              os.path.join(ISAAC_SIM_PATH, "python.sh"), 
              "isaacsimUtils/load_isaacsim_stage.py", 
              "--scene", 
              "omniverse://localhost/Projects/zero-to-slam/scene_turtle.usd"
            ],
            stdout=f, 
            stderr=f, 
            env=os.environ.copy(),
            preexec_fn=os.setsid,  # Creates a new process group
          )

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run multi SLAM with optional debug mode.")
    parser.add_argument("--debug", action="store_true", help="Enable debug mode")
    parser.add_argument("--amount", type=str, default="3", help="Amount of robots to use")
    args = parser.parse_args()
    if args.debug: print("In Debug mode.")

    # Robot namespaces
    robot_ids = []
    for i in range(int(args.amount)):
        robot_ids.append(f"robot{i+1}")

    ###--- Generate Target ---###

    # Choose a random target location for the robot
    ground_truth_occupancy_map = cv2.imread("groundTruth/occupancy_map30x30.png", cv2.IMREAD_GRAYSCALE)
    height, width = ground_truth_occupancy_map.shape

    while True:
        x = random.randint(0,width-1) - 4 # Undo the expected offset
        y = random.randint(0,height-1) - 4

        if ground_truth_occupancy_map[y, x] == 0 and x >= 10 and y >= 10:
            print(f"Chosen target: ({x}, {y})")
            target_map = cproc.to_single_pose_map(x, y)
            cv2.imwrite(f"channels/global/target_map.png", target_map)
            break
      
    
    ###--- Launch Processes ---###    

    try:
        # Source ROS2
        subprocess.run("source /opt/ros/humble/setup.bash", shell=True, executable="/bin/bash")
        subprocess.run("source ~/ros2_ws/install/setup.bash", shell=True, executable="/bin/bash")

        print("Launching Isaac Sim with ROS 2 bridge...")
        isaac_sim = launch_isaac_sim()
        time.sleep(15)  # Wait for Isaac Sim to stabilize

        print("Launching SLAM components ...")
        slam_processes = []
        for id in robot_ids:
            slam_command = f"ros2 launch slam_toolbox online_async_multirobot_launch.py namespace:={id} use_sim_time:=True"
            slam_process = run_ros_command(slam_command, f"logs/{id}", "slam_log.txt")
            slam_processes.append(slam_process)
        time.sleep(5) # Wait for SLAM to stabilize

        print("Launching Nav components ...")
        if args.debug: 
            nav_command = "ros2 launch turtle_navigation multiple_robot_turtle_navigation.launch.py"
        else:
            nav_command = "ros2 launch turtle_navigation multiple_robot_turtle_navigation.launch.py use_rviz:=false"
        nav_process = run_ros_command(nav_command, "logs", "nav_log.txt")

        print("Launching Pose Subscribers ...")
        pose_subscribers = []
        for id in robot_ids:
            pose_subscriber_command = f"python3 channelUtils/pose_subscriber.py --namespace {id}"
            pose_subscriber_process = run_ros_command(pose_subscriber_command, f"logs/{id}", "pose_subscriber.txt")
            pose_subscribers.append(pose_subscriber_process)

        print("Simulation is fully running! Ready to send navigation goals. CTRL C to close.")
        print()

        # Send navigation goals
        while True:
            nav_goals = []
            completion_events = []
            for robot_id in robot_ids:
                print("Input navigation goal for", robot_id)
                x = float(input("Enter goal X coordinate: "))
                y = float(input("Enter goal Y coordinate: "))
                event = threading.Event()
                nav_goals.append((robot_id, x, y, event))
                completion_events.append(event)

            for goal in nav_goals:
                send_nav_goal(goal[0], goal[1], goal[2], event=goal[3])
                print(f"Sent navigation goal to {goal[0]} at ({goal[1]}, {goal[2]})")
            
            # Wait for the goal to complete
            for event in completion_events:
                event.wait()
            print("All robots have reached their goals. Processing channels...")
            print()

            # Run channel processing
            channel_processing_command = "python3 channelUtils/channel_processing.py"
            subprocess.run(channel_processing_command, shell=True, executable="/bin/bash")

    except Exception as e:
        print(e)
    finally:
        print("Shutting down processes...")
        processes = [isaac_sim, nav_process] + slam_processes + pose_subscribers
        for proc in processes:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            except Exception as ex:
                print(f"Could not kill process group for pid {proc.pid}: {ex}")
        os.killpg(os.getpgid(isaac_sim.pid), signal.SIGTERM) # Make sure Isaac Sim is killed

        # Shutdown ROS2
        subprocess.run("ros2 daemon stop", shell=True, executable="/bin/bash")
        print("Processes terminated.")

        