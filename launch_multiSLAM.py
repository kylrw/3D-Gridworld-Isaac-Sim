import subprocess
import time
import os
import signal
import argparse

### Initialize Environment ###

# Set paths
HOME = os.path.expanduser("~")
ISAAC_SIM_PATH = os.path.join(HOME, "isaacsim")
ISAAC_SIM_SCRIPT = os.path.join(ISAAC_SIM_PATH, "isaac-sim.sh")  # Isaac Sim launcher

# ROS 2 Bridge Library Path
ROS2_BRIDGE_PATH = os.path.join(ISAAC_SIM_PATH, "exts/omni.isaac.ros2_bridge/humble/lib")

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

def run_ros_command(command, directory, log_file):
    """Run a ROS 2 command in a separate process"""

    # Check if the directory exists
    if not os.path.exists(f"{directory}"):
        os.makedirs(f"{directory}")

    log_file = f"{directory}/{log_file}"

    with open(log_file, "w") as f:
        return subprocess.Popen(
            command,
            shell=True,
            stdout=f,
            stderr=f,
            executable="/bin/bash",
            preexec_fn=os.setsid,  # Creates a new process group
        )

def send_nav_goal(robot_id, x, y, theta=0.0):
    """Send a navigation goal to a specific robot."""

    namespace = f"/{robot_id}"
    command = f"""
    ros2 action send_goal {namespace}/navigate_to_pose nav2_msgs/action/NavigateToPose "{{
      'pose': {{
        'header': {{
          'frame_id': 'map',
          'stamp': {{ 'sec': 0, 'nanosec': 0 }}
        }},
        'pose': {{
          'position': {{ 'x': {x}, 'y': {y}, 'z': 0.0 }},
          'orientation': {{ 'x': 0.0, 'y': 0.0, 'z': {theta}, 'w': 1.0 }}
        }}
      }}
    }}"
    """
    subprocess.run(command, shell=True, executable="/bin/bash")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run multi SLAM with optional debug mode.")
    parser.add_argument("--debug", action="store_true", help="Enable debug mode")
    parser.add_argument("--amount", type=str, default="3", help="Amount of robots to use")
    args = parser.parse_args()
    if args.debug: print("In Debug mode.")

    # Robot IDs
    robot_ids = []
    for i in range(int(args.amount)):
        robot_ids.append(f"robot{i+1}")

    try:

      # Source ROS2
      subprocess.run("source /opt/ros/humble/setup.bash", shell=True, executable="/bin/bash")
      subprocess.run("source ~/ros2_ws/install/setup.bash", shell=True, executable="/bin/bash")

      # Launch Isaac Sim
      print("Launching Isaac Sim with ROS 2 bridge...")
      isaac_sim = launch_isaac_sim()
      time.sleep(20)  # Wait for Isaac Sim to stabilize

      # Launch SLAM Toolbox for each robot
      print("Launching SLAM components ...")
      # slam1_command = "ros2 launch slam_toolbox online_async_multirobot_launch.py namespace:=robot1 use_sim_time:=True"
      # slam2_command = "ros2 launch slam_toolbox online_async_multirobot_launch.py namespace:=robot2 use_sim_time:=True"
      # slam3_command = "ros2 launch slam_toolbox online_async_multirobot_launch.py namespace:=robot3 use_sim_time:=True"
      # slam1_process = run_ros_command(slam1_command, "logs/slam1_log.txt")
      # slam2_process = run_ros_command(slam2_command, "logs/slam2_log.txt")
      # slam3_process = run_ros_command(slam3_command, "logs/slam3_log.txt")

      slam_processes = []
      for id in robot_ids:
          slam_command = f"ros2 launch slam_toolbox online_async_multirobot_launch.py namespace:={id} use_sim_time:=True"
          slam_process = run_ros_command(slam_command, f"logs/{id}", "slam_log.txt")
          slam_processes.append(slam_process)

      time.sleep(5)  

      # Launch Navigation for each robot
      print("Launching Nav components ...")
      if args.debug:
        nav_command = "ros2 launch turtle_navigation multiple_robot_turtle_navigation.launch.py"
      else:
        nav_command = "ros2 launch turtle_navigation multiple_robot_turtle_navigation.launch.py use_rviz:=false"
      nav_process = run_ros_command(nav_command, "logs", "nav_log.txt")

      print("Simulation is fully running! Ready to send navigation goals.")

      #map cmd: ros2 run nav2_map_server map_saver_cli -f robot1_map --ros-args -r __ns:=/robot1

      # Launch Channel Processing for each robot
      print("Launching Channel Processing ...")
      channel_processes = []
      for id in robot_ids:
          channel_processing_command = f"python3 channelProcessingUtils/channelProcessing.py --namespace {id}"
          channel_processing_process = run_ros_command(channel_processing_command, f"logs/{id}", "channel_processing.txt")
          channel_processes.append(channel_processing_process)

      

      while True:
          robot_id = input("Enter robot ID (robot1 or robot2): ").strip()
          x = float(input("Enter goal X coordinate: "))
          y = float(input("Enter goal Y coordinate: "))
          send_nav_goal(robot_id, x, y)

    except Exception as e:
        print(e)
    finally:
        print("Shutting down processes...")
        processes = [isaac_sim, nav_process] + slam_processes + channel_processes
        for proc in processes:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            except Exception as ex:
                print(f"Could not kill process group for pid {proc.pid}: {ex}")
        # slam1_process.terminate()
        # slam2_process.terminate()
        # slam3_process.terminate()
        # nav_process.terminate()
        # os.killpg(os.getpgid(slam1_process.pid), signal.SIGTERM)
        # os.killpg(os.getpgid(slam2_process.pid), signal.SIGTERM)
        # os.killpg(os.getpgid(slam3_process.pid), signal.SIGTERM)
        # os.killpg(os.getpgid(nav_process.pid), signal.SIGTERM)
        os.killpg(os.getpgid(isaac_sim.pid), signal.SIGTERM)


        