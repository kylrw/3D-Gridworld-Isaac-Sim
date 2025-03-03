import subprocess
import time
import os

# Set paths
HOME = os.path.expanduser("~")
ISAAC_SIM_PATH = os.path.join(HOME, "isaacsim")
ISAAC_SIM_SCRIPT = os.path.join(ISAAC_SIM_PATH, "isaac-sim.sh")  # Isaac Sim launcher

# ROS 2 Bridge Library Path
ROS2_BRIDGE_PATH = os.path.join(ISAAC_SIM_PATH, "exts/omni.isaac.ros2_bridge/humble/lib")

def launch_isaac_sim():
    print("Launching Isaac Sim with ROS 2 bridge...")
    log_file = "logs/isaacsim_log.txt"
    with open(log_file, "w") as f:
        # Call load_isaacsim_stage python file
        return subprocess.Popen([
            os.path.join(ISAAC_SIM_PATH, "python.sh"), "isaacsim_utils/load_isaacsim_stage.py", "--scene", "omniverse://localhost/Projects/zero-to-slam/scene12.usd"],
            stdout=f, stderr=f, env=os.environ.copy())

def run_ros_command(command, log_file):
    """Run a ROS 2 command in a separate process"""
    with open(log_file, "w") as f:
        return subprocess.Popen(command, shell=True, stdout=f, stderr=f, executable="/bin/bash")

def send_nav_goal(robot_id, x, y, theta=0.0):
    """Send a navigation goal to a specific robot."""
    namespace = f"/{robot_id}"
    command = f"""
    ros2 action send_goal {namespace}/navigate_to_pose nav2_msgs/action/NavigateToPose "{{
      'pose': {{
        'header': {{
          'frame_id': '{namespace}/map',
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
    isaac_sim = launch_isaac_sim()
    time.sleep(20)  # Wait for Isaac Sim to stabilize

    print("Launching ROS 2 components for robot1 and robot2...")

    # Start RViz (can be the same for both)
    rviz_command = "ros2 run rviz2 rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz --ros-args --remap use_sim_time:=True"
    rviz_process = run_ros_command(rviz_command, "logs/rviz_log.txt")

    # Launch SLAM Toolbox for each robot
    slam1_command = "ros2 launch slam_toolbox online_async_multirobot_launch.py namespace:=robot1 use_sim_time:=True "
    slam2_command = "ros2 launch slam_toolbox online_async_multirobot_launch.py namespace:=robot2 use_sim_time:=True "
    slam1_process = run_ros_command(slam1_command, "logs/slam1_log.txt")
    slam2_process = run_ros_command(slam2_command, "logs/slam2_log.txt")

    # # Launch Navigation for each robot
    # TODO

    print("Simulation is fully running! Ready to send navigation goals.")

    try:
        while True:
            robot_id = input("Enter robot ID (robot1 or robot2): ").strip()
            x = float(input("Enter goal X coordinate: "))
            y = float(input("Enter goal Y coordinate: "))
            send_nav_goal(robot_id, x, y)
    except KeyboardInterrupt:
        print("Shutting down processes...")
        rviz_process.terminate()
        slam1_process.terminate()
        slam2_process.terminate()


        