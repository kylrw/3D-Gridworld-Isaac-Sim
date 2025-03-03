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
            os.path.join(ISAAC_SIM_PATH, "python.sh"), "isaacsim_utils/load_isaacsim_stage.py"],
            stdout=f, stderr=f, env=os.environ.copy())

def run_ros_command(command, log_file):
    """Run a ROS 2 command in a separate process"""
    with open(log_file, "w") as f:
        return subprocess.Popen(command, shell=True, stdout=f, stderr=f, executable="/bin/bash")

def send_nav_goal(x, y, theta=0.0):
    """Send a navigation goal to Nav2"""
    command = f"""
    ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{{
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
    isaac_sim = launch_isaac_sim()
    time.sleep(20)  # Wait for Isaac Sim to stabilize

    print("Launching ROS 2 components...")

    # Launch ROS 2 Components
    rviz_command = "ros2 run rviz2 rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz --ros-args --remap use_sim_time:=True"
    slam_command = "ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True"
    nav_command = "ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True rolling_window:=True width:=10 height:=10"

    print("Launching Rviz2")
    rviz_process = run_ros_command(rviz_command, "logs/rviz_log.txt")
    time.sleep(2)
    print("Launching slam_toolbox")
    slam_process = run_ros_command(slam_command, "logs/slam_log.txt")
    time.sleep(2)
    print("Launching Nav2")
    nav_process = run_ros_command(nav_command, "logs/nav_log.txt")

    print("Simulation is fully running! Ready to send navigation goals.")

    try:
        while True:
            x = float(input("Enter goal X coordinate: "))
            y = float(input("Enter goal Y coordinate: "))
            send_nav_goal(x, y)
    except KeyboardInterrupt:
        print("Shutting down processes...")
        rviz_process.terminate()
        slam_process.terminate()
        nav_process.terminate()
        isaac_sim.terminate()