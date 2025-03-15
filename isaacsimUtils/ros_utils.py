import os
import subprocess
import threading

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

def send_nav_goal(robot_id, x, y, theta=0.0, event=None):
  """Send a navigation goal to a specific robot."""
  temp = x
  x = y
  y = temp

  def run_goal():
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

    log_file = f"logs/{robot_id}/nav_goal.txt"

    with open(log_file, "w") as f:
        process = subprocess.Popen(
            command,
            shell=True,
            stdout=f,
            stderr=f,
            executable="/bin/bash",
            preexec_fn=os.setsid,  # Creates a new process group
        )
        process.wait()

    # Check the status of the goal
    with open(log_file, "r") as f:
      output = f.read()
      if "SUCCEEDED" in output:
          print(f"Navigation goal for {robot_id} succeeded.")
      else:
          print(f"Navigation goal for {robot_id} failed.")
        
    if event:
       event.set()

  # Run the goal in a new thread
  threading.Thread(target=run_goal).start()