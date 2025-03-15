import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import argparse
import os

class PoseSubscriber(Node):
    def __init__(self, namespace: str):
        super().__init__('pose_subscriber')
        self.namespace = namespace
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            f"{namespace}/pose",
            self.pose_callback,
            10)
        self.subscription  # prevent unused variable warning

    def pose_callback(self, msg) -> None:
        x = round(msg.pose.pose.position.x)
        y = round(msg.pose.pose.position.y)

        self.get_logger().info(f"Robot Position: x={x}, y={y}")

        #Append latest robot position to a file
        with open(f"logs/{self.namespace}/pose.txt", "a") as f:
            f.write(f"{x},{y}\n")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Run a pose subscriber node. Outputs the robot's position to a text file.")
    parser.add_argument("--namespace", type=str, default="robot1", help="The namespace of the robot to subscribe to.")
    args = parser.parse_args()

    rclpy.init(args=None)

    node = PoseSubscriber(args.namespace)

    # Check if the logs directory exists
    if not os.path.exists("logs"):
        os.makedirs("logs")

    # Check if the robot's directory exists
    if not os.path.exists(f"logs/{args.namespace}"):
        os.makedirs(f"logs/{args.namespace}")

    # Clear the namespace_pose.txt file and add 0,0 as the starting position
    with open(f"logs/{args.namespace}/pose.txt", "w") as f:
        f.write("0,0\n")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()