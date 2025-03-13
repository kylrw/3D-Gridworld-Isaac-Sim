import subprocess
import os
import argparse
import sys

def channelProcessing(namespace: str = "robot1", size: int = 30) -> None:
    """
    Extracts the channels from the robot's map and saves them as images.
    """

    # Launches a pose subscriber node for the robot namespace
    pose_subscriber_command = f"python3 channelProcessingUtils/pose_subscriber.py --namespace {namespace}"
    pose_subscriber_process = subprocess.Popen(pose_subscriber_command, shell=True, executable="/bin/bash")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run channel processing for a robot.")
    parser.add_argument("--namespace", type=str, default="robot1", help="The namespace of the robot to process.")
    parser.add_argument("--size", type=int, default=30, help="The size of the map to process.")
    args = parser.parse_args()

    # Add the currnet directory to the python path
    sys.path.append(os.getcwd())


    channelProcessing(args.namespace, args.size)