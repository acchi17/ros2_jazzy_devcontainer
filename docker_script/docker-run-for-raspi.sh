#!/usr/bin/env bash
set -e

# Assumes an image named IMAGE_NAME has already been pulled beforehand.
IMAGE_NAME=acchi17/ros2-jazzy-image-for-raspi:latest
# Parent directory of the directory from which this script was invoked.
PARENT_DIR="$(dirname "$(pwd)")"
# Runs in the background so the shell becomes usable immediately; the interactive
# `exec bash` re-sources ~/.bashrc, which already sources /opt/ros/jazzy/setup.bash.
LAUNCH_CMD='source /opt/ros/jazzy/setup.bash && cd ~/work/ros2_ws && colcon build --symlink-install && source install/setup.bash && ros2 launch rc_driver rc_driver.launch.py & exec bash'

docker run -it --rm \
  --network host \
  -e ROS_DOMAIN_ID=30 \
  --user vscode \
  -v "$PARENT_DIR":/home/vscode/work \
  -w /home/vscode/work \
  "$IMAGE_NAME" \
  bash -c "$LAUNCH_CMD"
