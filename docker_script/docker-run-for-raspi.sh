#!/usr/bin/env bash
set -e

# Assumes an image named IMAGE_NAME has already been pulled beforehand.
IMAGE_NAME=acchi17/ros2-jazzy-image-for-raspi:latest
# Parent directory of the directory from which this script was invoked.
PARENT_DIR="$(dirname "$(pwd)")"
# Runs in the background so the shell becomes usable immediately; the interactive
# `exec bash` re-sources ~/.bashrc, which already sources /opt/ros/jazzy/setup.bash.
# The Zenoh router (rmw_zenohd) is started as its own background job so it's
# up independently of the colcon build/launch sequence; the PC-side router
# connects out to it (see zenoh/pc_router_config.json5).
LAUNCH_CMD='source /opt/ros/jazzy/setup.bash && ros2 run rmw_zenoh_cpp rmw_zenohd & source /opt/ros/jazzy/setup.bash && cd ~/work/ros2_ws && colcon build --symlink-install && source install/setup.bash && ros2 launch rc_driver rc_driver.launch.py & exec bash'

docker run -it --rm \
  --network host \
  --device=/dev/gpiochip0 \
  --group-add "$(getent group gpio | cut -d: -f3)" \
  -e ROS_DOMAIN_ID=30 \
  -e RMW_IMPLEMENTATION=rmw_zenoh_cpp \
  -e ZENOH_ROUTER_CONFIG_URI=/home/ubuntu/work/zenoh/raspi_router_config.json5 \
  --user ubuntu \
  -v "$PARENT_DIR":/home/ubuntu/work \
  -w /home/ubuntu/work \
  "$IMAGE_NAME" \
  bash -c "$LAUNCH_CMD"
