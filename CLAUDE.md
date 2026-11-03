# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

This is a **ROS 2 Jazzy** study repository that uses a Dev Container (Docker) environment. It contains:
- A Dev Container configuration for running ROS 2 Jazzy inside Docker
- A ROS 2 workspace (`ros2_ws/`)
- Documentation (`doc/`) and reference materials (`note/`)

## ROS 2 Workspace Commands

All commands below are run inside the container from `~/work/ros2_ws/`:

```bash
# Source ROS 2 (already in .bashrc)
source /opt/ros/jazzy/setup.bash

# Build all packages
cd ~/work/ros2_ws
colcon build --symlink-install

# Source the workspace after building
source install/setup.bash

# Build a single package
colcon build --symlink-install --packages-select my_package

# Run tests for a package
colcon test --packages-select my_package
colcon test-result --verbose

# Run a specific test file
cd ~/work/ros2_ws
python3 -m pytest src/my_package/test/test_flake8.py -v
```

## TurtleBot3 Simulation with Nav2

```bash
# Set robot model (waffle_pi, waffle, or burger)
export TURTLEBOT3_MODEL=waffle_pi

# Launch full simulation (Gazebo + Nav2 + RViz2)
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False

# Teleoperation (in a second terminal)
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/cmd_vel_nav

# SLAM mapping (in a third terminal)
ros2 launch slam_toolbox online_async_launch.py \
  use_sim_time:=true \
  params_file:=/opt/ros/jazzy/share/turtlebot3_navigation2/param/waffle_pi/slam_params.yaml

# Save map
ros2 run nav2_map_server map_saver_cli -f ~/map

# Launch navigation with saved map
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=true map:=$HOME/map.yaml
```
