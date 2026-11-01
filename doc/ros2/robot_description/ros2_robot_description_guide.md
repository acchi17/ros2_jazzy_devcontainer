# ROS 2 Robot Description

## Overview

A **robot description** is a file that describes the physical structure of a robot: its parts, how they are connected, and how they move. In ROS 2 it is written in **URDF** (Unified Robot Description Format), an XML format. **Xacro** is an XML macro language that makes URDF shorter and reusable.

Other ROS 2 components use the description for:

| Consumer | Purpose |
|---|---|
| `robot_state_publisher` | Publishes TF (coordinate frames) from joint states |
| RViz2 | Displays the robot model |
| Gazebo | Simulates the robot (visual, collision, inertia) |
| `ros2_control` | Defines which joints are controlled and which hardware drives them |
| Nav2 / MoveIt | Uses frames and geometry for navigation and planning |

The description is distributed as a string on the `robot_description` parameter of `robot_state_publisher`.

```
xacro file --(xacro)--> URDF string --> robot_state_publisher --> /tf, /robot_description
```

## Structure of a Robot Description

A robot is a **tree** of links connected by joints.

```
base_footprint
   └─(fixed)─ base_link
                 ├─(fixed)───── chassis_link
                 ├─(continuous) rear_wheel_link
                 └─(revolute)── front_steering_link
```

### `<link>`: a rigid body

A link has three optional parts.

| Element | Meaning | Used by |
|---|---|---|
| `<visual>` | Shape and color for display | RViz2, Gazebo |
| `<collision>` | Simplified shape for contact checking | Gazebo, Nav2 |
| `<inertial>` | Mass and inertia matrix | Gazebo (physics) |

```xml
<link name="chassis_link">
  <visual>
    <geometry><box size="0.4 0.18 0.08"/></geometry>
  </visual>
</link>
```

A link with no content (e.g. `base_link`) is valid. It works as a pure coordinate frame.

### `<joint>`: a connection between two links

A joint defines a `parent`, a `child`, and the child's pose relative to the parent (`origin`).

```xml
<joint name="steering_joint" type="revolute">
  <parent link="base_link"/>
  <child link="front_steering_link"/>
  <origin xyz="0.26 0 0" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-0.5" upper="0.5" effort="1.0" velocity="5.0"/>
</joint>
```

Main joint types:

| Type | Motion | Needs `<limit>` |
|---|---|---|
| `fixed` | None | No |
| `continuous` | Unlimited rotation (wheels) | `effort`, `velocity` |
| `revolute` | Rotation within a range | `lower`, `upper`, `effort`, `velocity` |
| `prismatic` | Linear motion within a range | `lower`, `upper`, `effort`, `velocity` |

### Conventions

- Units are SI: meters, radians, kilograms.
- Axes follow REP 103: x forward, y left, z up.
- `rpy` is roll-pitch-yaw in radians; `xyz` is in meters.
- Each link has exactly one parent joint, and the tree has exactly one root.
- Common frames: `base_link` (robot reference point) and `base_footprint` (its projection on the ground).

### `<ros2_control>`: hardware interface (optional)

This tag declares which joints `ros2_control` commands or reads, and which hardware plugin is used.

```xml
<ros2_control name="RcVehicleSystem" type="system">
  <hardware><plugin>rc_driver/RcDriverHardware</plugin></hardware>
  <joint name="traction_joint">
    <command_interface name="velocity"/>
    <state_interface name="velocity"/>
  </joint>
</ros2_control>
```

## How to Write a Robot Description

### 1. Create a description package

Keep the description in its own package (e.g. `<robot>_description`) so that other packages can share it.

```
rc_vehicle_description/
├── CMakeLists.txt
├── package.xml
├── urdf/rc_vehicle.urdf.xacro
└── ros2_control/rc_vehicle.ros2_control.xacro
```

Install the directories in `CMakeLists.txt`:

```cmake
install(DIRECTORY urdf ros2_control DESTINATION share/${PROJECT_NAME})
```

### 2. Write the links and joints

Start from `base_link` and add parts one by one. For each part, decide:

1. Which link is the parent?
2. Where is the child frame relative to the parent (`origin`)?
3. How does it move (`type`, `axis`, `limit`)?

Start with `<visual>` only; add `<collision>` and `<inertial>` when needed for simulation.

### 3. Use Xacro to avoid repetition

```xml
<robot name="rc_vehicle" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Argument: can be overridden from the command line or a launch file -->
  <xacro:arg name="wheelbase" default="0.26"/>
  <!-- Property: constant used in ${...} expressions -->
  <xacro:property name="wheelbase" value="$(arg wheelbase)"/>

  <!-- Macro: reusable block -->
  <xacro:macro name="wheel_link" params="name radius">
    <link name="${name}">
      <visual>
        <geometry><cylinder radius="${radius}" length="0.04"/></geometry>
      </visual>
    </link>
  </xacro:macro>

  <xacro:wheel_link name="rear_wheel_link" radius="0.05"/>
  <xacro:wheel_link name="front_wheel_link" radius="0.05"/>
</robot>
```

| Feature | Syntax |
|---|---|
| Argument | `<xacro:arg name="x" default="1"/>` and `$(arg x)` |
| Property | `<xacro:property name="x" value="1"/>` and `${x}` |
| Math | `${pi/2}`, `${wheelbase / 2.0}` |
| Macro | `<xacro:macro name="m" params="a b">` and `<xacro:m a="1" b="2"/>` |
| Include | `<xacro:include filename="$(find pkg)/file.xacro"/>` |

### 4. Check the result

Run these inside the container:

```bash
# Expand xacro into URDF
xacro rc_vehicle.urdf.xacro wheelbase:=0.3 > /tmp/rc_vehicle.urdf

# Validate the URDF and print the link/joint tree
check_urdf /tmp/rc_vehicle.urdf

# Draw the tree as a PDF
urdf_to_graphviz /tmp/rc_vehicle.urdf
```

### Tips

- Measure real dimensions and masses; wrong inertia makes simulation unstable.
- Keep values shared with controller parameters (e.g. `wheelbase`, `wheel_radius`) consistent.
- Use simple shapes (box, cylinder) for `<collision>`; avoid detailed meshes.
- See [rc_vehicle.urdf.xacro](../../../ros2_ws/src/rc_vehicle_description/urdf/rc_vehicle.urdf.xacro) for a working example in this repository.

## How to load a Robot Description

ROS 2 does not read the description file directly. A **launch file** expands the file into a URDF string and passes it to `robot_state_publisher` through the `robot_description` parameter.

```
launch file
  ├─ Command: run `xacro <file> <args>`  -> URDF string
  └─ Node: robot_state_publisher
        parameters: {'robot_description': <URDF string>}
              ├─> publishes the /robot_description topic
              └─> publishes /tf and /tf_static (using /joint_states)
```

### Launch file example

```python
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 1. Expand the xacro file into a URDF string
    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ',
            PathJoinSubstitution([
                FindPackageShare('rc_vehicle_description'),
                'urdf', 'rc_vehicle.urdf.xacro',
            ]),
            ' use_mock_hardware:=true',  # xacro arguments
        ]),
        value_type=str,
    )

    # 2. Pass the string to robot_state_publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
    )

    return LaunchDescription([robot_state_publisher])
```

Notes:

- `ParameterValue(..., value_type=str)` is required. Without it, ROS 2 may try to parse the URDF string as YAML and fail.
- Xacro arguments are appended to the command as `name:=value`. Use a `LaunchConfiguration` instead of a fixed value to set them from the command line.
- When running with Gazebo, add `use_sim_time` to the same `parameters` list.

### Check the result

```bash
ros2 launch <package_name> <launch_file>.py

# The description is published as a topic
ros2 topic echo /robot_description --once

# The frames are published as TF
ros2 run tf2_tools view_frames
```

To display the model, add a `RobotModel` display in RViz2 and set its description topic to `/robot_description`.

### Optional: start controller_manager with the description

When the description contains a `<ros2_control>` tag, `controller_manager` must be launched as well, and it also needs the description. `controller_manager` does not start by itself. It runs as a node (the `ros2_control_node` executable in the `controller_manager` package), so the launch file has to include it. The node uses the `<ros2_control>` tag to load the hardware plugin and to create the command/state interfaces.

There are two ways to give it the description:

| Way | How |
|---|---|
| Parameter | Pass the same `robot_description` string in the node's `parameters` |
| Topic | Start it without the parameter; it subscribes to `robot_description` published by `robot_state_publisher` |

The parameter way is explicit and does not depend on the startup order of nodes. Add the following to the launch file above (the `robot_description` variable is reused):

```python
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

controllers_yaml = PathJoinSubstitution([
    FindPackageShare('rc_vehicle_bringup'), 'config', 'controllers.yaml',
])

control_node = Node(
    package='controller_manager',
    executable='ros2_control_node',
    parameters=[{'robot_description': robot_description}, controllers_yaml],
    output='screen',
)

# Load and activate controllers after controller_manager has started
joint_state_broadcaster_spawner = Node(
    package='controller_manager',
    executable='spawner',
    arguments=['joint_state_broadcaster'],
)

return LaunchDescription([
    robot_state_publisher,
    control_node,
    joint_state_broadcaster_spawner,
])
```

Notes:

- `controllers.yaml` defines the controller types and parameters. It is separate from the description.
- `joint_state_broadcaster_spawner` loads and activates the `joint_state_broadcaster` controller. Once active, the controller publishes `/joint_states`, which `robot_state_publisher` uses to update the TF of movable joints.
- Joint names in `controllers.yaml` must match the joint names in the description.
- The spawner waits for `controller_manager`, so it can be started at the same time as `control_node`.
- Simulation with Gazebo does not use `ros2_control_node`. The `gz_ros2_control` plugin starts `controller_manager` inside Gazebo instead.

Check that the hardware and controllers were loaded:

```bash
ros2 control list_hardware_components
ros2 control list_controllers
```
