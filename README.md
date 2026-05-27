# my_delta_robot

ROS 2 package for a delta parallel robot. The package includes Cartesian trajectory planning, inverse kinematics, drawing path sequencing, RViz visualization, and Python user interfaces for sending drawing commands.

## Features

- `main_node`: receives line-segment motion commands, plans Cartesian trajectories, validates IK, and publishes `/joint_states`.
- `draw_node`: receives drawing commands and queues rectangle, triangle, or circle waypoints.
- `gui_user_interface_node.py`: Tkinter GUI for setting `vmax`, `amax`, and the target drawing action.
- `user_interface_node.py`: legacy terminal UI that publishes the same command topics as the GUI.
- Custom ROS 2 messages in `msg/`.
- URDF, RViz config, and launch files for visualization.

## Prerequisites

- ROS 2 Humble or compatible
- `colcon`
- `ament_cmake`
- `rclcpp`, `rclpy`, `sensor_msgs`, `geometry_msgs`, `robot_state_publisher`, `tf2_ros`, and `rviz2`
- Tkinter for the GUI (`python3-tk` on Ubuntu/Debian; usually bundled with Python on macOS)

## Build

From the workspace root, the directory that contains `src/`:

```bash
colcon build --symlink-install --packages-select my_delta_robot
source install/setup.bash
```

For zsh:

```bash
source install/setup.zsh
```

The helper script can also be used:

```bash
./build.zsh
source install/setup.zsh
```

## Run

Launch the robot model, RViz, `main_node`, and `draw_node`:

```bash
ros2 launch my_delta_robot display.launch.py
```

Run the GUI in another terminal:

```bash
source install/setup.bash
ros2 run my_delta_robot gui_user_interface_node.py
```

The terminal UI is still available:

```bash
source install/setup.bash
ros2 run my_delta_robot user_interface_node.py
```

## GUI Defaults

The GUI and terminal UI start with these motion limits:

| Field | Default |
|-------|---------|
| `vmax` | `5000.0` mm/s |
| `amax` | `100.0` mm/s^2 |

Both values must be numeric and positive before a drawing command can be sent.

## Communication

The user interface nodes reuse the existing topic-based command style.

| Publisher | Topic | Message | Receiver | Purpose |
|-----------|-------|---------|----------|---------|
| GUI or terminal UI | `/set_vmax_amax` | `my_delta_robot/msg/VmaxAmax` | `main_node` | Update velocity and acceleration limits |
| GUI or terminal UI | `/set_current_point` | `my_delta_robot/msg/Posicionxyz` | `draw_node` | Queue a drawing action |
| `draw_node` | `/input_ls_final` | `my_delta_robot/msg/LinearSpeedXYZ` | `main_node` | Send one Cartesian line segment |
| `main_node` | `/joint_states` | `sensor_msgs/msg/JointState` | `robot_state_publisher` / RViz | Visualize robot state |
| `main_node` | `/status_delta` | `std_msgs/msg/String` | `draw_node` | Notify that a segment is complete |

Drawing actions are encoded in the existing `Posicionxyz.type` field:

| GUI action | `type` | Behavior in `draw_node` |
|------------|--------|-------------------------|
| Rectangle | `6` | Queue rectangle/square path |
| Triangle | `7` | Queue triangle path |
| Circle | `8` | Queue approximated circle path |

No new custom message, service, or action is required for the GUI node.

## Example Topic Commands

Set the motion limits manually:

```bash
ros2 topic pub --once /set_vmax_amax my_delta_robot/msg/VmaxAmax \
  "{vmax: 5000.0, amax: 100.0}"
```

Draw a rectangle:

```bash
ros2 topic pub --once /set_current_point my_delta_robot/msg/Posicionxyz \
  "{x0: 0.0, y0: 0.0, z0: 0.0, type: 6}"
```

Draw a triangle:

```bash
ros2 topic pub --once /set_current_point my_delta_robot/msg/Posicionxyz \
  "{x0: 0.0, y0: 0.0, z0: 0.0, type: 7}"
```

Draw a circle:

```bash
ros2 topic pub --once /set_current_point my_delta_robot/msg/Posicionxyz \
  "{x0: 0.0, y0: 0.0, z0: 0.0, type: 8}"
```

Send a direct line segment to `main_node`:

```bash
ros2 topic pub --once /input_ls_final my_delta_robot/msg/LinearSpeedXYZ \
  "{xo: 0.0, yo: 0.0, zo: -375.0, xf: 0.0, yf: 0.0, zf: -490.0, gripper: 0}"
```

## `set_current_point` Type Codes

| Type | Meaning |
|------|---------|
| `-1` | Set current TCP position |
| `0` | Update circle reference point |
| `1` | Update rectangle/square reference point |
| `2` | Update triangle reference point |
| `3` | Set Z start offset |
| `4` | Set Z end offset |
| `5` | Set both Z offsets |
| `6` | Queue rectangle/square drawing path |
| `7` | Queue triangle drawing path |
| `8` | Queue circle drawing path |

## Tests

```bash
colcon build --symlink-install --packages-select my_delta_robot
source install/setup.bash
colcon test --packages-select my_delta_robot
colcon test-result --verbose
```

## Package Layout

```text
src/
|-- CMakeLists.txt
|-- package.xml
|-- delta_robot/          # IK and motion library
|-- launch/               # display and helper launch files
|-- msg/                  # Custom ROS 2 messages
|-- python_scripts/       # GUI, terminal UI, and helper Python nodes
|-- rviz/                 # RViz config
|-- src/                  # C++ ROS nodes
|-- test/                 # Unit tests
`-- urdf/                 # Delta robot URDF
```

## Notes

- The current runtime is intended for ROS simulation and visualization.
- Physical DRV8825/NEMA23 hardware still needs a lower-level step scheduler or embedded controller.
- `Reset Form` in the GUI resets the fields only; the current ROS command interface does not include a motion cancel command.
