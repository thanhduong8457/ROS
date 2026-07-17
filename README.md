# my_delta_robot

ROS 2 package for a delta parallel robot. The package includes Cartesian trajectory planning, inverse kinematics, drawing path sequencing, RViz visualization, and Python user interfaces for sending drawing commands.

For a current project-level summary, status, known gaps, and recommended next steps, see [PROJECT_OVERVIEW.md](PROJECT_OVERVIEW.md).

## Features

- `main_node`: validates line-segment commands, plans Cartesian trajectories, preflights IK, follows planned elapsed time, and publishes `/joint_states`.
- `draw_node`: generates drawing paths and executes them through a protected sequential waypoint queue; circles use one continuous 1 kHz trajectory without stops between polygon edges.
- Reusable validation and shape-generation helpers in `src/delta_robot/` keep ROS callbacks small and core behavior testable.
- `gui_user_interface_node.py`: operator control panel with live TCP feedback,
  motion presets, one-click shapes, direct Cartesian targets, and incremental jog controls.
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

Launch the complete simulator, including the control panel:

```bash
ros2 launch my_delta_robot control.launch.py
```

To launch only the robot model, RViz, `main_node`, and `draw_node`:

```bash
ros2 launch my_delta_robot display.launch.py
```

The GUI can also be started separately in another terminal:

```bash
source install/setup.bash
ros2 run my_delta_robot gui_user_interface_node.py
```

The terminal UI is still available:

```bash
source install/setup.bash
ros2 run my_delta_robot user_interface_node.py
```

## Operator Control Panel

The control panel shows whether `main_node` and `draw_node` are connected and
disables motion buttons while a command is active. It provides:

- Gentle, Normal, and Fast motion-limit presets, plus editable velocity and acceleration.
- One-click Rectangle, Triangle, and Smooth Circle commands.
- Live TCP X/Y/Z position in millimetres.
- Direct Cartesian target moves, Return Home, and Copy Current to Target.
- X/Y/Z jog buttons with 1, 2, 5, 10, or 20 mm steps.
- A timestamped activity log with drawing start, completion, and failure feedback.

The GUI and terminal UI start with these motion limits:

| Field | Default |
|-------|---------|
| `vmax` | `5000.0` mm/s |
| `amax` | `100.0` mm/s^2 |

Both values must be finite and positive. GUI manual moves also validate the
configured Z workspace of `-480` through `-375` mm. Commands execute immediately;
this simulation interface does not yet provide pause, cancel, or emergency stop.

## Communication

The user interface nodes reuse the existing topic-based command style.

Architecture diagrams and sequence flows are available in
[docs/software_architecture.md](docs/software_architecture.md).

| Publisher | Topic | Message | Receiver | Purpose |
|-----------|-------|---------|----------|---------|
| GUI or terminal UI | `/set_vmax_amax` | `my_delta_robot/msg/VmaxAmax` | `main_node` | Update velocity and acceleration limits |
| GUI or terminal UI | `/set_current_point` | `my_delta_robot/msg/Posicionxyz` | `draw_node` | Queue a drawing action or synchronize the path origin |
| GUI or `draw_node` | `/input_ls_final` | `my_delta_robot/msg/LinearSpeedXYZ` | `main_node` | Send one Cartesian line segment |
| `draw_node` | `/input_circle` | `my_delta_robot/msg/CircleXYZ` | `main_node` | Send one continuous circular trajectory |
| `main_node` | `/joint_states` | `sensor_msgs/msg/JointState` | GUI, `robot_state_publisher`, RViz | Report live TCP/joint state; 5 Hz while idle and trajectory rate while moving |
| `main_node` | `/status_delta` | `std_msgs/msg/String` | GUI, `draw_node` | Report `DONE ...` or `FAILED: ...` for a segment |
| `draw_node` | `/drawing_status` | `std_msgs/msg/String` | GUI | Report `STARTED:`, `DONE:`, or `FAILED:` for a complete shape |

Drawing actions are encoded in the existing `Posicionxyz.type` field:

| GUI action | `type` | Behavior in `draw_node` |
|------------|--------|-------------------------|
| Rectangle | `6` | Queue rectangle/square path |
| Triangle | `7` | Queue triangle path |
| Circle | `8` | Approach the circle, execute one continuous revolution, and return home |

The richer GUI reuses the existing custom messages and adds only a standard
`std_msgs/String` drawing-status topic.

The circle command uses a trapezoidal or triangular speed profile along the circumference. Position, tangent velocity, and radial centripetal acceleration are sampled continuously at 1 kHz, so the robot does not stop at intermediate circle points. Circle speed is curvature-limited so combined tangential and centripetal acceleration stays inside the configured acceleration envelope.

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
  "{xo: 0.0, yo: 0.0, zo: -375.0, xf: 0.0, yf: 0.0, zf: -450.0, gripper: 0}"
```

## `set_current_point` Type Codes

| Type | Meaning |
|------|---------|
| `-1` | Set current TCP position |
| `0` | Update path point A from `x0`, `y0`, `z0` |
| `1` | Update path point B from `x0`, `y0`, `z0` |
| `2` | Update path point C from `x0`, `y0`, `z0` |
| `3` | Set the drawing Z offset from `x0` |
| `4` | Set the legacy target Z offset from `x0` |
| `5` | Set drawing and legacy target Z offsets from `x0` and `y0` |
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

The current suite contains 13 tests covering IK, joint mapping, triangular and trapezoidal line motion, continuous clockwise/counter-clockwise circles, invalid planner input, sample-count protection, command validation, and shape closure.

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
- The control panel disables overlapping GUI commands, but the current ROS command interface does not include a motion cancel or emergency-stop command.
- A new shape command is rejected while another drawing sequence is active. A `FAILED:` segment status clears the remaining queue instead of advancing from an unconfirmed position.
