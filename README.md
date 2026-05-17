# my_delta_robot

ROS 2 package for a delta parallel robot: inverse kinematics, trapezoidal velocity profiles, trajectory publishing to RViz, and draw/waypoint sequencing.

## Features

- **Kinematics library** (`delta_robot/`): IK, linear paths, trapezoidal velocity profiles
- **main_node**: Subscribes to motion commands, publishes `/joint_states` and trajectory status
- **draw_node**: Queues waypoints and publishes segments to `main_node`
- **Visualization**: URDF, `robot_state_publisher`, RViz2 config

## Prerequisites

- ROS 2 (Humble or compatible) with `rclcpp`, `sensor_msgs`, `robot_state_publisher`, `rviz2`
- `colcon` and `ament_cmake`
- Optional: Conda env `ros2` (see [build.zsh](build.zsh))

## Build

From the **workspace root** (this directory, which contains `src/`):

```bash
# Default build
./build.zsh

# Or manually:
colcon build --symlink-install --packages-select my_delta_robot

# With conda / virtualenv Python:
colcon build --symlink-install \
  --cmake-args -DPython3_FIND_VIRTUALENV=ONLY \
  --packages-select my_delta_robot
```

```bash
source install/setup.bash   # or install/setup.zsh
```

## Run

Launch RViz, robot model, `draw_node`, and `main_node`:

```bash
ros2 launch my_delta_robot display.launch.py
```

RViz uses **`world`** as the fixed frame. A static transform `world` → `base_link` is published at launch so the TF tree is valid before the first `/joint_states` message. If RViz reports a missing frame, wait ~2 s (RViz starts after the other nodes) or run `ros2 run tf2_tools view_frames`.

Optional: publish initial pose only (normally `main_node` does this):

```bash
ros2 run my_delta_robot initial_pose_publisher.py
```

## Architecture

```
draw_node  --input_ls_final-->  main_node  --joint_states-->  robot_state_publisher / RViz
              ^                      |
              |                      v
         status_delta  <----  (trajectory done)
```

| Node | Role |
|------|------|
| `main_node` | IK + trajectory → `/joint_states`, `/status_delta` |
| `draw_node` | Waypoint queue → `/input_ls_final` |
| `robot_state_publisher` | URDF → TF |
| `rviz2` | Visualization |

Legacy sources (not built by default): `node_a.cpp`, `node_b.cpp`, `serial_module.cpp`.

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/input_ls_final` | `my_delta_robot/msg/LinearSpeedXYZ` | Line segment (start → end), mm |
| `/set_current_point` | `my_delta_robot/msg/Posicionxyz` | Configure pose / draw modes |
| `/set_num_point` | `my_delta_robot/msg/NumPoint` | Trajectory resolution |
| `/set_vmax_amax` | `my_delta_robot/msg/VmaxAmax` | Velocity limits |
| `/joint_states` | `sensor_msgs/msg/JointState` | Robot visualization |
| `/status_delta` | `std_msgs/msg/String` | Motion complete |

### Example commands

```bash
# Move line (mm)
ros2 topic pub --once /input_ls_final my_delta_robot/msg/LinearSpeedXYZ \
  "{xo: 0.0, yo: 0.0, zo: -375.0, xf: 0.0, yf: 0.0, zf: -490.0, gripper: 0}"

# Set current TCP position
ros2 topic pub --once /set_current_point my_delta_robot/msg/Posicionxyz \
  "{x0: 0.0, y0: 0.0, z0: -375.0, type: -1}"

# Draw square (type 6) / triangle (type 7)
ros2 topic pub --once /set_current_point my_delta_robot/msg/Posicionxyz \
  "{x0: 0.0, y0: 0.0, z0: 0.0, type: 6}"

# Trajectory tuning
ros2 topic pub --once /set_num_point my_delta_robot/msg/NumPoint "{resolution: 120}"
ros2 topic pub --once /set_vmax_amax my_delta_robot/msg/VmaxAmax "{vmax: 1500.0, amax: 200000.0}"
```

### `set_current_point` type codes

| type | Meaning |
|------|---------|
| -1 | Set current TCP position |
| 0–2 | Update circle / square / triangle corner references |
| 3–5 | Z offsets (start, end, both) |
| 6 | Queue square draw path |
| 7 | Queue triangle draw path |

## Tests

```bash
colcon build --packages-select my_delta_robot
source install/setup.bash
colcon test --packages-select my_delta_robot
colcon test-result --verbose
```

## Package layout

```
src/
├── delta_robot/          # IK + motion library
├── src/                  # main_node, draw_node
├── msg/                  # Custom messages
├── urdf/                 # delta_robot.urdf
├── rviz/                 # my_config.rviz
├── launch/               # display.launch.py
├── python_scripts/       # initial_pose_publisher.py
└── test/                 # test_delta_robot
```

## Hardware notes

- Stepper drivers: DRV8825 ×3, NEMA 23
- Serial bridge: `serial_module.cpp` (enable in `CMakeLists.txt` when hardware is connected)

## References

- [ROS 2 Humble docs](https://docs.ros.org/en/humble/)
- [Delta IK reference](https://www.marginallyclever.com/other/samples/fk-ik-test.html)
