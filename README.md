# ROS 2 Delta Robot

`my_delta_robot` is a ROS 2 simulation package for a three-arm delta robot. It
plans Cartesian line and circle trajectories, validates inverse kinematics,
publishes the robot joint state, and provides an RViz model plus graphical and
terminal controls.

> This project currently controls a simulation only. It does not implement
> homing, motor feedback, limit switches, an emergency stop, or a hardware-safe
> step scheduler.

## Quick start

The active development target is ROS 2 Humble. Start in a terminal where the
base ROS environment is already sourced (or the ROS Conda environment is
active), then run:

```bash
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
./build.zsh --test
./run.zsh control
```

`run.zsh` sources this workspace automatically. Its modes are:

| Command | Starts |
|---|---|
| `./run.zsh control` | Motion nodes, robot model, RViz, and graphical controls |
| `./run.zsh display` | Motion nodes, robot model, and RViz |
| `./run.zsh headless` | Motion nodes and robot model only |

Launch arguments can follow the mode, for example:

```bash
./run.zsh headless use_sim_time:=true
```

The equivalent ROS command is:

```bash
source install/setup.zsh
ros2 launch my_delta_robot bringup.launch.py gui:=true rviz:=true
```

On Bash, source `install/setup.bash` instead.

### Bring-up configuration

The canonical launcher exposes typed arguments instead of requiring source
edits:

| Argument | Default | Purpose |
|---|---:|---|
| `gui` | `true` | Start graphical controls |
| `rviz` | `true` | Start RViz |
| `use_sim_time` | `false` | Use `/clock` |
| `max_velocity_mm_s` | `5000.0` | Initial path velocity limit |
| `max_acceleration_mm_s2` | `100.0` | Initial path acceleration limit |
| `draw_offset_mm` | `20.0` | Drawing-plane Z offset |
| `circle_radius_mm` | `25.0` | Circle radius |
| `circle_center_x_mm` | `-100.0` | Circle center X |
| `circle_center_y_mm` | `-100.0` | Circle center Y |
| `circle_base_z_mm` | `-453.0` | Circle base Z before the drawing offset |

For example:

```bash
./run.zsh control circle_radius_mm:=40.0 draw_offset_mm:=15.0
```

## Operator controls

The graphical panel provides:

- connection and live TCP status;
- motion-limit presets and editable velocity/acceleration;
- rectangle, triangle, and continuous-circle commands;
- direct Cartesian targets and home return;
- a six-direction Cartesian joystick for Forward (`+Y`), Back (`-Y`),
  Left (`-X`), Right (`+X`), Up (`+Z`), and Down (`-Z`) movement with
  selectable 5–100 mm/s speed;
- command progress and failure feedback.

Press and hold a joystick button to move, then release it to stop at the latest
executed position. A 300 ms command-heartbeat timeout also stops jogging if the
UI closes or communication is interrupted. Line and drawing commands still
cannot be paused or cancelled, and the UI is not a hardware emergency stop.

The terminal interface remains available:

```bash
ros2 run my_delta_robot user_interface_node.py
```

## Runtime

The canonical `bringup.launch.py` starts:

- `main_node`, which plans, preflights, and executes Cartesian trajectories;
- `draw_node`, which sequences complete drawing paths;
- `robot_state_publisher` and the `world` to `base_link` transform;
- optional RViz and graphical controls.

Public Cartesian values and motion limits use millimetres, seconds, and their
derived units. The C++ planning and kinematics core uses SI units internally.
The home TCP is `(0, 0, -375)` mm; negative Z points downward.

| Topic | Message | Purpose |
|---|---|---|
| `/set_vmax_amax` | `VmaxAmax` | Set maximum path velocity and acceleration |
| `/set_current_point` | `Posicionxyz` | Configure drawing state or request a shape |
| `/input_ls_final` | `LinearSpeedXYZ` | Request a Cartesian line target |
| `/input_cartesian_jog` | `CartesianJog` | Start, refresh, or stop press-and-hold jogging |
| `/input_circle` | `CircleXYZ` | Request one continuous circle |
| `/joint_states` | `sensor_msgs/JointState` | Publish modeled joints and live TCP |
| `/status_delta` | `std_msgs/String` | Report line, circle, or jog completion/failure |
| `/drawing_status` | `std_msgs/String` | Report whole-shape progress |
| `/v_a_out` | `VmaxAmax` | Publish the current path velocity/acceleration |

`Posicionxyz.type` is retained for compatibility. Its message definition now
exposes named constants:

| Constant | Value | Meaning |
|---|---:|---|
| `SET_CURRENT_POINT` | `-1` | Synchronize `draw_node` with the live TCP |
| `SET_PATH_POINT_A/B/C` | `0/1/2` | Update stored shape points |
| `SET_DRAW_OFFSET` | `3` | Set drawing Z offset from `x0` |
| `DRAW_RECTANGLE` | `6` | Draw a closed four-sided path |
| `DRAW_TRIANGLE` | `7` | Draw a closed three-sided path |
| `DRAW_CIRCLE` | `8` | Draw one smooth continuous circle |

For diagrams and execution sequences, see
[docs/software_architecture.md](docs/software_architecture.md). For current
scope and roadmap, see [PROJECT_OVERVIEW.md](PROJECT_OVERVIEW.md).

## Command-line examples

Set motion limits:

```bash
ros2 topic pub --once /set_vmax_amax my_delta_robot/msg/VmaxAmax \
  "{vmax: 2000.0, amax: 100.0}"
```

Draw a rectangle:

```bash
ros2 topic pub --once /set_current_point my_delta_robot/msg/Posicionxyz \
  "{type: 6}"
```

Move directly to the home pose:

```bash
ros2 topic pub --once /input_ls_final my_delta_robot/msg/LinearSpeedXYZ \
  "{xf: 0.0, yf: 0.0, zf: -375.0}"
```

The line message retains its original start fields for wire compatibility, but
`main_node` uses its own live TCP as the authoritative trajectory start.

## Development

Build only:

```bash
./build.zsh
```

Build and run all registered tests:

```bash
./build.zsh --test
```

Or use the underlying commands:

```bash
colcon build --symlink-install --packages-select my_delta_robot
colcon test --packages-select my_delta_robot
colcon test-result --verbose
```

The active package layout is:

```text
src/
├── delta_robot/       # Kinematics, joint mapping, and trajectory planning
├── launch/            # Canonical bringup and compatibility launchers
├── msg/               # Active ROS interfaces
├── python_scripts/    # Graphical/terminal controls and shared helpers
├── src/               # Active C++ ROS nodes
├── test/              # Automated tests
├── urdf/ and rviz/    # Robot model and visualization
└── legacy/            # Archived, unsupported experiments
```

Generated `build/`, `install/`, and `log/` directories are workspace artifacts
and are not source code.

## Known limitations

- Simulation output stops at `/joint_states`; no physical motor controller is
  connected.
- Command completion still uses string topics rather than a typed ROS 2 action,
  so goal IDs, feedback, general trajectory cancellation, and robust
  multi-client arbitration are future work. Cartesian jogging has its own
  explicit stop command and dead-man timeout.
- Shape geometry is currently defined by `draw_node`; the operator UI exposes
  shape selection but not arbitrary dimensions.
- Node-level launch behavior, GUI widgets, and URDF/TF consistency have less
  automated coverage than the C++ math core.

Archived ROS 1 camera and serial experiments are documented in
[src/legacy/README.md](src/legacy/README.md).
