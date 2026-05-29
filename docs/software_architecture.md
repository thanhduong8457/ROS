# Software Architecture

This document illustrates the ROS 2 software architecture for `my_delta_robot`.
The active runtime path is the one launched by `display.launch.py`; older camera
and image-processing files are documented separately as optional or legacy flows.

## Active Runtime Architecture

```mermaid
flowchart LR
    subgraph UserLayer["User command layer"]
        GUI["gui_user_interface_node.py<br/>Tkinter ROS 2 UI"]
        TUI["user_interface_node.py<br/>terminal ROS 2 UI"]
        CLI["ros2 topic pub<br/>manual commands"]
    end

    subgraph CommandTopics["Command topics"]
        LimitsTopic["/set_vmax_amax<br/>VmaxAmax"]
        ShapeTopic["/set_current_point<br/>Posicionxyz"]
        DirectSegmentTopic["/input_ls_final<br/>LinearSpeedXYZ"]
        ResolutionTopic["/set_num_point<br/>NumPoint"]
    end

    subgraph MotionLayer["Motion orchestration layer"]
        DrawNode["draw_node<br/>shape waypoint queue"]
        MainNode["main_node<br/>trajectory executor"]
    end

    subgraph CoreLibrary["Delta robot C++ library"]
        Planner["CartesianTrajectoryGenerator<br/>trapezoid or triangular profile"]
        IK["delta_robot<br/>inverse_checked + joint mapping"]
        Config["joint_state_config.hpp<br/>joint names + home pose"]
    end

    subgraph Visualization["Visualization layer"]
        JointStates["/joint_states<br/>sensor_msgs/JointState"]
        VelocityOut["/v_a_out<br/>VmaxAmax runtime profile sample"]
        Status["/status_delta<br/>std_msgs/String"]
        RSP["robot_state_publisher<br/>URDF to TF"]
        StaticTF["static_transform_publisher<br/>world to base_link"]
        TF["/tf tree"]
        RViz["rviz2<br/>my_config.rviz"]
    end

    GUI --> LimitsTopic
    GUI --> ShapeTopic
    TUI --> LimitsTopic
    TUI --> ShapeTopic
    CLI --> LimitsTopic
    CLI --> ShapeTopic
    CLI --> DirectSegmentTopic
    CLI --> ResolutionTopic

    LimitsTopic --> MainNode
    ResolutionTopic --> MainNode
    ShapeTopic --> DrawNode
    DrawNode --> DirectSegmentTopic
    DirectSegmentTopic --> MainNode

    MainNode --> Planner
    MainNode --> IK
    IK --> Config
    MainNode --> JointStates
    MainNode --> VelocityOut
    MainNode --> Status
    Status --> DrawNode

    JointStates --> RSP
    StaticTF --> TF
    RSP --> TF
    TF --> RViz
```

## Runtime Responsibilities

| Component | Source | Responsibility |
|-----------|--------|----------------|
| `display.launch.py` | `src/launch/display.launch.py` | Starts `world_to_base_link`, `robot_state_publisher`, `main_node`, `draw_node`, then RViz after a short delay. |
| `gui_user_interface_node.py` | `src/python_scripts/gui_user_interface_node.py` | Publishes motion limits and a shape action from a Tkinter UI. |
| `user_interface_node.py` | `src/python_scripts/user_interface_node.py` | Legacy terminal UI that publishes the same active command topics. |
| `draw_node` | `src/src/draw_node.cpp` | Converts shape commands into ordered Cartesian waypoints and sends one segment at a time. |
| `main_node` | `src/src/main_node.cpp` | Accepts line segments, plans sampled Cartesian motion, validates IK, publishes joint states and completion status. |
| `delta_robot` library | `src/delta_robot/` | Provides inverse kinematics, motion limits, trapezoidal trajectory utilities, and RViz joint mapping. |
| `robot_state_publisher` | ROS 2 runtime | Converts `/joint_states` plus `delta_robot.urdf` into TF transforms for RViz. |

## Shape Command Sequence

```mermaid
sequenceDiagram
    autonumber
    participant UI as GUI or terminal UI
    participant Limits as /set_vmax_amax
    participant Shape as /set_current_point
    participant Draw as draw_node
    participant Segment as /input_ls_final
    participant Main as main_node
    participant Planner as CartesianTrajectoryGenerator
    participant Robot as delta_robot IK
    participant JS as /joint_states
    participant VOut as /v_a_out
    participant Status as /status_delta
    participant RSP as robot_state_publisher
    participant RViz as RViz

    UI->>Limits: Publish VmaxAmax(vmax, amax)
    Limits->>Main: Update motion_limits()
    UI->>Shape: Publish Posicionxyz(type 6, 7, or 8)
    Shape->>Draw: Receive drawing action
    Draw->>Draw: Clear old queue and enqueue shape waypoints
    Draw->>Segment: Publish first LinearSpeedXYZ(start, target)
    Segment->>Main: Receive segment command
    Main->>Planner: planLine(start_m, target_m, limits, 0.001 s)
    Planner-->>Main: Return sampled trajectory
    loop Preflight for every sample
        Main->>Robot: inverse_checked(sample.position_m)
        Robot-->>Main: IK ok or rejection reason
    end
    loop 1 ms motion timer
        Main->>Robot: inverse_checked(current sample)
        Main->>Robot: create_joint_state_list(position, theta)
        Main->>JS: Publish JointState
        Main->>VOut: Publish profile velocity and acceleration
        JS->>RSP: Consume joint positions
        RSP->>RViz: Publish TF transforms
    end
    Main->>Status: Publish segment DONE message
    Status->>Draw: Mark waypoint complete
    alt More queued waypoints
        Draw->>Segment: Publish next LinearSpeedXYZ
    else Queue empty
        Draw->>Draw: Wait for next command
    end
```

## Direct Segment Sequence

```mermaid
sequenceDiagram
    autonumber
    participant CLI as ros2 topic pub or custom node
    participant Segment as /input_ls_final
    participant Main as main_node
    participant Planner as CartesianTrajectoryGenerator
    participant Robot as delta_robot
    participant JS as /joint_states
    participant Status as /status_delta

    CLI->>Segment: Publish LinearSpeedXYZ(xo, yo, zo, xf, yf, zf)
    Segment->>Main: Segment callback
    alt Motion already active
        Main-->>CLI: Reject through ROS warning log
    else Start and target are identical
        Main-->>CLI: Ignore through ROS warning log
    else Segment accepted
        Main->>Planner: Generate sampled line profile
        Main->>Robot: Validate IK for every planned sample
        alt Planning or IK fails
            Main-->>CLI: Reject through ROS error log
        else Plan valid
            loop Motion timer at 1 kHz
                Main->>Robot: Compute IK and 12-joint RViz state
                Main->>JS: Publish JointState
            end
            Main->>Status: Publish DONE message
        end
    end
```

## Launch And Visualization Startup

```mermaid
sequenceDiagram
    autonumber
    participant Launch as display.launch.py
    participant StaticTF as static_transform_publisher
    participant RSP as robot_state_publisher
    participant Main as main_node
    participant Draw as draw_node
    participant RViz as rviz2

    Launch->>StaticTF: Start world to base_link transform
    Launch->>RSP: Start with delta_robot.urdf as robot_description
    Launch->>Main: Start trajectory executor
    Launch->>Draw: Start shape waypoint queue
    Main->>RSP: Publish initial /joint_states immediately
    loop 15 startup republishes every 200 ms
        Main->>RSP: Republish home pose to avoid startup race
    end
    Launch->>RViz: Start after 2 second TimerAction
    RSP->>RViz: TF tree and robot model become visible
```

## Optional Legacy Camera And Hardware Flow

These files are present in the repository but are not part of the active
`display.launch.py` build/run path.

```mermaid
flowchart LR
    Camera["camera_node.py / test_camera_node.py<br/>ROS 1 style rospy + OpenCV"]
    ImageTopic["/data_image<br/>ImagePos"]
    NodeA["node_a.cpp<br/>image point dispatcher<br/>not built by CMake"]
    SendToB["/send_to_node_b<br/>Posicionxyz"]
    DrawNode["draw_node<br/>still subscribes to send_to_node_b"]
    StatusToA["/status_to_node_a<br/>std_msgs/String"]
    StatusToImage["/status_to_image_node<br/>std_msgs/String"]
    Serial["serial_module.cpp<br/>serial hardware bridge<br/>not built by CMake"]
    JointStates["/joint_states"]
    Hardware["External controller<br/>/dev/ttyTHS1 JSON"]

    Camera --> ImageTopic
    ImageTopic --> NodeA
    NodeA --> SendToB
    SendToB --> DrawNode
    DrawNode --> StatusToA
    StatusToA --> NodeA
    NodeA --> StatusToImage
    StatusToImage --> Camera

    JointStates -. optional .-> Serial
    Serial -. optional .-> Hardware
```

## Topic Map

| Topic | Message | Publisher | Subscriber | Notes |
|-------|---------|-----------|------------|-------|
| `/set_vmax_amax` | `VmaxAmax` | GUI, terminal UI, CLI | `main_node` | Sets max velocity and acceleration in mm units at the command interface. |
| `/set_current_point` | `Posicionxyz` | GUI, terminal UI, CLI | `draw_node` | Type `6`, `7`, `8` request rectangle, triangle, circle. Types `-1` through `5` update draw-node reference state. |
| `/input_ls_final` | `LinearSpeedXYZ` | `draw_node`, CLI/custom nodes | `main_node` | One Cartesian line segment from start to target. |
| `/set_num_point` | `NumPoint` | CLI/custom nodes | `main_node` | Updates the legacy offline resolution value; runtime planner samples at 1 kHz. |
| `/joint_states` | `sensor_msgs/JointState` | `main_node` | `robot_state_publisher`, RViz, optional serial bridge | Contains 12 published joints matching `joint_state_config.hpp` and the URDF. |
| `/status_delta` | `std_msgs/String` | `main_node` | `draw_node` | Segment-complete handshake for queued waypoints. |
| `/v_a_out` | `VmaxAmax` | `main_node` | optional observers | Publishes current path velocity and acceleration sample values. |
| `/send_to_node_b` | `Posicionxyz` | legacy `node_a` or custom nodes | `draw_node` | Supported by `draw_node`, but active CMake does not build `node_a`. |
| `/status_to_node_a` | `std_msgs/String` | `draw_node` | legacy `node_a` | Used only by the legacy image-pipeline handshake. |
