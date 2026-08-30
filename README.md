# my_delta_robot — ROS2 Delta Robot

Delta parallel robot package: inverse kinematics, trapezoidal velocity profiles, RViz visualization.

---

## Build & Run

From the workspace root (parent of `src/`):

```bash
# Build
colcon build --symlink-install --packages-select my_delta_robot
# With conda/virtualenv Python:
colcon build --symlink-install --cmake-args -DPython3_FIND_VIRTUALENV=ONLY --packages-select my_delta_robot

# Source and launch (RViz + robot_state_publisher + draw_node + main_node)
source install/setup.bash
ros2 launch my_delta_robot display.launch.py
```

**Active nodes** (included in `display.launch.py`): `robot_state_publisher`, `rviz2`, `draw_node`, `main_node`. Optional/legacy: `node_a`, `node_b`, `serial_module` (commented out in CMakeLists; enable if needed).

---

## Step by step to create the project from scratch
## %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%# ROS2
# === link include with ROS.jgp picture above ===#
	https://docs.ros.org/en/galactic/index.html
	https://roboticsbackend.com/category/ros2/

# list of robot pakages: 
	https://moveit.ros.org/robots/

# link youtube about moveit
	https://www.youtube.com/channel/UCofPudSKrNzT3vccJD6VDyA/videos

# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#
https://www.udemy.com/course/mastering-rtos-hands-on-with-freertos-arduino-and-stm32fx/learn/lecture/25694576#overview 
# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#
DRV8825 x3
NEMA 23 Stepper Motor

# Udemy guide ROS2: 
https://banvien.udemy.com/course/ros2-for-beginners/learn/lecture/21305600#overview

# path to document
https://docs.ros.org/en/humble/index.html

# command create package: 
ros2 pkg create NAME_OF_PAKAGE --build-type ament_python/ament_cmake --dependencies rclcpp
ros2 pkg create urdf_tutorial --build-type ament_cmake --dependencies rclcpp

ros2 pkg create beginner_tutorials --build-type ament_cmake --dependencies rclcpp
ros2 pkg create my_delta_robot --build-type ament_cmake --dependencies rclcpp

ros2 pkg create --build-type ament_cmake --license Apache-2.0 my_delta_robot

# build specific package with command: 
# colcon build --packages-select NAME_OF_PAKAGE
colcon build --packages-select my_cpp_pkg
colcon build --packages-select my_py_pkg
colcon build --packages-select urdf_tutorial

colcon build --packages-select my_delta_robot

colcon build --symlink-install \
  -DPython3_EXECUTABLE:INTERNAL=$CONDA_PREFIX/bin/python3 \
  -DPython3_FIND_STRATEGY=LOCATION -DPython3_FIND_UNVERSIONED_NAMES=FIRST

# when we use the vitual env for build the package, the specify in percific python is required for building the package
colcon build --symlink-install --cmake-args -DPython3_FIND_VIRTUALENV=ONLY --packages-select my_delta_robot

colcon build --symlink-install --packages-select my_delta_robot

beginner_tutorials
delta_robot

# testing build message by command
ros2 interface show my_delta_robot/msg/Angulo

# install tool with specific verison: 
pip3 install setuptools==58.2.0 (setuptools is the name of the tool)

# Switch the env to ros2 by command: 
conda activate ros2
source the file .zsh in folder install after build to run the ros2 command

path to ROS2: /Users/thanhduong/miniconda3/envs/ros2/include

rqt --list-plugins

# this command to run gui debug rat
ros2 run rqt_gui rqt_gui --standalone rqt_graph.ros_graph.RosGraph

# IMPORTANCE!!!
# if you want to debug it then you need to change then base -> ros2 environment before open the vscode

# spider robot
https://hackaday.io/project/180534-zerobug-diy-hexapod-robot

# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#MOVEIT
Controlled by Servotronix softMC Motion Controller and CDHD Servo Drives, via EtherCAT
# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#MOVEIT

# Design Robot on solidwork
https://www.youtube.com/watch?v=p9c9KoKjEe0

# Export URDF file from solidwork
https://www.youtube.com/watch?v=OSL-zqw4cXs

# Run setup assitant with URDF file
https://www.youtube.com/watch?v=IS3JIV45rh0

# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#MOVEIT
# Design Delta robot on solidwork
https://www.youtube.com/watch?v=xt7HtHjXo4s

# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#MOVEIT
Simplified Delta Robot Kinematic Equations
https://www.youtube.com/watch?v=FTRCwuAnr6o&list=RDCMUCpTuKJrXFwybnpOG7HpTpZw&index=2
https://www.marginallyclever.com/other/samples/fk-ik-test.html
# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#MOVEIT

roslaunch panda_moveit_config demo.launch rviz_tutorial:=true

roslaunch moveit_setup_assistant setup_assistant.launch
	rosdep check rqt_graph

# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#MOVEIT
# check URDF file
	check_urdf my_delta_robot.urdf
	urdf_to_graphiz my_delta_robot.urdf
	evince Delta_robot.pdf

# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#MOVEIT
# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#MOVEIT
# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#MOVEIT
# [shell 1]: Activate ROS2 env
conda activate ros2
source install/setup.bash

# [shell 2]: Launch (Rviz + robot_state_publisher + draw_node + main_node)
ros2 launch my_delta_robot display.launch.py

# [shell 3]: Run rqt_graph to visualize nodes and topics
ros2 run rqt_gui rqt_gui --standalone rqt_graph.ros_graph.RosGraph

# ROS2 topic examples (trajectory / joint_states)
ros2 topic pub --once /input_ls_final my_delta_robot/msg/LinearSpeedXYZ "{xo: 0.0, yo: 0.0, zo: -375.0, xf: 0.0, yf: 0.0, zf: -490.0, vmax: 200.0, amax: 10000.0, gripper: 0}"
ros2 topic pub --once /input_ls_final my_delta_robot/msg/LinearSpeedXYZ "{xo: 0.0, yo: 0.0, zo: -490.0, xf: 0.0, yf: 0.0, zf: -375.0, vmax: 200.0, amax: 10000.0, gripper: 0}"

# set current point (draw_node)
ros2 topic pub --once /set_current_point my_delta_robot/msg/Posicionxyz "{x0: 0.0, y0: 0.0, z0: -375.0, type: -1}"
ros2 topic pub --once /set_current_point my_delta_robot/msg/Posicionxyz "{x0: 20.0, y0: 0.0, z0: 0.0, type: 3}"
ros2 topic pub --once /set_current_point my_delta_robot/msg/Posicionxyz "{x0: 0.0, y0: 0.0, z0: 0.0, type: 6}"
ros2 topic pub --once /set_current_point my_delta_robot/msg/Posicionxyz "{x0: 0.0, y0: 0.0, z0: 0.0, type: 7}"

# set resolution and vmax/amax
ros2 topic pub --once /set_num_point my_delta_robot/msg/NumPoint "{resolution: 120}"
ros2 topic pub --once /set_vmax_amax my_delta_robot/msg/VmaxAmax "{vmax: 1500.0, amax: 200000.0}"

# Another way to build project, 
cmake ../src/ -DPython3_FIND_VIRTUALENV=ONLY

ros2 launch my_delta_robot display.launch.py

colcon build --symlink-install --cmake-args -DPython3_FIND_VIRTUALENV=ONLY --packages-select my_delta_robot

# to run tests after a build:
source install/setup.bash
colcon test --packages-select my_delta_robot
