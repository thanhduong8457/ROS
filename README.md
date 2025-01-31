# Step by step to create the project from scratch
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
ros2 pkg create delta_robot --build-type ament_cmake --dependencies rclcpp

ros2 pkg create --build-type ament_cmake --license Apache-2.0 delta_robot

# build specific package with command: 
# colcon build --packages-select NAME_OF_PAKAGE
colcon build --packages-select my_cpp_pkg
colcon build --packages-select my_py_pkg
colcon build --packages-select urdf_tutorial

colcon build --packages-select delta_robot

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
	check_urdf delta_robot.urdf
	urdf_to_graphiz delta_robot.urdf
	evince Delta_robot.pdf

# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#MOVEIT
# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#MOVEIT
# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#MOVEIT
# [shell 1]: ROS Master
conda activate ros2

# [shell 2]: Run Rviz
roslaunch delta_robot my_delta.launch

# [shell 3]: Path Node and Torques
ros2 run delta_robot main_node

# [shell 4]: Run Node
ros2 run delta_robot node_b

# [shell 5]: Run Node
ros2 run delta_robot node_a

# [shell 6]: Run Node
ros2 run delta_robot camera_node.py

# [shell 7]: Run rqt_graph to visualize the relation between Node and Topic
ros2 run rqt_graph rqt_graph

# [shell 8]: Message for the torques and trajectories node
rostopic pub -1 /input_ls_final delta_robot/linear_speed_xyz -- 0.0 0.0 -375.0 0.0 0.0 -490.0 200.0 10000.0
rostopic pub -1 /input_ls_final delta_robot/linear_speed_xyz -- 0.0 0.0 -490.0 0.0 0.0 -375.0 200.0 10000.0
rostopic pub -1 /input_ls_final delta_robot/linear_speed_xyz -- 0.0 0.0 -375.0 0.0 150.0 -490.0 200.0 10000.0
rostopic pub -1 /input_ls_final delta_robot/linear_speed_xyz -- 0.0 150.0 -490.0 0.0 -150.0 -490.0 200.0 10000.0
rostopic pub -1 /input_ls_final delta_robot/linear_speed_xyz -- 0.0 -150.0 -490.0 0.0 0.0 -375.0 200.0 10000.0

rostopic pub -1 /status_to_image_node std_msgs/String -- "Done"

# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#MOVEIT
# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#MOVEIT
# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#MOVEIT

# set position of current point of robot delta
rostopic pub -1 /set_current_point delta_robot/Posicionxyz -- 0.0 0.0 -375 -1

# set start point grip distance
rostopic pub -1 /set_current_point delta_robot/Posicionxyz -- 20.0 0.0 0.0 3

# set end point grip distance
rostopic pub -1 /set_current_point delta_robot/Posicionxyz -- 20.0 0.0 0.0 4

# set start and end point grip distance
rostopic pub -1 /set_current_point delta_robot/Posicionxyz -- 20.0 20.0 0.0 5

# start to draw square if run node draw
rostopic pub -1 /set_current_point delta_robot/Posicionxyz -- 0.0 0.0 0.0 6
ros2 topic pub --once /set_current_point my_delta_robot/msg/Posicionxyz "{x0: 0.0, y0: 0.0, z0: 0.0, type: 6}"

# start to draw triangle if run node draw
rostopic pub -1 /set_current_point delta_robot/Posicionxyz -- 0.0 0.0 0.0 7
ros2 topic pub --once /set_current_point my_delta_robot/msg/Posicionxyz "{x0: 0.0, y0: 0.0, z0: 0.0, type: 7}"

# set position of circle point of robot delta when grip release
rostopic pub -1 /set_current_point delta_robot/Posicionxyz -- 20.0 20.0 0.0 0

# set position of square point of robot delta when grip release
rostopic pub -1 /set_current_point delta_robot/Posicionxyz -- 20.0 20.0 0.0 1

# set position of triangle point of robot delta when grip release
rostopic pub -1 /set_current_point delta_robot/Posicionxyz -- 20.0 20.0 0.0 2

# set numpoint 
rostopic pub -1 /set_num_point delta_robot/num_point -- 120 200

# set vmax and a max for robot
rostopic pub -1 /set_vmax_amax delta_robot/vmax_amax -- 1500.0 200000.0
ros2 topic pub --once /set_vmax_amax my_delta_robot/msg/VmaxAmax "{vmax: 1500.0, amax: 200000.0}"
ros2 topic pub --once /set_vmax_amax my_delta_robot/msg/VmaxAmax "{vmax: 10.0, amax: 200.0}"

# Another way to build project, 
cmake ../src/ -DPython3_FIND_VIRTUALENV=ONLY
