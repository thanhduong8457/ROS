##%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%# ROS2
#=== link include with ROS.jgp picture above ===#
	https://docs.ros.org/en/galactic/index.html
	https://roboticsbackend.com/category/ros2/

#list of robot pakages: 
	https://moveit.ros.org/robots/

#link of book:
	https://vn1lib.org/s/ROS 
	
# link yootube about moveit
	https://www.youtube.com/channel/UCofPudSKrNzT3vccJD6VDyA/videos

#ROS melodic:
	http://wiki.ros.org/ROS/Tutorials
#Tutorials:
	http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment
#moveit:
	http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/quickstart_in_rviz/quickstart_in_rviz_tutorial.html

	
##%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%##
- to reboot WSL, open powershell with admin and run this command:
	Get-Service LxssManager | Restart-Service

- power command help me successfully upgrade WSL 20.04 from WSL 18.04
	sudo apt-get purge snapd


#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#
https://www.udemy.com/course/mastering-rtos-hands-on-with-freertos-arduino-and-stm32fx/learn/lecture/25694576#overview 
#%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%#
DRV8825 x3
NEMA 23 Stepper Motor







# Udemy guide ROS2: 
https://banvien.udemy.com/course/ros2-for-beginners/learn/lecture/21305600#overview

# path to document
https://docs.ros.org/en/iron/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html

# command create package: 
ros2 pkg create NAME_OF_PAKAGE --build-type ament_python/ament_cmake --dependencies /rclcpp
ros2 pkg create urdf_tutorial --build-type ament_cmake --dependencies rclcpp

ros2 pkg create beginner_tutorials --build-type ament_cmake --dependencies rclcpp
ros2 pkg create my_delta_robot --build-type ament_cmake --dependencies rclcpp

# build specific package with command: 
colcon build --packages-select NAME_OF_PAKAGE
colcon build --packages-select my_cpp_pkg
colcon build --packages-select my_py_pkg
colcon build --packages-select urdf_tutorial

beginner_tutorials
my_delta_robot

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


