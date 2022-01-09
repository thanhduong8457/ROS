cmake_minimum_required(VERSION 3.0.2)
project(position_arms)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
   roscpp
   std_msgs
   sensor_msgs
   #moveit_msgs
   message_generation
   genmsg
   urdf
)

# find_package(Eigen3 REQUIRED)
# include_directories(include  ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})

# Declare ROS messages and services
add_message_files(FILES 
   PositionArm.msg
)

# add_service_files(FILES 
#     GetDistance.srv 
# )

## Generate added messages and services
generate_messages(
    DEPENDENCIES 
    std_msgs
)

catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES position_arms
   CATKIN_DEPENDS 
   roscpp 
   moveit_msgs 
   std_msgs
   message_runtime
   LIBRARIES ${PROJECT_NAME}
#  DEPENDS system_lib
)

include_directories(include 
   ${catkin_INCLUDE_DIRS} 
   ${Boost_INCLUDE_DIR} 
   SYSTEM ${EIGEN3_INCLUDE_DIRS}
)

add_executable(position src/position.cpp)
target_link_libraries(position ${catkin_LIBRARIES})
add_dependencies(position position_arms_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

add_executable(monitor src/monitor.cpp)
target_link_libraries(monitor ${catkin_LIBRARIES})
add_dependencies(monitor position_arms_generate_messages_cpp ${catkin_EXPORTED_TARGETS})