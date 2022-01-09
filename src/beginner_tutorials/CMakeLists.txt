cmake_minimum_required(VERSION 3.0.2)
project(beginner_tutorials)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS 
    roscpp 
    std_msgs 
    message_generation
)

## Declare ROS messages and services
add_message_files(FILES 
    Monitor.msg
    position_monitor.msg
)

add_service_files(FILES 
    GetDistance.srv 
)

## Generate added messages and services
generate_messages(
    DEPENDENCIES 
    std_msgs
)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(talker src/talker.cpp include/beginner_tutorials/PID.cpp include/beginner_tutorials/fuzzy_ctrl.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
add_dependencies(talker beginner_tutorials_generate_messages_cpp)

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
add_dependencies(listener beginner_tutorials_generate_messages_cpp)

# add_executable(add_numbers_server src/add_numbers_server.cpp)
# target_link_libraries(add_numbers_server ${catkin_LIBRARIES})
# add_dependencies(add_numbers_server beginner_tutorials_gencpp)

# add_executable(add_numbers_client src/add_numbers_client.cpp)
# target_link_libraries(add_numbers_client ${catkin_LIBRARIES})
# add_dependencies(add_numbers_client beginner_tutorials_gencpp)