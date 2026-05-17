cmake_minimum_required(VERSION 3.8)
project(my_delta_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Add the directory containing Findserial.cmake to CMAKE_MODULE_PATH
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DPython3_FIND_VIRTUALENV=ONLY")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development NumPy)
# find_package(serial REQUIRED)

include_directories(${Python3_NumPy_INCLUDE_DIRS})
# include_directories(${serial_INCLUDE_DIRS})

rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/Angulo.msg"
    "msg/LinearSpeedXYZ.msg"
    "msg/Posicionxyz.msg"
    "msg/PositionArm.msg"
    "msg/ImagePos.msg"
    "msg/VmaxAmax.msg"
    "msg/NumPoint.msg"
    DEPENDENCIES geometry_msgs
)

include_directories(
    delta_robot
    ${ament_INCLUDE_DIRS}
)

add_subdirectory(delta_robot)

add_executable(main_node src/main_node.cpp)
ament_target_dependencies(main_node rclcpp std_msgs sensor_msgs urdf geometry_msgs)
target_link_libraries(main_node delta_robot)
rosidl_target_interfaces(main_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

# add_executable(node_b src/node_b.cpp)
# ament_target_dependencies(node_b rclcpp std_msgs sensor_msgs urdf geometry_msgs)
# rosidl_target_interfaces(node_b ${PROJECT_NAME} "rosidl_typesupport_cpp")

# add_executable(node_a src/node_a.cpp)
# ament_target_dependencies(node_a rclcpp std_msgs sensor_msgs urdf geometry_msgs)
# rosidl_target_interfaces(node_a ${PROJECT_NAME} "rosidl_typesupport_cpp")

# add_executable(serial_module src/serial_module.cpp)
# ament_target_dependencies(serial_module rclcpp std_msgs sensor_msgs urdf geometry_msgs)
# target_link_libraries(serial_module ${serial_LIBRARIES})
# rosidl_target_interfaces(serial_module ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(draw_node src/draw_node.cpp)
ament_target_dependencies(draw_node rclcpp std_msgs sensor_msgs urdf geometry_msgs)
target_link_libraries(draw_node delta_robot)
rosidl_target_interfaces(draw_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS 
    main_node
    # node_a
    # node_b
    draw_node
    # test_acuracy
    DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
    python_scripts/initial_pose_publisher.py
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY urdf
    DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY rviz
    DESTINATION share/${PROJECT_NAME}
)

# Unit tests (optional: build with --cmake-args -DBUILD_TESTING=ON)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_delta_robot test/test_delta_robot.cpp)
  target_include_directories(test_delta_robot PRIVATE ${CMAKE_SOURCE_DIR}/delta_robot)
  target_link_libraries(test_delta_robot delta_robot)
endif()

ament_package()
