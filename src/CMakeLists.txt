cmake_minimum_required(VERSION 3.10)
project(my_delta_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

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

add_subdirectory(delta_robot)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

function(add_delta_robot_executable target source)
  add_executable(${target} ${source})
  ament_target_dependencies(${target}
    rclcpp std_msgs sensor_msgs geometry_msgs)
  target_include_directories(${target} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/delta_robot)
  target_link_libraries(${target}
    delta_robot
    "${cpp_typesupport_target}")
endfunction()

add_delta_robot_executable(main_node src/main_node.cpp)
add_delta_robot_executable(draw_node src/draw_node.cpp)

install(TARGETS main_node draw_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  python_scripts/initial_pose_publisher.py
  python_scripts/gui_user_interface_node.py
  python_scripts/user_interface_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_delta_robot test/test_delta_robot.cpp)
  target_include_directories(test_delta_robot PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/delta_robot)
  target_link_libraries(test_delta_robot delta_robot)
endif()

ament_package()
