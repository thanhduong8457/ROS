cmake_minimum_required(VERSION 3.10)
project(my_delta_robot)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CartesianJog.msg"
  "msg/CircleXYZ.msg"
  "msg/LinearSpeedXYZ.msg"
  "msg/Posicionxyz.msg"
  "msg/VmaxAmax.msg"
)

add_subdirectory(delta_robot)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

function(add_delta_robot_executable target source)
  add_executable(${target} ${source})
  ament_target_dependencies(${target}
    rclcpp std_msgs sensor_msgs)
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
  python_scripts/gui_user_interface_node.py
  python_scripts/user_interface_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  python_scripts/ros_bootstrap.py
  python_scripts/ui_config.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  PATTERN "__pycache__" EXCLUDE
  PATTERN ".DS_Store" EXCLUDE
)
install(
  DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
  PATTERN "__pycache__" EXCLUDE
  PATTERN ".DS_Store" EXCLUDE
)
install(
  DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
  PATTERN "__pycache__" EXCLUDE
  PATTERN ".DS_Store" EXCLUDE
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)

  ament_add_gtest(test_delta_robot test/test_delta_robot.cpp)
  target_include_directories(test_delta_robot PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/delta_robot)
  target_link_libraries(test_delta_robot delta_robot)

  add_test(
    NAME test_python_helpers
    COMMAND "${Python3_EXECUTABLE}"
            "${CMAKE_CURRENT_SOURCE_DIR}/test/test_python_helpers.py"
  )
endif()

ament_package()
