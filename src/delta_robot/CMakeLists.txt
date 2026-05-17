set(LIBRARY_NAME delta_robot)

add_library(${LIBRARY_NAME} SHARED
  delta_robot.cpp
)

target_include_directories(${LIBRARY_NAME} PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
)

target_compile_features(${LIBRARY_NAME} PUBLIC cxx_std_17)
