cmake_minimum_required(VERSION 3.10)
project(ekf_localization VERSION 1.0)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 3.3 REQUIRED)

add_executable(ekf_localization src/ekf.cpp src/localization_node.cpp src/main.cpp)

ament_target_dependencies(ekf_localization PUBLIC
  rclcpp 
  sensor_msgs 
  geometry_msgs
  Eigen3
)

install(TARGETS
  ekf_localization
  DESTINATION lib/${PROJECT_NAME}
)

target_include_directories(ekf_localization PUBLIC
  "${PROJECT_SOURCE_DIR}/include"
)


ament_package()
