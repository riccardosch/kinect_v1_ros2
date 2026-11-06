cmake_minimum_required(VERSION 3.16)
project(kinect_v1_bench)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(image_stats_component SHARED
  src/image_stats_node.cpp)
target_include_directories(image_stats_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(image_stats_component
  rclcpp::rclcpp
  rclcpp_components::component
  ${sensor_msgs_TARGETS})

rclcpp_components_register_node(image_stats_component
  PLUGIN "kinect_v1_bench::ImageStatsNode"
  EXECUTABLE image_stats_node)

install(TARGETS image_stats_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(PROGRAMS
  scripts/cpu_usage.sh
  scripts/check_zero_copy.py
  scripts/depth_stats.py
  scripts/imu_stats.py
  scripts/audio_stats.py
  scripts/check_camera_info.py
  scripts/cloud_stats.py
  scripts/remote_stats.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
