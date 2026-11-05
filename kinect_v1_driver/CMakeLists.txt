cmake_minimum_required(VERSION 3.16)
project(kinect_v1_driver)

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
find_package(std_msgs REQUIRED)
find_package(audio_common_msgs REQUIRED)

# libfreenect ships a minimal CMake config (variables only, no imported target).
find_package(libfreenect CONFIG REQUIRED)
find_library(FREENECT_LIBRARY
  NAMES ${FREENECT_LIBRARIES}
  HINTS ${FREENECT_LIBRARY_DIRS}
  REQUIRED)
message(STATUS "libfreenect ${FREENECT_VERSION}: ${FREENECT_LIBRARY}")

add_library(kinect_camera_component SHARED
  src/kinect_camera_node.cpp)
target_include_directories(kinect_camera_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
# Public headers include libfreenect.h.
target_include_directories(kinect_camera_component SYSTEM PUBLIC
  ${FREENECT_INCLUDE_DIRS})
target_link_libraries(kinect_camera_component
  ${FREENECT_LIBRARY}
  rclcpp::rclcpp
  rclcpp_components::component
  ${sensor_msgs_TARGETS})
target_compile_definitions(kinect_camera_component PRIVATE
  KINECT_V1_LIBFREENECT_VERSION="${FREENECT_VERSION}")

# Registers the component and generates a standalone executable wrapper.
rclcpp_components_register_node(kinect_camera_component
  PLUGIN "kinect_v1_driver::KinectCameraNode"
  EXECUTABLE kinect_camera_node)

# Aux component: LED + accelerometer (IMU) on the motor device, microphones
# on the audio device.
add_library(kinect_aux_component SHARED
  src/kinect_aux_node.cpp)
target_include_directories(kinect_aux_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_include_directories(kinect_aux_component SYSTEM PUBLIC
  ${FREENECT_INCLUDE_DIRS})
target_link_libraries(kinect_aux_component
  ${FREENECT_LIBRARY}
  rclcpp::rclcpp
  rclcpp_components::component
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${audio_common_msgs_TARGETS})

rclcpp_components_register_node(kinect_aux_component
  PLUGIN "kinect_v1_driver::KinectAuxNode"
  EXECUTABLE kinect_aux_node)

install(TARGETS kinect_camera_component kinect_aux_component
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Unit tests of pure logic (no hardware needed).
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_led_command test/test_led_command.cpp)
  target_link_libraries(test_led_command kinect_aux_component)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp rclcpp_components sensor_msgs std_msgs audio_common_msgs)
ament_package()
