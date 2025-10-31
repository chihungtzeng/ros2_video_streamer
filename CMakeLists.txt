cmake_minimum_required(VERSION 3.8)
project(ros2_video_streamer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)

ament_auto_add_executable(video_streamer_node src/video_streamer_node.cpp)
ament_target_dependencies(video_streamer_node
    rclcpp
    std_msgs
    sensor_msgs
    OpenCV)

install(
  TARGETS video_streamer_node
  EXPORT export_{PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_auto_package(INSTALL_TO_SHARE launch)
