cmake_minimum_required(VERSION 3.8)
project(line_follower)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(line_follower src/linefollower.cpp)
ament_target_dependencies(line_follower rclcpp sensor_msgs geometry_msgs nav_msgs tf2 tf2_geometry_msgs cv_bridge)
target_include_directories(line_follower PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS})
target_link_libraries(line_follower ${OpenCV_LIBS})

add_executable(camera_viewer src/camera_viewer.cpp)
ament_target_dependencies(camera_viewer rclcpp sensor_msgs cv_bridge image_transport)
target_include_directories(camera_viewer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS})
target_link_libraries(camera_viewer ${OpenCV_LIBS})

install(TARGETS line_follower camera_viewer DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(PROGRAMS
  scripts/camera_publisher.py
  scripts/lane_detector.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
