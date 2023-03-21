cmake_minimum_required(VERSION 3.5)
project(tt_cpp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(h_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(tt_cpp ${PROJECT_SOURCE_DIR}/include)

add_executable(elevator_server src/elevator_server.cpp)
ament_target_dependencies(elevator_server rclcpp h_interfaces)
target_link_libraries(elevator_server gpiod)

add_executable(elevator_client src/elevator_client.cpp)
ament_target_dependencies(elevator_client rclcpp h_interfaces)

add_executable(orientation_logger src/orientation_logger.cpp)
ament_target_dependencies(orientation_logger rclcpp geometry_msgs nav_msgs std_msgs sensor_msgs)

add_executable(odom_publisher src/odom_publisher.cpp)
ament_target_dependencies(odom_publisher rclcpp nav_msgs)

install(DIRECTORY
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
  )

install(TARGETS
  elevator_server
  elevator_client
  orientation_logger
  odom_publisher
  DESTINATION lib/${PROJECT_NAME})

ament_package()