cmake_minimum_required(VERSION 3.8)
project(dock_action)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/src
    )

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(action_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(irobot_create_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(depthai REQUIRED)


add_executable(dock_action_server src/dock_action.cpp)
ament_target_dependencies(dock_action_server rclcpp rclcpp_action rclcpp_components action_interfaces sensor_msgs irobot_create_msgs cv_bridge depthai OpenCV)

add_executable(test_publisher src/test_publisher.cpp)
ament_target_dependencies(test_publisher rclcpp rclcpp_action rclcpp_components action_interfaces sensor_msgs irobot_create_msgs cv_bridge depthai OpenCV)

include_directories( ${OpenCV_INCLUDE_DIRS} )
target_link_libraries(dock_action_server ${OpenCV_LIBS} )


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  dock_action_server
  test_publisher
  DESTINATION lib/${PROJECT_NAME})
ament_package()
