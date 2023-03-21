cmake_minimum_required(VERSION 3.8)
project(mci_rmf)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_srvs REQUIRED)
find_package(h_interfaces REQUIRED)
find_package(gui_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(irobot_create_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(action_interfaces REQUIRED)

## add include dirs
include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/src
    )

## create executables with dependencies
# task_planner
add_executable(task_planner src/TaskPlanner.cpp src/TaskPlanner_client.cpp src/TaskPlanner_receiver.cpp 
                            src/TaskPlanner_landmarks.cpp src/TaskPlanner_distributor.cpp src/TaskPlanner_misc_callbacks.cpp)
ament_target_dependencies(task_planner rclcpp h_interfaces gui_interfaces std_srvs)

# turtle_client
add_executable(turtle_client src/TurtleClient.cpp src/TurtleClient_clientState.cpp src/TurtleClient_clientTask.cpp
                             src/TurtleClient_misc_callbacks.cpp)
ament_target_dependencies(turtle_client rclcpp rclcpp_action rclcpp_components h_interfaces geometry_msgs
                                        irobot_create_msgs nav2_msgs sensor_msgs action_interfaces)

# task_generator
add_executable(task_generator src/TaskGenerator.cpp)
ament_target_dependencies(task_generator rclcpp h_interfaces gui_interfaces)

## install ros2 dirs
install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}
    )

## install nodes
install(TARGETS
    task_planner
    task_generator
    turtle_client
    DESTINATION lib/${PROJECT_NAME})

ament_package()