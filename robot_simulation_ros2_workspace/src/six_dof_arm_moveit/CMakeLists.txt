cmake_minimum_required(VERSION 3.22)
project(six_dof_arm_moveit)

find_package(ament_cmake REQUIRED)

ament_package()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

find_package(rclcpp REQUIRED)
find_package(ros2_igtl_bridge REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# C++ brain mesh processor
add_executable(mesh_processor src/my_robot_goal/mesh_processor.cpp)
ament_target_dependencies(brain_mesh_processor
  rclcpp
  ros2_igtl_bridge
  visualization_msgs
  geometry_msgs
  std_msgs
)

install(TARGETS
  brain_mesh_processor
  DESTINATION lib/${PROJECT_NAME}
)
