cmake_minimum_required(VERSION 3.5)
project(plannergan_iiwa)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Include C++ header files
include_directories(include)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(iiwa_description REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)

# Add the executable for the C++ node
add_executable(iiwa_motion_planning_node src/iiwa_motion_planning_node.cpp)
ament_target_dependencies(iiwa_motion_planning_node
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
)

# Declare executable
add_executable(motion_planning_node src/motion_planning_node.cpp)
ament_target_dependencies(motion_planning_node
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  moveit_core
  tf2
  tf2_ros
  tf2_eigen
)

# Declare executable
add_executable(traj2position_node src/traj2position_node.cpp)
ament_target_dependencies(traj2position_node
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  moveit_core
  tf2
  tf2_ros
  tf2_eigen
)

# Install executables
install(TARGETS
  motion_planning_node
  traj2position_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts
# install(PROGRAMS
#  src/rrt_planning.py
#  DESTINATION lib/${PROJECT_NAME}
# )

# Install C++ executables
install(TARGETS
  iiwa_motion_planning_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()