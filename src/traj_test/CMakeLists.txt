cmake_minimum_required(VERSION 3.8)
project(traj_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(offboard_state_machine REQUIRED)

include_directories(
  include
)

add_executable(traj_test_node
  src/traj_test_node.cpp
  src/traj_test_main.cpp
)

ament_target_dependencies(traj_test_node
  rclcpp
  std_msgs
  px4_msgs
  offboard_state_machine
)

add_executable(follow_traj_node
  src/follow_traj_node.cpp
  src/follow_traj_main.cpp
)

ament_target_dependencies(follow_traj_node
  rclcpp
  std_msgs
  px4_msgs
  offboard_state_machine
)

add_executable(swarm_coordinator_node
  src/swarm_coordinator_node.cpp
  src/swarm_coordinator_main.cpp
)

ament_target_dependencies(swarm_coordinator_node
  rclcpp
  std_msgs
)

add_executable(swarm_goto_coordinator_node
  src/swarm_goto_coordinator_node.cpp
  src/swarm_goto_coordinator_main.cpp
)

ament_target_dependencies(swarm_goto_coordinator_node
  rclcpp
  std_msgs
  px4_msgs
)

install(DIRECTORY include/
  DESTINATION include
)

# Install targets
install(TARGETS
  traj_test_node
  follow_traj_node
  swarm_coordinator_node
  swarm_goto_coordinator_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
