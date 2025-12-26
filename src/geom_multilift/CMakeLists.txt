cmake_minimum_required(VERSION 3.8)
project(geom_multilift)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)

add_executable(realflight_traj_node
  src/realflight_traj_node.cpp
  src/realflight_traj_main.cpp
)
ament_target_dependencies(realflight_traj_node
  rclcpp
  px4_msgs
  std_msgs
)

add_executable(geom_multilift_node
  src/data_loader.cpp
  src/geom_multilift_node.cpp
  src/geom_multilift_main.cpp
)
ament_target_dependencies(geom_multilift_node
  rclcpp
  geometry_msgs
  px4_msgs
  std_msgs
)
target_link_libraries(geom_multilift_node Eigen3::Eigen)

add_executable(geom_multilift_sitl_node
  src/data_loader.cpp
  src/data_loader_new.cpp
  src/geom_multilift_node.cpp  # reuse AccKalmanFilter implementation
  src/geom_multilift_sitl_node.cpp
  src/geom_multilift_sitl_main.cpp
)
ament_target_dependencies(geom_multilift_sitl_node
  rclcpp
  geometry_msgs
  nav_msgs
  px4_msgs
  std_msgs
)
target_link_libraries(geom_multilift_sitl_node Eigen3::Eigen)

add_executable(geom_multilift_realflight_node
  src/data_loader_new.cpp
  src/geom_multilift_realflight_node.cpp
  src/geom_multilift_realflight_main.cpp
)
ament_target_dependencies(geom_multilift_realflight_node
  rclcpp
  geometry_msgs
  px4_msgs
  std_msgs
)
target_link_libraries(geom_multilift_realflight_node Eigen3::Eigen)

add_executable(geom_multilift_hitl_node
  src/data_loader_new.cpp
  src/geom_multilift_hitl_node.cpp
  src/geom_multilift_hitl_main.cpp
)
ament_target_dependencies(geom_multilift_hitl_node
  rclcpp
  geometry_msgs
  nav_msgs
  px4_msgs
  std_msgs
)
target_link_libraries(geom_multilift_hitl_node Eigen3::Eigen)

install(TARGETS
  realflight_traj_node
  geom_multilift_node
  geom_multilift_sitl_node
  geom_multilift_realflight_node
  geom_multilift_hitl_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

install(FILES README.md
  DESTINATION share/${PROJECT_NAME})

ament_package()
