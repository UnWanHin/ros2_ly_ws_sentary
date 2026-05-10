cmake_minimum_required(VERSION 3.16)
project(navi_tf_bridge)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(auto_aim_common REQUIRED)
find_package(gimbal_driver REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(target_rel_to_goal_pos_node
  src/chase_area_limiter.cpp
  src/chase_pointer.cpp
  src/goal_output.cpp
  src/map_pointer.cpp
  src/pointer_debug.cpp
  src/pointer_solver.cpp
  src/target_rel_to_goal_pos_node.cpp
)

target_include_directories(target_rel_to_goal_pos_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_executable(map_aim_point_node
  src/pointer_solver.cpp
  src/pointer_solver_node.cpp
)

target_include_directories(map_aim_point_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(target_rel_to_goal_pos_node
  rclcpp
  std_msgs
  geometry_msgs
  auto_aim_common
  tf2
  tf2_ros
  tf2_geometry_msgs
)

ament_target_dependencies(map_aim_point_node
  rclcpp
  std_msgs
  geometry_msgs
  gimbal_driver
  tf2
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS
  target_rel_to_goal_pos_node
  map_aim_point_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  script/navitomap_input_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME navitomap_input_node
)

install(PROGRAMS
  script/navi_calib_simple.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME navi_calib_simple
)

install(PROGRAMS
  script/kabsch_calib.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME kabsch_calib
)

install(PROGRAMS
  script/affine_calib.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME affine_calib
)

install(PROGRAMS
  script/mock_gimbal_state_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME mock_gimbal_state_node
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
