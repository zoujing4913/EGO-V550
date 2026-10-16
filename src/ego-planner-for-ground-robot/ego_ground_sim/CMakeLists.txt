cmake_minimum_required(VERSION 3.8)
project(ego_ground_sim)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ego_planner REQUIRED)
find_package(bspline_opt REQUIRED)

add_executable(point_cloud_transformer src/point_cloud_transformer.cpp)
ament_target_dependencies(point_cloud_transformer PUBLIC
  rclcpp sensor_msgs geometry_msgs nav_msgs tf2 tf2_ros tf2_sensor_msgs)
target_link_libraries(point_cloud_transformer PRIVATE Eigen3::Eigen)

add_executable(cmd_vel_safety src/cmd_vel_safety.cpp)
ament_target_dependencies(cmd_vel_safety PUBLIC
  rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs tf2 tf2_ros tf2_geometry_msgs)
target_include_directories(cmd_vel_safety PRIVATE include)

add_executable(trajectory_visualizer src/trajectory_visualizer.cpp)
ament_target_dependencies(trajectory_visualizer rclcpp nav_msgs geometry_msgs ego_planner)
target_link_libraries(trajectory_visualizer bspline_opt::bspline_opt)

install(TARGETS point_cloud_transformer cmd_vel_safety trajectory_visualizer
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch urdf worlds config meshes
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/closed_loop_check.py
  scripts/indoor_free_space.py
  scripts/robot_motion_check.py
  scripts/v550_sim_check.py
  scripts/focus_v550.py
  scripts/save_v550_world.py
  scripts/make_v550_rooms.py
  DESTINATION share/${PROJECT_NAME}/scripts)

# ---------------------------------------------------------------------------
# CMU indoor scene (optional).
# The mesh already ships inside this repository (it belongs to the ROS 1
# simulation environment, which is not built). Installing it as a Gazebo model
# makes 'model://indoor' resolvable from worlds/cmu_indoor.world without
# sourcing or copying anything by hand.
# ---------------------------------------------------------------------------
set(_cmu_mesh "${CMAKE_CURRENT_SOURCE_DIR}/../autonomous_exploration_development_environment/src/vehicle_simulator/mesh")
if(EXISTS "${_cmu_mesh}/indoor/model.sdf")
  message(STATUS "${PROJECT_NAME}: installing CMU indoor model from ${_cmu_mesh}/indoor")
  install(DIRECTORY "${_cmu_mesh}/indoor"
    DESTINATION share/${PROJECT_NAME}/models
    PATTERN "preview" EXCLUDE)
else()
  message(WARNING
    "${PROJECT_NAME}: CMU indoor mesh not found at ${_cmu_mesh}/indoor - "
    "worlds/cmu_indoor.world will not be usable "
    "(run autonomous_exploration_development_environment/src/vehicle_simulator/"
    "mesh/download_environments.sh to get the other scenes too)")
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(scan_guard_test test/scan_guard_test.cpp)
  target_include_directories(scan_guard_test PRIVATE include)
endif()

ament_package()
