cmake_minimum_required(VERSION 3.8)
project(ego_planner)

# ROS 2 Humble port
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(plan_env REQUIRED)
find_package(path_searching REQUIRED)
find_package(bspline_opt REQUIRED)
find_package(traj_utils REQUIRED)
find_package(mpc_controller REQUIRED)

# Generate messages in the 'msg' folder
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Bspline.msg"
  "msg/DataDisp.msg"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

set(EGO_PLANNER_INCLUDE_DIRS
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(ego_planner_node
  src/ego_planner_node.cpp
  src/ego_replan_fsm.cpp
  src/planner_manager.cpp
)
target_include_directories(ego_planner_node PRIVATE ${EGO_PLANNER_INCLUDE_DIRS})
ament_target_dependencies(ego_planner_node PUBLIC
  rclcpp std_msgs geometry_msgs nav_msgs sensor_msgs visualization_msgs tf2)
target_link_libraries(ego_planner_node PRIVATE
  plan_env::plan_env
  path_searching::path_searching
  bspline_opt::bspline_opt
  traj_utils::traj_utils
  mpc_controller::mpc_controller
  Eigen3::Eigen
  ${PCL_LIBRARIES}
)

add_executable(traj_server src/traj_server.cpp)
target_include_directories(traj_server PRIVATE ${EGO_PLANNER_INCLUDE_DIRS})
ament_target_dependencies(traj_server PUBLIC
  rclcpp std_msgs geometry_msgs nav_msgs sensor_msgs visualization_msgs tf2)
target_link_libraries(traj_server PRIVATE
  bspline_opt::bspline_opt
  mpc_controller::mpc_controller
  Eigen3::Eigen
  ${PCL_LIBRARIES}
)

# make the generated interfaces available to the executables
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(ego_planner_node PRIVATE "${cpp_typesupport_target}")
target_link_libraries(traj_server PRIVATE "${cpp_typesupport_target}")

install(TARGETS ego_planner_node traj_server
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(inflation_escape_test test/inflation_escape_test.cpp)
  target_include_directories(inflation_escape_test PRIVATE include)
  target_link_libraries(inflation_escape_test Eigen3::Eigen)
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
