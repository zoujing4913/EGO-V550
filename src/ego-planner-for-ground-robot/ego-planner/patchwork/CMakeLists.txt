cmake_minimum_required(VERSION 3.8)
project(patchwork)

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
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
# pcl_conversions is header-only in ROS 2; only its include dirs are used (see plan_env).
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Node.msg"
  "msg/GroundEstimate.msg"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${pcl_conversions_INCLUDE_DIRS}
)

# Ground segmentation node used by the ego-planner pipeline
# (/pointcloud_scan -> /benchmark/{cloud,P,N})
add_executable(ground_segmentation nodes/offline_own_data.cpp)
ament_target_dependencies(ground_segmentation PUBLIC
  rclcpp sensor_msgs std_msgs geometry_msgs visualization_msgs
  tf2 tf2_ros tf2_sensor_msgs)
target_link_libraries(ground_segmentation PRIVATE
  ${PCL_LIBRARIES}
  Boost::boost
)

install(TARGETS ground_segmentation
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
