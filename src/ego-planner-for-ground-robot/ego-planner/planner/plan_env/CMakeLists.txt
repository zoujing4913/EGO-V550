cmake_minimum_required(VERSION 3.8)
project(plan_env)

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
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)

# NOTE: pcl_conversions is header-only in ROS 2 and exports a *broken* classic
# dependency list (pcl_msgs::... target names as plain libraries). Therefore only
# its include directory is consumed here instead of ament_target_dependencies().
link_directories(${PCL_LIBRARY_DIRS})

add_library(${PROJECT_NAME} SHARED
  src/grid_map.cpp
  src/raycast.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
target_compile_options(${PROJECT_NAME} PRIVATE -O3 -Wall -g)
ament_target_dependencies(${PROJECT_NAME} PUBLIC
  rclcpp
  cv_bridge
  message_filters
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_msgs
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  Eigen3::Eigen
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
target_include_directories(${PROJECT_NAME} PUBLIC ${pcl_conversions_INCLUDE_DIRS})

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp cv_bridge message_filters sensor_msgs geometry_msgs nav_msgs Eigen3 PCL OpenCV)

ament_package()
