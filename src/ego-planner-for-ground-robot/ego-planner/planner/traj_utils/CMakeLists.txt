cmake_minimum_required(VERSION 3.8)
project(traj_utils)

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
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(bspline_opt REQUIRED)
find_package(path_searching REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/planning_visualization.cpp
  src/polynomial_traj.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
target_compile_options(${PROJECT_NAME} PRIVATE -O3 -Wall -g)
ament_target_dependencies(${PROJECT_NAME} PUBLIC
  rclcpp
  geometry_msgs
  visualization_msgs
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  Eigen3::Eigen
  bspline_opt::bspline_opt
  path_searching::path_searching
  ${PCL_LIBRARIES}
)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp geometry_msgs visualization_msgs bspline_opt path_searching Eigen3 PCL)

ament_package()
