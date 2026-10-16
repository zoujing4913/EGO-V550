cmake_minimum_required(VERSION 3.8)
project(path_searching)

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
find_package(plan_env REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/dyn_a_star.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
target_compile_options(${PROJECT_NAME} PRIVATE -O3 -Wall -g)
ament_target_dependencies(${PROJECT_NAME} PUBLIC rclcpp)
target_link_libraries(${PROJECT_NAME} PUBLIC
  Eigen3::Eigen
  plan_env::plan_env
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
ament_export_dependencies(rclcpp plan_env Eigen3 PCL)

ament_package()
