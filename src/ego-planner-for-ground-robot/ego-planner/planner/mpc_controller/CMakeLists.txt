cmake_minimum_required(VERSION 3.8)
project(mpc_controller)

# ROS 2 Humble port
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-DHAVE_CSTDDEF -DHAVE_MPI_INITIALIZED)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# ---------------------------------------------------------------------------
# qpOASES: use an installed copy if available, otherwise build the copy that
# ships inside this repository (ego-planner/qpOASES).
# ---------------------------------------------------------------------------
find_library(QPOASES_LIBRARY NAMES qpOASES PATHS /usr/local/lib /usr/lib NO_CMAKE_SYSTEM_PATH)
find_path(QPOASES_INCLUDE_DIR NAMES qpOASES.hpp PATHS /usr/local/include /usr/include NO_CMAKE_SYSTEM_PATH)

if(QPOASES_LIBRARY AND QPOASES_INCLUDE_DIR)
  message(STATUS "mpc_controller: using installed qpOASES (${QPOASES_LIBRARY})")
  set(QPOASES_LINK ${QPOASES_LIBRARY})
  set(QPOASES_INCLUDE ${QPOASES_INCLUDE_DIR})
  set(QPOASES_INSTALL_INCLUDE ${QPOASES_INCLUDE_DIR})
  set(QPOASES_VENDORED FALSE)
else()
  set(_qpoases_src "${CMAKE_CURRENT_SOURCE_DIR}/../../qpOASES")
  if(NOT EXISTS "${_qpoases_src}/CMakeLists.txt")
    message(FATAL_ERROR "qpOASES not found. Install it into /usr/local (see README) "
                        "or keep the vendored copy at ${_qpoases_src}.")
  endif()
  message(STATUS "mpc_controller: building vendored qpOASES from ${_qpoases_src}")
  set(QPOASES_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  # qpOASES is linked into a shared library, so its objects must be position independent.
  set(CMAKE_POSITION_INDEPENDENT_CODE ON)
  add_subdirectory("${_qpoases_src}" "${CMAKE_CURRENT_BINARY_DIR}/qpOASES_build" EXCLUDE_FROM_ALL)
  # qpOASES is a plain (non-ROS) CMake project that is *not* installed into the
  # ament prefix, therefore it is linked privately. Its static objects are embedded
  # into libmpc_controller.so, so downstream packages only link mpc_controller.
  set(QPOASES_LINK qpOASES)
  set(QPOASES_INCLUDE "${_qpoases_src}/include")
  set(QPOASES_VENDORED TRUE)
  # The vendored headers are installed together with this package, so that
  # downstream packages (traj_server) can include <qpOASES.hpp>.
  set(QPOASES_INSTALL_INCLUDE "include")
endif()

if(QPOASES_VENDORED)
  install(DIRECTORY ${QPOASES_INCLUDE}/ DESTINATION include)
endif()

add_library(${PROJECT_NAME} SHARED
  src/MPC.cpp
  src/uniform_bspline.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${QPOASES_INCLUDE}>
  $<INSTALL_INTERFACE:${QPOASES_INSTALL_INCLUDE}>
  ${EIGEN3_INCLUDE_DIRS}
)
target_compile_options(${PROJECT_NAME} PRIVATE -O3 -Wall -g)
ament_target_dependencies(${PROJECT_NAME} PUBLIC rclcpp)
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen)
target_link_libraries(${PROJECT_NAME} PRIVATE ${QPOASES_LINK})

add_executable(qp_test app/qp_test.cpp)
target_include_directories(qp_test PRIVATE ${QPOASES_INCLUDE})
target_link_libraries(qp_test PRIVATE ${QPOASES_LINK} Eigen3::Eigen)

add_executable(mpc_test
  app/mpc_control_test.cpp
  src/uniform_bspline.cpp
  src/MPC.cpp
)
target_include_directories(mpc_test PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${QPOASES_INCLUDE}
  ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(mpc_test PUBLIC rclcpp geometry_msgs nav_msgs)
target_link_libraries(mpc_test PRIVATE Eigen3::Eigen ${QPOASES_LINK})

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS qp_test mpc_test DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp Eigen3)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(tracking_test test/tracking_test.cpp)
  target_link_libraries(tracking_test ${PROJECT_NAME})
endif()

ament_package()
