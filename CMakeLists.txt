cmake_minimum_required(VERSION 3.8)
project(bebop_simulator_r2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(mav_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
# find_package(cmake_modules REQUIRED)

find_package(
  Eigen3 REQUIRED
)

# add libraries
add_library(position_controller SHARED
   src/library/position_controller.cpp
)

add_library(position_controller_with_bebop SHARED
   src/library/position_controller_with_bebop.cpp
)

add_library(extendedKalmanFilter SHARED
   src/library/extendedKalmanFilter.cpp
)

add_library(waypoint_filter SHARED
   src/library/waypoint_filter.cpp
)
target_include_directories(position_controller PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_include_directories(position_controller_with_bebop PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_include_directories(extendedKalmanFilter PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_include_directories(waypoint_filter PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
# The ament_export_targets macro exports the targets for CMake. 
# This is necessary to allow your library’s clients to use the target_link_libraries(client my_library::my_library) syntax. 
# ament_export_include_directories("include/${PROJECT_NAME}")




ament_export_targets(export_position_controller HAS_LIBRARY_TARGET)
ament_export_targets(export_position_controller_with_bebop HAS_LIBRARY_TARGET)
ament_export_targets(export_extendedKalmanFilter HAS_LIBRARY_TARGET)
ament_export_targets(export_waypoint_filter HAS_LIBRARY_TARGET)

target_link_libraries(position_controller ${${LIBS}})
target_link_libraries(position_controller_with_bebop ${${LIBS}})
target_link_libraries(extendedKalmanFilter ${${LIBS}})
target_link_libraries(waypoint_filter ${${LIBS}})

install(
  DIRECTORY include/${PROJECT_NAME}/bebop_simulator
  DIRECTORY include/${PROJECT_NAME}/bebop_simulator_msgs/
  DIRECTORY include/${PROJECT_NAME}/bebop_msgs/
  DESTINATION include
  FILES_MATCHING PATTERN "*.h")

install(
  TARGETS 
  position_controller 
  position_controller_with_bebop 
  extendedKalmanFilter 
  waypoint_filter
  EXPORT 
  export_position_controller 
  export_position_controller_with_bebop 
  export_extendedKalmanFilter 
  export_waypoint_filter
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include

  )

# Add exe, nodes
add_executable(position_controller_node src/nodes/position_controller_node.cpp)
# This adds all files in the folder ${CMAKE_CURRENT_SOURCE_DIR}/include to the public interface during build time and all files
#  in the include folder (relative to ${CMAKE_INSTALL_DIR}) when being installed.
target_include_directories(position_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(position_controller_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(position_controller_node position_controller position_controller_with_bebop extendedKalmanFilter waypoint_filter ${${LIBS}})
  ament_target_dependencies(
    position_controller_node
    "rclcpp"
    "geometry_msgs"
    "gazebo_msgs"
    "mav_msgs"
    "nav_msgs"
    "std_msgs"
    "sensor_msgs"
    # "cmake_modules"
  )

install(TARGETS position_controller_node
  DESTINATION lib/${PROJECT_NAME})




ament_export_dependencies(rclcpp
geometry_msgs
  gazebo_msgs
  mav_msgs
  nav_msgs
  std_msgs
  roscpp
  sensor_msgs
  # cmake_modules
  Eigen
  )
ament_export_libraries(
  
)
ament_package()
