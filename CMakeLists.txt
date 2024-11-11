cmake_minimum_required(VERSION 3.8)
project(bebop_simulator_r2)

set(CMAKE_CXX_STANDARD 14)

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
find_package(trajectory_msgs REQUIRED)

find_package(
  Eigen3 REQUIRED HINTS ${Eigen3_HINT}
)
include_directories(/usr/include/eigen3)
# Install launch files.
install(DIRECTORY
  launch
  urdf
  worlds
  meshes
  DESTINATION share/${PROJECT_NAME}/
)

# add libraries
add_library(${PROJECT_NAME}_lib SHARED
   src/library/position_controller.cpp
   src/library/extendedKalmanFilter.cpp
   src/library/waypoint_filter.cpp
   #    
)
add_library(position_controller_with_bebop SHARED
  src/library/position_controller_with_bebop.cpp
)

ament_target_dependencies(${PROJECT_NAME}_lib
  "rclcpp"
  "geometry_msgs"
  "gazebo_msgs"
  "mav_msgs"
  "nav_msgs"
  "std_msgs"
  "sensor_msgs"
  "trajectory_msgs"
)
ament_target_dependencies(position_controller_with_bebop
  "rclcpp"
  "geometry_msgs"
  "gazebo_msgs"
  "mav_msgs"
  "nav_msgs"
  "std_msgs"
  "sensor_msgs"
  "trajectory_msgs"
)

target_include_directories(${PROJECT_NAME}_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_include_directories(position_controller_with_bebop PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

# The ament_export_targets macro exports the targets for CMake. 
# This is necessary to allow your library’s clients to use the target_link_libraries(client my_library::my_library) syntax. 
# ament_export_include_directories("include/${PROJECT_NAME}")




ament_export_targets(export_${PROJECT_NAME}_lib HAS_LIBRARY_TARGET)
# ament_export_targets(export_position_controller_with_bebop HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  # Eigen3::Eigen
)
target_link_libraries(${PROJECT_NAME}_lib
Eigen3::Eigen

)
target_link_libraries(position_controller_with_bebop
Eigen3::Eigen
${PROJECT_NAME}_lib
)


install(
  DIRECTORY include/${PROJECT_NAME}
  DIRECTORY include/bebop_simulator_msgs/
  DIRECTORY include/bebop_msgs/
  DESTINATION include
  FILES_MATCHING PATTERN "*.h")

install(
  TARGETS 
  ${PROJECT_NAME}_lib 
  position_controller_with_bebop 
  EXPORT 
  export_${PROJECT_NAME}_lib 
  # export_position_controller_with_bebop 
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include

  )

# Add exe, nodes
add_executable(position_controller_node 
  src/nodes/position_controller_node.cpp
)
ament_target_dependencies(position_controller_node
    "rclcpp"
    "geometry_msgs"
    "gazebo_msgs"
    "mav_msgs"
    "nav_msgs"
    "std_msgs"
    "sensor_msgs"
    "trajectory_msgs"
)
target_link_libraries(position_controller_node 
  ${PROJECT_NAME}_lib 
)
# This adds all files in the folder ${CMAKE_CURRENT_SOURCE_DIR}/include to the public interface during build time and all files
#  in the include folder (relative to ${CMAKE_INSTALL_DIR}) when being installed.
target_include_directories(position_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(position_controller_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS position_controller_node
  DESTINATION lib/${PROJECT_NAME}/)


# ament_export_dependencies(
#   rclcpp
#   geometry_msgs
#   gazebo_msgs
#   mav_msgs
#   nav_msgs
#   std_msgs
#   roscpp
#   sensor_msgs
#   # cmake_modules
#   Eigen
#   )
# ament_export_libraries(
  
# )
ament_package()
