cmake_minimum_required(VERSION 2.8.3)
project(bebop_simulator)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  gazebo_msgs
  mav_msgs
  nav_msgs
  std_msgs
  roscpp
  sensor_msgs
  cmake_modules
)

find_package(
  Eigen REQUIRED
)

catkin_package(
  INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS}
  LIBRARIES position_controller extendedKalmanFilter position_controller_with_bebop waypoint_filter
  CATKIN_DEPENDS geometry_msgs mav_msgs nav_msgs roscpp sensor_msgs std_msgs
  DEPENDS Eigen 
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_library(position_controller
   src/library/position_controller.cpp
)

add_library(position_controller_with_bebop
   src/library/position_controller_with_bebop.cpp
)

add_library(extendedKalmanFilter
   src/library/extendedKalmanFilter.cpp
)

add_library(waypoint_filter
   src/library/waypoint_filter.cpp
)

target_link_libraries(position_controller ${catkin_LIBRARIES})
add_dependencies(position_controller ${catkin_EXPORTED_TARGETS})

target_link_libraries(position_controller_with_bebop ${catkin_LIBRARIES})
add_dependencies(position_controller_with_bebop ${catkin_EXPORTED_TARGETS})

target_link_libraries(extendedKalmanFilter ${catkin_LIBRARIES})
add_dependencies(extendedKalmanFilter ${catkin_EXPORTED_TARGETS})

target_link_libraries(waypoint_filter ${catkin_LIBRARIES})
add_dependencies(waypoint_filter ${catkin_EXPORTED_TARGETS})

add_executable(position_controller_node src/nodes/position_controller_node.cpp)
add_dependencies(position_controller_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(position_controller_node
  position_controller extendedKalmanFilter waypoint_filter ${catkin_LIBRARIES})

add_executable(position_controller_with_bebop_node src/nodes/position_controller_with_bebop_node.cpp)
add_dependencies(position_controller_with_bebop_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(position_controller_with_bebop_node
  position_controller_with_bebop extendedKalmanFilter waypoint_filter  ${catkin_LIBRARIES})

add_executable(hovering_example src/nodes/hovering_example.cpp)
target_link_libraries(hovering_example ${catkin_LIBRARIES})
add_dependencies(hovering_example ${catkin_EXPORTED_TARGETS})

add_executable(waypoint_example src/nodes/waypoint_example.cpp)
target_link_libraries(waypoint_example ${catkin_LIBRARIES})
add_dependencies(waypoint_example ${catkin_EXPORTED_TARGETS})

add_executable(quaternion_to_rpy src/nodes/quaternion_to_rpy.cpp)
target_link_libraries(quaternion_to_rpy ${catkin_LIBRARIES})
add_dependencies(quaternion_to_rpy ${catkin_EXPORTED_TARGETS})

install(TARGETS position_controller extendedKalmanFilter position_controller_with_bebop waypoint_filter
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS position_controller position_controller_with_bebop_node hovering_example waypoint_example quaternion_to_rpy
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DIRECTORY include/bebop_simulator_msgs/
  DIRECTORY include/bebop_msgs/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
