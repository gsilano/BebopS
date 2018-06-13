cmake_minimum_required(VERSION 2.8.3)
project(teamsannio_med_control)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
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
  LIBRARIES position_controller extendedKalmanFilter position_controller_with_bebop
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

target_link_libraries(position_controller ${catkin_LIBRARIES})
add_dependencies(position_controller ${catkin_EXPORTED_TARGETS})

target_link_libraries(position_controller_with_bebop ${catkin_LIBRARIES})
add_dependencies(position_controller_with_bebop ${catkin_EXPORTED_TARGETS})

target_link_libraries(extendedKalmanFilter ${catkin_LIBRARIES})
add_dependencies(extendedKalmanFilter ${catkin_EXPORTED_TARGETS})

add_executable(position_controller_node src/nodes/position_controller_node.cpp)
add_dependencies(position_controller_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(position_controller_node
  position_controller extendedKalmanFilter ${catkin_LIBRARIES})

add_executable(position_controller_with_bebop_node src/nodes/position_controller_with_bebop_node.cpp)
add_dependencies(position_controller_with_bebop_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(position_controller_with_bebop_node
  position_controller_with_bebop extendedKalmanFilter ${catkin_LIBRARIES})

install(TARGETS position_controller extendedKalmanFilter position_controller_with_bebop
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS position_controller position_controller_with_bebop_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DIRECTORY include/teamsannio_msgs/
  DIRECTORY include/bebop_msgs/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
