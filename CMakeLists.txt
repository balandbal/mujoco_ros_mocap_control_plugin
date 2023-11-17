cmake_minimum_required(VERSION 3.0.2)
project(mujoco_ros_mocap_control_plugin)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED
  roscpp
  mujoco_ros
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mujoco_ros_mocap_control_plugin
  CATKIN_DEPENDS
    roscpp
    mujoco_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(mujoco_ros_mocap_control_plugin src/mocap_control_plugin.cpp)
target_include_directories(mujoco_ros_mocap_control_plugin SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(mujoco_ros_mocap_control_plugin ${catkin_LIBRARIES})
install(TARGETS mujoco_ros_mocap_control_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES
  mujoco_ros_mocap_control_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)