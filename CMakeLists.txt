cmake_minimum_required(VERSION 3.0.2)
project(mujoco_ros_mocap_control_plugin C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
include(ProjectOption)

find_package(catkin REQUIRED
  roscpp
  mujoco_ros
)

find_package(mujoco 2.3.6 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mujoco_ros_mocap_control_plugin mocap_syncronizer_callback
  CATKIN_DEPENDS
    roscpp
    mujoco_ros
  DEPENDS
    mujoco
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

add_library(mocap_syncronizer_callback
  src/mocap_syncronizer_callback.c
)
target_include_directories(mocap_syncronizer_callback SYSTEM PUBLIC
  ${mujoco_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(mocap_syncronizer_callback
  ${mujoco_LIBRARIES}
  ${catkin_LIBRARIES}
)
install(TARGETS mocap_syncronizer_callback
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)