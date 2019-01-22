cmake_minimum_required(VERSION 2.8.3)
project(gpd_utils)

#Set compiler according C++11 support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
  message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has C++11 support.")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -DNDEBUG -O3 -fopenmp -Wno-deprecated -Wenum-compare")
elseif(COMPILER_SUPPORTS_CXX0X)
  message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has C++0x support.")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -DNDEBUG -O3 -fopenmp -Wno-deprecated -Wenum-compare")
else()
  message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  cmake_modules
  actionlib_msgs
  geometry_msgs
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  visualization_msgs
  gpd
  pal_detection_msgs
  message_generation
  smach_c
  smach_c_pal_states
  smach_c_ros_states
  std_msgs
  tf
  message_filters
  pal_ariles
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

find_library(GENERATOR_LIB grasp_candidates_generator)
find_path(GENERATOR_LIB_INCLUDE_DIR gpg/grasp.h)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   BoundingBox.msg)

## Generate actions in the 'action' folder
add_action_files(
   FILES
   GraspCandidatesGeneration.action
 )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   actionlib_msgs sensor_msgs#   geometry_msgs#   visualization_msgs
)

set(STATES
  tabletop_segmentation_state
  tabletop_clustering_state
  generate_grasping_candidates_state
  object_cloud_extraction_state
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${STATES}
  CATKIN_DEPENDS actionlib actionlib_msgs geometry_msgs gpd pcl_conversions pcl_ros roscpp sensor_msgs visualization_msgs smach_c smach_c_pal_states smach_c_ros_states std_msgs tf message_filters pal_ariles
  DEPENDS system_lib Eigen PCL
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${GENERATOR_LIB_INCLUDE_DIR}
)

## Declare a C++ library
add_library(${PROJECT_NAME} src/pcl_filters.cpp src/tabletop_detector_class.cpp src/segment_table.cpp src/object_recognition_info.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(pal_grasp_detection_node src/pal_grasp_detection_node.cpp)
add_executable(pal_grasp_generation_server src/pal_grasp_generation_server.cpp)
add_executable(pal_grasp_generation_client src/pal_grasp_generation_client.cpp)

add_dependencies(pal_grasp_detection_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(pal_grasp_generation_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(pal_grasp_generation_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(pal_grasp_detection_node  ${catkin_LIBRARIES} ${GENERATOR_LIB})
target_link_libraries(pal_grasp_generation_server  ${catkin_LIBRARIES} ${GENERATOR_LIB})
target_link_libraries(pal_grasp_generation_client  ${catkin_LIBRARIES} ${GENERATOR_LIB})

foreach(state ${STATES})
    add_library(${state} src/states/${state}.cpp)
    add_dependencies(${state} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    target_link_libraries(${state} ${catkin_LIBRARIES} ${PROJECT_NAME})
endforeach()

add_executable(segment_table_node src/segment_table_node.cpp)
target_link_libraries(segment_table_node ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(state_machine_node src/state_machine_node.cpp)
target_link_libraries(state_machine_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${STATES})
add_dependencies(state_machine_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS pal_grasp_detection_node pal_grasp_generation_server pal_grasp_generation_client
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS segment_table_node state_machine_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)

