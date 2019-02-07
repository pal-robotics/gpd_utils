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
  pal_pcl
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

find_library(GENERATOR_LIB grasp_candidates_generator)
find_path(GENERATOR_LIB_INCLUDE_DIR gpg/grasp.h)

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
  CATKIN_DEPENDS actionlib actionlib_msgs geometry_msgs gpd pcl_conversions pcl_ros roscpp sensor_msgs visualization_msgs smach_c smach_c_pal_states smach_c_ros_states std_msgs tf message_filters pal_ariles pal_pcl
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

add_library(${PROJECT_NAME} src/states/tabletop_segmentation_state.cpp
    src/states/tabletop_clustering_state.cpp
    src/states/generate_grasping_candidates_state.cpp
    src/states/object_cloud_extraction_state.cpp
    src/tabletop_detector_class.cpp
    src/segment_table.cpp
    src/object_recognition_info.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GENERATOR_LIB})

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

add_executable(segment_table_node src/segment_table_node.cpp)
target_link_libraries(segment_table_node ${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(segment_table_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(tabletop_cloud_candidates_sm src/tabletop_cloud_candidates_sm.cpp)
target_link_libraries(tabletop_cloud_candidates_sm ${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(tabletop_cloud_candidates_sm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(object_cloud_candidates_sm src/object_cloud_candidates_sm.cpp)
target_link_libraries(object_cloud_candidates_sm ${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(object_cloud_candidates_sm ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

install(DIRECTORY launch config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS pal_grasp_detection_node pal_grasp_generation_server pal_grasp_generation_client
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS segment_table_node tabletop_cloud_candidates_sm object_cloud_candidates_sm
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)

#############
##  Tests  ##
#############
if(CATKIN_ENABLE_TESTING)

  find_package(rostest REQUIRED)
  include_directories(SYSTEM ${catkin_INCLUDE_DIRS})

  #### Setup targets to download data files
  catkin_download_test_data(
    download_${PROJECT_NAME}_test_files
    http://mars/test-binaries/${PROJECT_NAME}/test_files.tar.gz
    DESTINATION ${CMAKE_SOURCE_DIR}/test
    MD5 aa1e913f433c256eb11e70f729eacfed
  )
  add_custom_target(${PROJECT_NAME}_test_files
    COMMAND "tar" "xfv" "test_files.tar.gz"
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/test
    DEPENDS download_${PROJECT_NAME}_test_files
  )

  add_rostest_gtest(gpd_candidates-test test/gpd_candidates_test.test test/gpd_candidates_test.cpp)
  target_link_libraries(gpd_candidates-test ${catkin_LIBRARIES})
  add_dependencies(gpd_candidates-test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_test_files)

endif()
