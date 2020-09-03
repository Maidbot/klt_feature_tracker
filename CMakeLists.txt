cmake_minimum_required(VERSION 2.8.3)
project(klt_feature_tracker)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
  roslib
  message_generation
  message_runtime
  message_filters
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  TrackFeatures.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES klt_feature_tracker
)

find_package(OpenCV REQUIRED)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(klt_feature_tracker
  src/track_features.cpp
)

target_link_libraries(klt_feature_tracker
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

target_include_directories(klt_feature_tracker PUBLIC include)

set(${PROJECT_NAME}_INCLUDE_DIRS
    ${PROJECT_SOURCE_DIR}/include
    CACHE INTERNAL "${PROJECT_NAME}: Include Directories" FORCE)

## Declare a C++ executable
add_executable(feature_tracker_node
  src/nodes/feature_tracker_node.cpp
)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(feature_tracker_node klt_feature_tracker ${PROJECT_NAME}_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(feature_tracker_node
  klt_feature_tracker
  ${catkin_LIBRARIES}
)

# Install Targets

install(TARGETS klt_feature_tracker
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(TARGETS feature_tracker_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
