cmake_minimum_required(VERSION 3.0.2)
project(apsrc_v2x_rosbridge)

## c++11 feature
set(CMAKE_CXX_FLAGS "-std=c++17 -Wall -Wextra ${CMAKE_CXX_FLAGS}")

set(catkin_deps
    autoware_msgs
    apsrc_msgs
    geometry_msgs
    network_interface
    nodelet
    roscpp
    roslib
    roslint
    std_msgs
    visualization_msgs
    geographic_msgs
    geodesy
)

find_package(catkin REQUIRED ${catkin_deps})
find_package(Boost REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS ${catkin_deps}
    LIBRARIES ${PROJECT_NAME}_nodelets
)

roslint_cpp()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    include/${PROJECT_NAME}/asnJ2735
    include/${PROJECT_NAME}/asn1609dot2
)


## nodelets
add_library(${PROJECT_NAME}_nodelets
  src/apsrc_v2x_rosbridge_nodelet.cpp
)

target_link_libraries(${PROJECT_NAME}_nodelets
  ${catkin_LIBRARIES}
  asn1609dot2
  asnj2735
)

# nodes
add_executable(${PROJECT_NAME}_node
    src/apsrc_v2x_rosbridge_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS
  ${PROJECT_NAME}_node
  ${PROJECT_NAME}_nodelets
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY launch config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES nodelets.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
