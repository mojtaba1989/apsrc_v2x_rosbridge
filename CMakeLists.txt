cmake_minimum_required(VERSION 3.0.2)
project(apsrc_v2x_rosbridge)

## c++11 feature
set(CMAKE_CXX_FLAGS "-std=c++14 -Wall -Wextra ${CMAKE_CXX_FLAGS}")

set(catkin_deps
    autoware_msgs
    geometry_msgs
    network_interface
    nodelet
    roscpp
    roslib
    roslint
    std_msgs
)

find_package(catkin REQUIRED ${catkin_deps} message_generation)
find_package(Boost REQUIRED)

add_message_files(
  DIRECTORY msg
  FILES
    AccelerationSet4Way.msg
    AllowedManeuvers.msg
    BasicSafetyMessage.msg
    BrakeAppliedStatus.msg
    BrakeSystemStatus.msg
    BSMCore.msg
    BSMpartIIExtension.msg
    ConnectingLane.msg
    Connection.msg
    ConnectsTo.msg
    GenericLane.msg
    IntersectionGeometry.msg
    Intersections.msg
    LaneAttributes.msg
    LaneAttributesVehicle.msg
    LaneDirection.msg
    LaneSharing.msg
    MapData.msg
    NodeList.msg
    NodeOffsetPointXY.msg
    NodeXY.msg
    PathHistory.msg
    PathHistoryPoint.msg
    PathPrediction.msg
    Position3D.msg
    SignalGroup.msg
    VehicleSafetyExtensions.msg
    VehicleSize.msg  
)

generate_messages(
  DEPENDENCIES
    std_msgs
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS ${catkin_deps}
    CATKIN_DEPENDS message_runtime
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
    include/${PROJECT_NAME}/asn1609dot2
    include/${PROJECT_NAME}/asnJ2735
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
