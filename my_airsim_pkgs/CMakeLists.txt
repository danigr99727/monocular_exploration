cmake_minimum_required(VERSION 3.10.0)
project(my_airsim_pkgs)

# set this to path to AirSim root folder if you want your catkin workspace in a custom directory
set(AIRSIM_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/AirSim/)

set(CMAKE_CXX_STANDARD 11)
set(CXX_EXP_LIB "-nostdinc++ -I/usr/include/c++/8 -I/usr/include/x86_64-linux-gnu/c++/8 -nodefaultlibs 
-l/usr/lib/x86_64-linux-gnu/libc++.so -l/usr/lib/x86_64-linux-gnu/libc++abi.so
-lm -lc -lgcc_s -lgcc  
-lstdc++fs -fmax-errors=10 -Wnoexcept -Wstrict-null-sentinel") 

set(RPC_LIB_INCLUDES " ${AIRSIM_ROOT}/external/rpclib/rpclib-2.2.1/include")
set(RPC_LIB rpc) # name of .a file with lib prefix
message(STATUS "found RPC_LIB_INCLUDES=${RPC_LIB_INCLUDES}")

# todo eigen3 in AirLib already
# find_package(Eigen3 REQUIRED) 
find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
    cv_bridge
    image_transport
    mavros_msgs
    message_generation
    nav_msgs
    nodelet
    airsim_ros_pkgs
    roscpp
    rospy
    sensor_msgs
    std_msgs
    geographic_msgs
    geometry_msgs
    geometry_msgs
    std_srvs
    tf2
    tf2_ros
    tf2_sensor_msgs
    tf2_geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  # LIBRARIES airsim_ros
  CATKIN_DEPENDS  roscpp  std_msgs message_runtime # nodelet
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${AIRSIM_ROOT}/AirLib/deps/eigen3
  ${AIRSIM_ROOT}/AirLib/include
  ${RPC_LIB_INCLUDES}
  ${AIRSIM_ROOT}/MavLinkCom/include
  ${AIRSIM_ROOT}/MavLinkCom/common_utils
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library(airsim_nodelet src/airsim_nodelet.cpp)
target_link_libraries(airsim_nodelet  ${catkin_LIBRARIES} )#AirLib )#airsim_ros)

