cmake_minimum_required(VERSION 3.1)
project(my_pipeline LANGUAGES CXX C)

#################
# Find packages #
#################


find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        airsim_ros_pkgs
        tf
        )
#################################
# catkin specific configuration #
#################################

catkin_package(CATKIN_DEPENDS)

#########
# Build #
#########

include_directories(
        ${catkin_INCLUDE_DIRS})

link_libraries(
        ${catkin_LIBRARIES})

add_executable(performance_meter src/performance_meter.cpp)
target_link_libraries(performance_meter  ${catkin_LIBRARIES})