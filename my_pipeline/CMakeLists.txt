cmake_minimum_required(VERSION 3.1)
project(my_pipeline LANGUAGES CXX C)

#################
# Find packages #
#################


find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
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
