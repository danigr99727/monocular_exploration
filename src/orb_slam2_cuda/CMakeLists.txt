cmake_minimum_required(VERSION 2.8)
project(orb_slam2_cuda)


find_package(catkin REQUIRED COMPONENTS
        image_transport
        roscpp
        std_msgs
        nav_msgs
        visualization_msgs
        nodelet
        tf
        cv_bridge_4
        )
catkin_package()

MESSAGE("Build type: " ${ROS_BUILD_TYPE})

#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/ORB_SLAM2_CUDA/cmake_modules)

find_package(OpenCV 4.2.0 REQUIRED PATHS "/root/opencv-4.2.0/opencv-4.2.0/install")
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(CUDA REQUIRED)
#find_package(DBoW2 REQUIRED PATHS "/root/ORB_SLAM2_CUDA/Thirdparty/DBoW2/lib")

find_package( PCL REQUIRED )
include_directories(  )
link_directories(${PCL_LIBRARY_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/ORB_SLAM2_CUDA/Thirdparty/DBoW2/lib/
        ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/ORB_SLAM2_CUDA/Thirdparty/g2o/lib/
        )
add_definitions(${PCL_DEFINITIONS})

include_directories(
        ${catkin_INCLUDE_DIRS}  ${PCL_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}
   `    ${PROJECT_SOURCE_DIR}/lib
        ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/ORB_SLAM2_CUDA/
        ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/ORB_SLAM2_CUDA/include
        ${Pangolin_INCLUDE_DIRS}
        ${CUDA_INCLUDE_DIRS}
)

set(LIBS
${catkin_LIBRARIES}
${OpenCV_LIBS} 
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${PCL_LIBRARIES}
${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/ORB_SLAM2_CUDA/lib/libORB_SLAM2_CUDA.so
)

# Node for monocular camera
add_executable(Mono
        lib/orb_slam_wrapper.cc
        src/orb_slam_mono_node.cc
)

target_link_libraries(Mono
${LIBS}
)

add_library(orb_slam_nodelet src/orb_slam_mono_nodelet.cc)
target_link_libraries(orb_slam_nodelet  ${catkin_LIBRARIES})

# Node for stereo camera
#add_executable(Stereo
#src/ros_stereo.cc
#)

#target_link_libraries(Stereo
#${LIBS}
#)

# Node for RGB-D camera
#add_executable(RGBD
#src/ros_rgbd.cc
#)

#target_link_libraries(RGBD
#${LIBS}
#)

