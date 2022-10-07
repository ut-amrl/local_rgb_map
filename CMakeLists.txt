PROJECT(local_rgb_map)
CMAKE_MINIMUM_REQUIRED(VERSION 3.16)

MESSAGE(STATUS "Using compiler: ${CMAKE_CXX_COMPILER}")
MESSAGE(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
MESSAGE(STATUS "Arch: ${CMAKE_SYSTEM_PROCESSOR}")

SET(CMAKE_EXPORT_COMPILE_COMMANDS on)
SET(CMAKE_CXX_FLAGS "-std=c++14 -Wall -Werror")

IF(${CMAKE_BUILD_TYPE} MATCHES "Release")
  MESSAGE(STATUS "Additional Flags for Release mode")
  SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -fopenmp -O2 -DNDEBUG")
ELSEIF(${CMAKE_BUILD_TYPE} MATCHES "Debug")
  MESSAGE(STATUS "Additional Flags for Debug mode")
  SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -g")
ENDIF()

INCLUDE($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
ROSBUILD_INIT()
SET(ROS_BUILD_STATIC_LIBS true)
SET(ROS_BUILD_SHARED_LIBS false)

FIND_PACKAGE(OpenCV REQUIRED)

MESSAGE(STATUS "ROS-Overrride Build Type: ${CMAKE_BUILD_TYPE}")
MESSAGE(STATUS "CXX Flags: ${CMAKE_CXX_FLAGS}")

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

SET(libs roslib roscpp rosbag glog gflags amrl_shared_lib boost_system lua5.1
    pthread)

ADD_SUBDIRECTORY(src/shared)
INCLUDE_DIRECTORIES(src/shared)
INCLUDE_DIRECTORIES(src/config_reader/include)
INCLUDE_DIRECTORIES(src)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

ROSBUILD_ADD_EXECUTABLE(lmap
                        src/local_rgb_map/local_rgb_map_main.cc)
TARGET_LINK_LIBRARIES(lmap ${libs} ${OpenCV_LIBS})

rosbuild_add_executable(bev_node
  src/bev_node/bev_node_main.cc
  src/bev/bev.cc
  src/bev/bev_sticher.cc
)
target_link_libraries(bev_node
  ${libs}
  ${OpenCV_LIBS}
)

add_executable(bev_benchmark
  src/bev_node/bev_benchmark_main.cc
  src/bev/bev.cc
  src/bev/bev_sticher.cc
)
target_link_libraries(bev_benchmark
  ${libs}
  ${OpenCV_LIBS}
)
