cmake_minimum_required(VERSION 2.8.3)
project(mlmapping)

set(CMAKE_CXX_FLAGS "-std=c++17 ${CMAKE_CXX_FLAGS} -O3 -Wall -pthread") # -Wextra -Werror
set(CMAKE_BUILD_TYPE "RELEASE")

find_package (Eigen3 REQUIRED)
find_package (Sophus REQUIRED)
find_package (yaml-cpp REQUIRED)
find_package (PCL REQUIRED)

add_definitions( ${PCL_DEFINITIONS} )

find_package(catkin REQUIRED COMPONENTS
    nodelet
    roscpp
    rostime
    message_filters
    message_generation
    std_msgs
    sensor_msgs
    geometry_msgs
    tf
    tf2
    )

add_message_files(
    FILES
    awareness2local.msg
    awareness.msg
    localmap.msg
    esdfs.msg
    )

generate_messages(
    DEPENDENCIES
    std_msgs
    sensor_msgs
    geometry_msgs
    )

## Declare a catkin package
catkin_package(
    CATKIN_DEPENDS message_runtime
    )

include_directories(SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${CSPARSE_INCLUDE_DIR}
    ${Sophus_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    "${CMAKE_CURRENT_SOURCE_DIR}/src/"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/utils/"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/msgs/"
    )

## Declare a C++ library
add_library(mlmapping
    src/nodelet_awareness.cpp
    src/nodelet_local.cpp
    src/map_awareness.cpp
    src/map_local.cpp
    src/map_warehouse.cpp
    src/rviz_vis.cpp
    src/msgs/msg_awareness2local.cpp
    src/msgs/msg_awareness.cpp
    src/msgs/msg_localmap.cpp
    src/independent_modules/l2grid2d.cpp
    src/independent_modules/l2esdfs_batch_3d.cpp
    )

add_dependencies(mlmapping
    mlmapping_generate_messages_cpp
    ${catkin_EXPORTED_TARGETS}
    )

target_link_libraries(mlmapping
    ${catkin_LIBRARIES}
    ${CSPARSE_LIBRARY}
    ${Sophus_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
    ${Boost_SYSTEM_LIBRARY}
    )

add_executable(visualization
    src/node_visualization.cpp
    src/map_awareness.cpp
    src/map_warehouse.cpp
    src/map_local.cpp
    src/rviz_vis.cpp
    src/msgs/msg_awareness.cpp
    src/msgs/msg_localmap.cpp
    )
add_dependencies(visualization
    mlmapping_generate_messages_cpp
    ${catkin_EXPORTED_TARGETS}
    )
target_link_libraries(visualization
    ${catkin_LIBRARIES}
    ${CSPARSE_LIBRARY}
    ${Sophus_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
    )

add_executable(republish_record
    src/independent_modules/repub_rec.cpp
    )
target_link_libraries(republish_record
    ${catkin_LIBRARIES}
    )

