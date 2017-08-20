cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(ch4_3)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (pcl_visualizer_demo pcl_visualizer_demo.cpp)
target_link_libraries (pcl_visualizer_demo ${PCL_LIBRARIES})


add_executable (histogram histogram.cpp)
target_link_libraries (histogram ${PCL_LIBRARIES})


add_executable (passthrough  passthrough.cpp)
target_link_libraries (passthrough ${PCL_LIBRARIES})









