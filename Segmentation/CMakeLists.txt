cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(ch4_1)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (cloud_viewer cloud_viewer.cpp)
target_link_libraries (cloud_viewer ${PCL_LIBRARIES})

add_executable (euclidean euclidean.cpp)
target_link_libraries (euclidean ${PCL_LIBRARIES})


add_executable (conditional_euclidean  conditional_euclidean.cpp)
target_link_libraries (conditional_euclidean  ${PCL_LIBRARIES})


add_executable (VFH  VFH.cpp)
target_link_libraries (VFH  ${PCL_LIBRARIES})
