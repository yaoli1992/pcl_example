cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(ch4_2)

find_package(PCL 1.3 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (range_image_visualization range_image_visualization.cpp)
target_link_libraries (range_image_visualization ${PCL_LIBRARIES})

add_executable (using_indices using_indices.cpp)
target_link_libraries (using_indices ${PCL_LIBRARIES})

add_executable (nans nans.cpp)
target_link_libraries (nans ${PCL_LIBRARIES})

