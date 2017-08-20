cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(ch12_1)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (resampling resampling.cpp)
target_link_libraries (resampling ${PCL_LIBRARIES})

add_executable (concave_hull_2d concave_hull_2d.cpp)
target_link_libraries (concave_hull_2d ${PCL_LIBRARIES})

add_executable (greedy_projection greedy_projection.cpp)
target_link_libraries (greedy_projection ${PCL_LIBRARIES})



add_executable (passion passion.cpp)
target_link_libraries (passion ${PCL_LIBRARIES})
