cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(ch13_1)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (template_alignment template_alignment.cpp)
target_link_libraries (template_alignment ${PCL_LIBRARIES})



add_executable (normal_distributions_transform normal_distributions_transform.cpp)
target_link_libraries (normal_distributions_transform ${PCL_LIBRARIES})
