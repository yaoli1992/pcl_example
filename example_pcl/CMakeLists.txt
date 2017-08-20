cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(ch4_3)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (match_linemod_template match_linemod_template.cpp)
target_link_libraries (match_linemod_template ${PCL_LIBRARIES})

add_executable (linemod_detection linemod_detection.cpp)
target_link_libraries (linemod_detection ${PCL_LIBRARIES})

add_executable (train_linemod_template train_linemod_template.cpp)
target_link_libraries (train_linemod_template ${PCL_LIBRARIES})


add_executable (normal_estimation normal_estimation.cpp)
target_link_libraries (normal_estimation ${PCL_LIBRARIES})





