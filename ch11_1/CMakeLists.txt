cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(ch11_1)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (planar_segmentation planar_segmentation.cpp)
target_link_libraries (planar_segmentation ${PCL_LIBRARIES})

add_executable (cylinder_segmentation cylinder_segmentation.cpp)
target_link_libraries (cylinder_segmentation ${PCL_LIBRARIES})


add_executable (cluster_extraction cluster_extraction.cpp)
target_link_libraries (cluster_extraction ${PCL_LIBRARIES})


add_executable (euclidean euclidean.cpp)
target_link_libraries (euclidean ${PCL_LIBRARIES})


add_executable (viewer viewer.cpp)
target_link_libraries (viewer ${PCL_LIBRARIES})
