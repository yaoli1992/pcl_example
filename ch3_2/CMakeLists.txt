cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(point_cloud_compression)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (point_cloud_compression point_cloud_compression.cpp)
target_link_libraries (point_cloud_compression ${PCL_LIBRARIES})

add_executable (openni_viewer openni_viewer.cpp)
target_link_libraries (openni_viewer ${PCL_LIBRARIES})


add_executable (kinect_test kinect_test.cpp)
target_link_libraries (kinect_test ${PCL_LIBRARIES})

 
add_executable (openni_viewer_simple  openni_viewer_simple.cpp)
target_link_libraries (openni_viewer_simple  ${PCL_LIBRARIES})



