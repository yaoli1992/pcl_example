cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(ch9_1)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
	
add_executable (normal_estimation normal_estimation.cpp)
target_link_libraries (normal_estimation ${PCL_LIBRARIES})

 add_executable (normal_estimation_using_integral_images normal_estimation_using_integral_images.cpp)
target_link_libraries (normal_estimation_using_integral_images ${PCL_LIBRARIES})

add_executable (narf_feature_extraction narf_feature_extraction.cpp)
target_link_libraries (narf_feature_extraction ${PCL_LIBRARIES})

add_executable (fpfh_radius fpfh_radius.cpp)
target_link_libraries (fpfh_radius ${PCL_LIBRARIES})


