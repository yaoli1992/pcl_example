cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(feature)

find_package(PCL 1.3 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (narf_feature_extraction narf_feature_extraction.cpp)
target_link_libraries (narf_feature_extraction ${PCL_LIBRARIES})


add_executable (normal_3d normal_3d.cpp)
target_link_libraries (normal_3d ${PCL_LIBRARIES})

add_executable (pfh pfh.cpp)
target_link_libraries (pfh ${PCL_LIBRARIES})


add_executable (fpfh fpfh.cpp)
target_link_libraries (fpfh ${PCL_LIBRARIES})

#This code will only compile with PCL versions 1.8 and above (the current trunk).
#add_executable (rsd rsd.cpp)
#target_link_libraries (rsd ${PCL_LIBRARIES})

add_executable (rangeimage rangeimage.cpp)
target_link_libraries (rangeimage ${PCL_LIBRARIES})

add_executable (extracting extracting.cpp)
target_link_libraries (extracting ${PCL_LIBRARIES})

add_executable (narfkeypoint narfkeypoint.cpp)
target_link_libraries (narfkeypoint ${PCL_LIBRARIES})


add_executable (pclhistogramvisualizer  pclhistogramvisualizer.cpp)
target_link_libraries (pclhistogramvisualizer  ${PCL_LIBRARIES})

add_executable (sift  sift.cpp)
target_link_libraries (sift  ${PCL_LIBRARIES})

