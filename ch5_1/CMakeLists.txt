cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(ch5_1)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (passthrough passthrough.cpp)
target_link_libraries (passthrough ${PCL_LIBRARIES})

add_executable (voxel_grid voxel_grid.cpp)
target_link_libraries (voxel_grid ${PCL_LIBRARIES})

add_executable (statistical_removal statistical_removal.cpp)
target_link_libraries (statistical_removal ${PCL_LIBRARIES})

add_executable (project_inliers project_inliers.cpp)
target_link_libraries (project_inliers ${PCL_LIBRARIES})

add_executable (extract_indices extract_indices.cpp)
target_link_libraries (extract_indices ${PCL_LIBRARIES})

add_executable (remove_outliers remove_outliers.cpp)
target_link_libraries (remove_outliers ${PCL_LIBRARIES})

add_executable (yl_voxel_grid yl_voxel_grid.cpp)
target_link_libraries(yl_voxel_grid ${PCL_LIBRARIES})

#add_executable (yl_statistical_removal yl_statistical_removal.cpp)
#target_link_libraries(yl_statistical_removal ${PCL_LIBRARIES})



