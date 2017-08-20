cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(kinect_processing)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (openni_viewer openni_viewer.cpp)
target_link_libraries (openni_viewer ${PCL_LIBRARIES})


add_executable (openni openni.cpp)
target_link_libraries (openni ${PCL_LIBRARIES})


add_executable (add add.cpp)
target_link_libraries (add ${PCL_LIBRARIES})

add_executable (matrix matrix.cpp)
target_link_libraries (matrix ${PCL_LIBRARIES})

add_executable (normals normals.cpp)
target_link_libraries (normals ${PCL_LIBRARIES})


add_executable (integral_image integral_image.cpp)
target_link_libraries (integral_image ${PCL_LIBRARIES})

add_executable (passthrough  passthrough.cpp)
target_link_libraries (passthrough ${PCL_LIBRARIES})

add_executable (unsampling  unsampling.cpp)
target_link_libraries (unsampling ${PCL_LIBRARIES})

add_executable (icp  icp.cpp)
target_link_libraries (icp ${PCL_LIBRARIES})

add_executable (ransac  ransac.cpp)
target_link_libraries (ransac ${PCL_LIBRARIES})

add_executable (viewer  viewer.cpp)
target_link_libraries (viewer ${PCL_LIBRARIES})

#add_executable (min_cut  min_cut.cpp)
#target_link_libraries (min_cut ${PCL_LIBRARIES})


add_executable (plane_model  plane_model.cpp)
target_link_libraries (plane_model ${PCL_LIBRARIES})



add_executable (concavehull  concavehull.cpp)
target_link_libraries (concavehull ${PCL_LIBRARIES})

add_executable (triangulation  triangulation.cpp)
target_link_libraries (triangulation ${PCL_LIBRARIES})

add_executable (convexhull  convexhull.cpp)
target_link_libraries (convexhull ${PCL_LIBRARIES})
