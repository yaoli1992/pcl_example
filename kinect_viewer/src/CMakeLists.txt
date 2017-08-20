find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (kdtree_search kdtree_search.cpp)
target_link_libraries (kdtree_search ${PCL_LIBRARIES})


add_executable (image_grabber_viewer  image_grabber_viewer.cpp)
target_link_libraries (image_grabber_viewer ${PCL_LIBRARIES})

add_executable (image_grabber_saver image_grabber_saver.cpp)
target_link_libraries (image_grabber_saver ${PCL_LIBRARIES})


add_executable (openni_viewer_simple openni_viewer_simple.cpp)
target_link_libraries (openni_viewer_simple ${PCL_LIBRARIES})

add_executable (openni_viewer openni_viewer.cpp)
target_link_libraries (openni_viewer ${PCL_LIBRARIES})

add_executable (kinect_viewer kinect_viewer.cpp)
target_link_libraries (kinect_viewer ${PCL_LIBRARIES})


add_executable (openni_grabber_depth_example openni_grabber_depth_example.cpp)
target_link_libraries (openni_grabber_depth_example ${PCL_LIBRARIES})




add_executable (openni_grabber_example  openni_grabber_example.cpp)
target_link_libraries (openni_grabber_example  ${PCL_LIBRARIES})























