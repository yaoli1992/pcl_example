# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.2

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list

# Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/salm/myopencv/yl_pcl/example_pcl

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/salm/myopencv/yl_pcl/example_pcl/build

# Include any dependencies generated for this target.
include CMakeFiles/boundary_estimation.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/boundary_estimation.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/boundary_estimation.dir/flags.make

CMakeFiles/boundary_estimation.dir/boundary_estimation.cpp.o: CMakeFiles/boundary_estimation.dir/flags.make
CMakeFiles/boundary_estimation.dir/boundary_estimation.cpp.o: ../boundary_estimation.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/salm/myopencv/yl_pcl/example_pcl/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/boundary_estimation.dir/boundary_estimation.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/boundary_estimation.dir/boundary_estimation.cpp.o -c /home/salm/myopencv/yl_pcl/example_pcl/boundary_estimation.cpp

CMakeFiles/boundary_estimation.dir/boundary_estimation.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/boundary_estimation.dir/boundary_estimation.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/salm/myopencv/yl_pcl/example_pcl/boundary_estimation.cpp > CMakeFiles/boundary_estimation.dir/boundary_estimation.cpp.i

CMakeFiles/boundary_estimation.dir/boundary_estimation.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/boundary_estimation.dir/boundary_estimation.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/salm/myopencv/yl_pcl/example_pcl/boundary_estimation.cpp -o CMakeFiles/boundary_estimation.dir/boundary_estimation.cpp.s

CMakeFiles/boundary_estimation.dir/boundary_estimation.cpp.o.requires:
.PHONY : CMakeFiles/boundary_estimation.dir/boundary_estimation.cpp.o.requires

CMakeFiles/boundary_estimation.dir/boundary_estimation.cpp.o.provides: CMakeFiles/boundary_estimation.dir/boundary_estimation.cpp.o.requires
	$(MAKE) -f CMakeFiles/boundary_estimation.dir/build.make CMakeFiles/boundary_estimation.dir/boundary_estimation.cpp.o.provides.build
.PHONY : CMakeFiles/boundary_estimation.dir/boundary_estimation.cpp.o.provides

CMakeFiles/boundary_estimation.dir/boundary_estimation.cpp.o.provides.build: CMakeFiles/boundary_estimation.dir/boundary_estimation.cpp.o

# Object files for target boundary_estimation
boundary_estimation_OBJECTS = \
"CMakeFiles/boundary_estimation.dir/boundary_estimation.cpp.o"

# External object files for target boundary_estimation
boundary_estimation_EXTERNAL_OBJECTS =

boundary_estimation: CMakeFiles/boundary_estimation.dir/boundary_estimation.cpp.o
boundary_estimation: CMakeFiles/boundary_estimation.dir/build.make
boundary_estimation: /usr/lib/i386-linux-gnu/libboost_system.so
boundary_estimation: /usr/lib/i386-linux-gnu/libboost_filesystem.so
boundary_estimation: /usr/lib/i386-linux-gnu/libboost_thread.so
boundary_estimation: /usr/lib/i386-linux-gnu/libboost_date_time.so
boundary_estimation: /usr/lib/i386-linux-gnu/libboost_iostreams.so
boundary_estimation: /usr/lib/i386-linux-gnu/libboost_serialization.so
boundary_estimation: /usr/lib/i386-linux-gnu/libboost_chrono.so
boundary_estimation: /usr/lib/i386-linux-gnu/libpthread.so
boundary_estimation: /usr/local/lib/libpcl_common.so
boundary_estimation: /usr/lib/i386-linux-gnu/libflann_cpp_s.a
boundary_estimation: /usr/local/lib/libpcl_kdtree.so
boundary_estimation: /usr/local/lib/libpcl_octree.so
boundary_estimation: /usr/local/lib/libpcl_search.so
boundary_estimation: /usr/local/lib/libpcl_sample_consensus.so
boundary_estimation: /usr/local/lib/libpcl_filters.so
boundary_estimation: /usr/lib/libOpenNI.so
boundary_estimation: /usr/lib/libOpenNI2.so
boundary_estimation: /usr/local/lib/libpcl_io.so
boundary_estimation: /usr/local/lib/libpcl_features.so
boundary_estimation: /usr/local/lib/libpcl_visualization.so
boundary_estimation: /usr/local/lib/libpcl_ml.so
boundary_estimation: /usr/local/lib/libpcl_segmentation.so
boundary_estimation: /usr/local/lib/libpcl_people.so
boundary_estimation: /usr/lib/i386-linux-gnu/libqhull.so
boundary_estimation: /usr/local/lib/libpcl_surface.so
boundary_estimation: /usr/local/lib/libpcl_keypoints.so
boundary_estimation: /usr/local/lib/libpcl_outofcore.so
boundary_estimation: /usr/local/lib/libpcl_stereo.so
boundary_estimation: /usr/local/lib/libpcl_registration.so
boundary_estimation: /usr/local/lib/libpcl_recognition.so
boundary_estimation: /usr/local/lib/libpcl_tracking.so
boundary_estimation: /usr/lib/i386-linux-gnu/libboost_system.so
boundary_estimation: /usr/lib/i386-linux-gnu/libboost_filesystem.so
boundary_estimation: /usr/lib/i386-linux-gnu/libboost_thread.so
boundary_estimation: /usr/lib/i386-linux-gnu/libboost_date_time.so
boundary_estimation: /usr/lib/i386-linux-gnu/libboost_iostreams.so
boundary_estimation: /usr/lib/i386-linux-gnu/libboost_serialization.so
boundary_estimation: /usr/lib/i386-linux-gnu/libboost_chrono.so
boundary_estimation: /usr/lib/i386-linux-gnu/libpthread.so
boundary_estimation: /usr/lib/i386-linux-gnu/libqhull.so
boundary_estimation: /usr/lib/libOpenNI.so
boundary_estimation: /usr/lib/libOpenNI2.so
boundary_estimation: /usr/lib/i386-linux-gnu/libflann_cpp_s.a
boundary_estimation: /usr/lib/libvtkGenericFiltering.so.5.8.0
boundary_estimation: /usr/lib/libvtkGeovis.so.5.8.0
boundary_estimation: /usr/lib/libvtkCharts.so.5.8.0
boundary_estimation: /usr/local/lib/libpcl_common.so
boundary_estimation: /usr/local/lib/libpcl_kdtree.so
boundary_estimation: /usr/local/lib/libpcl_octree.so
boundary_estimation: /usr/local/lib/libpcl_search.so
boundary_estimation: /usr/local/lib/libpcl_sample_consensus.so
boundary_estimation: /usr/local/lib/libpcl_filters.so
boundary_estimation: /usr/local/lib/libpcl_io.so
boundary_estimation: /usr/local/lib/libpcl_features.so
boundary_estimation: /usr/local/lib/libpcl_visualization.so
boundary_estimation: /usr/local/lib/libpcl_ml.so
boundary_estimation: /usr/local/lib/libpcl_segmentation.so
boundary_estimation: /usr/local/lib/libpcl_people.so
boundary_estimation: /usr/local/lib/libpcl_surface.so
boundary_estimation: /usr/local/lib/libpcl_keypoints.so
boundary_estimation: /usr/local/lib/libpcl_outofcore.so
boundary_estimation: /usr/local/lib/libpcl_stereo.so
boundary_estimation: /usr/local/lib/libpcl_registration.so
boundary_estimation: /usr/local/lib/libpcl_recognition.so
boundary_estimation: /usr/local/lib/libpcl_tracking.so
boundary_estimation: /usr/lib/libvtkViews.so.5.8.0
boundary_estimation: /usr/lib/libvtkInfovis.so.5.8.0
boundary_estimation: /usr/lib/libvtkWidgets.so.5.8.0
boundary_estimation: /usr/lib/libvtkVolumeRendering.so.5.8.0
boundary_estimation: /usr/lib/libvtkHybrid.so.5.8.0
boundary_estimation: /usr/lib/libvtkParallel.so.5.8.0
boundary_estimation: /usr/lib/libvtkRendering.so.5.8.0
boundary_estimation: /usr/lib/libvtkImaging.so.5.8.0
boundary_estimation: /usr/lib/libvtkGraphics.so.5.8.0
boundary_estimation: /usr/lib/libvtkIO.so.5.8.0
boundary_estimation: /usr/lib/libvtkFiltering.so.5.8.0
boundary_estimation: /usr/lib/libvtkCommon.so.5.8.0
boundary_estimation: /usr/lib/libvtksys.so.5.8.0
boundary_estimation: CMakeFiles/boundary_estimation.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable boundary_estimation"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/boundary_estimation.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/boundary_estimation.dir/build: boundary_estimation
.PHONY : CMakeFiles/boundary_estimation.dir/build

CMakeFiles/boundary_estimation.dir/requires: CMakeFiles/boundary_estimation.dir/boundary_estimation.cpp.o.requires
.PHONY : CMakeFiles/boundary_estimation.dir/requires

CMakeFiles/boundary_estimation.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/boundary_estimation.dir/cmake_clean.cmake
.PHONY : CMakeFiles/boundary_estimation.dir/clean

CMakeFiles/boundary_estimation.dir/depend:
	cd /home/salm/myopencv/yl_pcl/example_pcl/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/salm/myopencv/yl_pcl/example_pcl /home/salm/myopencv/yl_pcl/example_pcl /home/salm/myopencv/yl_pcl/example_pcl/build /home/salm/myopencv/yl_pcl/example_pcl/build /home/salm/myopencv/yl_pcl/example_pcl/build/CMakeFiles/boundary_estimation.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/boundary_estimation.dir/depend
