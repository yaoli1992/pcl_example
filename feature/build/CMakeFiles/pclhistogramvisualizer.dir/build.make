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
CMAKE_SOURCE_DIR = /home/salm/myopencv/yl_pcl/feature

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/salm/myopencv/yl_pcl/feature/build

# Include any dependencies generated for this target.
include CMakeFiles/pclhistogramvisualizer.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/pclhistogramvisualizer.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/pclhistogramvisualizer.dir/flags.make

CMakeFiles/pclhistogramvisualizer.dir/pclhistogramvisualizer.cpp.o: CMakeFiles/pclhistogramvisualizer.dir/flags.make
CMakeFiles/pclhistogramvisualizer.dir/pclhistogramvisualizer.cpp.o: ../pclhistogramvisualizer.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/salm/myopencv/yl_pcl/feature/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/pclhistogramvisualizer.dir/pclhistogramvisualizer.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/pclhistogramvisualizer.dir/pclhistogramvisualizer.cpp.o -c /home/salm/myopencv/yl_pcl/feature/pclhistogramvisualizer.cpp

CMakeFiles/pclhistogramvisualizer.dir/pclhistogramvisualizer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pclhistogramvisualizer.dir/pclhistogramvisualizer.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/salm/myopencv/yl_pcl/feature/pclhistogramvisualizer.cpp > CMakeFiles/pclhistogramvisualizer.dir/pclhistogramvisualizer.cpp.i

CMakeFiles/pclhistogramvisualizer.dir/pclhistogramvisualizer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pclhistogramvisualizer.dir/pclhistogramvisualizer.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/salm/myopencv/yl_pcl/feature/pclhistogramvisualizer.cpp -o CMakeFiles/pclhistogramvisualizer.dir/pclhistogramvisualizer.cpp.s

CMakeFiles/pclhistogramvisualizer.dir/pclhistogramvisualizer.cpp.o.requires:
.PHONY : CMakeFiles/pclhistogramvisualizer.dir/pclhistogramvisualizer.cpp.o.requires

CMakeFiles/pclhistogramvisualizer.dir/pclhistogramvisualizer.cpp.o.provides: CMakeFiles/pclhistogramvisualizer.dir/pclhistogramvisualizer.cpp.o.requires
	$(MAKE) -f CMakeFiles/pclhistogramvisualizer.dir/build.make CMakeFiles/pclhistogramvisualizer.dir/pclhistogramvisualizer.cpp.o.provides.build
.PHONY : CMakeFiles/pclhistogramvisualizer.dir/pclhistogramvisualizer.cpp.o.provides

CMakeFiles/pclhistogramvisualizer.dir/pclhistogramvisualizer.cpp.o.provides.build: CMakeFiles/pclhistogramvisualizer.dir/pclhistogramvisualizer.cpp.o

# Object files for target pclhistogramvisualizer
pclhistogramvisualizer_OBJECTS = \
"CMakeFiles/pclhistogramvisualizer.dir/pclhistogramvisualizer.cpp.o"

# External object files for target pclhistogramvisualizer
pclhistogramvisualizer_EXTERNAL_OBJECTS =

pclhistogramvisualizer: CMakeFiles/pclhistogramvisualizer.dir/pclhistogramvisualizer.cpp.o
pclhistogramvisualizer: CMakeFiles/pclhistogramvisualizer.dir/build.make
pclhistogramvisualizer: /usr/lib/i386-linux-gnu/libboost_system.so
pclhistogramvisualizer: /usr/lib/i386-linux-gnu/libboost_filesystem.so
pclhistogramvisualizer: /usr/lib/i386-linux-gnu/libboost_thread.so
pclhistogramvisualizer: /usr/lib/i386-linux-gnu/libboost_date_time.so
pclhistogramvisualizer: /usr/lib/i386-linux-gnu/libboost_iostreams.so
pclhistogramvisualizer: /usr/lib/i386-linux-gnu/libboost_serialization.so
pclhistogramvisualizer: /usr/lib/i386-linux-gnu/libboost_chrono.so
pclhistogramvisualizer: /usr/lib/i386-linux-gnu/libpthread.so
pclhistogramvisualizer: /usr/lib/libpcl_common.so
pclhistogramvisualizer: /usr/lib/i386-linux-gnu/libflann_cpp_s.a
pclhistogramvisualizer: /usr/lib/libpcl_kdtree.so
pclhistogramvisualizer: /usr/lib/libpcl_octree.so
pclhistogramvisualizer: /usr/lib/libpcl_search.so
pclhistogramvisualizer: /usr/lib/i386-linux-gnu/libqhull.so
pclhistogramvisualizer: /usr/lib/libpcl_surface.so
pclhistogramvisualizer: /usr/lib/libpcl_sample_consensus.so
pclhistogramvisualizer: /usr/lib/libOpenNI.so
pclhistogramvisualizer: /usr/lib/libOpenNI2.so
pclhistogramvisualizer: /usr/lib/libpcl_io.so
pclhistogramvisualizer: /usr/lib/libpcl_filters.so
pclhistogramvisualizer: /usr/lib/libpcl_features.so
pclhistogramvisualizer: /usr/lib/libpcl_keypoints.so
pclhistogramvisualizer: /usr/lib/libpcl_registration.so
pclhistogramvisualizer: /usr/lib/libpcl_segmentation.so
pclhistogramvisualizer: /usr/lib/libpcl_recognition.so
pclhistogramvisualizer: /usr/lib/libpcl_visualization.so
pclhistogramvisualizer: /usr/lib/libpcl_people.so
pclhistogramvisualizer: /usr/lib/libpcl_outofcore.so
pclhistogramvisualizer: /usr/lib/libpcl_tracking.so
pclhistogramvisualizer: /usr/lib/libpcl_apps.so
pclhistogramvisualizer: /usr/lib/i386-linux-gnu/libboost_system.so
pclhistogramvisualizer: /usr/lib/i386-linux-gnu/libboost_filesystem.so
pclhistogramvisualizer: /usr/lib/i386-linux-gnu/libboost_thread.so
pclhistogramvisualizer: /usr/lib/i386-linux-gnu/libboost_date_time.so
pclhistogramvisualizer: /usr/lib/i386-linux-gnu/libboost_iostreams.so
pclhistogramvisualizer: /usr/lib/i386-linux-gnu/libboost_serialization.so
pclhistogramvisualizer: /usr/lib/i386-linux-gnu/libboost_chrono.so
pclhistogramvisualizer: /usr/lib/i386-linux-gnu/libpthread.so
pclhistogramvisualizer: /usr/lib/i386-linux-gnu/libqhull.so
pclhistogramvisualizer: /usr/lib/libOpenNI.so
pclhistogramvisualizer: /usr/lib/libOpenNI2.so
pclhistogramvisualizer: /usr/lib/i386-linux-gnu/libflann_cpp_s.a
pclhistogramvisualizer: /usr/lib/libvtkGenericFiltering.so.5.8.0
pclhistogramvisualizer: /usr/lib/libvtkGeovis.so.5.8.0
pclhistogramvisualizer: /usr/lib/libvtkCharts.so.5.8.0
pclhistogramvisualizer: /usr/lib/libpcl_common.so
pclhistogramvisualizer: /usr/lib/libpcl_kdtree.so
pclhistogramvisualizer: /usr/lib/libpcl_octree.so
pclhistogramvisualizer: /usr/lib/libpcl_search.so
pclhistogramvisualizer: /usr/lib/libpcl_surface.so
pclhistogramvisualizer: /usr/lib/libpcl_sample_consensus.so
pclhistogramvisualizer: /usr/lib/libpcl_io.so
pclhistogramvisualizer: /usr/lib/libpcl_filters.so
pclhistogramvisualizer: /usr/lib/libpcl_features.so
pclhistogramvisualizer: /usr/lib/libpcl_keypoints.so
pclhistogramvisualizer: /usr/lib/libpcl_registration.so
pclhistogramvisualizer: /usr/lib/libpcl_segmentation.so
pclhistogramvisualizer: /usr/lib/libpcl_recognition.so
pclhistogramvisualizer: /usr/lib/libpcl_visualization.so
pclhistogramvisualizer: /usr/lib/libpcl_people.so
pclhistogramvisualizer: /usr/lib/libpcl_outofcore.so
pclhistogramvisualizer: /usr/lib/libpcl_tracking.so
pclhistogramvisualizer: /usr/lib/libpcl_apps.so
pclhistogramvisualizer: /usr/lib/libvtkViews.so.5.8.0
pclhistogramvisualizer: /usr/lib/libvtkInfovis.so.5.8.0
pclhistogramvisualizer: /usr/lib/libvtkWidgets.so.5.8.0
pclhistogramvisualizer: /usr/lib/libvtkVolumeRendering.so.5.8.0
pclhistogramvisualizer: /usr/lib/libvtkHybrid.so.5.8.0
pclhistogramvisualizer: /usr/lib/libvtkParallel.so.5.8.0
pclhistogramvisualizer: /usr/lib/libvtkRendering.so.5.8.0
pclhistogramvisualizer: /usr/lib/libvtkImaging.so.5.8.0
pclhistogramvisualizer: /usr/lib/libvtkGraphics.so.5.8.0
pclhistogramvisualizer: /usr/lib/libvtkIO.so.5.8.0
pclhistogramvisualizer: /usr/lib/libvtkFiltering.so.5.8.0
pclhistogramvisualizer: /usr/lib/libvtkCommon.so.5.8.0
pclhistogramvisualizer: /usr/lib/libvtksys.so.5.8.0
pclhistogramvisualizer: CMakeFiles/pclhistogramvisualizer.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable pclhistogramvisualizer"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/pclhistogramvisualizer.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/pclhistogramvisualizer.dir/build: pclhistogramvisualizer
.PHONY : CMakeFiles/pclhistogramvisualizer.dir/build

CMakeFiles/pclhistogramvisualizer.dir/requires: CMakeFiles/pclhistogramvisualizer.dir/pclhistogramvisualizer.cpp.o.requires
.PHONY : CMakeFiles/pclhistogramvisualizer.dir/requires

CMakeFiles/pclhistogramvisualizer.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/pclhistogramvisualizer.dir/cmake_clean.cmake
.PHONY : CMakeFiles/pclhistogramvisualizer.dir/clean

CMakeFiles/pclhistogramvisualizer.dir/depend:
	cd /home/salm/myopencv/yl_pcl/feature/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/salm/myopencv/yl_pcl/feature /home/salm/myopencv/yl_pcl/feature /home/salm/myopencv/yl_pcl/feature/build /home/salm/myopencv/yl_pcl/feature/build /home/salm/myopencv/yl_pcl/feature/build/CMakeFiles/pclhistogramvisualizer.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/pclhistogramvisualizer.dir/depend
