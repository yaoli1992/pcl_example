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
CMAKE_SOURCE_DIR = /home/salm/myopencv/yl_pcl/ch2_1

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/salm/myopencv/yl_pcl/ch2_1/build

# Include any dependencies generated for this target.
include CMakeFiles/openni_image.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/openni_image.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/openni_image.dir/flags.make

CMakeFiles/openni_image.dir/openni_image.cpp.o: CMakeFiles/openni_image.dir/flags.make
CMakeFiles/openni_image.dir/openni_image.cpp.o: ../openni_image.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/salm/myopencv/yl_pcl/ch2_1/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/openni_image.dir/openni_image.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/openni_image.dir/openni_image.cpp.o -c /home/salm/myopencv/yl_pcl/ch2_1/openni_image.cpp

CMakeFiles/openni_image.dir/openni_image.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/openni_image.dir/openni_image.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/salm/myopencv/yl_pcl/ch2_1/openni_image.cpp > CMakeFiles/openni_image.dir/openni_image.cpp.i

CMakeFiles/openni_image.dir/openni_image.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/openni_image.dir/openni_image.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/salm/myopencv/yl_pcl/ch2_1/openni_image.cpp -o CMakeFiles/openni_image.dir/openni_image.cpp.s

CMakeFiles/openni_image.dir/openni_image.cpp.o.requires:
.PHONY : CMakeFiles/openni_image.dir/openni_image.cpp.o.requires

CMakeFiles/openni_image.dir/openni_image.cpp.o.provides: CMakeFiles/openni_image.dir/openni_image.cpp.o.requires
	$(MAKE) -f CMakeFiles/openni_image.dir/build.make CMakeFiles/openni_image.dir/openni_image.cpp.o.provides.build
.PHONY : CMakeFiles/openni_image.dir/openni_image.cpp.o.provides

CMakeFiles/openni_image.dir/openni_image.cpp.o.provides.build: CMakeFiles/openni_image.dir/openni_image.cpp.o

# Object files for target openni_image
openni_image_OBJECTS = \
"CMakeFiles/openni_image.dir/openni_image.cpp.o"

# External object files for target openni_image
openni_image_EXTERNAL_OBJECTS =

openni_image: CMakeFiles/openni_image.dir/openni_image.cpp.o
openni_image: CMakeFiles/openni_image.dir/build.make
openni_image: /usr/lib/i386-linux-gnu/libboost_system.so
openni_image: /usr/lib/i386-linux-gnu/libboost_filesystem.so
openni_image: /usr/lib/i386-linux-gnu/libboost_thread.so
openni_image: /usr/lib/i386-linux-gnu/libboost_date_time.so
openni_image: /usr/lib/i386-linux-gnu/libboost_iostreams.so
openni_image: /usr/lib/i386-linux-gnu/libboost_serialization.so
openni_image: /usr/lib/i386-linux-gnu/libboost_chrono.so
openni_image: /usr/lib/i386-linux-gnu/libpthread.so
openni_image: /usr/local/lib/libpcl_common.so
openni_image: /usr/lib/i386-linux-gnu/libflann_cpp_s.a
openni_image: /usr/local/lib/libpcl_kdtree.so
openni_image: /usr/local/lib/libpcl_octree.so
openni_image: /usr/local/lib/libpcl_search.so
openni_image: /usr/local/lib/libpcl_sample_consensus.so
openni_image: /usr/local/lib/libpcl_filters.so
openni_image: /usr/lib/libOpenNI.so
openni_image: /usr/lib/libOpenNI2.so
openni_image: /usr/local/lib/libpcl_io.so
openni_image: /usr/local/lib/libpcl_features.so
openni_image: /usr/local/lib/libpcl_visualization.so
openni_image: /usr/local/lib/libpcl_ml.so
openni_image: /usr/local/lib/libpcl_segmentation.so
openni_image: /usr/local/lib/libpcl_people.so
openni_image: /usr/lib/i386-linux-gnu/libqhull.so
openni_image: /usr/local/lib/libpcl_surface.so
openni_image: /usr/local/lib/libpcl_keypoints.so
openni_image: /usr/local/lib/libpcl_outofcore.so
openni_image: /usr/local/lib/libpcl_stereo.so
openni_image: /usr/local/lib/libpcl_registration.so
openni_image: /usr/local/lib/libpcl_recognition.so
openni_image: /usr/local/lib/libpcl_tracking.so
openni_image: /usr/lib/i386-linux-gnu/libboost_system.so
openni_image: /usr/lib/i386-linux-gnu/libboost_filesystem.so
openni_image: /usr/lib/i386-linux-gnu/libboost_thread.so
openni_image: /usr/lib/i386-linux-gnu/libboost_date_time.so
openni_image: /usr/lib/i386-linux-gnu/libboost_iostreams.so
openni_image: /usr/lib/i386-linux-gnu/libboost_serialization.so
openni_image: /usr/lib/i386-linux-gnu/libboost_chrono.so
openni_image: /usr/lib/i386-linux-gnu/libpthread.so
openni_image: /usr/lib/i386-linux-gnu/libqhull.so
openni_image: /usr/lib/libOpenNI.so
openni_image: /usr/lib/libOpenNI2.so
openni_image: /usr/lib/i386-linux-gnu/libflann_cpp_s.a
openni_image: /usr/lib/libvtkGenericFiltering.so.5.8.0
openni_image: /usr/lib/libvtkGeovis.so.5.8.0
openni_image: /usr/lib/libvtkCharts.so.5.8.0
openni_image: /usr/local/lib/libpcl_common.so
openni_image: /usr/local/lib/libpcl_kdtree.so
openni_image: /usr/local/lib/libpcl_octree.so
openni_image: /usr/local/lib/libpcl_search.so
openni_image: /usr/local/lib/libpcl_sample_consensus.so
openni_image: /usr/local/lib/libpcl_filters.so
openni_image: /usr/local/lib/libpcl_io.so
openni_image: /usr/local/lib/libpcl_features.so
openni_image: /usr/local/lib/libpcl_visualization.so
openni_image: /usr/local/lib/libpcl_ml.so
openni_image: /usr/local/lib/libpcl_segmentation.so
openni_image: /usr/local/lib/libpcl_people.so
openni_image: /usr/local/lib/libpcl_surface.so
openni_image: /usr/local/lib/libpcl_keypoints.so
openni_image: /usr/local/lib/libpcl_outofcore.so
openni_image: /usr/local/lib/libpcl_stereo.so
openni_image: /usr/local/lib/libpcl_registration.so
openni_image: /usr/local/lib/libpcl_recognition.so
openni_image: /usr/local/lib/libpcl_tracking.so
openni_image: /usr/lib/libvtkViews.so.5.8.0
openni_image: /usr/lib/libvtkInfovis.so.5.8.0
openni_image: /usr/lib/libvtkWidgets.so.5.8.0
openni_image: /usr/lib/libvtkVolumeRendering.so.5.8.0
openni_image: /usr/lib/libvtkHybrid.so.5.8.0
openni_image: /usr/lib/libvtkParallel.so.5.8.0
openni_image: /usr/lib/libvtkRendering.so.5.8.0
openni_image: /usr/lib/libvtkImaging.so.5.8.0
openni_image: /usr/lib/libvtkGraphics.so.5.8.0
openni_image: /usr/lib/libvtkIO.so.5.8.0
openni_image: /usr/lib/libvtkFiltering.so.5.8.0
openni_image: /usr/lib/libvtkCommon.so.5.8.0
openni_image: /usr/lib/libvtksys.so.5.8.0
openni_image: CMakeFiles/openni_image.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable openni_image"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/openni_image.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/openni_image.dir/build: openni_image
.PHONY : CMakeFiles/openni_image.dir/build

CMakeFiles/openni_image.dir/requires: CMakeFiles/openni_image.dir/openni_image.cpp.o.requires
.PHONY : CMakeFiles/openni_image.dir/requires

CMakeFiles/openni_image.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/openni_image.dir/cmake_clean.cmake
.PHONY : CMakeFiles/openni_image.dir/clean

CMakeFiles/openni_image.dir/depend:
	cd /home/salm/myopencv/yl_pcl/ch2_1/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/salm/myopencv/yl_pcl/ch2_1 /home/salm/myopencv/yl_pcl/ch2_1 /home/salm/myopencv/yl_pcl/ch2_1/build /home/salm/myopencv/yl_pcl/ch2_1/build /home/salm/myopencv/yl_pcl/ch2_1/build/CMakeFiles/openni_image.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/openni_image.dir/depend

