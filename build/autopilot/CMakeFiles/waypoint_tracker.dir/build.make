# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/jay/autopilot_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jay/autopilot_ws/build

# Include any dependencies generated for this target.
include autopilot/CMakeFiles/waypoint_tracker.dir/depend.make

# Include the progress variables for this target.
include autopilot/CMakeFiles/waypoint_tracker.dir/progress.make

# Include the compile flags for this target's objects.
include autopilot/CMakeFiles/waypoint_tracker.dir/flags.make

autopilot/CMakeFiles/waypoint_tracker.dir/src/waypoint_navigation.cpp.o: autopilot/CMakeFiles/waypoint_tracker.dir/flags.make
autopilot/CMakeFiles/waypoint_tracker.dir/src/waypoint_navigation.cpp.o: /home/jay/autopilot_ws/src/autopilot/src/waypoint_navigation.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/jay/autopilot_ws/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object autopilot/CMakeFiles/waypoint_tracker.dir/src/waypoint_navigation.cpp.o"
	cd /home/jay/autopilot_ws/build/autopilot && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/waypoint_tracker.dir/src/waypoint_navigation.cpp.o -c /home/jay/autopilot_ws/src/autopilot/src/waypoint_navigation.cpp

autopilot/CMakeFiles/waypoint_tracker.dir/src/waypoint_navigation.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/waypoint_tracker.dir/src/waypoint_navigation.cpp.i"
	cd /home/jay/autopilot_ws/build/autopilot && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/jay/autopilot_ws/src/autopilot/src/waypoint_navigation.cpp > CMakeFiles/waypoint_tracker.dir/src/waypoint_navigation.cpp.i

autopilot/CMakeFiles/waypoint_tracker.dir/src/waypoint_navigation.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/waypoint_tracker.dir/src/waypoint_navigation.cpp.s"
	cd /home/jay/autopilot_ws/build/autopilot && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/jay/autopilot_ws/src/autopilot/src/waypoint_navigation.cpp -o CMakeFiles/waypoint_tracker.dir/src/waypoint_navigation.cpp.s

autopilot/CMakeFiles/waypoint_tracker.dir/src/waypoint_navigation.cpp.o.requires:
.PHONY : autopilot/CMakeFiles/waypoint_tracker.dir/src/waypoint_navigation.cpp.o.requires

autopilot/CMakeFiles/waypoint_tracker.dir/src/waypoint_navigation.cpp.o.provides: autopilot/CMakeFiles/waypoint_tracker.dir/src/waypoint_navigation.cpp.o.requires
	$(MAKE) -f autopilot/CMakeFiles/waypoint_tracker.dir/build.make autopilot/CMakeFiles/waypoint_tracker.dir/src/waypoint_navigation.cpp.o.provides.build
.PHONY : autopilot/CMakeFiles/waypoint_tracker.dir/src/waypoint_navigation.cpp.o.provides

autopilot/CMakeFiles/waypoint_tracker.dir/src/waypoint_navigation.cpp.o.provides.build: autopilot/CMakeFiles/waypoint_tracker.dir/src/waypoint_navigation.cpp.o

# Object files for target waypoint_tracker
waypoint_tracker_OBJECTS = \
"CMakeFiles/waypoint_tracker.dir/src/waypoint_navigation.cpp.o"

# External object files for target waypoint_tracker
waypoint_tracker_EXTERNAL_OBJECTS =

/home/jay/autopilot_ws/devel/lib/autopilot/waypoint_tracker: autopilot/CMakeFiles/waypoint_tracker.dir/src/waypoint_navigation.cpp.o
/home/jay/autopilot_ws/devel/lib/autopilot/waypoint_tracker: /opt/ros/hydro/lib/libcamera_info_manager.so
/home/jay/autopilot_ws/devel/lib/autopilot/waypoint_tracker: /opt/ros/hydro/lib/libcv_bridge.so
/home/jay/autopilot_ws/devel/lib/autopilot/waypoint_tracker: /opt/ros/hydro/lib/libopencv_videostab.so.2.4.9
/home/jay/autopilot_ws/devel/lib/autopilot/waypoint_tracker: /opt/ros/hydro/lib/libopencv_video.so.2.4.9
/home/jay/autopilot_ws/devel/lib/autopilot/waypoint_tracker: /opt/ros/hydro/lib/libopencv_superres.so.2.4.9
/home/jay/autopilot_ws/devel/lib/autopilot/waypoint_tracker: /opt/ros/hydro/lib/libopencv_stitching.so.2.4.9
/home/jay/autopilot_ws/devel/lib/autopilot/waypoint_tracker: /opt/ros/hydro/lib/libopencv_photo.so.2.4.9
/home/jay/autopilot_ws/devel/lib/autopilot/waypoint_tracker: /opt/ros/hydro/lib/libopencv_ocl.so.2.4.9
/home/jay/autopilot_ws/devel/lib/autopilot/waypoint_tracker: /opt/ros/hydro/lib/libopencv_objdetect.so.2.4.9
/home/jay/autopilot_ws/devel/lib/autopilot/waypoint_tracker: /opt/ros/hydro/lib/libopencv_nonfree.so.2.4.9
/home/jay/autopilot_ws/devel/lib/autopilot/waypoint_tracker: /opt/ros/hydro/lib/libopencv_ml.so.2.4.9
/home/jay/autopilot_ws/devel/lib/autopilot/waypoint_tracker: /opt/ros/hydro/lib/libopencv_legacy.so.2.4.9
/home/jay/autopilot_ws/devel/lib/autopilot/waypoint_tracker: /opt/ros/hydro/lib/libopencv_imgproc.so.2.4.9
/home/jay/autopilot_ws/devel/lib/autopilot/waypoint_tracker: /opt/ros/hydro/lib/libopencv_highgui.so.2.4.9
/home/jay/autopilot_ws/devel/lib/autopilot/waypoint_tracker: /opt/ros/hydro/lib/libopencv_gpu.so.2.4.9
/home/jay/autopilot_ws/devel/lib/autopilot/waypoint_tracker: /opt/ros/hydro/lib/libopencv_flann.so.2.4.9
/home/jay/autopilot_ws/devel/lib/autopilot/waypoint_tracker: /opt/ros/hydro/lib/libopencv_features2d.so.2.4.9
/home/jay/autopilot_ws/devel/lib/autopilot/waypoint_tracker: /opt/ros/hydro/lib/libopencv_core.so.2.4.9
/home/jay/autopilot_ws/devel/lib/autopilot/waypoint_tracker: /opt/ros/hydro/lib/libopencv_contrib.so.2.4.9
/home/jay/autopilot_ws/devel/lib/autopilot/waypoint_tracker: /opt/ros/hydro/lib/libopencv_calib3d.so.2.4.9
/home/jay/autopilot_ws/devel/lib/autopilot/waypoint_tracker: /opt/ros/hydro/lib/libimage_transport.so
/home/jay/autopilot_ws/devel/lib/autopilot/waypoint_tracker: /usr/lib/libtinyxml.so
/home/jay/autopilot_ws/devel/lib/autopilot/waypoint_tracker: /opt/ros/hydro/lib/libclass_loader.so
/home/jay/autopilot_ws/devel/lib/autopilot/waypoint_tracker: /usr/lib/libPocoFoundation.so
/home/jay/autopilot_ws/devel/lib/autopilot/waypoint_tracker: /usr/lib/x86_64-linux-gnu/libdl.so
/home/jay/autopilot_ws/devel/lib/autopilot/waypoint_tracker: /opt/ros/hydro/lib/libroslib.so
/home/jay/autopilot_ws/devel/lib/autopilot/waypoint_tracker: /opt/ros/hydro/lib/libcamera_calibration_parsers.so
/home/jay/autopilot_ws/devel/lib/autopilot/waypoint_tracker: /opt/ros/hydro/lib/libtf.so
/home/jay/autopilot_ws/devel/lib/autopilot/waypoint_tracker: /opt/ros/hydro/lib/libtf2_ros.so
/home/jay/autopilot_ws/devel/lib/autopilot/waypoint_tracker: /opt/ros/hydro/lib/libactionlib.so
/home/jay/autopilot_ws/devel/lib/autopilot/waypoint_tracker: /opt/ros/hydro/lib/libmessage_filters.so
/home/jay/autopilot_ws/devel/lib/autopilot/waypoint_tracker: /opt/ros/hydro/lib/libroscpp.so
/home/jay/autopilot_ws/devel/lib/autopilot/waypoint_tracker: /usr/lib/libboost_signals-mt.so
/home/jay/autopilot_ws/devel/lib/autopilot/waypoint_tracker: /usr/lib/libboost_filesystem-mt.so
/home/jay/autopilot_ws/devel/lib/autopilot/waypoint_tracker: /opt/ros/hydro/lib/libxmlrpcpp.so
/home/jay/autopilot_ws/devel/lib/autopilot/waypoint_tracker: /opt/ros/hydro/lib/libtf2.so
/home/jay/autopilot_ws/devel/lib/autopilot/waypoint_tracker: /opt/ros/hydro/lib/libroscpp_serialization.so
/home/jay/autopilot_ws/devel/lib/autopilot/waypoint_tracker: /opt/ros/hydro/lib/librosconsole.so
/home/jay/autopilot_ws/devel/lib/autopilot/waypoint_tracker: /opt/ros/hydro/lib/librosconsole_log4cxx.so
/home/jay/autopilot_ws/devel/lib/autopilot/waypoint_tracker: /opt/ros/hydro/lib/librosconsole_backend_interface.so
/home/jay/autopilot_ws/devel/lib/autopilot/waypoint_tracker: /usr/lib/liblog4cxx.so
/home/jay/autopilot_ws/devel/lib/autopilot/waypoint_tracker: /usr/lib/libboost_regex-mt.so
/home/jay/autopilot_ws/devel/lib/autopilot/waypoint_tracker: /opt/ros/hydro/lib/librostime.so
/home/jay/autopilot_ws/devel/lib/autopilot/waypoint_tracker: /usr/lib/libboost_date_time-mt.so
/home/jay/autopilot_ws/devel/lib/autopilot/waypoint_tracker: /usr/lib/libboost_system-mt.so
/home/jay/autopilot_ws/devel/lib/autopilot/waypoint_tracker: /usr/lib/libboost_thread-mt.so
/home/jay/autopilot_ws/devel/lib/autopilot/waypoint_tracker: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/jay/autopilot_ws/devel/lib/autopilot/waypoint_tracker: /opt/ros/hydro/lib/libcpp_common.so
/home/jay/autopilot_ws/devel/lib/autopilot/waypoint_tracker: /opt/ros/hydro/lib/libconsole_bridge.so
/home/jay/autopilot_ws/devel/lib/autopilot/waypoint_tracker: /opt/ros/hydro/lib/libopencv_videostab.so.2.4.9
/home/jay/autopilot_ws/devel/lib/autopilot/waypoint_tracker: /opt/ros/hydro/lib/libopencv_video.so.2.4.9
/home/jay/autopilot_ws/devel/lib/autopilot/waypoint_tracker: /opt/ros/hydro/lib/libopencv_superres.so.2.4.9
/home/jay/autopilot_ws/devel/lib/autopilot/waypoint_tracker: /opt/ros/hydro/lib/libopencv_stitching.so.2.4.9
/home/jay/autopilot_ws/devel/lib/autopilot/waypoint_tracker: /opt/ros/hydro/lib/libopencv_photo.so.2.4.9
/home/jay/autopilot_ws/devel/lib/autopilot/waypoint_tracker: /opt/ros/hydro/lib/libopencv_ocl.so.2.4.9
/home/jay/autopilot_ws/devel/lib/autopilot/waypoint_tracker: /opt/ros/hydro/lib/libopencv_objdetect.so.2.4.9
/home/jay/autopilot_ws/devel/lib/autopilot/waypoint_tracker: /opt/ros/hydro/lib/libopencv_nonfree.so.2.4.9
/home/jay/autopilot_ws/devel/lib/autopilot/waypoint_tracker: /opt/ros/hydro/lib/libopencv_ml.so.2.4.9
/home/jay/autopilot_ws/devel/lib/autopilot/waypoint_tracker: /opt/ros/hydro/lib/libopencv_legacy.so.2.4.9
/home/jay/autopilot_ws/devel/lib/autopilot/waypoint_tracker: /opt/ros/hydro/lib/libopencv_imgproc.so.2.4.9
/home/jay/autopilot_ws/devel/lib/autopilot/waypoint_tracker: /opt/ros/hydro/lib/libopencv_highgui.so.2.4.9
/home/jay/autopilot_ws/devel/lib/autopilot/waypoint_tracker: /opt/ros/hydro/lib/libopencv_gpu.so.2.4.9
/home/jay/autopilot_ws/devel/lib/autopilot/waypoint_tracker: /opt/ros/hydro/lib/libopencv_flann.so.2.4.9
/home/jay/autopilot_ws/devel/lib/autopilot/waypoint_tracker: /opt/ros/hydro/lib/libopencv_features2d.so.2.4.9
/home/jay/autopilot_ws/devel/lib/autopilot/waypoint_tracker: /opt/ros/hydro/lib/libopencv_core.so.2.4.9
/home/jay/autopilot_ws/devel/lib/autopilot/waypoint_tracker: /opt/ros/hydro/lib/libopencv_contrib.so.2.4.9
/home/jay/autopilot_ws/devel/lib/autopilot/waypoint_tracker: /opt/ros/hydro/lib/libopencv_calib3d.so.2.4.9
/home/jay/autopilot_ws/devel/lib/autopilot/waypoint_tracker: /opt/ros/hydro/lib/libopencv_nonfree.so.2.4.9
/home/jay/autopilot_ws/devel/lib/autopilot/waypoint_tracker: /opt/ros/hydro/lib/libopencv_ocl.so.2.4.9
/home/jay/autopilot_ws/devel/lib/autopilot/waypoint_tracker: /opt/ros/hydro/lib/libopencv_gpu.so.2.4.9
/home/jay/autopilot_ws/devel/lib/autopilot/waypoint_tracker: /opt/ros/hydro/lib/libopencv_photo.so.2.4.9
/home/jay/autopilot_ws/devel/lib/autopilot/waypoint_tracker: /opt/ros/hydro/lib/libopencv_objdetect.so.2.4.9
/home/jay/autopilot_ws/devel/lib/autopilot/waypoint_tracker: /opt/ros/hydro/lib/libopencv_legacy.so.2.4.9
/home/jay/autopilot_ws/devel/lib/autopilot/waypoint_tracker: /opt/ros/hydro/lib/libopencv_video.so.2.4.9
/home/jay/autopilot_ws/devel/lib/autopilot/waypoint_tracker: /opt/ros/hydro/lib/libopencv_ml.so.2.4.9
/home/jay/autopilot_ws/devel/lib/autopilot/waypoint_tracker: /opt/ros/hydro/lib/libopencv_calib3d.so.2.4.9
/home/jay/autopilot_ws/devel/lib/autopilot/waypoint_tracker: /opt/ros/hydro/lib/libopencv_features2d.so.2.4.9
/home/jay/autopilot_ws/devel/lib/autopilot/waypoint_tracker: /opt/ros/hydro/lib/libopencv_highgui.so.2.4.9
/home/jay/autopilot_ws/devel/lib/autopilot/waypoint_tracker: /opt/ros/hydro/lib/libopencv_imgproc.so.2.4.9
/home/jay/autopilot_ws/devel/lib/autopilot/waypoint_tracker: /opt/ros/hydro/lib/libopencv_flann.so.2.4.9
/home/jay/autopilot_ws/devel/lib/autopilot/waypoint_tracker: /opt/ros/hydro/lib/libopencv_core.so.2.4.9
/home/jay/autopilot_ws/devel/lib/autopilot/waypoint_tracker: autopilot/CMakeFiles/waypoint_tracker.dir/build.make
/home/jay/autopilot_ws/devel/lib/autopilot/waypoint_tracker: autopilot/CMakeFiles/waypoint_tracker.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable /home/jay/autopilot_ws/devel/lib/autopilot/waypoint_tracker"
	cd /home/jay/autopilot_ws/build/autopilot && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/waypoint_tracker.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
autopilot/CMakeFiles/waypoint_tracker.dir/build: /home/jay/autopilot_ws/devel/lib/autopilot/waypoint_tracker
.PHONY : autopilot/CMakeFiles/waypoint_tracker.dir/build

autopilot/CMakeFiles/waypoint_tracker.dir/requires: autopilot/CMakeFiles/waypoint_tracker.dir/src/waypoint_navigation.cpp.o.requires
.PHONY : autopilot/CMakeFiles/waypoint_tracker.dir/requires

autopilot/CMakeFiles/waypoint_tracker.dir/clean:
	cd /home/jay/autopilot_ws/build/autopilot && $(CMAKE_COMMAND) -P CMakeFiles/waypoint_tracker.dir/cmake_clean.cmake
.PHONY : autopilot/CMakeFiles/waypoint_tracker.dir/clean

autopilot/CMakeFiles/waypoint_tracker.dir/depend:
	cd /home/jay/autopilot_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jay/autopilot_ws/src /home/jay/autopilot_ws/src/autopilot /home/jay/autopilot_ws/build /home/jay/autopilot_ws/build/autopilot /home/jay/autopilot_ws/build/autopilot/CMakeFiles/waypoint_tracker.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : autopilot/CMakeFiles/waypoint_tracker.dir/depend

