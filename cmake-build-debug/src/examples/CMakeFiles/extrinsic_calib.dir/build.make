# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.19

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Disable VCS-based implicit rules.
% : %,v


# Disable VCS-based implicit rules.
% : RCS/%


# Disable VCS-based implicit rules.
% : RCS/%,v


# Disable VCS-based implicit rules.
% : SCCS/s.%


# Disable VCS-based implicit rules.
% : s.%


.SUFFIXES: .hpux_make_needs_suffix_list


# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /home/zoukaixiang/3rdparty/clion-2021.1.1/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/zoukaixiang/3rdparty/clion-2021.1.1/bin/cmake/linux/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/zoukaixiang/code/camodocal

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/zoukaixiang/code/camodocal/cmake-build-debug

# Include any dependencies generated for this target.
include src/examples/CMakeFiles/extrinsic_calib.dir/depend.make

# Include the progress variables for this target.
include src/examples/CMakeFiles/extrinsic_calib.dir/progress.make

# Include the compile flags for this target's objects.
include src/examples/CMakeFiles/extrinsic_calib.dir/flags.make

src/examples/CMakeFiles/extrinsic_calib.dir/extrinsic_calib.cc.o: src/examples/CMakeFiles/extrinsic_calib.dir/flags.make
src/examples/CMakeFiles/extrinsic_calib.dir/extrinsic_calib.cc.o: ../src/examples/extrinsic_calib.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zoukaixiang/code/camodocal/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/examples/CMakeFiles/extrinsic_calib.dir/extrinsic_calib.cc.o"
	cd /home/zoukaixiang/code/camodocal/cmake-build-debug/src/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/extrinsic_calib.dir/extrinsic_calib.cc.o -c /home/zoukaixiang/code/camodocal/src/examples/extrinsic_calib.cc

src/examples/CMakeFiles/extrinsic_calib.dir/extrinsic_calib.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/extrinsic_calib.dir/extrinsic_calib.cc.i"
	cd /home/zoukaixiang/code/camodocal/cmake-build-debug/src/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zoukaixiang/code/camodocal/src/examples/extrinsic_calib.cc > CMakeFiles/extrinsic_calib.dir/extrinsic_calib.cc.i

src/examples/CMakeFiles/extrinsic_calib.dir/extrinsic_calib.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/extrinsic_calib.dir/extrinsic_calib.cc.s"
	cd /home/zoukaixiang/code/camodocal/cmake-build-debug/src/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zoukaixiang/code/camodocal/src/examples/extrinsic_calib.cc -o CMakeFiles/extrinsic_calib.dir/extrinsic_calib.cc.s

# Object files for target extrinsic_calib
extrinsic_calib_OBJECTS = \
"CMakeFiles/extrinsic_calib.dir/extrinsic_calib.cc.o"

# External object files for target extrinsic_calib
extrinsic_calib_EXTERNAL_OBJECTS =

bin/extrinsic_calib: src/examples/CMakeFiles/extrinsic_calib.dir/extrinsic_calib.cc.o
bin/extrinsic_calib: src/examples/CMakeFiles/extrinsic_calib.dir/build.make
bin/extrinsic_calib: /usr/lib/x86_64-linux-gnu/libdl.so
bin/extrinsic_calib: /usr/lib/x86_64-linux-gnu/libnsl.so
bin/extrinsic_calib: /usr/lib/x86_64-linux-gnu/libm.so
bin/extrinsic_calib: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
bin/extrinsic_calib: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
bin/extrinsic_calib: /usr/lib/x86_64-linux-gnu/libboost_system.so
bin/extrinsic_calib: lib/libcamodocal_calib.so
bin/extrinsic_calib: lib/libcamodocal_pose_graph.so
bin/extrinsic_calib: lib/libcamodocal_camera_systems.so
bin/extrinsic_calib: lib/libcamodocal_pugixml.so
bin/extrinsic_calib: lib/libcamodocal_location_recognition.so
bin/extrinsic_calib: lib/libcamodocal_dbow2.so
bin/extrinsic_calib: lib/libcamodocal_dutilscv.so
bin/extrinsic_calib: lib/libcamodocal_dvision.so
bin/extrinsic_calib: lib/libcamodocal_dutils.so
bin/extrinsic_calib: lib/libcamodocal_visual_odometry.so
bin/extrinsic_calib: lib/libcamodocal_camera_models.so
bin/extrinsic_calib: lib/libcamodocal_gpl.so
bin/extrinsic_calib: lib/libcamodocal_sparse_graph.so
bin/extrinsic_calib: lib/libcamodocal_pose_estimation.so
bin/extrinsic_calib: lib/libcamodocal_brisk.so
bin/extrinsic_calib: lib/libagast.so
bin/extrinsic_calib: lib/libcamodocal_features2d.so
bin/extrinsic_calib: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
bin/extrinsic_calib: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
bin/extrinsic_calib: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
bin/extrinsic_calib: /usr/lib/x86_64-linux-gnu/libboost_system.so
bin/extrinsic_calib: /usr/lib/x86_64-linux-gnu/libboost_thread.so
bin/extrinsic_calib: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
bin/extrinsic_calib: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
bin/extrinsic_calib: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
bin/extrinsic_calib: lib/libcamodocal_fivepoint.so
bin/extrinsic_calib: /usr/lib/x86_64-linux-gnu/libdl.so
bin/extrinsic_calib: /usr/lib/x86_64-linux-gnu/libnsl.so
bin/extrinsic_calib: /usr/lib/x86_64-linux-gnu/libm.so
bin/extrinsic_calib: /usr/local/lib/libopencv_stitching.so.3.4.1
bin/extrinsic_calib: /usr/local/lib/libopencv_videostab.so.3.4.1
bin/extrinsic_calib: /usr/local/lib/libopencv_superres.so.3.4.1
bin/extrinsic_calib: /usr/local/lib/libopencv_img_hash.so.3.4.1
bin/extrinsic_calib: /usr/local/lib/libopencv_hfs.so.3.4.1
bin/extrinsic_calib: /usr/local/lib/libopencv_face.so.3.4.1
bin/extrinsic_calib: /usr/local/lib/libopencv_photo.so.3.4.1
bin/extrinsic_calib: /usr/local/lib/libopencv_tracking.so.3.4.1
bin/extrinsic_calib: /usr/local/lib/libopencv_datasets.so.3.4.1
bin/extrinsic_calib: /usr/local/lib/libopencv_aruco.so.3.4.1
bin/extrinsic_calib: /usr/local/lib/libopencv_freetype.so.3.4.1
bin/extrinsic_calib: /usr/local/lib/libopencv_stereo.so.3.4.1
bin/extrinsic_calib: /usr/local/lib/libopencv_surface_matching.so.3.4.1
bin/extrinsic_calib: /usr/local/lib/libopencv_saliency.so.3.4.1
bin/extrinsic_calib: /usr/local/lib/libopencv_fuzzy.so.3.4.1
bin/extrinsic_calib: /usr/local/lib/libopencv_plot.so.3.4.1
bin/extrinsic_calib: /usr/local/lib/libopencv_structured_light.so.3.4.1
bin/extrinsic_calib: /usr/local/lib/libopencv_phase_unwrapping.so.3.4.1
bin/extrinsic_calib: /usr/local/lib/libopencv_bgsegm.so.3.4.1
bin/extrinsic_calib: /usr/local/lib/libopencv_reg.so.3.4.1
bin/extrinsic_calib: /usr/local/lib/libopencv_bioinspired.so.3.4.1
bin/extrinsic_calib: /usr/local/lib/libopencv_optflow.so.3.4.1
bin/extrinsic_calib: /usr/local/lib/libopencv_ximgproc.so.3.4.1
bin/extrinsic_calib: /usr/local/lib/libopencv_line_descriptor.so.3.4.1
bin/extrinsic_calib: /usr/local/lib/libopencv_dpm.so.3.4.1
bin/extrinsic_calib: /usr/local/lib/libopencv_xobjdetect.so.3.4.1
bin/extrinsic_calib: /usr/local/lib/libopencv_objdetect.so.3.4.1
bin/extrinsic_calib: /usr/local/lib/libopencv_rgbd.so.3.4.1
bin/extrinsic_calib: /usr/local/lib/libopencv_dnn_objdetect.so.3.4.1
bin/extrinsic_calib: /usr/local/lib/libopencv_dnn.so.3.4.1
bin/extrinsic_calib: /usr/local/lib/libopencv_ccalib.so.3.4.1
bin/extrinsic_calib: /usr/local/lib/libopencv_sfm.so.3.4.1
bin/extrinsic_calib: /usr/local/lib/libopencv_xfeatures2d.so.3.4.1
bin/extrinsic_calib: /usr/local/lib/libopencv_ml.so.3.4.1
bin/extrinsic_calib: /usr/local/lib/libopencv_shape.so.3.4.1
bin/extrinsic_calib: /usr/local/lib/libopencv_video.so.3.4.1
bin/extrinsic_calib: /usr/local/lib/libopencv_calib3d.so.3.4.1
bin/extrinsic_calib: /usr/local/lib/libopencv_features2d.so.3.4.1
bin/extrinsic_calib: /usr/local/lib/libopencv_flann.so.3.4.1
bin/extrinsic_calib: /usr/local/lib/libopencv_highgui.so.3.4.1
bin/extrinsic_calib: /usr/local/lib/libopencv_videoio.so.3.4.1
bin/extrinsic_calib: /usr/local/lib/libopencv_imgcodecs.so.3.4.1
bin/extrinsic_calib: /usr/local/lib/libopencv_xphoto.so.3.4.1
bin/extrinsic_calib: /usr/local/lib/libopencv_imgproc.so.3.4.1
bin/extrinsic_calib: /usr/local/lib/libopencv_core.so.3.4.1
bin/extrinsic_calib: /usr/local/lib/libceres.a
bin/extrinsic_calib: /usr/local/lib/libglog.so.0.5.0
bin/extrinsic_calib: /usr/local/lib/libgflags.so.2.2.2
bin/extrinsic_calib: /usr/local/lib/libgflags.so.2.2.2
bin/extrinsic_calib: src/examples/CMakeFiles/extrinsic_calib.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/zoukaixiang/code/camodocal/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../bin/extrinsic_calib"
	cd /home/zoukaixiang/code/camodocal/cmake-build-debug/src/examples && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/extrinsic_calib.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/examples/CMakeFiles/extrinsic_calib.dir/build: bin/extrinsic_calib

.PHONY : src/examples/CMakeFiles/extrinsic_calib.dir/build

src/examples/CMakeFiles/extrinsic_calib.dir/clean:
	cd /home/zoukaixiang/code/camodocal/cmake-build-debug/src/examples && $(CMAKE_COMMAND) -P CMakeFiles/extrinsic_calib.dir/cmake_clean.cmake
.PHONY : src/examples/CMakeFiles/extrinsic_calib.dir/clean

src/examples/CMakeFiles/extrinsic_calib.dir/depend:
	cd /home/zoukaixiang/code/camodocal/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/zoukaixiang/code/camodocal /home/zoukaixiang/code/camodocal/src/examples /home/zoukaixiang/code/camodocal/cmake-build-debug /home/zoukaixiang/code/camodocal/cmake-build-debug/src/examples /home/zoukaixiang/code/camodocal/cmake-build-debug/src/examples/CMakeFiles/extrinsic_calib.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/examples/CMakeFiles/extrinsic_calib.dir/depend
