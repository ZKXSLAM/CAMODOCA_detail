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
include src/gpl/CMakeFiles/camodocal_gpl.dir/depend.make

# Include the progress variables for this target.
include src/gpl/CMakeFiles/camodocal_gpl.dir/progress.make

# Include the compile flags for this target's objects.
include src/gpl/CMakeFiles/camodocal_gpl.dir/flags.make

src/gpl/CMakeFiles/camodocal_gpl.dir/CubicSpline.cc.o: src/gpl/CMakeFiles/camodocal_gpl.dir/flags.make
src/gpl/CMakeFiles/camodocal_gpl.dir/CubicSpline.cc.o: ../src/gpl/CubicSpline.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zoukaixiang/code/camodocal/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/gpl/CMakeFiles/camodocal_gpl.dir/CubicSpline.cc.o"
	cd /home/zoukaixiang/code/camodocal/cmake-build-debug/src/gpl && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/camodocal_gpl.dir/CubicSpline.cc.o -c /home/zoukaixiang/code/camodocal/src/gpl/CubicSpline.cc

src/gpl/CMakeFiles/camodocal_gpl.dir/CubicSpline.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/camodocal_gpl.dir/CubicSpline.cc.i"
	cd /home/zoukaixiang/code/camodocal/cmake-build-debug/src/gpl && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zoukaixiang/code/camodocal/src/gpl/CubicSpline.cc > CMakeFiles/camodocal_gpl.dir/CubicSpline.cc.i

src/gpl/CMakeFiles/camodocal_gpl.dir/CubicSpline.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/camodocal_gpl.dir/CubicSpline.cc.s"
	cd /home/zoukaixiang/code/camodocal/cmake-build-debug/src/gpl && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zoukaixiang/code/camodocal/src/gpl/CubicSpline.cc -o CMakeFiles/camodocal_gpl.dir/CubicSpline.cc.s

src/gpl/CMakeFiles/camodocal_gpl.dir/EigenQuaternionParameterization.cc.o: src/gpl/CMakeFiles/camodocal_gpl.dir/flags.make
src/gpl/CMakeFiles/camodocal_gpl.dir/EigenQuaternionParameterization.cc.o: ../src/gpl/EigenQuaternionParameterization.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zoukaixiang/code/camodocal/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object src/gpl/CMakeFiles/camodocal_gpl.dir/EigenQuaternionParameterization.cc.o"
	cd /home/zoukaixiang/code/camodocal/cmake-build-debug/src/gpl && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/camodocal_gpl.dir/EigenQuaternionParameterization.cc.o -c /home/zoukaixiang/code/camodocal/src/gpl/EigenQuaternionParameterization.cc

src/gpl/CMakeFiles/camodocal_gpl.dir/EigenQuaternionParameterization.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/camodocal_gpl.dir/EigenQuaternionParameterization.cc.i"
	cd /home/zoukaixiang/code/camodocal/cmake-build-debug/src/gpl && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zoukaixiang/code/camodocal/src/gpl/EigenQuaternionParameterization.cc > CMakeFiles/camodocal_gpl.dir/EigenQuaternionParameterization.cc.i

src/gpl/CMakeFiles/camodocal_gpl.dir/EigenQuaternionParameterization.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/camodocal_gpl.dir/EigenQuaternionParameterization.cc.s"
	cd /home/zoukaixiang/code/camodocal/cmake-build-debug/src/gpl && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zoukaixiang/code/camodocal/src/gpl/EigenQuaternionParameterization.cc -o CMakeFiles/camodocal_gpl.dir/EigenQuaternionParameterization.cc.s

src/gpl/CMakeFiles/camodocal_gpl.dir/gpl.cc.o: src/gpl/CMakeFiles/camodocal_gpl.dir/flags.make
src/gpl/CMakeFiles/camodocal_gpl.dir/gpl.cc.o: ../src/gpl/gpl.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zoukaixiang/code/camodocal/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object src/gpl/CMakeFiles/camodocal_gpl.dir/gpl.cc.o"
	cd /home/zoukaixiang/code/camodocal/cmake-build-debug/src/gpl && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/camodocal_gpl.dir/gpl.cc.o -c /home/zoukaixiang/code/camodocal/src/gpl/gpl.cc

src/gpl/CMakeFiles/camodocal_gpl.dir/gpl.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/camodocal_gpl.dir/gpl.cc.i"
	cd /home/zoukaixiang/code/camodocal/cmake-build-debug/src/gpl && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zoukaixiang/code/camodocal/src/gpl/gpl.cc > CMakeFiles/camodocal_gpl.dir/gpl.cc.i

src/gpl/CMakeFiles/camodocal_gpl.dir/gpl.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/camodocal_gpl.dir/gpl.cc.s"
	cd /home/zoukaixiang/code/camodocal/cmake-build-debug/src/gpl && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zoukaixiang/code/camodocal/src/gpl/gpl.cc -o CMakeFiles/camodocal_gpl.dir/gpl.cc.s

# Object files for target camodocal_gpl
camodocal_gpl_OBJECTS = \
"CMakeFiles/camodocal_gpl.dir/CubicSpline.cc.o" \
"CMakeFiles/camodocal_gpl.dir/EigenQuaternionParameterization.cc.o" \
"CMakeFiles/camodocal_gpl.dir/gpl.cc.o"

# External object files for target camodocal_gpl
camodocal_gpl_EXTERNAL_OBJECTS =

lib/libcamodocal_gpl.so: src/gpl/CMakeFiles/camodocal_gpl.dir/CubicSpline.cc.o
lib/libcamodocal_gpl.so: src/gpl/CMakeFiles/camodocal_gpl.dir/EigenQuaternionParameterization.cc.o
lib/libcamodocal_gpl.so: src/gpl/CMakeFiles/camodocal_gpl.dir/gpl.cc.o
lib/libcamodocal_gpl.so: src/gpl/CMakeFiles/camodocal_gpl.dir/build.make
lib/libcamodocal_gpl.so: /usr/lib/x86_64-linux-gnu/libdl.so
lib/libcamodocal_gpl.so: /usr/lib/x86_64-linux-gnu/libnsl.so
lib/libcamodocal_gpl.so: /usr/lib/x86_64-linux-gnu/libm.so
lib/libcamodocal_gpl.so: /usr/local/lib/libopencv_stitching.so.3.4.1
lib/libcamodocal_gpl.so: /usr/local/lib/libopencv_videostab.so.3.4.1
lib/libcamodocal_gpl.so: /usr/local/lib/libopencv_superres.so.3.4.1
lib/libcamodocal_gpl.so: /usr/local/lib/libopencv_img_hash.so.3.4.1
lib/libcamodocal_gpl.so: /usr/local/lib/libopencv_hfs.so.3.4.1
lib/libcamodocal_gpl.so: /usr/local/lib/libopencv_face.so.3.4.1
lib/libcamodocal_gpl.so: /usr/local/lib/libopencv_aruco.so.3.4.1
lib/libcamodocal_gpl.so: /usr/local/lib/libopencv_freetype.so.3.4.1
lib/libcamodocal_gpl.so: /usr/local/lib/libopencv_stereo.so.3.4.1
lib/libcamodocal_gpl.so: /usr/local/lib/libopencv_surface_matching.so.3.4.1
lib/libcamodocal_gpl.so: /usr/local/lib/libopencv_saliency.so.3.4.1
lib/libcamodocal_gpl.so: /usr/local/lib/libopencv_fuzzy.so.3.4.1
lib/libcamodocal_gpl.so: /usr/local/lib/libopencv_structured_light.so.3.4.1
lib/libcamodocal_gpl.so: /usr/local/lib/libopencv_bgsegm.so.3.4.1
lib/libcamodocal_gpl.so: /usr/local/lib/libopencv_reg.so.3.4.1
lib/libcamodocal_gpl.so: /usr/local/lib/libopencv_bioinspired.so.3.4.1
lib/libcamodocal_gpl.so: /usr/local/lib/libopencv_optflow.so.3.4.1
lib/libcamodocal_gpl.so: /usr/local/lib/libopencv_line_descriptor.so.3.4.1
lib/libcamodocal_gpl.so: /usr/local/lib/libopencv_dpm.so.3.4.1
lib/libcamodocal_gpl.so: /usr/local/lib/libopencv_xobjdetect.so.3.4.1
lib/libcamodocal_gpl.so: /usr/local/lib/libopencv_rgbd.so.3.4.1
lib/libcamodocal_gpl.so: /usr/local/lib/libopencv_dnn_objdetect.so.3.4.1
lib/libcamodocal_gpl.so: /usr/local/lib/libopencv_ccalib.so.3.4.1
lib/libcamodocal_gpl.so: /usr/local/lib/libopencv_sfm.so.3.4.1
lib/libcamodocal_gpl.so: /usr/local/lib/libopencv_xphoto.so.3.4.1
lib/libcamodocal_gpl.so: /usr/local/lib/libceres.a
lib/libcamodocal_gpl.so: /usr/local/lib/libceres.a
lib/libcamodocal_gpl.so: /usr/local/lib/libopencv_photo.so.3.4.1
lib/libcamodocal_gpl.so: /usr/local/lib/libopencv_tracking.so.3.4.1
lib/libcamodocal_gpl.so: /usr/local/lib/libopencv_datasets.so.3.4.1
lib/libcamodocal_gpl.so: /usr/local/lib/libopencv_plot.so.3.4.1
lib/libcamodocal_gpl.so: /usr/local/lib/libopencv_phase_unwrapping.so.3.4.1
lib/libcamodocal_gpl.so: /usr/local/lib/libopencv_ximgproc.so.3.4.1
lib/libcamodocal_gpl.so: /usr/local/lib/libopencv_objdetect.so.3.4.1
lib/libcamodocal_gpl.so: /usr/local/lib/libopencv_dnn.so.3.4.1
lib/libcamodocal_gpl.so: /usr/local/lib/libopencv_xfeatures2d.so.3.4.1
lib/libcamodocal_gpl.so: /usr/local/lib/libopencv_ml.so.3.4.1
lib/libcamodocal_gpl.so: /usr/local/lib/libopencv_shape.so.3.4.1
lib/libcamodocal_gpl.so: /usr/local/lib/libopencv_video.so.3.4.1
lib/libcamodocal_gpl.so: /usr/local/lib/libopencv_calib3d.so.3.4.1
lib/libcamodocal_gpl.so: /usr/local/lib/libopencv_features2d.so.3.4.1
lib/libcamodocal_gpl.so: /usr/local/lib/libopencv_flann.so.3.4.1
lib/libcamodocal_gpl.so: /usr/local/lib/libopencv_highgui.so.3.4.1
lib/libcamodocal_gpl.so: /usr/local/lib/libopencv_videoio.so.3.4.1
lib/libcamodocal_gpl.so: /usr/local/lib/libopencv_imgcodecs.so.3.4.1
lib/libcamodocal_gpl.so: /usr/local/lib/libopencv_imgproc.so.3.4.1
lib/libcamodocal_gpl.so: /usr/local/lib/libopencv_core.so.3.4.1
lib/libcamodocal_gpl.so: /usr/local/lib/libglog.so.0.5.0
lib/libcamodocal_gpl.so: /usr/local/lib/libgflags.so.2.2.2
lib/libcamodocal_gpl.so: /usr/local/lib/libgflags.so.2.2.2
lib/libcamodocal_gpl.so: src/gpl/CMakeFiles/camodocal_gpl.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/zoukaixiang/code/camodocal/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX shared library ../../lib/libcamodocal_gpl.so"
	cd /home/zoukaixiang/code/camodocal/cmake-build-debug/src/gpl && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/camodocal_gpl.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/gpl/CMakeFiles/camodocal_gpl.dir/build: lib/libcamodocal_gpl.so

.PHONY : src/gpl/CMakeFiles/camodocal_gpl.dir/build

src/gpl/CMakeFiles/camodocal_gpl.dir/clean:
	cd /home/zoukaixiang/code/camodocal/cmake-build-debug/src/gpl && $(CMAKE_COMMAND) -P CMakeFiles/camodocal_gpl.dir/cmake_clean.cmake
.PHONY : src/gpl/CMakeFiles/camodocal_gpl.dir/clean

src/gpl/CMakeFiles/camodocal_gpl.dir/depend:
	cd /home/zoukaixiang/code/camodocal/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/zoukaixiang/code/camodocal /home/zoukaixiang/code/camodocal/src/gpl /home/zoukaixiang/code/camodocal/cmake-build-debug /home/zoukaixiang/code/camodocal/cmake-build-debug/src/gpl /home/zoukaixiang/code/camodocal/cmake-build-debug/src/gpl/CMakeFiles/camodocal_gpl.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/gpl/CMakeFiles/camodocal_gpl.dir/depend
