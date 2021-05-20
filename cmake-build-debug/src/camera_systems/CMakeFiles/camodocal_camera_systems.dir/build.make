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
include src/camera_systems/CMakeFiles/camodocal_camera_systems.dir/depend.make

# Include the progress variables for this target.
include src/camera_systems/CMakeFiles/camodocal_camera_systems.dir/progress.make

# Include the compile flags for this target's objects.
include src/camera_systems/CMakeFiles/camodocal_camera_systems.dir/flags.make

src/camera_systems/CMakeFiles/camodocal_camera_systems.dir/CameraSystem.cc.o: src/camera_systems/CMakeFiles/camodocal_camera_systems.dir/flags.make
src/camera_systems/CMakeFiles/camodocal_camera_systems.dir/CameraSystem.cc.o: ../src/camera_systems/CameraSystem.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zoukaixiang/code/camodocal/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/camera_systems/CMakeFiles/camodocal_camera_systems.dir/CameraSystem.cc.o"
	cd /home/zoukaixiang/code/camodocal/cmake-build-debug/src/camera_systems && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/camodocal_camera_systems.dir/CameraSystem.cc.o -c /home/zoukaixiang/code/camodocal/src/camera_systems/CameraSystem.cc

src/camera_systems/CMakeFiles/camodocal_camera_systems.dir/CameraSystem.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/camodocal_camera_systems.dir/CameraSystem.cc.i"
	cd /home/zoukaixiang/code/camodocal/cmake-build-debug/src/camera_systems && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zoukaixiang/code/camodocal/src/camera_systems/CameraSystem.cc > CMakeFiles/camodocal_camera_systems.dir/CameraSystem.cc.i

src/camera_systems/CMakeFiles/camodocal_camera_systems.dir/CameraSystem.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/camodocal_camera_systems.dir/CameraSystem.cc.s"
	cd /home/zoukaixiang/code/camodocal/cmake-build-debug/src/camera_systems && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zoukaixiang/code/camodocal/src/camera_systems/CameraSystem.cc -o CMakeFiles/camodocal_camera_systems.dir/CameraSystem.cc.s

# Object files for target camodocal_camera_systems
camodocal_camera_systems_OBJECTS = \
"CMakeFiles/camodocal_camera_systems.dir/CameraSystem.cc.o"

# External object files for target camodocal_camera_systems
camodocal_camera_systems_EXTERNAL_OBJECTS =

lib/libcamodocal_camera_systems.so: src/camera_systems/CMakeFiles/camodocal_camera_systems.dir/CameraSystem.cc.o
lib/libcamodocal_camera_systems.so: src/camera_systems/CMakeFiles/camodocal_camera_systems.dir/build.make
lib/libcamodocal_camera_systems.so: /usr/lib/x86_64-linux-gnu/libdl.so
lib/libcamodocal_camera_systems.so: /usr/lib/x86_64-linux-gnu/libnsl.so
lib/libcamodocal_camera_systems.so: /usr/lib/x86_64-linux-gnu/libm.so
lib/libcamodocal_camera_systems.so: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
lib/libcamodocal_camera_systems.so: /usr/lib/x86_64-linux-gnu/libboost_system.so
lib/libcamodocal_camera_systems.so: lib/libcamodocal_camera_models.so
lib/libcamodocal_camera_systems.so: lib/libcamodocal_pugixml.so
lib/libcamodocal_camera_systems.so: lib/libcamodocal_gpl.so
lib/libcamodocal_camera_systems.so: /usr/lib/x86_64-linux-gnu/libdl.so
lib/libcamodocal_camera_systems.so: /usr/lib/x86_64-linux-gnu/libnsl.so
lib/libcamodocal_camera_systems.so: /usr/lib/x86_64-linux-gnu/libm.so
lib/libcamodocal_camera_systems.so: /usr/local/lib/libopencv_stitching.so.3.4.1
lib/libcamodocal_camera_systems.so: /usr/local/lib/libopencv_videostab.so.3.4.1
lib/libcamodocal_camera_systems.so: /usr/local/lib/libopencv_superres.so.3.4.1
lib/libcamodocal_camera_systems.so: /usr/local/lib/libopencv_img_hash.so.3.4.1
lib/libcamodocal_camera_systems.so: /usr/local/lib/libopencv_hfs.so.3.4.1
lib/libcamodocal_camera_systems.so: /usr/local/lib/libopencv_face.so.3.4.1
lib/libcamodocal_camera_systems.so: /usr/local/lib/libopencv_photo.so.3.4.1
lib/libcamodocal_camera_systems.so: /usr/local/lib/libopencv_tracking.so.3.4.1
lib/libcamodocal_camera_systems.so: /usr/local/lib/libopencv_datasets.so.3.4.1
lib/libcamodocal_camera_systems.so: /usr/local/lib/libopencv_aruco.so.3.4.1
lib/libcamodocal_camera_systems.so: /usr/local/lib/libopencv_freetype.so.3.4.1
lib/libcamodocal_camera_systems.so: /usr/local/lib/libopencv_stereo.so.3.4.1
lib/libcamodocal_camera_systems.so: /usr/local/lib/libopencv_surface_matching.so.3.4.1
lib/libcamodocal_camera_systems.so: /usr/local/lib/libopencv_saliency.so.3.4.1
lib/libcamodocal_camera_systems.so: /usr/local/lib/libopencv_fuzzy.so.3.4.1
lib/libcamodocal_camera_systems.so: /usr/local/lib/libopencv_plot.so.3.4.1
lib/libcamodocal_camera_systems.so: /usr/local/lib/libopencv_structured_light.so.3.4.1
lib/libcamodocal_camera_systems.so: /usr/local/lib/libopencv_phase_unwrapping.so.3.4.1
lib/libcamodocal_camera_systems.so: /usr/local/lib/libopencv_bgsegm.so.3.4.1
lib/libcamodocal_camera_systems.so: /usr/local/lib/libopencv_reg.so.3.4.1
lib/libcamodocal_camera_systems.so: /usr/local/lib/libopencv_bioinspired.so.3.4.1
lib/libcamodocal_camera_systems.so: /usr/local/lib/libopencv_optflow.so.3.4.1
lib/libcamodocal_camera_systems.so: /usr/local/lib/libopencv_ximgproc.so.3.4.1
lib/libcamodocal_camera_systems.so: /usr/local/lib/libopencv_line_descriptor.so.3.4.1
lib/libcamodocal_camera_systems.so: /usr/local/lib/libopencv_dpm.so.3.4.1
lib/libcamodocal_camera_systems.so: /usr/local/lib/libopencv_xobjdetect.so.3.4.1
lib/libcamodocal_camera_systems.so: /usr/local/lib/libopencv_objdetect.so.3.4.1
lib/libcamodocal_camera_systems.so: /usr/local/lib/libopencv_rgbd.so.3.4.1
lib/libcamodocal_camera_systems.so: /usr/local/lib/libopencv_dnn_objdetect.so.3.4.1
lib/libcamodocal_camera_systems.so: /usr/local/lib/libopencv_dnn.so.3.4.1
lib/libcamodocal_camera_systems.so: /usr/local/lib/libopencv_ccalib.so.3.4.1
lib/libcamodocal_camera_systems.so: /usr/local/lib/libopencv_sfm.so.3.4.1
lib/libcamodocal_camera_systems.so: /usr/local/lib/libopencv_xfeatures2d.so.3.4.1
lib/libcamodocal_camera_systems.so: /usr/local/lib/libopencv_ml.so.3.4.1
lib/libcamodocal_camera_systems.so: /usr/local/lib/libopencv_shape.so.3.4.1
lib/libcamodocal_camera_systems.so: /usr/local/lib/libopencv_video.so.3.4.1
lib/libcamodocal_camera_systems.so: /usr/local/lib/libopencv_calib3d.so.3.4.1
lib/libcamodocal_camera_systems.so: /usr/local/lib/libopencv_features2d.so.3.4.1
lib/libcamodocal_camera_systems.so: /usr/local/lib/libopencv_flann.so.3.4.1
lib/libcamodocal_camera_systems.so: /usr/local/lib/libopencv_highgui.so.3.4.1
lib/libcamodocal_camera_systems.so: /usr/local/lib/libopencv_videoio.so.3.4.1
lib/libcamodocal_camera_systems.so: /usr/local/lib/libopencv_imgcodecs.so.3.4.1
lib/libcamodocal_camera_systems.so: /usr/local/lib/libopencv_xphoto.so.3.4.1
lib/libcamodocal_camera_systems.so: /usr/local/lib/libopencv_imgproc.so.3.4.1
lib/libcamodocal_camera_systems.so: /usr/local/lib/libopencv_core.so.3.4.1
lib/libcamodocal_camera_systems.so: /usr/local/lib/libceres.a
lib/libcamodocal_camera_systems.so: /usr/local/lib/libgflags.so.2.2.2
lib/libcamodocal_camera_systems.so: /usr/local/lib/libglog.so.0.5.0
lib/libcamodocal_camera_systems.so: /usr/local/lib/libgflags.so.2.2.2
lib/libcamodocal_camera_systems.so: src/camera_systems/CMakeFiles/camodocal_camera_systems.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/zoukaixiang/code/camodocal/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library ../../lib/libcamodocal_camera_systems.so"
	cd /home/zoukaixiang/code/camodocal/cmake-build-debug/src/camera_systems && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/camodocal_camera_systems.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/camera_systems/CMakeFiles/camodocal_camera_systems.dir/build: lib/libcamodocal_camera_systems.so

.PHONY : src/camera_systems/CMakeFiles/camodocal_camera_systems.dir/build

src/camera_systems/CMakeFiles/camodocal_camera_systems.dir/clean:
	cd /home/zoukaixiang/code/camodocal/cmake-build-debug/src/camera_systems && $(CMAKE_COMMAND) -P CMakeFiles/camodocal_camera_systems.dir/cmake_clean.cmake
.PHONY : src/camera_systems/CMakeFiles/camodocal_camera_systems.dir/clean

src/camera_systems/CMakeFiles/camodocal_camera_systems.dir/depend:
	cd /home/zoukaixiang/code/camodocal/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/zoukaixiang/code/camodocal /home/zoukaixiang/code/camodocal/src/camera_systems /home/zoukaixiang/code/camodocal/cmake-build-debug /home/zoukaixiang/code/camodocal/cmake-build-debug/src/camera_systems /home/zoukaixiang/code/camodocal/cmake-build-debug/src/camera_systems/CMakeFiles/camodocal_camera_systems.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/camera_systems/CMakeFiles/camodocal_camera_systems.dir/depend

