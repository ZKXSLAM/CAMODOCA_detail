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
include src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/depend.make

# Include the progress variables for this target.
include src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/progress.make

# Include the compile flags for this target's objects.
include src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/flags.make

src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/BinaryFile.cpp.o: src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/flags.make
src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/BinaryFile.cpp.o: ../src/dbow2/DUtils/BinaryFile.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zoukaixiang/code/camodocal/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/BinaryFile.cpp.o"
	cd /home/zoukaixiang/code/camodocal/cmake-build-debug/src/dbow2/DUtils && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/camodocal_dutils.dir/BinaryFile.cpp.o -c /home/zoukaixiang/code/camodocal/src/dbow2/DUtils/BinaryFile.cpp

src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/BinaryFile.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/camodocal_dutils.dir/BinaryFile.cpp.i"
	cd /home/zoukaixiang/code/camodocal/cmake-build-debug/src/dbow2/DUtils && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zoukaixiang/code/camodocal/src/dbow2/DUtils/BinaryFile.cpp > CMakeFiles/camodocal_dutils.dir/BinaryFile.cpp.i

src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/BinaryFile.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/camodocal_dutils.dir/BinaryFile.cpp.s"
	cd /home/zoukaixiang/code/camodocal/cmake-build-debug/src/dbow2/DUtils && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zoukaixiang/code/camodocal/src/dbow2/DUtils/BinaryFile.cpp -o CMakeFiles/camodocal_dutils.dir/BinaryFile.cpp.s

src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/ConfigFile.cpp.o: src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/flags.make
src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/ConfigFile.cpp.o: ../src/dbow2/DUtils/ConfigFile.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zoukaixiang/code/camodocal/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/ConfigFile.cpp.o"
	cd /home/zoukaixiang/code/camodocal/cmake-build-debug/src/dbow2/DUtils && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/camodocal_dutils.dir/ConfigFile.cpp.o -c /home/zoukaixiang/code/camodocal/src/dbow2/DUtils/ConfigFile.cpp

src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/ConfigFile.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/camodocal_dutils.dir/ConfigFile.cpp.i"
	cd /home/zoukaixiang/code/camodocal/cmake-build-debug/src/dbow2/DUtils && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zoukaixiang/code/camodocal/src/dbow2/DUtils/ConfigFile.cpp > CMakeFiles/camodocal_dutils.dir/ConfigFile.cpp.i

src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/ConfigFile.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/camodocal_dutils.dir/ConfigFile.cpp.s"
	cd /home/zoukaixiang/code/camodocal/cmake-build-debug/src/dbow2/DUtils && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zoukaixiang/code/camodocal/src/dbow2/DUtils/ConfigFile.cpp -o CMakeFiles/camodocal_dutils.dir/ConfigFile.cpp.s

src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/DebugFunctions.cpp.o: src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/flags.make
src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/DebugFunctions.cpp.o: ../src/dbow2/DUtils/DebugFunctions.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zoukaixiang/code/camodocal/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/DebugFunctions.cpp.o"
	cd /home/zoukaixiang/code/camodocal/cmake-build-debug/src/dbow2/DUtils && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/camodocal_dutils.dir/DebugFunctions.cpp.o -c /home/zoukaixiang/code/camodocal/src/dbow2/DUtils/DebugFunctions.cpp

src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/DebugFunctions.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/camodocal_dutils.dir/DebugFunctions.cpp.i"
	cd /home/zoukaixiang/code/camodocal/cmake-build-debug/src/dbow2/DUtils && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zoukaixiang/code/camodocal/src/dbow2/DUtils/DebugFunctions.cpp > CMakeFiles/camodocal_dutils.dir/DebugFunctions.cpp.i

src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/DebugFunctions.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/camodocal_dutils.dir/DebugFunctions.cpp.s"
	cd /home/zoukaixiang/code/camodocal/cmake-build-debug/src/dbow2/DUtils && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zoukaixiang/code/camodocal/src/dbow2/DUtils/DebugFunctions.cpp -o CMakeFiles/camodocal_dutils.dir/DebugFunctions.cpp.s

src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/FileFunctions.cpp.o: src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/flags.make
src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/FileFunctions.cpp.o: ../src/dbow2/DUtils/FileFunctions.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zoukaixiang/code/camodocal/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/FileFunctions.cpp.o"
	cd /home/zoukaixiang/code/camodocal/cmake-build-debug/src/dbow2/DUtils && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/camodocal_dutils.dir/FileFunctions.cpp.o -c /home/zoukaixiang/code/camodocal/src/dbow2/DUtils/FileFunctions.cpp

src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/FileFunctions.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/camodocal_dutils.dir/FileFunctions.cpp.i"
	cd /home/zoukaixiang/code/camodocal/cmake-build-debug/src/dbow2/DUtils && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zoukaixiang/code/camodocal/src/dbow2/DUtils/FileFunctions.cpp > CMakeFiles/camodocal_dutils.dir/FileFunctions.cpp.i

src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/FileFunctions.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/camodocal_dutils.dir/FileFunctions.cpp.s"
	cd /home/zoukaixiang/code/camodocal/cmake-build-debug/src/dbow2/DUtils && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zoukaixiang/code/camodocal/src/dbow2/DUtils/FileFunctions.cpp -o CMakeFiles/camodocal_dutils.dir/FileFunctions.cpp.s

src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/LineFile.cpp.o: src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/flags.make
src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/LineFile.cpp.o: ../src/dbow2/DUtils/LineFile.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zoukaixiang/code/camodocal/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/LineFile.cpp.o"
	cd /home/zoukaixiang/code/camodocal/cmake-build-debug/src/dbow2/DUtils && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/camodocal_dutils.dir/LineFile.cpp.o -c /home/zoukaixiang/code/camodocal/src/dbow2/DUtils/LineFile.cpp

src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/LineFile.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/camodocal_dutils.dir/LineFile.cpp.i"
	cd /home/zoukaixiang/code/camodocal/cmake-build-debug/src/dbow2/DUtils && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zoukaixiang/code/camodocal/src/dbow2/DUtils/LineFile.cpp > CMakeFiles/camodocal_dutils.dir/LineFile.cpp.i

src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/LineFile.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/camodocal_dutils.dir/LineFile.cpp.s"
	cd /home/zoukaixiang/code/camodocal/cmake-build-debug/src/dbow2/DUtils && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zoukaixiang/code/camodocal/src/dbow2/DUtils/LineFile.cpp -o CMakeFiles/camodocal_dutils.dir/LineFile.cpp.s

src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/LUT.cpp.o: src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/flags.make
src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/LUT.cpp.o: ../src/dbow2/DUtils/LUT.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zoukaixiang/code/camodocal/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/LUT.cpp.o"
	cd /home/zoukaixiang/code/camodocal/cmake-build-debug/src/dbow2/DUtils && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/camodocal_dutils.dir/LUT.cpp.o -c /home/zoukaixiang/code/camodocal/src/dbow2/DUtils/LUT.cpp

src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/LUT.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/camodocal_dutils.dir/LUT.cpp.i"
	cd /home/zoukaixiang/code/camodocal/cmake-build-debug/src/dbow2/DUtils && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zoukaixiang/code/camodocal/src/dbow2/DUtils/LUT.cpp > CMakeFiles/camodocal_dutils.dir/LUT.cpp.i

src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/LUT.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/camodocal_dutils.dir/LUT.cpp.s"
	cd /home/zoukaixiang/code/camodocal/cmake-build-debug/src/dbow2/DUtils && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zoukaixiang/code/camodocal/src/dbow2/DUtils/LUT.cpp -o CMakeFiles/camodocal_dutils.dir/LUT.cpp.s

src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/Profiler.cpp.o: src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/flags.make
src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/Profiler.cpp.o: ../src/dbow2/DUtils/Profiler.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zoukaixiang/code/camodocal/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/Profiler.cpp.o"
	cd /home/zoukaixiang/code/camodocal/cmake-build-debug/src/dbow2/DUtils && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/camodocal_dutils.dir/Profiler.cpp.o -c /home/zoukaixiang/code/camodocal/src/dbow2/DUtils/Profiler.cpp

src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/Profiler.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/camodocal_dutils.dir/Profiler.cpp.i"
	cd /home/zoukaixiang/code/camodocal/cmake-build-debug/src/dbow2/DUtils && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zoukaixiang/code/camodocal/src/dbow2/DUtils/Profiler.cpp > CMakeFiles/camodocal_dutils.dir/Profiler.cpp.i

src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/Profiler.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/camodocal_dutils.dir/Profiler.cpp.s"
	cd /home/zoukaixiang/code/camodocal/cmake-build-debug/src/dbow2/DUtils && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zoukaixiang/code/camodocal/src/dbow2/DUtils/Profiler.cpp -o CMakeFiles/camodocal_dutils.dir/Profiler.cpp.s

src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/Random.cpp.o: src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/flags.make
src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/Random.cpp.o: ../src/dbow2/DUtils/Random.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zoukaixiang/code/camodocal/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/Random.cpp.o"
	cd /home/zoukaixiang/code/camodocal/cmake-build-debug/src/dbow2/DUtils && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/camodocal_dutils.dir/Random.cpp.o -c /home/zoukaixiang/code/camodocal/src/dbow2/DUtils/Random.cpp

src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/Random.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/camodocal_dutils.dir/Random.cpp.i"
	cd /home/zoukaixiang/code/camodocal/cmake-build-debug/src/dbow2/DUtils && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zoukaixiang/code/camodocal/src/dbow2/DUtils/Random.cpp > CMakeFiles/camodocal_dutils.dir/Random.cpp.i

src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/Random.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/camodocal_dutils.dir/Random.cpp.s"
	cd /home/zoukaixiang/code/camodocal/cmake-build-debug/src/dbow2/DUtils && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zoukaixiang/code/camodocal/src/dbow2/DUtils/Random.cpp -o CMakeFiles/camodocal_dutils.dir/Random.cpp.s

src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/StringFunctions.cpp.o: src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/flags.make
src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/StringFunctions.cpp.o: ../src/dbow2/DUtils/StringFunctions.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zoukaixiang/code/camodocal/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building CXX object src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/StringFunctions.cpp.o"
	cd /home/zoukaixiang/code/camodocal/cmake-build-debug/src/dbow2/DUtils && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/camodocal_dutils.dir/StringFunctions.cpp.o -c /home/zoukaixiang/code/camodocal/src/dbow2/DUtils/StringFunctions.cpp

src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/StringFunctions.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/camodocal_dutils.dir/StringFunctions.cpp.i"
	cd /home/zoukaixiang/code/camodocal/cmake-build-debug/src/dbow2/DUtils && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zoukaixiang/code/camodocal/src/dbow2/DUtils/StringFunctions.cpp > CMakeFiles/camodocal_dutils.dir/StringFunctions.cpp.i

src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/StringFunctions.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/camodocal_dutils.dir/StringFunctions.cpp.s"
	cd /home/zoukaixiang/code/camodocal/cmake-build-debug/src/dbow2/DUtils && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zoukaixiang/code/camodocal/src/dbow2/DUtils/StringFunctions.cpp -o CMakeFiles/camodocal_dutils.dir/StringFunctions.cpp.s

src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/TimeManager.cpp.o: src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/flags.make
src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/TimeManager.cpp.o: ../src/dbow2/DUtils/TimeManager.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zoukaixiang/code/camodocal/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Building CXX object src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/TimeManager.cpp.o"
	cd /home/zoukaixiang/code/camodocal/cmake-build-debug/src/dbow2/DUtils && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/camodocal_dutils.dir/TimeManager.cpp.o -c /home/zoukaixiang/code/camodocal/src/dbow2/DUtils/TimeManager.cpp

src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/TimeManager.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/camodocal_dutils.dir/TimeManager.cpp.i"
	cd /home/zoukaixiang/code/camodocal/cmake-build-debug/src/dbow2/DUtils && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zoukaixiang/code/camodocal/src/dbow2/DUtils/TimeManager.cpp > CMakeFiles/camodocal_dutils.dir/TimeManager.cpp.i

src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/TimeManager.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/camodocal_dutils.dir/TimeManager.cpp.s"
	cd /home/zoukaixiang/code/camodocal/cmake-build-debug/src/dbow2/DUtils && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zoukaixiang/code/camodocal/src/dbow2/DUtils/TimeManager.cpp -o CMakeFiles/camodocal_dutils.dir/TimeManager.cpp.s

src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/Timestamp.cpp.o: src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/flags.make
src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/Timestamp.cpp.o: ../src/dbow2/DUtils/Timestamp.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zoukaixiang/code/camodocal/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_11) "Building CXX object src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/Timestamp.cpp.o"
	cd /home/zoukaixiang/code/camodocal/cmake-build-debug/src/dbow2/DUtils && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/camodocal_dutils.dir/Timestamp.cpp.o -c /home/zoukaixiang/code/camodocal/src/dbow2/DUtils/Timestamp.cpp

src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/Timestamp.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/camodocal_dutils.dir/Timestamp.cpp.i"
	cd /home/zoukaixiang/code/camodocal/cmake-build-debug/src/dbow2/DUtils && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zoukaixiang/code/camodocal/src/dbow2/DUtils/Timestamp.cpp > CMakeFiles/camodocal_dutils.dir/Timestamp.cpp.i

src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/Timestamp.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/camodocal_dutils.dir/Timestamp.cpp.s"
	cd /home/zoukaixiang/code/camodocal/cmake-build-debug/src/dbow2/DUtils && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zoukaixiang/code/camodocal/src/dbow2/DUtils/Timestamp.cpp -o CMakeFiles/camodocal_dutils.dir/Timestamp.cpp.s

# Object files for target camodocal_dutils
camodocal_dutils_OBJECTS = \
"CMakeFiles/camodocal_dutils.dir/BinaryFile.cpp.o" \
"CMakeFiles/camodocal_dutils.dir/ConfigFile.cpp.o" \
"CMakeFiles/camodocal_dutils.dir/DebugFunctions.cpp.o" \
"CMakeFiles/camodocal_dutils.dir/FileFunctions.cpp.o" \
"CMakeFiles/camodocal_dutils.dir/LineFile.cpp.o" \
"CMakeFiles/camodocal_dutils.dir/LUT.cpp.o" \
"CMakeFiles/camodocal_dutils.dir/Profiler.cpp.o" \
"CMakeFiles/camodocal_dutils.dir/Random.cpp.o" \
"CMakeFiles/camodocal_dutils.dir/StringFunctions.cpp.o" \
"CMakeFiles/camodocal_dutils.dir/TimeManager.cpp.o" \
"CMakeFiles/camodocal_dutils.dir/Timestamp.cpp.o"

# External object files for target camodocal_dutils
camodocal_dutils_EXTERNAL_OBJECTS =

lib/libcamodocal_dutils.so: src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/BinaryFile.cpp.o
lib/libcamodocal_dutils.so: src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/ConfigFile.cpp.o
lib/libcamodocal_dutils.so: src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/DebugFunctions.cpp.o
lib/libcamodocal_dutils.so: src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/FileFunctions.cpp.o
lib/libcamodocal_dutils.so: src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/LineFile.cpp.o
lib/libcamodocal_dutils.so: src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/LUT.cpp.o
lib/libcamodocal_dutils.so: src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/Profiler.cpp.o
lib/libcamodocal_dutils.so: src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/Random.cpp.o
lib/libcamodocal_dutils.so: src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/StringFunctions.cpp.o
lib/libcamodocal_dutils.so: src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/TimeManager.cpp.o
lib/libcamodocal_dutils.so: src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/Timestamp.cpp.o
lib/libcamodocal_dutils.so: src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/build.make
lib/libcamodocal_dutils.so: src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/zoukaixiang/code/camodocal/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_12) "Linking CXX shared library ../../../lib/libcamodocal_dutils.so"
	cd /home/zoukaixiang/code/camodocal/cmake-build-debug/src/dbow2/DUtils && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/camodocal_dutils.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/build: lib/libcamodocal_dutils.so

.PHONY : src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/build

src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/clean:
	cd /home/zoukaixiang/code/camodocal/cmake-build-debug/src/dbow2/DUtils && $(CMAKE_COMMAND) -P CMakeFiles/camodocal_dutils.dir/cmake_clean.cmake
.PHONY : src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/clean

src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/depend:
	cd /home/zoukaixiang/code/camodocal/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/zoukaixiang/code/camodocal /home/zoukaixiang/code/camodocal/src/dbow2/DUtils /home/zoukaixiang/code/camodocal/cmake-build-debug /home/zoukaixiang/code/camodocal/cmake-build-debug/src/dbow2/DUtils /home/zoukaixiang/code/camodocal/cmake-build-debug/src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/dbow2/DUtils/CMakeFiles/camodocal_dutils.dir/depend

