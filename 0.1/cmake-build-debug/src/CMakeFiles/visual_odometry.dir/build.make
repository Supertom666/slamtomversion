# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.17

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
CMAKE_COMMAND = /snap/clion/129/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /snap/clion/129/bin/cmake/linux/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/tom/CLionProjects/0.1

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/tom/CLionProjects/0.1/cmake-build-debug

# Include any dependencies generated for this target.
include src/CMakeFiles/visual_odometry.dir/depend.make

# Include the progress variables for this target.
include src/CMakeFiles/visual_odometry.dir/progress.make

# Include the compile flags for this target's objects.
include src/CMakeFiles/visual_odometry.dir/flags.make

src/CMakeFiles/visual_odometry.dir/visual_odometry.cpp.o: src/CMakeFiles/visual_odometry.dir/flags.make
src/CMakeFiles/visual_odometry.dir/visual_odometry.cpp.o: ../src/visual_odometry.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/tom/CLionProjects/0.1/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/CMakeFiles/visual_odometry.dir/visual_odometry.cpp.o"
	cd /home/tom/CLionProjects/0.1/cmake-build-debug/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/visual_odometry.dir/visual_odometry.cpp.o -c /home/tom/CLionProjects/0.1/src/visual_odometry.cpp

src/CMakeFiles/visual_odometry.dir/visual_odometry.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/visual_odometry.dir/visual_odometry.cpp.i"
	cd /home/tom/CLionProjects/0.1/cmake-build-debug/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/tom/CLionProjects/0.1/src/visual_odometry.cpp > CMakeFiles/visual_odometry.dir/visual_odometry.cpp.i

src/CMakeFiles/visual_odometry.dir/visual_odometry.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/visual_odometry.dir/visual_odometry.cpp.s"
	cd /home/tom/CLionProjects/0.1/cmake-build-debug/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/tom/CLionProjects/0.1/src/visual_odometry.cpp -o CMakeFiles/visual_odometry.dir/visual_odometry.cpp.s

# Object files for target visual_odometry
visual_odometry_OBJECTS = \
"CMakeFiles/visual_odometry.dir/visual_odometry.cpp.o"

# External object files for target visual_odometry
visual_odometry_EXTERNAL_OBJECTS =

../bin/visual_odometry: src/CMakeFiles/visual_odometry.dir/visual_odometry.cpp.o
../bin/visual_odometry: src/CMakeFiles/visual_odometry.dir/build.make
../bin/visual_odometry: src/CMakeFiles/visual_odometry.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/tom/CLionProjects/0.1/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../bin/visual_odometry"
	cd /home/tom/CLionProjects/0.1/cmake-build-debug/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/visual_odometry.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/CMakeFiles/visual_odometry.dir/build: ../bin/visual_odometry

.PHONY : src/CMakeFiles/visual_odometry.dir/build

src/CMakeFiles/visual_odometry.dir/clean:
	cd /home/tom/CLionProjects/0.1/cmake-build-debug/src && $(CMAKE_COMMAND) -P CMakeFiles/visual_odometry.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/visual_odometry.dir/clean

src/CMakeFiles/visual_odometry.dir/depend:
	cd /home/tom/CLionProjects/0.1/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/tom/CLionProjects/0.1 /home/tom/CLionProjects/0.1/src /home/tom/CLionProjects/0.1/cmake-build-debug /home/tom/CLionProjects/0.1/cmake-build-debug/src /home/tom/CLionProjects/0.1/cmake-build-debug/src/CMakeFiles/visual_odometry.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/visual_odometry.dir/depend
