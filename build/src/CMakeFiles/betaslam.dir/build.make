# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


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
CMAKE_SOURCE_DIR = /home/winter/Desktop/slam/myslam

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/winter/Desktop/slam/myslam/build

# Include any dependencies generated for this target.
include src/CMakeFiles/betaslam.dir/depend.make

# Include the progress variables for this target.
include src/CMakeFiles/betaslam.dir/progress.make

# Include the compile flags for this target's objects.
include src/CMakeFiles/betaslam.dir/flags.make

src/CMakeFiles/betaslam.dir/config.cpp.o: src/CMakeFiles/betaslam.dir/flags.make
src/CMakeFiles/betaslam.dir/config.cpp.o: ../src/config.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/winter/Desktop/slam/myslam/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/CMakeFiles/betaslam.dir/config.cpp.o"
	cd /home/winter/Desktop/slam/myslam/build/src && g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/betaslam.dir/config.cpp.o -c /home/winter/Desktop/slam/myslam/src/config.cpp

src/CMakeFiles/betaslam.dir/config.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/betaslam.dir/config.cpp.i"
	cd /home/winter/Desktop/slam/myslam/build/src && g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/winter/Desktop/slam/myslam/src/config.cpp > CMakeFiles/betaslam.dir/config.cpp.i

src/CMakeFiles/betaslam.dir/config.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/betaslam.dir/config.cpp.s"
	cd /home/winter/Desktop/slam/myslam/build/src && g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/winter/Desktop/slam/myslam/src/config.cpp -o CMakeFiles/betaslam.dir/config.cpp.s

src/CMakeFiles/betaslam.dir/config.cpp.o.requires:

.PHONY : src/CMakeFiles/betaslam.dir/config.cpp.o.requires

src/CMakeFiles/betaslam.dir/config.cpp.o.provides: src/CMakeFiles/betaslam.dir/config.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/betaslam.dir/build.make src/CMakeFiles/betaslam.dir/config.cpp.o.provides.build
.PHONY : src/CMakeFiles/betaslam.dir/config.cpp.o.provides

src/CMakeFiles/betaslam.dir/config.cpp.o.provides.build: src/CMakeFiles/betaslam.dir/config.cpp.o


# Object files for target betaslam
betaslam_OBJECTS = \
"CMakeFiles/betaslam.dir/config.cpp.o"

# External object files for target betaslam
betaslam_EXTERNAL_OBJECTS =

../lib/libbetaslam.so: src/CMakeFiles/betaslam.dir/config.cpp.o
../lib/libbetaslam.so: src/CMakeFiles/betaslam.dir/build.make
../lib/libbetaslam.so: src/CMakeFiles/betaslam.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/winter/Desktop/slam/myslam/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library ../../lib/libbetaslam.so"
	cd /home/winter/Desktop/slam/myslam/build/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/betaslam.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/CMakeFiles/betaslam.dir/build: ../lib/libbetaslam.so

.PHONY : src/CMakeFiles/betaslam.dir/build

src/CMakeFiles/betaslam.dir/requires: src/CMakeFiles/betaslam.dir/config.cpp.o.requires

.PHONY : src/CMakeFiles/betaslam.dir/requires

src/CMakeFiles/betaslam.dir/clean:
	cd /home/winter/Desktop/slam/myslam/build/src && $(CMAKE_COMMAND) -P CMakeFiles/betaslam.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/betaslam.dir/clean

src/CMakeFiles/betaslam.dir/depend:
	cd /home/winter/Desktop/slam/myslam/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/winter/Desktop/slam/myslam /home/winter/Desktop/slam/myslam/src /home/winter/Desktop/slam/myslam/build /home/winter/Desktop/slam/myslam/build/src /home/winter/Desktop/slam/myslam/build/src/CMakeFiles/betaslam.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/betaslam.dir/depend

