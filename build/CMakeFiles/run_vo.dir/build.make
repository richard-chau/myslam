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
CMAKE_SOURCE_DIR = /home/winter/Desktop/slam/slambook-master/project/newslam

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/winter/Desktop/slam/slambook-master/project/newslam/build

# Include any dependencies generated for this target.
include CMakeFiles/run_vo.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/run_vo.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/run_vo.dir/flags.make

CMakeFiles/run_vo.dir/main.cpp.o: CMakeFiles/run_vo.dir/flags.make
CMakeFiles/run_vo.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/winter/Desktop/slam/slambook-master/project/newslam/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/run_vo.dir/main.cpp.o"
	g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/run_vo.dir/main.cpp.o -c /home/winter/Desktop/slam/slambook-master/project/newslam/main.cpp

CMakeFiles/run_vo.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/run_vo.dir/main.cpp.i"
	g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/winter/Desktop/slam/slambook-master/project/newslam/main.cpp > CMakeFiles/run_vo.dir/main.cpp.i

CMakeFiles/run_vo.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/run_vo.dir/main.cpp.s"
	g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/winter/Desktop/slam/slambook-master/project/newslam/main.cpp -o CMakeFiles/run_vo.dir/main.cpp.s

CMakeFiles/run_vo.dir/main.cpp.o.requires:

.PHONY : CMakeFiles/run_vo.dir/main.cpp.o.requires

CMakeFiles/run_vo.dir/main.cpp.o.provides: CMakeFiles/run_vo.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/run_vo.dir/build.make CMakeFiles/run_vo.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/run_vo.dir/main.cpp.o.provides

CMakeFiles/run_vo.dir/main.cpp.o.provides.build: CMakeFiles/run_vo.dir/main.cpp.o


# Object files for target run_vo
run_vo_OBJECTS = \
"CMakeFiles/run_vo.dir/main.cpp.o"

# External object files for target run_vo
run_vo_EXTERNAL_OBJECTS =

../bin/run_vo: CMakeFiles/run_vo.dir/main.cpp.o
../bin/run_vo: CMakeFiles/run_vo.dir/build.make
../bin/run_vo: ../lib/libbetaslam.so
../bin/run_vo: /usr/local/lib/libopencv_dnn.so.3.4.0
../bin/run_vo: /usr/local/lib/libopencv_ml.so.3.4.0
../bin/run_vo: /usr/local/lib/libopencv_objdetect.so.3.4.0
../bin/run_vo: /usr/local/lib/libopencv_shape.so.3.4.0
../bin/run_vo: /usr/local/lib/libopencv_stitching.so.3.4.0
../bin/run_vo: /usr/local/lib/libopencv_superres.so.3.4.0
../bin/run_vo: /usr/local/lib/libopencv_videostab.so.3.4.0
../bin/run_vo: /usr/local/lib/libopencv_calib3d.so.3.4.0
../bin/run_vo: /usr/local/lib/libopencv_features2d.so.3.4.0
../bin/run_vo: /usr/local/lib/libopencv_flann.so.3.4.0
../bin/run_vo: /usr/local/lib/libopencv_highgui.so.3.4.0
../bin/run_vo: /usr/local/lib/libopencv_photo.so.3.4.0
../bin/run_vo: /usr/local/lib/libopencv_video.so.3.4.0
../bin/run_vo: /usr/local/lib/libopencv_videoio.so.3.4.0
../bin/run_vo: /usr/local/lib/libopencv_imgcodecs.so.3.4.0
../bin/run_vo: /usr/local/lib/libopencv_imgproc.so.3.4.0
../bin/run_vo: /usr/local/lib/libopencv_viz.so.3.4.0
../bin/run_vo: /usr/local/lib/libopencv_core.so.3.4.0
../bin/run_vo: /home/winter/Desktop/slam/slambook-master/3rdparty/Sophus/build/libSophus.so
../bin/run_vo: CMakeFiles/run_vo.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/winter/Desktop/slam/slambook-master/project/newslam/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../bin/run_vo"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/run_vo.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/run_vo.dir/build: ../bin/run_vo

.PHONY : CMakeFiles/run_vo.dir/build

CMakeFiles/run_vo.dir/requires: CMakeFiles/run_vo.dir/main.cpp.o.requires

.PHONY : CMakeFiles/run_vo.dir/requires

CMakeFiles/run_vo.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/run_vo.dir/cmake_clean.cmake
.PHONY : CMakeFiles/run_vo.dir/clean

CMakeFiles/run_vo.dir/depend:
	cd /home/winter/Desktop/slam/slambook-master/project/newslam/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/winter/Desktop/slam/slambook-master/project/newslam /home/winter/Desktop/slam/slambook-master/project/newslam /home/winter/Desktop/slam/slambook-master/project/newslam/build /home/winter/Desktop/slam/slambook-master/project/newslam/build /home/winter/Desktop/slam/slambook-master/project/newslam/build/CMakeFiles/run_vo.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/run_vo.dir/depend

