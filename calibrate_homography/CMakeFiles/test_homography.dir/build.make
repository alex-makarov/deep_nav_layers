# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/alex/src/catkin_ws/src/deep_nav_layers/calibrate_homography

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/alex/src/catkin_ws/src/deep_nav_layers/calibrate_homography

# Include any dependencies generated for this target.
include CMakeFiles/test_homography.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/test_homography.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/test_homography.dir/flags.make

CMakeFiles/test_homography.dir/test_homography.cpp.o: CMakeFiles/test_homography.dir/flags.make
CMakeFiles/test_homography.dir/test_homography.cpp.o: test_homography.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/alex/src/catkin_ws/src/deep_nav_layers/calibrate_homography/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/test_homography.dir/test_homography.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test_homography.dir/test_homography.cpp.o -c /home/alex/src/catkin_ws/src/deep_nav_layers/calibrate_homography/test_homography.cpp

CMakeFiles/test_homography.dir/test_homography.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_homography.dir/test_homography.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/alex/src/catkin_ws/src/deep_nav_layers/calibrate_homography/test_homography.cpp > CMakeFiles/test_homography.dir/test_homography.cpp.i

CMakeFiles/test_homography.dir/test_homography.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_homography.dir/test_homography.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/alex/src/catkin_ws/src/deep_nav_layers/calibrate_homography/test_homography.cpp -o CMakeFiles/test_homography.dir/test_homography.cpp.s

CMakeFiles/test_homography.dir/test_homography.cpp.o.requires:

.PHONY : CMakeFiles/test_homography.dir/test_homography.cpp.o.requires

CMakeFiles/test_homography.dir/test_homography.cpp.o.provides: CMakeFiles/test_homography.dir/test_homography.cpp.o.requires
	$(MAKE) -f CMakeFiles/test_homography.dir/build.make CMakeFiles/test_homography.dir/test_homography.cpp.o.provides.build
.PHONY : CMakeFiles/test_homography.dir/test_homography.cpp.o.provides

CMakeFiles/test_homography.dir/test_homography.cpp.o.provides.build: CMakeFiles/test_homography.dir/test_homography.cpp.o


# Object files for target test_homography
test_homography_OBJECTS = \
"CMakeFiles/test_homography.dir/test_homography.cpp.o"

# External object files for target test_homography
test_homography_EXTERNAL_OBJECTS =

test_homography: CMakeFiles/test_homography.dir/test_homography.cpp.o
test_homography: CMakeFiles/test_homography.dir/build.make
test_homography: /usr/lib/libopencv_dnn.so.3.3.1
test_homography: /usr/lib/libopencv_ml.so.3.3.1
test_homography: /usr/lib/libopencv_objdetect.so.3.3.1
test_homography: /usr/lib/libopencv_shape.so.3.3.1
test_homography: /usr/lib/libopencv_stitching.so.3.3.1
test_homography: /usr/lib/libopencv_superres.so.3.3.1
test_homography: /usr/lib/libopencv_videostab.so.3.3.1
test_homography: /usr/lib/libopencv_calib3d.so.3.3.1
test_homography: /usr/lib/libopencv_features2d.so.3.3.1
test_homography: /usr/lib/libopencv_flann.so.3.3.1
test_homography: /usr/lib/libopencv_highgui.so.3.3.1
test_homography: /usr/lib/libopencv_photo.so.3.3.1
test_homography: /usr/lib/libopencv_video.so.3.3.1
test_homography: /usr/lib/libopencv_videoio.so.3.3.1
test_homography: /usr/lib/libopencv_imgcodecs.so.3.3.1
test_homography: /usr/lib/libopencv_imgproc.so.3.3.1
test_homography: /usr/lib/libopencv_core.so.3.3.1
test_homography: CMakeFiles/test_homography.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/alex/src/catkin_ws/src/deep_nav_layers/calibrate_homography/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable test_homography"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test_homography.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/test_homography.dir/build: test_homography

.PHONY : CMakeFiles/test_homography.dir/build

CMakeFiles/test_homography.dir/requires: CMakeFiles/test_homography.dir/test_homography.cpp.o.requires

.PHONY : CMakeFiles/test_homography.dir/requires

CMakeFiles/test_homography.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/test_homography.dir/cmake_clean.cmake
.PHONY : CMakeFiles/test_homography.dir/clean

CMakeFiles/test_homography.dir/depend:
	cd /home/alex/src/catkin_ws/src/deep_nav_layers/calibrate_homography && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/alex/src/catkin_ws/src/deep_nav_layers/calibrate_homography /home/alex/src/catkin_ws/src/deep_nav_layers/calibrate_homography /home/alex/src/catkin_ws/src/deep_nav_layers/calibrate_homography /home/alex/src/catkin_ws/src/deep_nav_layers/calibrate_homography /home/alex/src/catkin_ws/src/deep_nav_layers/calibrate_homography/CMakeFiles/test_homography.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/test_homography.dir/depend

