# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_SOURCE_DIR = /home/lin/Projects/vio_ws/src/lvio

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/lin/Projects/vio_ws/src/lvio/build

# Include any dependencies generated for this target.
include CMakeFiles/test_frame.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/test_frame.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/test_frame.dir/flags.make

CMakeFiles/test_frame.dir/test/test_frame.cpp.o: CMakeFiles/test_frame.dir/flags.make
CMakeFiles/test_frame.dir/test/test_frame.cpp.o: ../test/test_frame.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lin/Projects/vio_ws/src/lvio/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/test_frame.dir/test/test_frame.cpp.o"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test_frame.dir/test/test_frame.cpp.o -c /home/lin/Projects/vio_ws/src/lvio/test/test_frame.cpp

CMakeFiles/test_frame.dir/test/test_frame.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_frame.dir/test/test_frame.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lin/Projects/vio_ws/src/lvio/test/test_frame.cpp > CMakeFiles/test_frame.dir/test/test_frame.cpp.i

CMakeFiles/test_frame.dir/test/test_frame.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_frame.dir/test/test_frame.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lin/Projects/vio_ws/src/lvio/test/test_frame.cpp -o CMakeFiles/test_frame.dir/test/test_frame.cpp.s

# Object files for target test_frame
test_frame_OBJECTS = \
"CMakeFiles/test_frame.dir/test/test_frame.cpp.o"

# External object files for target test_frame
test_frame_EXTERNAL_OBJECTS =

../bin/test_frame: CMakeFiles/test_frame.dir/test/test_frame.cpp.o
../bin/test_frame: CMakeFiles/test_frame.dir/build.make
../bin/test_frame: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.4.2.0
../bin/test_frame: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.4.2.0
../bin/test_frame: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.4.2.0
../bin/test_frame: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.4.2.0
../bin/test_frame: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.4.2.0
../bin/test_frame: /usr/lib/x86_64-linux-gnu/libopencv_dnn_objdetect.so.4.2.0
../bin/test_frame: /usr/lib/x86_64-linux-gnu/libopencv_dnn_superres.so.4.2.0
../bin/test_frame: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.4.2.0
../bin/test_frame: /usr/lib/x86_64-linux-gnu/libopencv_face.so.4.2.0
../bin/test_frame: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.4.2.0
../bin/test_frame: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.4.2.0
../bin/test_frame: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.4.2.0
../bin/test_frame: /usr/lib/x86_64-linux-gnu/libopencv_hfs.so.4.2.0
../bin/test_frame: /usr/lib/x86_64-linux-gnu/libopencv_img_hash.so.4.2.0
../bin/test_frame: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.4.2.0
../bin/test_frame: /usr/lib/x86_64-linux-gnu/libopencv_quality.so.4.2.0
../bin/test_frame: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.4.2.0
../bin/test_frame: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.4.2.0
../bin/test_frame: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.4.2.0
../bin/test_frame: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.4.2.0
../bin/test_frame: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.4.2.0
../bin/test_frame: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.4.2.0
../bin/test_frame: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.4.2.0
../bin/test_frame: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.4.2.0
../bin/test_frame: /usr/lib/x86_64-linux-gnu/libopencv_tracking.so.4.2.0
../bin/test_frame: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.4.2.0
../bin/test_frame: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.4.2.0
../bin/test_frame: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.4.2.0
../bin/test_frame: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.4.2.0
../bin/test_frame: ../lib/liblvio.so
../bin/test_frame: ../thirdparty/DBoW2/lib/libDBoW2.so
../bin/test_frame: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.4.2.0
../bin/test_frame: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.4.2.0
../bin/test_frame: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.4.2.0
../bin/test_frame: /usr/lib/x86_64-linux-gnu/libopencv_text.so.4.2.0
../bin/test_frame: /usr/lib/x86_64-linux-gnu/libopencv_dnn.so.4.2.0
../bin/test_frame: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.4.2.0
../bin/test_frame: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.4.2.0
../bin/test_frame: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.4.2.0
../bin/test_frame: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.4.2.0
../bin/test_frame: /usr/lib/x86_64-linux-gnu/libopencv_video.so.4.2.0
../bin/test_frame: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.4.2.0
../bin/test_frame: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.4.2.0
../bin/test_frame: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.4.2.0
../bin/test_frame: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.4.2.0
../bin/test_frame: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.4.2.0
../bin/test_frame: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.4.2.0
../bin/test_frame: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.4.2.0
../bin/test_frame: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.4.2.0
../bin/test_frame: /usr/lib/x86_64-linux-gnu/libopencv_core.so.4.2.0
../bin/test_frame: CMakeFiles/test_frame.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/lin/Projects/vio_ws/src/lvio/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../bin/test_frame"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test_frame.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/test_frame.dir/build: ../bin/test_frame

.PHONY : CMakeFiles/test_frame.dir/build

CMakeFiles/test_frame.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/test_frame.dir/cmake_clean.cmake
.PHONY : CMakeFiles/test_frame.dir/clean

CMakeFiles/test_frame.dir/depend:
	cd /home/lin/Projects/vio_ws/src/lvio/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/lin/Projects/vio_ws/src/lvio /home/lin/Projects/vio_ws/src/lvio /home/lin/Projects/vio_ws/src/lvio/build /home/lin/Projects/vio_ws/src/lvio/build /home/lin/Projects/vio_ws/src/lvio/build/CMakeFiles/test_frame.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/test_frame.dir/depend

