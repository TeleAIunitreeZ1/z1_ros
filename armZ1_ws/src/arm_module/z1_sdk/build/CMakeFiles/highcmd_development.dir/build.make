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
CMAKE_SOURCE_DIR = /home/jianghao/armZ1_ws/src/arm_module/z1_sdk

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jianghao/armZ1_ws/src/arm_module/z1_sdk/build

# Include any dependencies generated for this target.
include CMakeFiles/highcmd_development.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/highcmd_development.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/highcmd_development.dir/flags.make

CMakeFiles/highcmd_development.dir/examples/highcmd_development.cpp.o: CMakeFiles/highcmd_development.dir/flags.make
CMakeFiles/highcmd_development.dir/examples/highcmd_development.cpp.o: ../examples/highcmd_development.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jianghao/armZ1_ws/src/arm_module/z1_sdk/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/highcmd_development.dir/examples/highcmd_development.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/highcmd_development.dir/examples/highcmd_development.cpp.o -c /home/jianghao/armZ1_ws/src/arm_module/z1_sdk/examples/highcmd_development.cpp

CMakeFiles/highcmd_development.dir/examples/highcmd_development.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/highcmd_development.dir/examples/highcmd_development.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jianghao/armZ1_ws/src/arm_module/z1_sdk/examples/highcmd_development.cpp > CMakeFiles/highcmd_development.dir/examples/highcmd_development.cpp.i

CMakeFiles/highcmd_development.dir/examples/highcmd_development.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/highcmd_development.dir/examples/highcmd_development.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jianghao/armZ1_ws/src/arm_module/z1_sdk/examples/highcmd_development.cpp -o CMakeFiles/highcmd_development.dir/examples/highcmd_development.cpp.s

# Object files for target highcmd_development
highcmd_development_OBJECTS = \
"CMakeFiles/highcmd_development.dir/examples/highcmd_development.cpp.o"

# External object files for target highcmd_development
highcmd_development_EXTERNAL_OBJECTS =

highcmd_development: CMakeFiles/highcmd_development.dir/examples/highcmd_development.cpp.o
highcmd_development: CMakeFiles/highcmd_development.dir/build.make
highcmd_development: CMakeFiles/highcmd_development.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/jianghao/armZ1_ws/src/arm_module/z1_sdk/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable highcmd_development"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/highcmd_development.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/highcmd_development.dir/build: highcmd_development

.PHONY : CMakeFiles/highcmd_development.dir/build

CMakeFiles/highcmd_development.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/highcmd_development.dir/cmake_clean.cmake
.PHONY : CMakeFiles/highcmd_development.dir/clean

CMakeFiles/highcmd_development.dir/depend:
	cd /home/jianghao/armZ1_ws/src/arm_module/z1_sdk/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jianghao/armZ1_ws/src/arm_module/z1_sdk /home/jianghao/armZ1_ws/src/arm_module/z1_sdk /home/jianghao/armZ1_ws/src/arm_module/z1_sdk/build /home/jianghao/armZ1_ws/src/arm_module/z1_sdk/build /home/jianghao/armZ1_ws/src/arm_module/z1_sdk/build/CMakeFiles/highcmd_development.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/highcmd_development.dir/depend

