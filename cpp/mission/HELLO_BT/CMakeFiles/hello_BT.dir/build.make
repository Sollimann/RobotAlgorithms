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
CMAKE_SOURCE_DIR = /home/kristoffer/Hobby/RobotAlgorithms/cpp/mission/HELLO_BT

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/kristoffer/Hobby/RobotAlgorithms/cpp/mission/HELLO_BT

# Include any dependencies generated for this target.
include CMakeFiles/hello_BT.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/hello_BT.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/hello_BT.dir/flags.make

CMakeFiles/hello_BT.dir/hello_BT.cpp.o: CMakeFiles/hello_BT.dir/flags.make
CMakeFiles/hello_BT.dir/hello_BT.cpp.o: hello_BT.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/kristoffer/Hobby/RobotAlgorithms/cpp/mission/HELLO_BT/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/hello_BT.dir/hello_BT.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/hello_BT.dir/hello_BT.cpp.o -c /home/kristoffer/Hobby/RobotAlgorithms/cpp/mission/HELLO_BT/hello_BT.cpp

CMakeFiles/hello_BT.dir/hello_BT.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/hello_BT.dir/hello_BT.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/kristoffer/Hobby/RobotAlgorithms/cpp/mission/HELLO_BT/hello_BT.cpp > CMakeFiles/hello_BT.dir/hello_BT.cpp.i

CMakeFiles/hello_BT.dir/hello_BT.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/hello_BT.dir/hello_BT.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/kristoffer/Hobby/RobotAlgorithms/cpp/mission/HELLO_BT/hello_BT.cpp -o CMakeFiles/hello_BT.dir/hello_BT.cpp.s

CMakeFiles/hello_BT.dir/hello_BT.cpp.o.requires:

.PHONY : CMakeFiles/hello_BT.dir/hello_BT.cpp.o.requires

CMakeFiles/hello_BT.dir/hello_BT.cpp.o.provides: CMakeFiles/hello_BT.dir/hello_BT.cpp.o.requires
	$(MAKE) -f CMakeFiles/hello_BT.dir/build.make CMakeFiles/hello_BT.dir/hello_BT.cpp.o.provides.build
.PHONY : CMakeFiles/hello_BT.dir/hello_BT.cpp.o.provides

CMakeFiles/hello_BT.dir/hello_BT.cpp.o.provides.build: CMakeFiles/hello_BT.dir/hello_BT.cpp.o


# Object files for target hello_BT
hello_BT_OBJECTS = \
"CMakeFiles/hello_BT.dir/hello_BT.cpp.o"

# External object files for target hello_BT
hello_BT_EXTERNAL_OBJECTS =

hello_BT: CMakeFiles/hello_BT.dir/hello_BT.cpp.o
hello_BT: CMakeFiles/hello_BT.dir/build.make
hello_BT: /usr/local/lib/libbehaviortree_cpp_v3.so
hello_BT: /usr/lib/x86_64-linux-gnu/libboost_coroutine.so
hello_BT: /usr/lib/x86_64-linux-gnu/libboost_context.so
hello_BT: /usr/lib/x86_64-linux-gnu/libboost_system.so
hello_BT: /usr/lib/x86_64-linux-gnu/libboost_thread.so
hello_BT: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
hello_BT: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
hello_BT: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
hello_BT: /usr/lib/x86_64-linux-gnu/libpthread.so
hello_BT: /usr/lib/x86_64-linux-gnu/libzmq.so
hello_BT: CMakeFiles/hello_BT.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/kristoffer/Hobby/RobotAlgorithms/cpp/mission/HELLO_BT/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable hello_BT"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/hello_BT.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/hello_BT.dir/build: hello_BT

.PHONY : CMakeFiles/hello_BT.dir/build

CMakeFiles/hello_BT.dir/requires: CMakeFiles/hello_BT.dir/hello_BT.cpp.o.requires

.PHONY : CMakeFiles/hello_BT.dir/requires

CMakeFiles/hello_BT.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/hello_BT.dir/cmake_clean.cmake
.PHONY : CMakeFiles/hello_BT.dir/clean

CMakeFiles/hello_BT.dir/depend:
	cd /home/kristoffer/Hobby/RobotAlgorithms/cpp/mission/HELLO_BT && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/kristoffer/Hobby/RobotAlgorithms/cpp/mission/HELLO_BT /home/kristoffer/Hobby/RobotAlgorithms/cpp/mission/HELLO_BT /home/kristoffer/Hobby/RobotAlgorithms/cpp/mission/HELLO_BT /home/kristoffer/Hobby/RobotAlgorithms/cpp/mission/HELLO_BT /home/kristoffer/Hobby/RobotAlgorithms/cpp/mission/HELLO_BT/CMakeFiles/hello_BT.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/hello_BT.dir/depend

