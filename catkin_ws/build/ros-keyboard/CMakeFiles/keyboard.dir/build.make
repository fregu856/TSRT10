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
CMAKE_SOURCE_DIR = /home/fregu856/TSRT10/catkin_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/fregu856/TSRT10/catkin_ws/build

# Include any dependencies generated for this target.
include ros-keyboard/CMakeFiles/keyboard.dir/depend.make

# Include the progress variables for this target.
include ros-keyboard/CMakeFiles/keyboard.dir/progress.make

# Include the compile flags for this target's objects.
include ros-keyboard/CMakeFiles/keyboard.dir/flags.make

ros-keyboard/CMakeFiles/keyboard.dir/src/main.cpp.o: ros-keyboard/CMakeFiles/keyboard.dir/flags.make
ros-keyboard/CMakeFiles/keyboard.dir/src/main.cpp.o: /home/fregu856/TSRT10/catkin_ws/src/ros-keyboard/src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/fregu856/TSRT10/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object ros-keyboard/CMakeFiles/keyboard.dir/src/main.cpp.o"
	cd /home/fregu856/TSRT10/catkin_ws/build/ros-keyboard && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/keyboard.dir/src/main.cpp.o -c /home/fregu856/TSRT10/catkin_ws/src/ros-keyboard/src/main.cpp

ros-keyboard/CMakeFiles/keyboard.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/keyboard.dir/src/main.cpp.i"
	cd /home/fregu856/TSRT10/catkin_ws/build/ros-keyboard && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/fregu856/TSRT10/catkin_ws/src/ros-keyboard/src/main.cpp > CMakeFiles/keyboard.dir/src/main.cpp.i

ros-keyboard/CMakeFiles/keyboard.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/keyboard.dir/src/main.cpp.s"
	cd /home/fregu856/TSRT10/catkin_ws/build/ros-keyboard && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/fregu856/TSRT10/catkin_ws/src/ros-keyboard/src/main.cpp -o CMakeFiles/keyboard.dir/src/main.cpp.s

ros-keyboard/CMakeFiles/keyboard.dir/src/main.cpp.o.requires:

.PHONY : ros-keyboard/CMakeFiles/keyboard.dir/src/main.cpp.o.requires

ros-keyboard/CMakeFiles/keyboard.dir/src/main.cpp.o.provides: ros-keyboard/CMakeFiles/keyboard.dir/src/main.cpp.o.requires
	$(MAKE) -f ros-keyboard/CMakeFiles/keyboard.dir/build.make ros-keyboard/CMakeFiles/keyboard.dir/src/main.cpp.o.provides.build
.PHONY : ros-keyboard/CMakeFiles/keyboard.dir/src/main.cpp.o.provides

ros-keyboard/CMakeFiles/keyboard.dir/src/main.cpp.o.provides.build: ros-keyboard/CMakeFiles/keyboard.dir/src/main.cpp.o


ros-keyboard/CMakeFiles/keyboard.dir/src/keyboard.cpp.o: ros-keyboard/CMakeFiles/keyboard.dir/flags.make
ros-keyboard/CMakeFiles/keyboard.dir/src/keyboard.cpp.o: /home/fregu856/TSRT10/catkin_ws/src/ros-keyboard/src/keyboard.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/fregu856/TSRT10/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object ros-keyboard/CMakeFiles/keyboard.dir/src/keyboard.cpp.o"
	cd /home/fregu856/TSRT10/catkin_ws/build/ros-keyboard && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/keyboard.dir/src/keyboard.cpp.o -c /home/fregu856/TSRT10/catkin_ws/src/ros-keyboard/src/keyboard.cpp

ros-keyboard/CMakeFiles/keyboard.dir/src/keyboard.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/keyboard.dir/src/keyboard.cpp.i"
	cd /home/fregu856/TSRT10/catkin_ws/build/ros-keyboard && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/fregu856/TSRT10/catkin_ws/src/ros-keyboard/src/keyboard.cpp > CMakeFiles/keyboard.dir/src/keyboard.cpp.i

ros-keyboard/CMakeFiles/keyboard.dir/src/keyboard.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/keyboard.dir/src/keyboard.cpp.s"
	cd /home/fregu856/TSRT10/catkin_ws/build/ros-keyboard && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/fregu856/TSRT10/catkin_ws/src/ros-keyboard/src/keyboard.cpp -o CMakeFiles/keyboard.dir/src/keyboard.cpp.s

ros-keyboard/CMakeFiles/keyboard.dir/src/keyboard.cpp.o.requires:

.PHONY : ros-keyboard/CMakeFiles/keyboard.dir/src/keyboard.cpp.o.requires

ros-keyboard/CMakeFiles/keyboard.dir/src/keyboard.cpp.o.provides: ros-keyboard/CMakeFiles/keyboard.dir/src/keyboard.cpp.o.requires
	$(MAKE) -f ros-keyboard/CMakeFiles/keyboard.dir/build.make ros-keyboard/CMakeFiles/keyboard.dir/src/keyboard.cpp.o.provides.build
.PHONY : ros-keyboard/CMakeFiles/keyboard.dir/src/keyboard.cpp.o.provides

ros-keyboard/CMakeFiles/keyboard.dir/src/keyboard.cpp.o.provides.build: ros-keyboard/CMakeFiles/keyboard.dir/src/keyboard.cpp.o


# Object files for target keyboard
keyboard_OBJECTS = \
"CMakeFiles/keyboard.dir/src/main.cpp.o" \
"CMakeFiles/keyboard.dir/src/keyboard.cpp.o"

# External object files for target keyboard
keyboard_EXTERNAL_OBJECTS =

/home/fregu856/TSRT10/catkin_ws/devel/lib/keyboard/keyboard: ros-keyboard/CMakeFiles/keyboard.dir/src/main.cpp.o
/home/fregu856/TSRT10/catkin_ws/devel/lib/keyboard/keyboard: ros-keyboard/CMakeFiles/keyboard.dir/src/keyboard.cpp.o
/home/fregu856/TSRT10/catkin_ws/devel/lib/keyboard/keyboard: ros-keyboard/CMakeFiles/keyboard.dir/build.make
/home/fregu856/TSRT10/catkin_ws/devel/lib/keyboard/keyboard: /usr/lib/x86_64-linux-gnu/libSDLmain.a
/home/fregu856/TSRT10/catkin_ws/devel/lib/keyboard/keyboard: /usr/lib/x86_64-linux-gnu/libSDL.so
/home/fregu856/TSRT10/catkin_ws/devel/lib/keyboard/keyboard: /opt/ros/kinetic/lib/libroscpp.so
/home/fregu856/TSRT10/catkin_ws/devel/lib/keyboard/keyboard: /usr/lib/x86_64-linux-gnu/libboost_signals.so
/home/fregu856/TSRT10/catkin_ws/devel/lib/keyboard/keyboard: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/fregu856/TSRT10/catkin_ws/devel/lib/keyboard/keyboard: /opt/ros/kinetic/lib/librosconsole.so
/home/fregu856/TSRT10/catkin_ws/devel/lib/keyboard/keyboard: /opt/ros/kinetic/lib/librosconsole_log4cxx.so
/home/fregu856/TSRT10/catkin_ws/devel/lib/keyboard/keyboard: /opt/ros/kinetic/lib/librosconsole_backend_interface.so
/home/fregu856/TSRT10/catkin_ws/devel/lib/keyboard/keyboard: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/fregu856/TSRT10/catkin_ws/devel/lib/keyboard/keyboard: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/fregu856/TSRT10/catkin_ws/devel/lib/keyboard/keyboard: /opt/ros/kinetic/lib/libxmlrpcpp.so
/home/fregu856/TSRT10/catkin_ws/devel/lib/keyboard/keyboard: /opt/ros/kinetic/lib/libroscpp_serialization.so
/home/fregu856/TSRT10/catkin_ws/devel/lib/keyboard/keyboard: /opt/ros/kinetic/lib/librostime.so
/home/fregu856/TSRT10/catkin_ws/devel/lib/keyboard/keyboard: /opt/ros/kinetic/lib/libcpp_common.so
/home/fregu856/TSRT10/catkin_ws/devel/lib/keyboard/keyboard: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/fregu856/TSRT10/catkin_ws/devel/lib/keyboard/keyboard: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/fregu856/TSRT10/catkin_ws/devel/lib/keyboard/keyboard: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/fregu856/TSRT10/catkin_ws/devel/lib/keyboard/keyboard: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/fregu856/TSRT10/catkin_ws/devel/lib/keyboard/keyboard: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/fregu856/TSRT10/catkin_ws/devel/lib/keyboard/keyboard: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/fregu856/TSRT10/catkin_ws/devel/lib/keyboard/keyboard: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
/home/fregu856/TSRT10/catkin_ws/devel/lib/keyboard/keyboard: ros-keyboard/CMakeFiles/keyboard.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/fregu856/TSRT10/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable /home/fregu856/TSRT10/catkin_ws/devel/lib/keyboard/keyboard"
	cd /home/fregu856/TSRT10/catkin_ws/build/ros-keyboard && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/keyboard.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
ros-keyboard/CMakeFiles/keyboard.dir/build: /home/fregu856/TSRT10/catkin_ws/devel/lib/keyboard/keyboard

.PHONY : ros-keyboard/CMakeFiles/keyboard.dir/build

ros-keyboard/CMakeFiles/keyboard.dir/requires: ros-keyboard/CMakeFiles/keyboard.dir/src/main.cpp.o.requires
ros-keyboard/CMakeFiles/keyboard.dir/requires: ros-keyboard/CMakeFiles/keyboard.dir/src/keyboard.cpp.o.requires

.PHONY : ros-keyboard/CMakeFiles/keyboard.dir/requires

ros-keyboard/CMakeFiles/keyboard.dir/clean:
	cd /home/fregu856/TSRT10/catkin_ws/build/ros-keyboard && $(CMAKE_COMMAND) -P CMakeFiles/keyboard.dir/cmake_clean.cmake
.PHONY : ros-keyboard/CMakeFiles/keyboard.dir/clean

ros-keyboard/CMakeFiles/keyboard.dir/depend:
	cd /home/fregu856/TSRT10/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/fregu856/TSRT10/catkin_ws/src /home/fregu856/TSRT10/catkin_ws/src/ros-keyboard /home/fregu856/TSRT10/catkin_ws/build /home/fregu856/TSRT10/catkin_ws/build/ros-keyboard /home/fregu856/TSRT10/catkin_ws/build/ros-keyboard/CMakeFiles/keyboard.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : ros-keyboard/CMakeFiles/keyboard.dir/depend

