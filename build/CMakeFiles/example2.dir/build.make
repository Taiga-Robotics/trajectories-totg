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
CMAKE_SOURCE_DIR = /home/iris/devel/trajectories-totg

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/iris/devel/trajectories-totg/build

# Include any dependencies generated for this target.
include CMakeFiles/example2.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/example2.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/example2.dir/flags.make

CMakeFiles/example2.dir/Example2.cpp.o: CMakeFiles/example2.dir/flags.make
CMakeFiles/example2.dir/Example2.cpp.o: ../Example2.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/iris/devel/trajectories-totg/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/example2.dir/Example2.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/example2.dir/Example2.cpp.o -c /home/iris/devel/trajectories-totg/Example2.cpp

CMakeFiles/example2.dir/Example2.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/example2.dir/Example2.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/iris/devel/trajectories-totg/Example2.cpp > CMakeFiles/example2.dir/Example2.cpp.i

CMakeFiles/example2.dir/Example2.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/example2.dir/Example2.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/iris/devel/trajectories-totg/Example2.cpp -o CMakeFiles/example2.dir/Example2.cpp.s

CMakeFiles/example2.dir/Path.cpp.o: CMakeFiles/example2.dir/flags.make
CMakeFiles/example2.dir/Path.cpp.o: ../Path.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/iris/devel/trajectories-totg/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/example2.dir/Path.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/example2.dir/Path.cpp.o -c /home/iris/devel/trajectories-totg/Path.cpp

CMakeFiles/example2.dir/Path.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/example2.dir/Path.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/iris/devel/trajectories-totg/Path.cpp > CMakeFiles/example2.dir/Path.cpp.i

CMakeFiles/example2.dir/Path.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/example2.dir/Path.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/iris/devel/trajectories-totg/Path.cpp -o CMakeFiles/example2.dir/Path.cpp.s

CMakeFiles/example2.dir/Trajectory.cpp.o: CMakeFiles/example2.dir/flags.make
CMakeFiles/example2.dir/Trajectory.cpp.o: ../Trajectory.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/iris/devel/trajectories-totg/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/example2.dir/Trajectory.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/example2.dir/Trajectory.cpp.o -c /home/iris/devel/trajectories-totg/Trajectory.cpp

CMakeFiles/example2.dir/Trajectory.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/example2.dir/Trajectory.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/iris/devel/trajectories-totg/Trajectory.cpp > CMakeFiles/example2.dir/Trajectory.cpp.i

CMakeFiles/example2.dir/Trajectory.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/example2.dir/Trajectory.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/iris/devel/trajectories-totg/Trajectory.cpp -o CMakeFiles/example2.dir/Trajectory.cpp.s

# Object files for target example2
example2_OBJECTS = \
"CMakeFiles/example2.dir/Example2.cpp.o" \
"CMakeFiles/example2.dir/Path.cpp.o" \
"CMakeFiles/example2.dir/Trajectory.cpp.o"

# External object files for target example2
example2_EXTERNAL_OBJECTS =

example2: CMakeFiles/example2.dir/Example2.cpp.o
example2: CMakeFiles/example2.dir/Path.cpp.o
example2: CMakeFiles/example2.dir/Trajectory.cpp.o
example2: CMakeFiles/example2.dir/build.make
example2: CMakeFiles/example2.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/iris/devel/trajectories-totg/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable example2"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/example2.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/example2.dir/build: example2

.PHONY : CMakeFiles/example2.dir/build

CMakeFiles/example2.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/example2.dir/cmake_clean.cmake
.PHONY : CMakeFiles/example2.dir/clean

CMakeFiles/example2.dir/depend:
	cd /home/iris/devel/trajectories-totg/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/iris/devel/trajectories-totg /home/iris/devel/trajectories-totg /home/iris/devel/trajectories-totg/build /home/iris/devel/trajectories-totg/build /home/iris/devel/trajectories-totg/build/CMakeFiles/example2.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/example2.dir/depend

