# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/ccmake

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/juiz/team-nust-robocup-spl/Resources/Simulator

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/juiz/team-nust-robocup-spl/Resources/Simulator/build

# Include any dependencies generated for this target.
include CMakeFiles/VREP-Naoqi-Sim.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/VREP-Naoqi-Sim.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/VREP-Naoqi-Sim.dir/flags.make

CMakeFiles/VREP-Naoqi-Sim.dir/main.cpp.o: CMakeFiles/VREP-Naoqi-Sim.dir/flags.make
CMakeFiles/VREP-Naoqi-Sim.dir/main.cpp.o: ../main.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/juiz/team-nust-robocup-spl/Resources/Simulator/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/VREP-Naoqi-Sim.dir/main.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/VREP-Naoqi-Sim.dir/main.cpp.o -c /home/juiz/team-nust-robocup-spl/Resources/Simulator/main.cpp

CMakeFiles/VREP-Naoqi-Sim.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/VREP-Naoqi-Sim.dir/main.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/juiz/team-nust-robocup-spl/Resources/Simulator/main.cpp > CMakeFiles/VREP-Naoqi-Sim.dir/main.cpp.i

CMakeFiles/VREP-Naoqi-Sim.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/VREP-Naoqi-Sim.dir/main.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/juiz/team-nust-robocup-spl/Resources/Simulator/main.cpp -o CMakeFiles/VREP-Naoqi-Sim.dir/main.cpp.s

CMakeFiles/VREP-Naoqi-Sim.dir/main.cpp.o.requires:
.PHONY : CMakeFiles/VREP-Naoqi-Sim.dir/main.cpp.o.requires

CMakeFiles/VREP-Naoqi-Sim.dir/main.cpp.o.provides: CMakeFiles/VREP-Naoqi-Sim.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/VREP-Naoqi-Sim.dir/build.make CMakeFiles/VREP-Naoqi-Sim.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/VREP-Naoqi-Sim.dir/main.cpp.o.provides

CMakeFiles/VREP-Naoqi-Sim.dir/main.cpp.o.provides.build: CMakeFiles/VREP-Naoqi-Sim.dir/main.cpp.o

CMakeFiles/VREP-Naoqi-Sim.dir/VREPInterface.cpp.o: CMakeFiles/VREP-Naoqi-Sim.dir/flags.make
CMakeFiles/VREP-Naoqi-Sim.dir/VREPInterface.cpp.o: ../VREPInterface.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/juiz/team-nust-robocup-spl/Resources/Simulator/build/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/VREP-Naoqi-Sim.dir/VREPInterface.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/VREP-Naoqi-Sim.dir/VREPInterface.cpp.o -c /home/juiz/team-nust-robocup-spl/Resources/Simulator/VREPInterface.cpp

CMakeFiles/VREP-Naoqi-Sim.dir/VREPInterface.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/VREP-Naoqi-Sim.dir/VREPInterface.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/juiz/team-nust-robocup-spl/Resources/Simulator/VREPInterface.cpp > CMakeFiles/VREP-Naoqi-Sim.dir/VREPInterface.cpp.i

CMakeFiles/VREP-Naoqi-Sim.dir/VREPInterface.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/VREP-Naoqi-Sim.dir/VREPInterface.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/juiz/team-nust-robocup-spl/Resources/Simulator/VREPInterface.cpp -o CMakeFiles/VREP-Naoqi-Sim.dir/VREPInterface.cpp.s

CMakeFiles/VREP-Naoqi-Sim.dir/VREPInterface.cpp.o.requires:
.PHONY : CMakeFiles/VREP-Naoqi-Sim.dir/VREPInterface.cpp.o.requires

CMakeFiles/VREP-Naoqi-Sim.dir/VREPInterface.cpp.o.provides: CMakeFiles/VREP-Naoqi-Sim.dir/VREPInterface.cpp.o.requires
	$(MAKE) -f CMakeFiles/VREP-Naoqi-Sim.dir/build.make CMakeFiles/VREP-Naoqi-Sim.dir/VREPInterface.cpp.o.provides.build
.PHONY : CMakeFiles/VREP-Naoqi-Sim.dir/VREPInterface.cpp.o.provides

CMakeFiles/VREP-Naoqi-Sim.dir/VREPInterface.cpp.o.provides.build: CMakeFiles/VREP-Naoqi-Sim.dir/VREPInterface.cpp.o

CMakeFiles/VREP-Naoqi-Sim.dir/extApi.c.o: CMakeFiles/VREP-Naoqi-Sim.dir/flags.make
CMakeFiles/VREP-Naoqi-Sim.dir/extApi.c.o: ../extApi.c
	$(CMAKE_COMMAND) -E cmake_progress_report /home/juiz/team-nust-robocup-spl/Resources/Simulator/build/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building C object CMakeFiles/VREP-Naoqi-Sim.dir/extApi.c.o"
	/usr/bin/cc  $(C_DEFINES) $(C_FLAGS) -o CMakeFiles/VREP-Naoqi-Sim.dir/extApi.c.o   -c /home/juiz/team-nust-robocup-spl/Resources/Simulator/extApi.c

CMakeFiles/VREP-Naoqi-Sim.dir/extApi.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/VREP-Naoqi-Sim.dir/extApi.c.i"
	/usr/bin/cc  $(C_DEFINES) $(C_FLAGS) -E /home/juiz/team-nust-robocup-spl/Resources/Simulator/extApi.c > CMakeFiles/VREP-Naoqi-Sim.dir/extApi.c.i

CMakeFiles/VREP-Naoqi-Sim.dir/extApi.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/VREP-Naoqi-Sim.dir/extApi.c.s"
	/usr/bin/cc  $(C_DEFINES) $(C_FLAGS) -S /home/juiz/team-nust-robocup-spl/Resources/Simulator/extApi.c -o CMakeFiles/VREP-Naoqi-Sim.dir/extApi.c.s

CMakeFiles/VREP-Naoqi-Sim.dir/extApi.c.o.requires:
.PHONY : CMakeFiles/VREP-Naoqi-Sim.dir/extApi.c.o.requires

CMakeFiles/VREP-Naoqi-Sim.dir/extApi.c.o.provides: CMakeFiles/VREP-Naoqi-Sim.dir/extApi.c.o.requires
	$(MAKE) -f CMakeFiles/VREP-Naoqi-Sim.dir/build.make CMakeFiles/VREP-Naoqi-Sim.dir/extApi.c.o.provides.build
.PHONY : CMakeFiles/VREP-Naoqi-Sim.dir/extApi.c.o.provides

CMakeFiles/VREP-Naoqi-Sim.dir/extApi.c.o.provides.build: CMakeFiles/VREP-Naoqi-Sim.dir/extApi.c.o

CMakeFiles/VREP-Naoqi-Sim.dir/extApiPlatform.c.o: CMakeFiles/VREP-Naoqi-Sim.dir/flags.make
CMakeFiles/VREP-Naoqi-Sim.dir/extApiPlatform.c.o: ../extApiPlatform.c
	$(CMAKE_COMMAND) -E cmake_progress_report /home/juiz/team-nust-robocup-spl/Resources/Simulator/build/CMakeFiles $(CMAKE_PROGRESS_4)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building C object CMakeFiles/VREP-Naoqi-Sim.dir/extApiPlatform.c.o"
	/usr/bin/cc  $(C_DEFINES) $(C_FLAGS) -o CMakeFiles/VREP-Naoqi-Sim.dir/extApiPlatform.c.o   -c /home/juiz/team-nust-robocup-spl/Resources/Simulator/extApiPlatform.c

CMakeFiles/VREP-Naoqi-Sim.dir/extApiPlatform.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/VREP-Naoqi-Sim.dir/extApiPlatform.c.i"
	/usr/bin/cc  $(C_DEFINES) $(C_FLAGS) -E /home/juiz/team-nust-robocup-spl/Resources/Simulator/extApiPlatform.c > CMakeFiles/VREP-Naoqi-Sim.dir/extApiPlatform.c.i

CMakeFiles/VREP-Naoqi-Sim.dir/extApiPlatform.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/VREP-Naoqi-Sim.dir/extApiPlatform.c.s"
	/usr/bin/cc  $(C_DEFINES) $(C_FLAGS) -S /home/juiz/team-nust-robocup-spl/Resources/Simulator/extApiPlatform.c -o CMakeFiles/VREP-Naoqi-Sim.dir/extApiPlatform.c.s

CMakeFiles/VREP-Naoqi-Sim.dir/extApiPlatform.c.o.requires:
.PHONY : CMakeFiles/VREP-Naoqi-Sim.dir/extApiPlatform.c.o.requires

CMakeFiles/VREP-Naoqi-Sim.dir/extApiPlatform.c.o.provides: CMakeFiles/VREP-Naoqi-Sim.dir/extApiPlatform.c.o.requires
	$(MAKE) -f CMakeFiles/VREP-Naoqi-Sim.dir/build.make CMakeFiles/VREP-Naoqi-Sim.dir/extApiPlatform.c.o.provides.build
.PHONY : CMakeFiles/VREP-Naoqi-Sim.dir/extApiPlatform.c.o.provides

CMakeFiles/VREP-Naoqi-Sim.dir/extApiPlatform.c.o.provides.build: CMakeFiles/VREP-Naoqi-Sim.dir/extApiPlatform.c.o

# Object files for target VREP-Naoqi-Sim
VREP__Naoqi__Sim_OBJECTS = \
"CMakeFiles/VREP-Naoqi-Sim.dir/main.cpp.o" \
"CMakeFiles/VREP-Naoqi-Sim.dir/VREPInterface.cpp.o" \
"CMakeFiles/VREP-Naoqi-Sim.dir/extApi.c.o" \
"CMakeFiles/VREP-Naoqi-Sim.dir/extApiPlatform.c.o"

# External object files for target VREP-Naoqi-Sim
VREP__Naoqi__Sim_EXTERNAL_OBJECTS =

VREP-Naoqi-Sim: CMakeFiles/VREP-Naoqi-Sim.dir/main.cpp.o
VREP-Naoqi-Sim: CMakeFiles/VREP-Naoqi-Sim.dir/VREPInterface.cpp.o
VREP-Naoqi-Sim: CMakeFiles/VREP-Naoqi-Sim.dir/extApi.c.o
VREP-Naoqi-Sim: CMakeFiles/VREP-Naoqi-Sim.dir/extApiPlatform.c.o
VREP-Naoqi-Sim: CMakeFiles/VREP-Naoqi-Sim.dir/build.make
VREP-Naoqi-Sim: /home/juiz/TeamNustResources/simulator-sdk-2.1.2.17-linux64/lib/libalnaosim.so
VREP-Naoqi-Sim: /home/juiz/TeamNustResources/simulator-sdk-2.1.2.17-linux64/lib/libalsimutils.so
VREP-Naoqi-Sim: /home/juiz/TeamNustResources/simulator-sdk-2.1.2.17-linux64/lib/libalrobotmodel.so
VREP-Naoqi-Sim: /home/juiz/TeamNustResources/simulator-sdk-2.1.2.17-linux64/lib/libalproxies.so
VREP-Naoqi-Sim: /home/juiz/TeamNustResources/simulator-sdk-2.1.2.17-linux64/lib/libalmemoryfastaccess.so
VREP-Naoqi-Sim: /home/juiz/TeamNustResources/simulator-sdk-2.1.2.17-linux64/lib/libalcommon.so
VREP-Naoqi-Sim: /home/juiz/TeamNustResources/simulator-sdk-2.1.2.17-linux64/lib/libqi.so
VREP-Naoqi-Sim: /home/juiz/TeamNustResources/simulator-sdk-2.1.2.17-linux64/lib/libqitype.so
VREP-Naoqi-Sim: /home/juiz/TeamNustResources/simulator-sdk-2.1.2.17-linux64/lib/libboost_system.so
VREP-Naoqi-Sim: CMakeFiles/VREP-Naoqi-Sim.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable VREP-Naoqi-Sim"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/VREP-Naoqi-Sim.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/VREP-Naoqi-Sim.dir/build: VREP-Naoqi-Sim
.PHONY : CMakeFiles/VREP-Naoqi-Sim.dir/build

CMakeFiles/VREP-Naoqi-Sim.dir/requires: CMakeFiles/VREP-Naoqi-Sim.dir/main.cpp.o.requires
CMakeFiles/VREP-Naoqi-Sim.dir/requires: CMakeFiles/VREP-Naoqi-Sim.dir/VREPInterface.cpp.o.requires
CMakeFiles/VREP-Naoqi-Sim.dir/requires: CMakeFiles/VREP-Naoqi-Sim.dir/extApi.c.o.requires
CMakeFiles/VREP-Naoqi-Sim.dir/requires: CMakeFiles/VREP-Naoqi-Sim.dir/extApiPlatform.c.o.requires
.PHONY : CMakeFiles/VREP-Naoqi-Sim.dir/requires

CMakeFiles/VREP-Naoqi-Sim.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/VREP-Naoqi-Sim.dir/cmake_clean.cmake
.PHONY : CMakeFiles/VREP-Naoqi-Sim.dir/clean

CMakeFiles/VREP-Naoqi-Sim.dir/depend:
	cd /home/juiz/team-nust-robocup-spl/Resources/Simulator/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/juiz/team-nust-robocup-spl/Resources/Simulator /home/juiz/team-nust-robocup-spl/Resources/Simulator /home/juiz/team-nust-robocup-spl/Resources/Simulator/build /home/juiz/team-nust-robocup-spl/Resources/Simulator/build /home/juiz/team-nust-robocup-spl/Resources/Simulator/build/CMakeFiles/VREP-Naoqi-Sim.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/VREP-Naoqi-Sim.dir/depend

