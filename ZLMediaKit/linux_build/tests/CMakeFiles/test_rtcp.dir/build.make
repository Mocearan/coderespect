# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.20

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

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/coderespect/ZLMediaKit

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/coderespect/ZLMediaKit/build

# Include any dependencies generated for this target.
include tests/CMakeFiles/test_rtcp.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include tests/CMakeFiles/test_rtcp.dir/compiler_depend.make

# Include the progress variables for this target.
include tests/CMakeFiles/test_rtcp.dir/progress.make

# Include the compile flags for this target's objects.
include tests/CMakeFiles/test_rtcp.dir/flags.make

tests/CMakeFiles/test_rtcp.dir/test_rtcp.cpp.o: tests/CMakeFiles/test_rtcp.dir/flags.make
tests/CMakeFiles/test_rtcp.dir/test_rtcp.cpp.o: ../tests/test_rtcp.cpp
tests/CMakeFiles/test_rtcp.dir/test_rtcp.cpp.o: tests/CMakeFiles/test_rtcp.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/coderespect/ZLMediaKit/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object tests/CMakeFiles/test_rtcp.dir/test_rtcp.cpp.o"
	cd /home/coderespect/ZLMediaKit/build/tests && /usr/local/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT tests/CMakeFiles/test_rtcp.dir/test_rtcp.cpp.o -MF CMakeFiles/test_rtcp.dir/test_rtcp.cpp.o.d -o CMakeFiles/test_rtcp.dir/test_rtcp.cpp.o -c /home/coderespect/ZLMediaKit/tests/test_rtcp.cpp

tests/CMakeFiles/test_rtcp.dir/test_rtcp.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_rtcp.dir/test_rtcp.cpp.i"
	cd /home/coderespect/ZLMediaKit/build/tests && /usr/local/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/coderespect/ZLMediaKit/tests/test_rtcp.cpp > CMakeFiles/test_rtcp.dir/test_rtcp.cpp.i

tests/CMakeFiles/test_rtcp.dir/test_rtcp.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_rtcp.dir/test_rtcp.cpp.s"
	cd /home/coderespect/ZLMediaKit/build/tests && /usr/local/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/coderespect/ZLMediaKit/tests/test_rtcp.cpp -o CMakeFiles/test_rtcp.dir/test_rtcp.cpp.s

# Object files for target test_rtcp
test_rtcp_OBJECTS = \
"CMakeFiles/test_rtcp.dir/test_rtcp.cpp.o"

# External object files for target test_rtcp
test_rtcp_EXTERNAL_OBJECTS =

../release/linux/Debug/test_rtcp: tests/CMakeFiles/test_rtcp.dir/test_rtcp.cpp.o
../release/linux/Debug/test_rtcp: tests/CMakeFiles/test_rtcp.dir/build.make
../release/linux/Debug/test_rtcp: ../release/linux/Debug/libzlmediakit.a
../release/linux/Debug/test_rtcp: ../release/linux/Debug/libzltoolkit.a
../release/linux/Debug/test_rtcp: /usr/lib64/libssl.so
../release/linux/Debug/test_rtcp: /usr/lib64/libcrypto.so
../release/linux/Debug/test_rtcp: ../release/linux/Debug/libmpeg.a
../release/linux/Debug/test_rtcp: ../release/linux/Debug/libmov.a
../release/linux/Debug/test_rtcp: ../release/linux/Debug/libflv.a
../release/linux/Debug/test_rtcp: tests/CMakeFiles/test_rtcp.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/coderespect/ZLMediaKit/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../release/linux/Debug/test_rtcp"
	cd /home/coderespect/ZLMediaKit/build/tests && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test_rtcp.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
tests/CMakeFiles/test_rtcp.dir/build: ../release/linux/Debug/test_rtcp
.PHONY : tests/CMakeFiles/test_rtcp.dir/build

tests/CMakeFiles/test_rtcp.dir/clean:
	cd /home/coderespect/ZLMediaKit/build/tests && $(CMAKE_COMMAND) -P CMakeFiles/test_rtcp.dir/cmake_clean.cmake
.PHONY : tests/CMakeFiles/test_rtcp.dir/clean

tests/CMakeFiles/test_rtcp.dir/depend:
	cd /home/coderespect/ZLMediaKit/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/coderespect/ZLMediaKit /home/coderespect/ZLMediaKit/tests /home/coderespect/ZLMediaKit/build /home/coderespect/ZLMediaKit/build/tests /home/coderespect/ZLMediaKit/build/tests/CMakeFiles/test_rtcp.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : tests/CMakeFiles/test_rtcp.dir/depend

