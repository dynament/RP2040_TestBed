# CMAKE generated file: DO NOT EDIT!
# Generated by "MinGW Makefiles" Generator, CMake Version 3.25

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

SHELL = cmd.exe

# The CMake executable.
CMAKE_COMMAND = "C:\Program Files\CMake\bin\cmake.exe"

# The command to remove a file.
RM = "C:\Program Files\CMake\bin\cmake.exe" -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = C:\VSARM\sdk\pico\pico-sdk\tools\pioasm

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = C:\Users\craig.hemingway\VisualCode_Workspace\RP2040_TestBed\build\pioasm

# Include any dependencies generated for this target.
include CMakeFiles/pioasm.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/pioasm.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/pioasm.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/pioasm.dir/flags.make

CMakeFiles/pioasm.dir/main.cpp.obj: CMakeFiles/pioasm.dir/flags.make
CMakeFiles/pioasm.dir/main.cpp.obj: CMakeFiles/pioasm.dir/includes_CXX.rsp
CMakeFiles/pioasm.dir/main.cpp.obj: C:/VSARM/sdk/pico/pico-sdk/tools/pioasm/main.cpp
CMakeFiles/pioasm.dir/main.cpp.obj: CMakeFiles/pioasm.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=C:\Users\craig.hemingway\VisualCode_Workspace\RP2040_TestBed\build\pioasm\CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/pioasm.dir/main.cpp.obj"
	C:\VSARM\mingw\mingw32\bin\c++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/pioasm.dir/main.cpp.obj -MF CMakeFiles\pioasm.dir\main.cpp.obj.d -o CMakeFiles\pioasm.dir\main.cpp.obj -c C:\VSARM\sdk\pico\pico-sdk\tools\pioasm\main.cpp

CMakeFiles/pioasm.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pioasm.dir/main.cpp.i"
	C:\VSARM\mingw\mingw32\bin\c++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E C:\VSARM\sdk\pico\pico-sdk\tools\pioasm\main.cpp > CMakeFiles\pioasm.dir\main.cpp.i

CMakeFiles/pioasm.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pioasm.dir/main.cpp.s"
	C:\VSARM\mingw\mingw32\bin\c++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S C:\VSARM\sdk\pico\pico-sdk\tools\pioasm\main.cpp -o CMakeFiles\pioasm.dir\main.cpp.s

CMakeFiles/pioasm.dir/pio_assembler.cpp.obj: CMakeFiles/pioasm.dir/flags.make
CMakeFiles/pioasm.dir/pio_assembler.cpp.obj: CMakeFiles/pioasm.dir/includes_CXX.rsp
CMakeFiles/pioasm.dir/pio_assembler.cpp.obj: C:/VSARM/sdk/pico/pico-sdk/tools/pioasm/pio_assembler.cpp
CMakeFiles/pioasm.dir/pio_assembler.cpp.obj: CMakeFiles/pioasm.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=C:\Users\craig.hemingway\VisualCode_Workspace\RP2040_TestBed\build\pioasm\CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/pioasm.dir/pio_assembler.cpp.obj"
	C:\VSARM\mingw\mingw32\bin\c++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/pioasm.dir/pio_assembler.cpp.obj -MF CMakeFiles\pioasm.dir\pio_assembler.cpp.obj.d -o CMakeFiles\pioasm.dir\pio_assembler.cpp.obj -c C:\VSARM\sdk\pico\pico-sdk\tools\pioasm\pio_assembler.cpp

CMakeFiles/pioasm.dir/pio_assembler.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pioasm.dir/pio_assembler.cpp.i"
	C:\VSARM\mingw\mingw32\bin\c++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E C:\VSARM\sdk\pico\pico-sdk\tools\pioasm\pio_assembler.cpp > CMakeFiles\pioasm.dir\pio_assembler.cpp.i

CMakeFiles/pioasm.dir/pio_assembler.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pioasm.dir/pio_assembler.cpp.s"
	C:\VSARM\mingw\mingw32\bin\c++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S C:\VSARM\sdk\pico\pico-sdk\tools\pioasm\pio_assembler.cpp -o CMakeFiles\pioasm.dir\pio_assembler.cpp.s

CMakeFiles/pioasm.dir/pio_disassembler.cpp.obj: CMakeFiles/pioasm.dir/flags.make
CMakeFiles/pioasm.dir/pio_disassembler.cpp.obj: CMakeFiles/pioasm.dir/includes_CXX.rsp
CMakeFiles/pioasm.dir/pio_disassembler.cpp.obj: C:/VSARM/sdk/pico/pico-sdk/tools/pioasm/pio_disassembler.cpp
CMakeFiles/pioasm.dir/pio_disassembler.cpp.obj: CMakeFiles/pioasm.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=C:\Users\craig.hemingway\VisualCode_Workspace\RP2040_TestBed\build\pioasm\CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/pioasm.dir/pio_disassembler.cpp.obj"
	C:\VSARM\mingw\mingw32\bin\c++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/pioasm.dir/pio_disassembler.cpp.obj -MF CMakeFiles\pioasm.dir\pio_disassembler.cpp.obj.d -o CMakeFiles\pioasm.dir\pio_disassembler.cpp.obj -c C:\VSARM\sdk\pico\pico-sdk\tools\pioasm\pio_disassembler.cpp

CMakeFiles/pioasm.dir/pio_disassembler.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pioasm.dir/pio_disassembler.cpp.i"
	C:\VSARM\mingw\mingw32\bin\c++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E C:\VSARM\sdk\pico\pico-sdk\tools\pioasm\pio_disassembler.cpp > CMakeFiles\pioasm.dir\pio_disassembler.cpp.i

CMakeFiles/pioasm.dir/pio_disassembler.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pioasm.dir/pio_disassembler.cpp.s"
	C:\VSARM\mingw\mingw32\bin\c++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S C:\VSARM\sdk\pico\pico-sdk\tools\pioasm\pio_disassembler.cpp -o CMakeFiles\pioasm.dir\pio_disassembler.cpp.s

CMakeFiles/pioasm.dir/gen/lexer.cpp.obj: CMakeFiles/pioasm.dir/flags.make
CMakeFiles/pioasm.dir/gen/lexer.cpp.obj: CMakeFiles/pioasm.dir/includes_CXX.rsp
CMakeFiles/pioasm.dir/gen/lexer.cpp.obj: C:/VSARM/sdk/pico/pico-sdk/tools/pioasm/gen/lexer.cpp
CMakeFiles/pioasm.dir/gen/lexer.cpp.obj: CMakeFiles/pioasm.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=C:\Users\craig.hemingway\VisualCode_Workspace\RP2040_TestBed\build\pioasm\CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/pioasm.dir/gen/lexer.cpp.obj"
	C:\VSARM\mingw\mingw32\bin\c++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/pioasm.dir/gen/lexer.cpp.obj -MF CMakeFiles\pioasm.dir\gen\lexer.cpp.obj.d -o CMakeFiles\pioasm.dir\gen\lexer.cpp.obj -c C:\VSARM\sdk\pico\pico-sdk\tools\pioasm\gen\lexer.cpp

CMakeFiles/pioasm.dir/gen/lexer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pioasm.dir/gen/lexer.cpp.i"
	C:\VSARM\mingw\mingw32\bin\c++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E C:\VSARM\sdk\pico\pico-sdk\tools\pioasm\gen\lexer.cpp > CMakeFiles\pioasm.dir\gen\lexer.cpp.i

CMakeFiles/pioasm.dir/gen/lexer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pioasm.dir/gen/lexer.cpp.s"
	C:\VSARM\mingw\mingw32\bin\c++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S C:\VSARM\sdk\pico\pico-sdk\tools\pioasm\gen\lexer.cpp -o CMakeFiles\pioasm.dir\gen\lexer.cpp.s

CMakeFiles/pioasm.dir/gen/parser.cpp.obj: CMakeFiles/pioasm.dir/flags.make
CMakeFiles/pioasm.dir/gen/parser.cpp.obj: CMakeFiles/pioasm.dir/includes_CXX.rsp
CMakeFiles/pioasm.dir/gen/parser.cpp.obj: C:/VSARM/sdk/pico/pico-sdk/tools/pioasm/gen/parser.cpp
CMakeFiles/pioasm.dir/gen/parser.cpp.obj: CMakeFiles/pioasm.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=C:\Users\craig.hemingway\VisualCode_Workspace\RP2040_TestBed\build\pioasm\CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/pioasm.dir/gen/parser.cpp.obj"
	C:\VSARM\mingw\mingw32\bin\c++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/pioasm.dir/gen/parser.cpp.obj -MF CMakeFiles\pioasm.dir\gen\parser.cpp.obj.d -o CMakeFiles\pioasm.dir\gen\parser.cpp.obj -c C:\VSARM\sdk\pico\pico-sdk\tools\pioasm\gen\parser.cpp

CMakeFiles/pioasm.dir/gen/parser.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pioasm.dir/gen/parser.cpp.i"
	C:\VSARM\mingw\mingw32\bin\c++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E C:\VSARM\sdk\pico\pico-sdk\tools\pioasm\gen\parser.cpp > CMakeFiles\pioasm.dir\gen\parser.cpp.i

CMakeFiles/pioasm.dir/gen/parser.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pioasm.dir/gen/parser.cpp.s"
	C:\VSARM\mingw\mingw32\bin\c++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S C:\VSARM\sdk\pico\pico-sdk\tools\pioasm\gen\parser.cpp -o CMakeFiles\pioasm.dir\gen\parser.cpp.s

CMakeFiles/pioasm.dir/c_sdk_output.cpp.obj: CMakeFiles/pioasm.dir/flags.make
CMakeFiles/pioasm.dir/c_sdk_output.cpp.obj: CMakeFiles/pioasm.dir/includes_CXX.rsp
CMakeFiles/pioasm.dir/c_sdk_output.cpp.obj: C:/VSARM/sdk/pico/pico-sdk/tools/pioasm/c_sdk_output.cpp
CMakeFiles/pioasm.dir/c_sdk_output.cpp.obj: CMakeFiles/pioasm.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=C:\Users\craig.hemingway\VisualCode_Workspace\RP2040_TestBed\build\pioasm\CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/pioasm.dir/c_sdk_output.cpp.obj"
	C:\VSARM\mingw\mingw32\bin\c++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/pioasm.dir/c_sdk_output.cpp.obj -MF CMakeFiles\pioasm.dir\c_sdk_output.cpp.obj.d -o CMakeFiles\pioasm.dir\c_sdk_output.cpp.obj -c C:\VSARM\sdk\pico\pico-sdk\tools\pioasm\c_sdk_output.cpp

CMakeFiles/pioasm.dir/c_sdk_output.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pioasm.dir/c_sdk_output.cpp.i"
	C:\VSARM\mingw\mingw32\bin\c++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E C:\VSARM\sdk\pico\pico-sdk\tools\pioasm\c_sdk_output.cpp > CMakeFiles\pioasm.dir\c_sdk_output.cpp.i

CMakeFiles/pioasm.dir/c_sdk_output.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pioasm.dir/c_sdk_output.cpp.s"
	C:\VSARM\mingw\mingw32\bin\c++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S C:\VSARM\sdk\pico\pico-sdk\tools\pioasm\c_sdk_output.cpp -o CMakeFiles\pioasm.dir\c_sdk_output.cpp.s

CMakeFiles/pioasm.dir/python_output.cpp.obj: CMakeFiles/pioasm.dir/flags.make
CMakeFiles/pioasm.dir/python_output.cpp.obj: CMakeFiles/pioasm.dir/includes_CXX.rsp
CMakeFiles/pioasm.dir/python_output.cpp.obj: C:/VSARM/sdk/pico/pico-sdk/tools/pioasm/python_output.cpp
CMakeFiles/pioasm.dir/python_output.cpp.obj: CMakeFiles/pioasm.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=C:\Users\craig.hemingway\VisualCode_Workspace\RP2040_TestBed\build\pioasm\CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object CMakeFiles/pioasm.dir/python_output.cpp.obj"
	C:\VSARM\mingw\mingw32\bin\c++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/pioasm.dir/python_output.cpp.obj -MF CMakeFiles\pioasm.dir\python_output.cpp.obj.d -o CMakeFiles\pioasm.dir\python_output.cpp.obj -c C:\VSARM\sdk\pico\pico-sdk\tools\pioasm\python_output.cpp

CMakeFiles/pioasm.dir/python_output.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pioasm.dir/python_output.cpp.i"
	C:\VSARM\mingw\mingw32\bin\c++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E C:\VSARM\sdk\pico\pico-sdk\tools\pioasm\python_output.cpp > CMakeFiles\pioasm.dir\python_output.cpp.i

CMakeFiles/pioasm.dir/python_output.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pioasm.dir/python_output.cpp.s"
	C:\VSARM\mingw\mingw32\bin\c++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S C:\VSARM\sdk\pico\pico-sdk\tools\pioasm\python_output.cpp -o CMakeFiles\pioasm.dir\python_output.cpp.s

CMakeFiles/pioasm.dir/hex_output.cpp.obj: CMakeFiles/pioasm.dir/flags.make
CMakeFiles/pioasm.dir/hex_output.cpp.obj: CMakeFiles/pioasm.dir/includes_CXX.rsp
CMakeFiles/pioasm.dir/hex_output.cpp.obj: C:/VSARM/sdk/pico/pico-sdk/tools/pioasm/hex_output.cpp
CMakeFiles/pioasm.dir/hex_output.cpp.obj: CMakeFiles/pioasm.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=C:\Users\craig.hemingway\VisualCode_Workspace\RP2040_TestBed\build\pioasm\CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object CMakeFiles/pioasm.dir/hex_output.cpp.obj"
	C:\VSARM\mingw\mingw32\bin\c++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/pioasm.dir/hex_output.cpp.obj -MF CMakeFiles\pioasm.dir\hex_output.cpp.obj.d -o CMakeFiles\pioasm.dir\hex_output.cpp.obj -c C:\VSARM\sdk\pico\pico-sdk\tools\pioasm\hex_output.cpp

CMakeFiles/pioasm.dir/hex_output.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pioasm.dir/hex_output.cpp.i"
	C:\VSARM\mingw\mingw32\bin\c++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E C:\VSARM\sdk\pico\pico-sdk\tools\pioasm\hex_output.cpp > CMakeFiles\pioasm.dir\hex_output.cpp.i

CMakeFiles/pioasm.dir/hex_output.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pioasm.dir/hex_output.cpp.s"
	C:\VSARM\mingw\mingw32\bin\c++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S C:\VSARM\sdk\pico\pico-sdk\tools\pioasm\hex_output.cpp -o CMakeFiles\pioasm.dir\hex_output.cpp.s

CMakeFiles/pioasm.dir/ada_output.cpp.obj: CMakeFiles/pioasm.dir/flags.make
CMakeFiles/pioasm.dir/ada_output.cpp.obj: CMakeFiles/pioasm.dir/includes_CXX.rsp
CMakeFiles/pioasm.dir/ada_output.cpp.obj: C:/VSARM/sdk/pico/pico-sdk/tools/pioasm/ada_output.cpp
CMakeFiles/pioasm.dir/ada_output.cpp.obj: CMakeFiles/pioasm.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=C:\Users\craig.hemingway\VisualCode_Workspace\RP2040_TestBed\build\pioasm\CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building CXX object CMakeFiles/pioasm.dir/ada_output.cpp.obj"
	C:\VSARM\mingw\mingw32\bin\c++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/pioasm.dir/ada_output.cpp.obj -MF CMakeFiles\pioasm.dir\ada_output.cpp.obj.d -o CMakeFiles\pioasm.dir\ada_output.cpp.obj -c C:\VSARM\sdk\pico\pico-sdk\tools\pioasm\ada_output.cpp

CMakeFiles/pioasm.dir/ada_output.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pioasm.dir/ada_output.cpp.i"
	C:\VSARM\mingw\mingw32\bin\c++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E C:\VSARM\sdk\pico\pico-sdk\tools\pioasm\ada_output.cpp > CMakeFiles\pioasm.dir\ada_output.cpp.i

CMakeFiles/pioasm.dir/ada_output.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pioasm.dir/ada_output.cpp.s"
	C:\VSARM\mingw\mingw32\bin\c++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S C:\VSARM\sdk\pico\pico-sdk\tools\pioasm\ada_output.cpp -o CMakeFiles\pioasm.dir\ada_output.cpp.s

# Object files for target pioasm
pioasm_OBJECTS = \
"CMakeFiles/pioasm.dir/main.cpp.obj" \
"CMakeFiles/pioasm.dir/pio_assembler.cpp.obj" \
"CMakeFiles/pioasm.dir/pio_disassembler.cpp.obj" \
"CMakeFiles/pioasm.dir/gen/lexer.cpp.obj" \
"CMakeFiles/pioasm.dir/gen/parser.cpp.obj" \
"CMakeFiles/pioasm.dir/c_sdk_output.cpp.obj" \
"CMakeFiles/pioasm.dir/python_output.cpp.obj" \
"CMakeFiles/pioasm.dir/hex_output.cpp.obj" \
"CMakeFiles/pioasm.dir/ada_output.cpp.obj"

# External object files for target pioasm
pioasm_EXTERNAL_OBJECTS =

pioasm.exe: CMakeFiles/pioasm.dir/main.cpp.obj
pioasm.exe: CMakeFiles/pioasm.dir/pio_assembler.cpp.obj
pioasm.exe: CMakeFiles/pioasm.dir/pio_disassembler.cpp.obj
pioasm.exe: CMakeFiles/pioasm.dir/gen/lexer.cpp.obj
pioasm.exe: CMakeFiles/pioasm.dir/gen/parser.cpp.obj
pioasm.exe: CMakeFiles/pioasm.dir/c_sdk_output.cpp.obj
pioasm.exe: CMakeFiles/pioasm.dir/python_output.cpp.obj
pioasm.exe: CMakeFiles/pioasm.dir/hex_output.cpp.obj
pioasm.exe: CMakeFiles/pioasm.dir/ada_output.cpp.obj
pioasm.exe: CMakeFiles/pioasm.dir/build.make
pioasm.exe: CMakeFiles/pioasm.dir/linkLibs.rsp
pioasm.exe: CMakeFiles/pioasm.dir/objects1
pioasm.exe: CMakeFiles/pioasm.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=C:\Users\craig.hemingway\VisualCode_Workspace\RP2040_TestBed\build\pioasm\CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Linking CXX executable pioasm.exe"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles\pioasm.dir\link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/pioasm.dir/build: pioasm.exe
.PHONY : CMakeFiles/pioasm.dir/build

CMakeFiles/pioasm.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles\pioasm.dir\cmake_clean.cmake
.PHONY : CMakeFiles/pioasm.dir/clean

CMakeFiles/pioasm.dir/depend:
	$(CMAKE_COMMAND) -E cmake_depends "MinGW Makefiles" C:\VSARM\sdk\pico\pico-sdk\tools\pioasm C:\VSARM\sdk\pico\pico-sdk\tools\pioasm C:\Users\craig.hemingway\VisualCode_Workspace\RP2040_TestBed\build\pioasm C:\Users\craig.hemingway\VisualCode_Workspace\RP2040_TestBed\build\pioasm C:\Users\craig.hemingway\VisualCode_Workspace\RP2040_TestBed\build\pioasm\CMakeFiles\pioasm.dir\DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/pioasm.dir/depend

