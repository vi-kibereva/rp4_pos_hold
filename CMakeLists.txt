#cmake_minimum_required(VERSION 3.10)
#project(rp4_pos_hold1 LANGUAGES CXX)
#
## Require modern C++
#set(CMAKE_CXX_STANDARD 17)
#set(CMAKE_CXX_STANDARD_REQUIRED ON)
#
## Make headers in include/ visible
#include_directories(${CMAKE_SOURCE_DIR}/include)
#
## Collect all .cpp files in src/
#file(GLOB_RECURSE ALL_SRC "${CMAKE_SOURCE_DIR}/src/*.cpp")
#
## Remove any file whose name starts with main_gr
#list(FILTER ALL_SRC EXCLUDE REGEX ".*/main_gr.*\\.cpp$")
#
## Create the executable
#add_executable(rp4_pos_hold1 ${ALL_SRC})
cmake_minimum_required(VERSION 3.10)
project(rp4_pos_hold1 LANGUAGES CXX)

# Require modern C++
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set default build type if not specified
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Compiler flags for different build types
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  message(STATUS "Building in Debug mode - no optimizations, debug symbols enabled")
  # Debug flags: no optimization, debug symbols, all warnings
  set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3 -Wall -Wextra -pedantic")
elseif(CMAKE_BUILD_TYPE STREQUAL "Release")
  message(STATUS "Building in Release mode - optimizations enabled")
  # Release flags: optimize for performance
  set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
endif()

# Include headers
include_directories(${CMAKE_SOURCE_DIR}/include)

# Collect all .cpp files in src/
file(GLOB_RECURSE ALL_SRC "${CMAKE_SOURCE_DIR}/src/*.cpp")

# --- First executable: rp4_pos_hold1 ---
# Exclude main_gr.cpp
list(FILTER ALL_SRC EXCLUDE REGEX ".*/main_gr\\.cpp$")

add_executable(rp4_pos_hold1 ${ALL_SRC})

# --- Second executable: main_gr_exec ---
# Collect all sources except main.cpp
file(GLOB_RECURSE MAIN_GR_SRC "${CMAKE_SOURCE_DIR}/src/*.cpp")
list(FILTER MAIN_GR_SRC EXCLUDE REGEX ".*/main\\.cpp$")

# Add main_gr.cpp explicitly
list(APPEND MAIN_GR_SRC "${CMAKE_SOURCE_DIR}/src/main_gr.cpp")

add_executable(main_gr_exec ${MAIN_GR_SRC})
