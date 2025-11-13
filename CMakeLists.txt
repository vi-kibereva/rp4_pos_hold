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

#OpenCV
find_package( OpenCV REQUIRED )


find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBCAMERA REQUIRED libcamera)

include_directories(${LIBCAMERA_INCLUDE_DIRS} ${OPENCV_INCLUDE_DIRS})

# Collect all .cpp files in src/
file(GLOB_RECURSE ALL_SRC "${CMAKE_SOURCE_DIR}/src/*.cpp")

# --- First executable: rp4_pos_hold1 ---
add_executable(rp4_pos_hold1 ${ALL_SRC})
target_link_libraries(rp4_pos_hold1 -llccv ${OpenCV_LIBS})