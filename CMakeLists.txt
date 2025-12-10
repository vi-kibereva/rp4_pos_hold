cmake_minimum_required(VERSION 3.10)
project(rp4_pos_hold1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  message(STATUS "Building in Debug mode - no optimizations, debug symbols enabled")
  set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3 -Wall -Wextra -pedantic")
elseif(CMAKE_BUILD_TYPE STREQUAL "Release")
  message(STATUS "Building in Release mode - optimizations enabled")
  set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
endif()

include_directories(${CMAKE_SOURCE_DIR}/include)

#OpenCV
find_package( OpenCV REQUIRED )

find_package(PkgConfig)

# Check for libcamera (optional - only needed for rp4_pos_hold1 and rp4_recorder)
if(PkgConfig_FOUND)
  pkg_check_modules(LIBCAMERA IMPORTED_TARGET libcamera)
endif()

# Collect all .cpp files in src/
file(GLOB_RECURSE ALL_SRC "${CMAKE_SOURCE_DIR}/src/*.cpp")

list(FILTER ALL_SRC EXCLUDE REGEX ".*/main\\.cpp$")
list(FILTER ALL_SRC EXCLUDE REGEX ".*/record_main\\.cpp$")
list(FILTER ALL_SRC EXCLUDE REGEX ".*/analyze_main\\.cpp$")
list(FILTER ALL_SRC EXCLUDE REGEX ".*/OfflineDroneAdapter\\.cpp$")

set(COMMON_SRC ${ALL_SRC})

# --- First executable: rp4_pos_hold1 ---
if(LIBCAMERA_FOUND)
  message(STATUS "libcamera found - building rp4_pos_hold1 and rp4_recorder")
  add_executable(rp4_pos_hold1 ${CMAKE_SOURCE_DIR}/src/main.cpp ${COMMON_SRC})

  target_include_directories(rp4_pos_hold1 PRIVATE
  ${LIBCAMERA_INCLUDE_DIRS}
)

  target_link_directories(rp4_pos_hold1 PRIVATE ${LIBCAMERA_LIBRARY_DIRS})

  target_link_libraries(
  rp4_pos_hold1 PRIVATE
  ${LIBCAMERA_LIBRARIES}
  ${OpenCV_LIBS}
  lccv
)

  add_executable(
  rp4_recorder
  ${CMAKE_SOURCE_DIR}/src/record_main.cpp
  ${COMMON_SRC}
)

  target_include_directories(rp4_recorder PRIVATE
  ${LIBCAMERA_INCLUDE_DIRS}
)

  target_link_directories(rp4_recorder PRIVATE ${LIBCAMERA_LIBRARY_DIRS})

  target_link_libraries(
  rp4_recorder PRIVATE
  ${LIBCAMERA_LIBRARIES}
  ${OpenCV_LIBS}
  lccv
)
else()
  message(STATUS "libcamera not found - skipping rp4_pos_hold1 and rp4_recorder (Raspberry Pi only)")
endif()

# --- Third executable: rp4_analyzer ---
message(STATUS "Building rp4_analyzer (works on all platforms)")
add_executable(
  rp4_analyzer
  ${CMAKE_SOURCE_DIR}/src/analyze_main.cpp
  ${CMAKE_SOURCE_DIR}/src/pid/pid.cpp
  ${CMAKE_SOURCE_DIR}/src/posHold/OfflineDroneAdapter.cpp
)

target_include_directories(rp4_analyzer PRIVATE
  ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(
  rp4_analyzer PRIVATE
  ${OpenCV_LIBS}
)
