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

find_package(PkgConfig REQUIRED)

pkg_check_modules(LIBCAMERA REQUIRED IMPORTED_TARGET libcamera)

# Collect all .cpp files in src/
file(GLOB_RECURSE ALL_SRC "${CMAKE_SOURCE_DIR}/src/*.cpp")

list(FILTER ALL_SRC EXCLUDE REGEX ".*/main\\.cpp$")
list(FILTER ALL_SRC EXCLUDE REGEX ".*/record_main\\.cpp$")

set(COMMON_SRC ${ALL_SRC})

# --- First executable: rp4_pos_hold1 ---
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
