cmake_minimum_required(VERSION 3.20)

project(bitaflught_msp_example
  VERSION 0.1.0
  LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Use modern C++
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Nice warnings (tweak to taste)
if (MSVC)
  add_compile_options(/W4 /permissive-)
else()
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Library with your MSP implementation
add_library(bitaflught_msp STATIC
  src/msp/bitaflught_msp.cpp
  src/msp/serial_stream.cpp
  src/utils.cpp
)

target_include_directories(bitaflught_msp
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Main executable
add_executable(main_gr
  src/main_gr.cpp
)

target_link_libraries(main_gr
  PRIVATE bitaflught_msp
)

# Optional: faster/safer defaults for debug & release
if (CMAKE_BUILD_TYPE STREQUAL "Release")
  if (CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    target_compile_options(bitaflught_msp PRIVATE -O2)
    target_compile_options(main_gr        PRIVATE -O2)
  endif()
else()
  if (CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    target_compile_options(bitaflught_msp PRIVATE -O0 -g3)
    target_compile_options(main_gr        PRIVATE -O0 -g3)
  endif()
endif()
