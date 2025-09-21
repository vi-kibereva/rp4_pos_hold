cmake_minimum_required(VERSION 3.10)
project(rp4_pos_hold1 LANGUAGES CXX)

# Require modern C++
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Make headers in include/ visible
include_directories(${CMAKE_SOURCE_DIR}/include)

# Collect all .cpp files in src/
file(GLOB_RECURSE ALL_SRC "${CMAKE_SOURCE_DIR}/src/*.cpp")

# Remove any file whose name starts with main_gr
list(FILTER ALL_SRC EXCLUDE REGEX ".*/main_gr.*\\.cpp$")

# Create the executable
add_executable(rp4_pos_hold1 ${ALL_SRC})
