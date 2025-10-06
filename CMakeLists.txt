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
file(GLOB_recurse MAIN_GR_SRC "${CMAKE_SOURCE_DIR}/src/*.cpp")
list(FILTER MAIN_GR_SRC EXCLUDE REGEX ".*/main\\.cpp$")

# Add main_gr.cpp explicitly
list(APPEND MAIN_GR_SRC "${CMAKE_SOURCE_DIR}/src/main_gr.cpp")

add_executable(main_gr_exec ${MAIN_GR_SRC})
