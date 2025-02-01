# Minimum version of CMake required
cmake_minimum_required(VERSION 3.10)

# Project name and version
project(Geo3DConsole VERSION 1.0)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Add include directory
include_directories(include)

# Add source and header files
set(SOURCES
    src/main.cpp
    src/Geometry.cpp
)

set(HEADERS
    include/Geometry.h
)

# Add the executable
add_executable(Geo3DConsole ${SOURCES} ${HEADERS})

# Optionally install the executable
install(TARGETS Geo3DConsole DESTINATION bin)