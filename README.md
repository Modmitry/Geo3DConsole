# Geo3DConsole

Geo3DConsole is a simple C++ console application that calculates the minimum distance between two line segments in 3D space. 
The application prompts the user to enter four 3D points (each specified by x, y, and z coordinates) that define two line segments (points 1-2 and points 3-4),
and then computes the minimum distance between these segments.

## Prerequisites

- A C++ compiler that supports C++17.
- [CMake](https://cmake.org/) version 3.10 or later.

## Installation and Build Instructions

1. **Install Required Tools:**

   - [Download and install CMake](https://cmake.org/download/).
   - Install a C++ compiler such as Visual Studio (which includes MSVC) or MinGW.

2. **Clone the Repository:**
   - git clone https://github.com/Modmitry/Geo3DConsole.git

4. **Create a Build Directory:**
   - cd /path/to/Geo3DConsole
   - mkdir build
   - cd build   
5. **Run CMake to Generate Build Files:**
   - cmake ..
   - Optional: To specify a particular generator (e.g., Visual Studio 2022), you can add: cmake -G "Visual Studio 17 2022" ..

6. **Build the project:**
  - Open the generated solution file Geo3DConsole.sln [Windows]
  - make - [Linux]
