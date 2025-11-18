# CMakeLists.txt
cmake_minimum_required(VERSION 3.16)

# Définir les optimisations avant include
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -funroll-loops -ffast-math")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -funroll-loops -ffast-math")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(optical_flow_lidar)