cmake_minimum_required(VERSION 3.16)
project(self-driving-car)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 14)
# add_definitions(-DARMA_DONT_USE_WRAPPER)
set(LIBS "${PROJECT_SOURCE_DIR}/libs")
add_subdirectory(${LIBS})
add_subdirectory(src)

