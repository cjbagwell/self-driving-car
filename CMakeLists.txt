cmake_minimum_required(VERSION 3.16)
project(self-driving-car)
# find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
set(LIBS "${PROJECT_SOURCE_DIR}/libs")
add_subdirectory(${LIBS}/pybind11)
add_subdirectory(src)

