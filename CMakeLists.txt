cmake_minimum_required(VERSION 3.16)
project(self-driving-car)

add_definitions(-DARMA_DONT_USE_WRAPPER)
set(LIBS "${PROJECT_SOURCE_DIR}/libs")
add_subdirectory(${LIBS})
add_subdirectory(src)

