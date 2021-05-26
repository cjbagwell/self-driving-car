cmake_minimum_required(VERSION 3.4)
project(pybind11_test)
add_subdirectory(pybind11)
pybind11_add_module(module_name main.cpp)