set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_library(
    controller
    controller/Controller2D.h
    controller/Controller2D.cpp
)
target_include_directories(controller PUBLIC "${CMAKE_SOURCE_DIR}/libs/pybind11/include")
pybind11_add_module(py_controller controller/Controller2D.cpp)
