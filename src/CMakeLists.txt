set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_library(
    controller
    controller/Controller2D.h
    controller/Controller2D.cpp
)
target_include_directories(controller PUBLIC "${LIBS}/pybind11/include")

pybind11_add_module(py_controller controller/Controller2D.cpp)

target_link_libraries(
    py_controller
    PRIVATE pybind11::embed
    armadillo
)


include_directories(/usr/include/python3.8)
