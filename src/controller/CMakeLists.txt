add_library(
    controller
    Controller2D.h
    Controller2D.cpp
)

target_include_directories(controller PUBLIC "${LIBS}/pybind11/include")

pybind11_add_module(py_controller Controller2D.cpp)

target_link_libraries(
    py_controller
    PRIVATE pybind11::embed
    armadillo
)
