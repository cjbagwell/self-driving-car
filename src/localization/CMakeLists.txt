include_directories(/usr/include/python3.8)

add_library(localization
    EsEkf.h
    EsEkf.cpp
    ImuMeasurement.h
    rotations.h
    State.h
)

target_include_directories(localization 
    PUBLIC "${LIBS}/pybind11/include"
    /usr/include/python3.8)

pybind11_add_module(py_localization EsEkf.cpp)

target_link_libraries(
    py_localization
    PRIVATE pybind11::embed
    armadillo
)