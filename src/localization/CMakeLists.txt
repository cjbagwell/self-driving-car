
cmake_minimum_required(VERSION 3.16)

include_directories(/usr/include/python3.8)
include_directories(${LIBS}/**)
add_subdirectory(test)
enable_testing()

find_package( OpenCV REQUIRED )
find_package( Python3 REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )


set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 14)

add_library(localization
    EsEkf.h
    EsEkf.cpp
    ImuMeasurement.h
    rotations.h
    State.h
    VisualOdometer.h
    VisualOdometer.cpp
)

target_include_directories(localization 
    PRIVATE "${LIBS}/pybind11/include"
    PRIVATE "${LIBS}/matplotlib-cpp"
    PRIVATE ${PYTHON_INCLUDE_DIRS}
)

target_link_libraries(localization
    ${PYTHON_LIBRARIES}
)

add_executable(
    VoTest
    VisualOdometer.cpp
)

target_link_libraries(VoTest
    ${OpenCV_LIBS}
    ${PYTHON_LIBRARIES}
    armadillo
)

target_include_directories(VoTest
    PRIVATE "${LIBS}/matplotlib-cpp"
    PRIVATE ${PYTHON_INCLUDE_DIRS}
)

pybind11_add_module(py_localization EsEkf.cpp)

target_link_libraries(
    py_localization
    PRIVATE pybind11::embed
    armadillo
)