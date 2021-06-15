include_directories(/usr/include/python3.8)
include_directories(${LIBS}/**)
add_subdirectory(test)
enable_testing()

find_package( OpenCV REQUIRED )
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
    PUBLIC "${LIBS}/pybind11/include"
    /usr/include/python3.8
)

add_executable(
    VoTest
    VisualOdometer.cpp
)

target_link_libraries(VoTest
    ${OpenCV_LIBS}
    armadillo
)

pybind11_add_module(py_localization EsEkf.cpp)

target_link_libraries(
    py_localization
    PRIVATE pybind11::embed
    armadillo
)