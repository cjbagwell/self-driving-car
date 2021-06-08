include_directories(${LIBS}/**)
set(This LoalizationTests)
set(Sources 
    LocalizationTests.cpp
)

add_executable(${This} ${Sources})
target_link_libraries(${This} PUBLIC
    gtest_main
    localization
    armadillo
)

add_test(
    NAME ${This}
    COMMAND ${This}
)