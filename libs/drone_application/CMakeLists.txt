add_library(drone_application
        src/drone.cpp
        inc/drone.h
)
target_include_directories(drone_application PUBLIC inc)

enable_testing()
add_executable(drone_tests
        tests/test_drone.cpp
)
target_link_libraries(drone_tests
        drone_application
        GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(drone_tests)