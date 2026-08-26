add_library(protocol
        src/protocol.cpp
        inc/protocol.h
)
target_include_directories(protocol PUBLIC inc)

enable_testing()
add_executable(protocol_tests
        tests/test_protocol.cpp
)
target_link_libraries(protocol_tests
        protocol
        GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(protocol_tests)