add_executable(gcs_integration_tests
        test_integration.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../gcs/src/gcs_comm.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../libs/dcs_protocol/src/protocol.cpp
)
target_include_directories(gcs_integration_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../gcs/inc
        ${CMAKE_CURRENT_SOURCE_DIR}/../libs/dcs_protocol/inc
        ${CMAKE_CURRENT_SOURCE_DIR}/../libs/config/inc
)
find_package(Threads REQUIRED)
target_link_libraries(gcs_integration_tests PRIVATE Threads::Threads GTest::gtest_main)