add_library(gcs_lib
        inc/gcs_comm.h
        src/gcs_comm.cpp
        inc/gcs_ui.h
        src/gcs_ui.cpp
)
target_include_directories(gcs_lib PUBLIC
        inc
        ${CMAKE_CURRENT_SOURCE_DIR}/../libs/dcs_protocol/inc
        ${CMAKE_CURRENT_SOURCE_DIR}/../libs/config/inc
        ${CMAKE_CURRENT_SOURCE_DIR}/../libs/imgui
        ${CMAKE_CURRENT_SOURCE_DIR}/../libs/imgui/backends
        ${CMAKE_CURRENT_SOURCE_DIR}/../libs/glfw/include
)
target_sources(gcs_lib PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../libs/dcs_protocol/src/protocol.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../libs/imgui/imgui.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../libs/imgui/imgui_draw.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../libs/imgui/imgui_widgets.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../libs/imgui/imgui_tables.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../libs/imgui/backends/imgui_impl_glfw.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../libs/imgui/backends/imgui_impl_opengl3.cpp
)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../libs/glfw ${CMAKE_CURRENT_BINARY_DIR}/glfw_build)
target_link_libraries(gcs_lib PUBLIC glfw)

find_package(OpenGL REQUIRED)
target_link_libraries(gcs_lib PUBLIC OpenGL::GL)

find_package(Threads REQUIRED)
target_link_libraries(gcs_lib PUBLIC Threads::Threads)

add_executable(gcs src/gcs.cpp)
target_link_libraries(gcs PRIVATE gcs_lib)

enable_testing()
add_executable(gcs_tests
        tests/test_gcs.cpp
        src/gcs_comm.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../libs/dcs_protocol/src/protocol.cpp
)
target_include_directories(gcs_tests PRIVATE
        inc
        ${CMAKE_CURRENT_SOURCE_DIR}/../libs/dcs_protocol/inc
        ${CMAKE_CURRENT_SOURCE_DIR}/../libs/config/inc
)
target_compile_definitions(gcs_tests PRIVATE GCS_UNIT_TEST)
target_link_libraries(gcs_tests PRIVATE Threads::Threads GTest::gtest_main)

include(GoogleTest)
gtest_discover_tests(gcs_tests)