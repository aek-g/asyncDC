cmake_minimum_required(VERSION 4.3)
project(asyncDC)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_subdirectory(libs/googletest)
add_subdirectory(libs/dcs_protocol)
add_subdirectory(libs/drone_application)
