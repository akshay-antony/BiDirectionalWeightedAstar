cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(BAstar VERSION 1.0)
add_executable(result main.cpp)
target_link_libraries(result pthread)
