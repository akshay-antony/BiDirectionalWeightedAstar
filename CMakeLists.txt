cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(BAstar VERSION 1.0)
add_executable(result src/main.cpp)
target_include_directories(result PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(result pthread)
