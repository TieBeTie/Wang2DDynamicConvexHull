cmake_minimum_required(VERSION 3.14)
project(Pennant CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(${PROJECT_SOURCE_DIR}/include)

add_executable(pennant_exe
    src/main.cpp
)

enable_testing()
add_subdirectory(tests)
