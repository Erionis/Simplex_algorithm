cmake_minimum_required(VERSION 3.17)

set(CMAKE_C_COMPILER "C:/MinGW/bin/gcc.exe")
set(CMAKE_CXX_COMPILER "C:/MinGW/bin/g++.exe")

project(esame)

add_executable(esame main.cpp)
add_executable(mainEQ include/mainEQ.cpp)

target_include_directories(esame PRIVATE include)