cmake_minimum_required(VERSION 3.24)

set(CMAKE_CXX_STANDARD 11)

project(Simplex_algorithm)

add_executable(main main.cpp)
add_executable(testEQ testEQ.cpp)
add_executable(TestINFEASIBLE TestINFEASIBLE.cpp)
add_executable(testMIN testMIN.cpp)
add_executable(testOUNB testOUNB.cpp)


