cmake_minimum_required(VERSION 3.17)

set(CMAKE_C_COMPILER "C:/MinGW/bin/gcc.exe")
set(CMAKE_CXX_COMPILER "C:/MinGW/bin/g++.exe")

project(esame)

add_executable(esame main.cpp)
add_executable(mainEQ include/mainEQ.cpp)

add_executable(main main.cpp)
add_executable(main_print main.cpp)

add_executable(mainINFEASIBLE mainINFEASIBLE.cpp)
add_executable(mainINFEASIBLE_print mainINFEASIBLE.cpp)

add_executable(mainMIN mainMIN.cpp)
add_executable(mainMIN_print mainMIN.cpp)

add_executable(mainTest mainTest.cpp)
add_executable(mainTest_print mainTest.cpp)

add_executable(mainUNBOUND mainUNBOUND.cpp)
add_executable(mainUNBOUND_print mainUNBOUND.cpp)


target_compile_definitions(esame_print PUBLIC PRINT)
target_compile_definitions(mainINFEASIBLE_print PUBLIC PRINT)
target_compile_definitions(mainMIN_print PUBLIC PRINT)
target_compile_definitions(mainTest_print PUBLIC PRINT)
target_compile_definitions(mainUNBOUND_print PUBLIC PRINT)
