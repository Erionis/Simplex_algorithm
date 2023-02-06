

cmake_minimum_required(VERSION 3.17)

project(SimplexAlg)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_executable(main main.cpp)
add_executable(main_print main.cpp)

add_executable(mainINFEASIBLE mainINFEASIBLE.cpp)
add_executable(mainINFEASIBLE_print mainINFEASIBLE.cpp)

add_executable(mainMIN mainMIN.cpp)
add_executable(mainMIN_print mainMIN.cpp)

add_executable(mainUNBOUND mainUNBOUND.cpp)
add_executable(mainUNBOUND_print mainUNBOUND.cpp)

# Compile libraries
include_directories(${CMAKE_SOURCE_DIR}/include)
link_directories(${CMAKE_SOURCE_DIR}/lib)
add_library(core SHARED ${SOURCES})

# # Compile example files
file( GLOB EXAMPLES ./examples/*.cpp )
foreach( EXAMPLE ${EXAMPLES} )
    get_filename_component(EXAMPLE_NAME ${EXAMPLE} NAME_WE)
    add_executable( ${EXAMPLE_NAME} ${EXAMPLE} )
    target_link_libraries(${EXAMPLE_NAME} core)
endforeach()

target_compile_definitions(main_print PUBLIC PRINT)
target_compile_definitions(mainINFEASIBLE_print PUBLIC PRINT)
target_compile_definitions(mainMIN_print PUBLIC PRINT)
target_compile_definitions(mainUNBOUND_print PUBLIC PRINT)








