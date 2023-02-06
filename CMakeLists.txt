

cmake_minimum_required(VERSION 3.17)

project(SimplexAlg)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# For each source file 2 versions will be generated, e.g. example and example_print,
# the former returns the solution to the problem and the latter also prints each step of the algorithm

file( GLOB EXAMPLES ./examples/*.cpp )
foreach(EXAMPLE ${EXAMPLES} )
    get_filename_component(EXAMPLE_NAME ${EXAMPLE} NAME_WE)
    add_executable( ${EXAMPLE_NAME} ${EXAMPLE} )
    add_executable( ${EXAMPLE_NAME}_print ${EXAMPLE} )
    target_compile_definitions(${EXAMPLE_NAME}_print PUBLIC PRINT)
    set_target_properties(${EXAMPLE_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ./examples)
    set_target_properties(${EXAMPLE_NAME}_print PROPERTIES RUNTIME_OUTPUT_DIRECTORY ./examples)
endforeach()







