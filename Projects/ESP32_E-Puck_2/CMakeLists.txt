cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(myProject)

# idf_component_register( ... )
target_link_libraries(foo PRIVATE idf::esp-idf-cxx)
target_compile_options(${COMPONENT_LIB} PRIVATE -std=gnu++11)
