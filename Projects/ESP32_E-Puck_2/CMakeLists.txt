cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(ESP32_E_Puck_2)

# List all defined targets
message(STATUS "IDF_TARGETS: ${IDF_TARGETS}")


# idf_component_register( ... )
target_link_libraries(ESP32_E_Puck_2 PRIVATE idf::esp-idf-cxx)
target_compile_options(${COMPONENT_LIB} PRIVATE -std=gnu++11)
