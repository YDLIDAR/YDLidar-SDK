#cmake_policy(SET CMP0078 NEW)
#cmake_policy(SET CMP0086 OLD)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
add_definitions(-std=c++11) # Use C++11

#Include directories
INCLUDE_DIRECTORIES(
     ${CMAKE_SOURCE_DIR}
     ${CMAKE_SOURCE_DIR}/../
     ${CMAKE_CURRENT_BINARY_DIR}
     ${CMAKE_BINARY_DIR}
)
SET(MODULE_NAME ydlidar_csharp)

#################################3
add_subdirectory(examples)
############################################################################
# build ydlidar sdk python version
###############################################################################
# This is a CMake YDLIDAR SDK for Python
INCLUDE(${SWIG_USE_FILE})
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})
SET(CMAKE_SWIG_FLAGS "")
#set(CMAKE_SWIG_OUTDIR "${CMAKE_BINARY_DIR}")
SET(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_BINARY_DIR})
set_property(SOURCE ydlidar_sdk.i PROPERTY SWIG_FLAGS "-module" "ydlidar")
SET_SOURCE_FILES_PROPERTIES(ydlidar_sdk.i PROPERTIES CPLUSPLUS ON)
###############################################################################
# This is a CMake YDLIDAR SDK for Python
if (${CMAKE_VERSION} VERSION_LESS "3.8.0")
    swig_add_module(${MODULE_NAME} csharp ydlidar_sdk.i)
else()
    swig_add_library(${MODULE_NAME} LANGUAGE csharp SOURCES ydlidar_sdk.i)
endif()
SWIG_LINK_LIBRARIES(${MODULE_NAME} ydlidar_sdk)
#set_target_properties(${MODULE_NAME} PROPERTIES OUTPUT_NAME "ydlidar")
