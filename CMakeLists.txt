cmake_minimum_required(VERSION 3.7.0)
project(geodesic)

find_package(Git REQUIRED)

execute_process(
    COMMAND ${GIT_EXECUTABLE} describe --always --tags
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE GEODESIC_VERSION
    ERROR_QUIET
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

message( STATUS "VERSION: ${GEODESIC_VERSION}")

add_definitions(-DGEODESIC_VERSION="${GEODESIC_VERSION}")

set(CMAKE_CXX_STANDARD 11)

OPTION(PYTHON_BINDING "Set when you want to build PYTHON_BINDING (Python bindings for the library)" ON)

#################################################################################
# GEODESIC LIBRARY
#################################################################################
add_library(geodesic_interface INTERFACE)
target_include_directories(geodesic_interface INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include/geodesic)
target_sources(geodesic_interface INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include/geodesic/geodesic_algorithm_base.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/geodesic/geodesic_algorithm_dijkstra.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/geodesic/geodesic_algorithm_dijkstra_alternative.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/geodesic/geodesic_algorithm_exact.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/geodesic/geodesic_algorithm_exact_elements.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/geodesic/geodesic_algorithm_graph_base.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/geodesic/geodesic_algorithm_subdivision.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/geodesic/geodesic_constants_and_simple_functions.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/geodesic/geodesic_memory.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/geodesic/geodesic_mesh.h
    ${CMAKE_CURRENT_SOURCE_DIR}/include/geodesic/geodesic_mesh_elements.h
)
install(FILES $<TARGET_PROPERTY:geodesic_interface,INTERFACE_SOURCES> DESTINATION include)

#################################################################################
# MATLAB LIBRARY
#################################################################################
add_library(geodesic_matlab SHARED matlab/geodesic_matlab_api.h matlab/geodesic_matlab_api.cpp)
target_link_libraries(geodesic_matlab geodesic_interface)
install(TARGETS geodesic_matlab DESTINATION matlab)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/matlab/examples/ DESTINATION matlab FILES_MATCHING PATTERN "*.m")

#################################################################################
# PYTHON BINDING
#################################################################################
IF(PYTHON_BINDING)
    include(FetchContent)
    FetchContent_Declare(
        pybind11
        GIT_REPOSITORY https://github.com/pybind/pybind11
        GIT_TAG v2.2.4
    )

    FetchContent_GetProperties(pybind11)
    if(NOT pybind11_POPULATED)
        FetchContent_Populate(pybind11)
        add_subdirectory(${pybind11_SOURCE_DIR} ${pybind11_BINARY_DIR})
    endif()

    pybind11_add_module(geodesic python/geodesic.cpp)
    target_link_libraries(geodesic LINK_PUBLIC geodesic_interface)

    install(TARGETS geodesic LIBRARY DESTINATION python)
    install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/python/example.py DESTINATION python)
    install(FILES tests/flat_triangular_mesh.txt DESTINATION python)
ENDIF(PYTHON_BINDING)
#################################################################################
# TESTS
#################################################################################
add_executable(example0 tests/example0.cpp)
target_link_libraries(example0 geodesic_interface)

add_executable(example1 tests/example1.cpp)
target_link_libraries(example1 geodesic_interface)

install(TARGETS example0 RUNTIME DESTINATION tests)
install(TARGETS example1 RUNTIME DESTINATION tests)
install(FILES tests/flat_triangular_mesh.txt tests/hedgehog_mesh.txt DESTINATION tests)

#################################################################################
# CPACK
#################################################################################
set(CPACK_PACKAGE_NAME ${CMAKE_PROJECT_NAME})
set(CPACK_PACKAGE_VERSION ${GEODESIC_VERSION})
set(CPACK_PACKAGE_VENDOR "com.github.mojocorp")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Multiple source/target exact geodesic (shortest path) algorithm for triangular mesh (triangulated 2D surface in 3D)")
set(CPACK_PACKAGE_INSTALL_DIRECTORY "${CMAKE_PROJECT_NAME}")

include(CPack)
