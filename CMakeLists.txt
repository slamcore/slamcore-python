# =========================================================================== #
#
# Slamcore Confidential
# ---------------------
#
# Slamcore Limited
# All Rights Reserved.
# (C) Copyright 2023
#
# NOTICE:
#
# All information contained herein is, and remains the property of Slamcore
# Limited and its suppliers, if any. The intellectual and technical concepts
# contained herein are proprietary to Slamcore Limited and its suppliers and
# may be covered by patents in process, and are protected by trade secret or
# copyright law. Dissemination of this information or reproduction of this
# material is strictly forbidden unless prior written permission is obtained
# from Slamcore Limited.
#
# =========================================================================== #

cmake_minimum_required(VERSION 3.4...3.18)

project(slamcore-python)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(GetGitTagVersion)

set(SLAMCORE_PYTHON_SO_NAME _slamcore_python)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(pybind11 2.10.3 REQUIRED)
find_package(PythonPip REQUIRED)
find_package(slamcore_sdk REQUIRED)
find_package(Eigen3 REQUIRED)

find_program(
  STUBGEN_BIN
  NAMES stubgen
  DOC "Path to stubgen"
)

pybind11_add_module(${SLAMCORE_PYTHON_SO_NAME}
    src/errors.cpp
    src/logging.cpp
    src/objects.cpp
    src/slam.cpp
    src/slamcore_python.cpp
    src/subsystems.cpp
    src/types.cpp
    src/version.cpp)

set_target_properties(${SLAMCORE_PYTHON_SO_NAME} PROPERTIES
                                                 C_STANDARD 99
                                                 CXX_STANDARD 17
                                                 CXX_STANDARD_REQUIRED YES
                                                 CXX_EXTENSIONS NO)

message(STATUS "Building python bindings for slamcore_sdk - using python: ${PYTHON_EXECUTABLE}")

target_link_libraries(${SLAMCORE_PYTHON_SO_NAME} PRIVATE slamcore::slamcore)

target_link_libraries(${SLAMCORE_PYTHON_SO_NAME} PRIVATE Eigen3::Eigen)

target_include_directories(${SLAMCORE_PYTHON_SO_NAME} PRIVATE src/include)

set(SLAMCORE_PYTHON_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/python)
set(SLAMCORE_PYTHON_MODULE_OUTPUT_DIRECTORY ${SLAMCORE_PYTHON_OUTPUT_DIRECTORY}/slamcore)

set_target_properties(${SLAMCORE_PYTHON_SO_NAME} PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${SLAMCORE_PYTHON_MODULE_OUTPUT_DIRECTORY})

add_custom_command(
    TARGET ${SLAMCORE_PYTHON_SO_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
            ${CMAKE_CURRENT_LIST_DIR}/slamcore_python/__init__.py
            ${SLAMCORE_PYTHON_MODULE_OUTPUT_DIRECTORY}/__init__.py
    COMMENT "Configure the slamcore_python __init__.py file")

add_custom_command(
    TARGET ${SLAMCORE_PYTHON_SO_NAME} POST_BUILD
    COMMAND ${PYTHON_EXECUTABLE} -c "import slamcore.${SLAMCORE_PYTHON_SO_NAME}"
    COMMAND ${PYTHON_EXECUTABLE} -c "from slamcore import core"
    WORKING_DIRECTORY ${SLAMCORE_PYTHON_OUTPUT_DIRECTORY}
    COMMENT "Check basic slamcore module imports")

set(BYPRODUCT_PYI_FILES
    ${SLAMCORE_PYTHON_MODULE_OUTPUT_DIRECTORY}/${SLAMCORE_PYTHON_SO_NAME}/__init__.pyi
    ${SLAMCORE_PYTHON_MODULE_OUTPUT_DIRECTORY}/${SLAMCORE_PYTHON_SO_NAME}/core.pyi
    ${SLAMCORE_PYTHON_MODULE_OUTPUT_DIRECTORY}/${SLAMCORE_PYTHON_SO_NAME}/errors.pyi
    ${SLAMCORE_PYTHON_MODULE_OUTPUT_DIRECTORY}/${SLAMCORE_PYTHON_SO_NAME}/logging.pyi
    ${SLAMCORE_PYTHON_MODULE_OUTPUT_DIRECTORY}/${SLAMCORE_PYTHON_SO_NAME}/slam.pyi
    ${SLAMCORE_PYTHON_MODULE_OUTPUT_DIRECTORY}/${SLAMCORE_PYTHON_SO_NAME}/subsystems.pyi)

add_custom_command(
    TARGET ${SLAMCORE_PYTHON_SO_NAME} POST_BUILD
    BYPRODUCTS ${BYPRODUCT_PYI_FILES}
    COMMAND ${STUBGEN_BIN} -o . -p slamcore.${SLAMCORE_PYTHON_SO_NAME} | grep -q -v  'Failed'
    WORKING_DIRECTORY ${SLAMCORE_PYTHON_OUTPUT_DIRECTORY}
    COMMENT "Generate .pyi files for type annotations")

add_custom_target(slamcore_python COMMENT "Generate python bindings")
add_dependencies(slamcore_python ${SLAMCORE_PYTHON_SO_NAME})

get_property(SLAMCORE_PYTHON_SO_SUFFIX TARGET ${SLAMCORE_PYTHON_SO_NAME} PROPERTY SUFFIX)
set(SLAMCORE_PYTHON_SO_FULL_NAME "${SLAMCORE_PYTHON_SO_NAME}${SLAMCORE_PYTHON_SO_SUFFIX}")
configure_file(setup.py.in ${SLAMCORE_PYTHON_OUTPUT_DIRECTORY}/setup.py @ONLY)

file(WRITE ${SLAMCORE_PYTHON_MODULE_OUTPUT_DIRECTORY}/py.typed)

add_custom_target(slamcore_python_wheel ${PIP_EXECUTABLE} wheel --no-deps .
    WORKING_DIRECTORY ${SLAMCORE_PYTHON_OUTPUT_DIRECTORY}
    DEPENDS ${SLAMCORE_PYTHON_OUTPUT_DIRECTORY}/setup.py
    COMMENT "Generate the python wheel package")

add_dependencies(slamcore_python_wheel slamcore_python)
