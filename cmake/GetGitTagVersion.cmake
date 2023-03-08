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

find_package(Git)

# =========================================================================== #
# This logic implements converting the output of `git describe --tags` into
# a format compatible with PEP 440.
# The output must be parsed into:
#  1. The version number, which will be validated as a PEP 440 public version
#     identifier; and,
#  2. An optional local version label.
# Some examples:
# git describe --tags       | public version     | local version
# --------------------------+--------------------+----------------------------
# v1.2.3-10-gb183a83        | 1.2.3              | +10.b183a83
# v21.01.1                  | 21.01.1            | <empty>
# <no tags>                 | 0.1.0              | +0.<short-hash>
# =========================================================================== #
if(GIT_EXECUTABLE)

  # Find the closest git tag to the current commit. Version tags are expected
  # to beging with "v".
  execute_process(
    COMMAND ${GIT_EXECUTABLE} describe --tags --match "v*"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    OUTPUT_VARIABLE GIT_DESCRIBE_OUTPUT
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
  if(NOT "${GIT_DESCRIBE_OUTPUT}" STREQUAL "")
    message(DEBUG "GIT_DESCRIBE_OUTPUT=${GIT_DESCRIBE_OUTPUT}")
    string(REGEX REPLACE "-.*" "" GIT_TAG "${GIT_DESCRIBE_OUTPUT}")
    message(DEBUG "GIT_TAG=${GIT_TAG}")
    string(REGEX REPLACE "^v" "" GIT_TAG_VERSION "${GIT_TAG}")
    message(DEBUG "GIT_TAG_VERSION=${GIT_TAG_VERSION}")
    string(REGEX REPLACE "^${GIT_TAG}-?" "" GIT_DESCRIBE_SUFFIX "${GIT_DESCRIBE_OUTPUT}")
    message(DEBUG "GIT_DESCRIBE_SUFFIX=${GIT_DESCRIBE_SUFFIX}")
    if(NOT "${GIT_DESCRIBE_SUFFIX}" STREQUAL "")
      string(REGEX REPLACE "-g" "." SLAMCORE_PYTHON_LOCAL_VERSION "+${GIT_DESCRIBE_SUFFIX}")
    endif()
    message(DEBUG "SLAMCORE_PYTHON_LOCAL_VERSION=${SLAMCORE_PYTHON_LOCAL_VERSION}")
    set(SLAMCORE_PYTHON_VERSION ${GIT_TAG_VERSION})
  endif()

  # Fallback: no tags
  if(NOT DEFINED SLAMCORE_PYTHON_VERSION)
    execute_process(
      COMMAND ${GIT_EXECUTABLE} rev-parse --short HEAD
      WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
      OUTPUT_VARIABLE GIT_REV_PARSE_OUTPUT
      OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    set(SLAMCORE_PYTHON_VERSION 0.1.0)
    set(SLAMCORE_PYTHON_LOCAL_VERSION "+0.${GIT_REV_PARSE_OUTPUT}")
  endif()
else()
  # No git
  set(SLAMCORE_PYTHON_VERSION 0.1.0)
  set(SLAMCORE_PYTHON_LOCAL_VERSION "+0.unknown")
endif()

