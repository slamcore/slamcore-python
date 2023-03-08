# ============================================================================ #
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
# ============================================================================ #

find_program(
  PIP_EXECUTABLE
  NAMES pip3
  DOC "Path to pip executable")

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(PythonPip "Failed to find pip3 executable"
                                  PIP_EXECUTABLE)
