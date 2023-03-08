/******************************************************************************
 *
 * Slamcore Confidential
 * ---------------------
 *
 * Slamcore Limited
 * All Rights Reserved.
 * (C) Copyright 2021
 *
 * NOTICE:
 *
 * All information contained herein is, and remains the property of Slamcore
 * Limited and its suppliers, if any. The intellectual and technical concepts
 * contained herein are proprietary to Slamcore Limited and its suppliers and
 * may be covered by patents in process, and are protected by trade secret or
 * copyright law. Dissemination of this information or reproduction of this
 * material is strictly forbidden unless prior written permission is obtained
 * from Slamcore Limited.
 *
 ******************************************************************************/

#include "slamcore/logging.hpp"

#include <pybind11/chrono.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace slamcore::python
{

void logging(pybind11::module& module)
{
  auto loggingModule = module.def_submodule("logging");

  pybind11::enum_<LogSeverity>(loggingModule, "LogSeverity")
    .value("Info", LogSeverity::Info)
    .value("Warning", LogSeverity::Warning)
    .value("Error", LogSeverity::Error)
    .value("Fatal", LogSeverity::Fatal);

  pybind11::class_<LogMessageInterface>(loggingModule, "LogMessage")
    .def_property_readonly("severity", &LogMessageInterface::getSeverity)
    .def_property_readonly("timestamp", &LogMessageInterface::getTimestamp)
    .def_property_readonly("message", &LogMessageInterface::getMessage);
}

} // namespace slamcore::python
