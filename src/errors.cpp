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

#include "slamcore/errors.hpp"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

namespace slamcore::python
{

void errors(pybind11::module& module)
{
  auto errorModule = module.def_submodule("errors");

  py::class_<std::error_category>(errorModule, "ErrorCategory")
    .def_property_readonly("name", &std::error_category::name)
    .def_property_readonly("message", &std::error_category::message);

  auto pyErrorCode = py::class_<std::error_code>(errorModule, "SystemErrorCode")
                       .def_property_readonly("value", &std::error_code::value)
                       .def_property_readonly("category", &std::error_code::category)
                       .def_property_readonly("message", &std::error_code::message)
                       .def("__bool__", &std::error_code::operator bool);

  py::enum_<slamcore::errc>(pyErrorCode, "ErrorValues")
    .value("hardware_problem", slamcore::errc::hardware_problem)
    .value("device_not_open", slamcore::errc::device_not_open)
    .value("device_already_open", slamcore::errc::device_already_open)
    .value("device_not_running", slamcore::errc::device_not_running)
    .value("device_already_running", slamcore::errc::device_already_running)
    .value("not_initialized_yet", slamcore::errc::not_initialized_yet)
    .value("initialization_failure", slamcore::errc::initialization_failure)
    .value("input_data_problem", slamcore::errc::input_data_problem)
    .value("compression_format_not_supported", slamcore::errc::compression_format_not_supported)
    .value("stream_not_supported", slamcore::errc::stream_not_supported)
    .value("property_not_supported", slamcore::errc::property_not_supported)
    .value("property_is_read_only", slamcore::errc::property_is_read_only)
    .value("wrong_type_for_property", slamcore::errc::wrong_type_for_property)
    .value("unhandled_exception", slamcore::errc::unhandled_exception)
    .value("no_odometry_listener", slamcore::errc::no_odometry_listener)
    .value("end_of_dataset", slamcore::errc::end_of_dataset)
    .value("end_of_data", slamcore::errc::end_of_data)
    .value("invalid_subsystem", slamcore::errc::invalid_subsystem);
}

} // namespace slamcore::python
