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

#include "slamcore/types/version.hpp"

#include <pybind11/operators.h>
#include <pybind11/pybind11.h>

namespace slamcore::python
{

void version(pybind11::module& module)
{
  auto coreModule = module.def_submodule("core");

  pybind11::class_<slamcore::Version>(coreModule, "Version")
    .def(pybind11::init<>())
    .def(pybind11::init<uint8_t, uint8_t, uint8_t>())
    .def(pybind11::self == pybind11::self)
    .def(pybind11::self < pybind11::self)
    .def(pybind11::self <= pybind11::self)
    .def(pybind11::self > pybind11::self)
    .def(pybind11::self != pybind11::self)
    .def(pybind11::self >= pybind11::self)
    .def("get_major", &slamcore::Version::getMajor)
    .def("get_minor", &slamcore::Version::getMinor)
    .def("get_patch", &slamcore::Version::getPatch);
}

} // namespace slamcore::python
