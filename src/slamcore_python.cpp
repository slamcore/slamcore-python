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

#include <pybind11/pybind11.h>

namespace slamcore::python
{

extern void errors(pybind11::module&);
extern void logging(pybind11::module&);
extern void objects(pybind11::module&);
extern void slam(pybind11::module&);
extern void subsystems(pybind11::module&);
extern void types(pybind11::module&);
extern void version(pybind11::module&);

PYBIND11_MODULE(_slamcore_python, module)
{
  slamcore::python::version(module);
  slamcore::python::errors(module);
  slamcore::python::logging(module);
  slamcore::python::types(module);
  slamcore::python::objects(module);
  slamcore::python::subsystems(module);
  slamcore::python::slam(module);
}

} // namespace slamcore::python
