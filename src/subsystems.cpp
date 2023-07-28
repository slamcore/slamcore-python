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

#include "slamcore/subsystems/height_mapping.hpp"
#include "slamcore/subsystems/optimised_trajectory.hpp"
#include "slamcore/subsystems/panoptic_segmentation.hpp"
#include "slamcore/subsystems/sensors_info.hpp"

#include <matrix_converter.hpp>
#include <pybind11/chrono.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

namespace slamcore::python
{

void subsystems(py::module& module)
{
  auto subsystemsModule = module.def_submodule("subsystems");

  py::class_<SensorsInfoInterface, std::shared_ptr<SensorsInfoInterface>>(subsystemsModule,
                                                                          "SensorsInfo")
    .def_property_readonly("camera_list", &SensorsInfoInterface::getCameraList)
    .def("sensor_size", &SensorsInfoInterface::getCameraSensorSize, py::arg("camera_id"))
    .def("image_format", &SensorsInfoInterface::getCameraImageFormat, py::arg("camera_id"))
    .def("factory_focal_length",
         &SensorsInfoInterface::getCameraFactoryFocalLength,
         py::arg("camera_id"))
    .def("factory_principal_point",
         &SensorsInfoInterface::getCameraFactoryPrincipalPoint,
         py::arg("camera_id"))
    .def("factory_distortion_model",
         &SensorsInfoInterface::getCameraFactoryDistortionModel,
         py::arg("camera_id"))
    .def("factory_distortion_params",
         &SensorsInfoInterface::getCameraFactoryDistortionParams,
         py::arg("camera_id"))
    .def("factory_TSC", &SensorsInfoInterface::getCameraFactoryTSC, py::arg("camera_id"));

  py::class_<Map2DAccessInterface, std::shared_ptr<Map2DAccessInterface>>(subsystemsModule,
                                                                          "Map2DAccessInterface")
    .def(
      "fetch",
      [](Map2DAccessInterface& obj, std::chrono::nanoseconds timeout) { return obj.fetch(timeout); },
      py::arg("timeout"),
      py::call_guard<py::gil_scoped_release>())
    .def("get", &Map2DAccessInterface::get, py::return_value_policy::reference_internal);

  py::class_<HeightMappingSubsystemInterface,
             Map2DAccessInterface,
             std::shared_ptr<HeightMappingSubsystemInterface>>(subsystemsModule,
                                                               "HeightMappingSubsystem",
                                                               py::multiple_inheritance());

  auto trajectorySubsystem = py::class_<OptimisedTrajectorySubsystemInterface,
                                        std::shared_ptr<OptimisedTrajectorySubsystemInterface>>(
    subsystemsModule, "OptimisedTrajectorySubsystem");

  trajectorySubsystem
    .def(
      "fetch",
      [](OptimisedTrajectorySubsystemInterface& obj, std::chrono::nanoseconds timeout)
      { return obj.fetch(timeout); },
      py::arg("timeout"),
      py::call_guard<py::gil_scoped_release>())
    .def_property_readonly("trajectory",
                           &OptimisedTrajectorySubsystemInterface::getTrajectory,
                           py::return_value_policy::reference_internal);

  py::class_<PanopticSegmentationRunnerInterface, std::shared_ptr<PanopticSegmentationRunnerInterface>>(
    subsystemsModule, "PanopticSegmentationRunnerInterface")
    .def("run_inference", &PanopticSegmentationRunnerInterface::runInference, py::arg("image"))
    .def_property_readonly("label_mapping",
                           &PanopticSegmentationRunnerInterface::getLabelMapping,
                           py::return_value_policy::reference_internal);

  py::class_<PanopticSegmentationSubsystemInterface,
             std::shared_ptr<PanopticSegmentationSubsystemInterface>>(subsystemsModule,
                                                                      "PanopticSegmentationSubsystem")
    .def("register_plugin",
         &PanopticSegmentationSubsystemInterface::registerPlugin,
         py::arg("path"),
         py::call_guard<py::gil_scoped_release>())
    .def("register_default_plugin",
         &PanopticSegmentationSubsystemInterface::registerDefaultPlugin,
         py::call_guard<py::gil_scoped_release>())
    .def("get_runner",
         &PanopticSegmentationSubsystemInterface::getRunner,
         py::return_value_policy::reference_internal);
}

} // namespace slamcore::python
