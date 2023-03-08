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

#include "slamcore/objects/image.hpp"
#include "slamcore/types/multi_session_id.hpp"
#include "slamcore/types/positioning_mode.hpp"
#include "slamcore/types/range.hpp"
#include "slamcore/types/reference_frame.hpp"
#include "slamcore/types/reference_frame_category.hpp"
#include "slamcore/types/sensor_id.hpp"
#include "slamcore/types/slam_event.hpp"
#include "slamcore/types/tracking_status.hpp"

#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <matrix_converter.hpp>

#include <sstream>

namespace py = pybind11;

namespace slamcore::python
{

void types(pybind11::module& module)
{
  auto coreModule = module.def_submodule("core");

  pybind11::class_<MultiSessionID>(coreModule, "MultiSessionID")
    .def(pybind11::init<>())
    .def(pybind11::init<SessionIDT, MultiSessionID::ValueT>())
    .def("value", &MultiSessionID::value)
    .def("session", &MultiSessionID::session)
    .def("__str__",
         [](const MultiSessionID& self)
         {
           std::stringstream ss;
           ss << self;
           return ss.str();
         });

  pybind11::class_<LandmarkId>(coreModule, "LandmarkId")
    .def(pybind11::init<>())
    .def(pybind11::init<SessionIDT, LandmarkId::ValueT>())
    .def("value", &LandmarkId::value)
    .def("session", &LandmarkId::session)
    .def("__str__",
         [](const LandmarkId& self)
         {
           std::stringstream ss;
           ss << self;
           return ss.str();
         });

  pybind11::class_<FrameId>(coreModule, "FrameId")
    .def(pybind11::init<>())
    .def(pybind11::init<SessionIDT, FrameId::ValueT>())
    .def("value", &FrameId::value)
    .def("session", &FrameId::session)
    .def("__str__",
         [](const FrameId& self)
         {
           std::stringstream ss;
           ss << self;
           return ss.str();
         });

  pybind11::enum_<SensorType>(coreModule, "SensorType")
    .value("SLAM", SensorType::SLAM)
    .value("Visible", SensorType::Visible)
    .value("Depth", SensorType::Depth)
    .value("Infrared", SensorType::Infrared)
    .value("Confidence", SensorType::Confidence)
    .value("Phase", SensorType::Phase)
    .value("Accelerometer", SensorType::Accelerometer)
    .value("Gyroscope", SensorType::Gyroscope)
    .value("Magnetometer", SensorType::Magnetometer)
    .value("GPS", SensorType::GPS)
    .value("Odometry", SensorType::Odometry)
    .value("Encoder", SensorType::Encoder)
    .value("Pose", SensorType::Pose)
    .value("Bumper", SensorType::Bumper)
    .value("Proximity", SensorType::Proximity)
    .value("LIDAR", SensorType::LIDAR);

  pybind11::class_<ReferenceFrame>(coreModule, "ReferenceFrame")
    .def(py::init<>())
    .def(py::init<ReferenceFrameCategory, SensorIndexT>())
    .def("name", &ReferenceFrame::name)
    .def("category", &ReferenceFrame::category)
    .def("index", &ReferenceFrame::index)
    .def("__str__", &ReferenceFrame::name)
    .def("__eq__", &ReferenceFrame::operator==, py::is_operator())
    .def("__ne__", &ReferenceFrame::operator!=, py::is_operator())
    .def("__lt__", &ReferenceFrame::operator<, py::is_operator());

  pybind11::enum_<ReferenceFrameCategory>(coreModule, "ReferenceFrameCategory")
    .value("Unknown", ReferenceFrameCategory::Unknown)
    .value("World", ReferenceFrameCategory::World)
    .value("Camera", ReferenceFrameCategory::Camera)
    .value("IMU", ReferenceFrameCategory::IMU)
    .value("Body", ReferenceFrameCategory::Body)
    .value("Odometry", ReferenceFrameCategory::Odometry)
    .value("LIDAR", ReferenceFrameCategory::LIDAR)
    .value("GPS", ReferenceFrameCategory::GPS);

  pybind11::enum_<ImageFormat>(coreModule, "ImageFormat")
    .value("Custom", ImageFormat::Custom)
    .value("Mono_8", ImageFormat::Mono_8)
    .value("Mono_16", ImageFormat::Mono_16)
    .value("Mono_F", ImageFormat::Mono_F)
    .value("RGB_8", ImageFormat::RGB_8)
    .value("RGBA_8", ImageFormat::RGBA_8)
    .value("RGB_F", ImageFormat::RGB_F)
    .value("RGBA_F", ImageFormat::RGBA_F)
    .value("JPEG", ImageFormat::JPEG)
    .value("Mono_32", ImageFormat::Mono_32)
    .value("Mono_64", ImageFormat::Mono_64);

  pybind11::enum_<TrackingStatus>(coreModule, "TrackingStatus")
    .value("Not_Initialised", TrackingStatus::NOT_INITIALISED)
    .value("Ok", TrackingStatus::OK)
    .value("Lost", TrackingStatus::LOST);

  pybind11::enum_<SLAMEvent>(coreModule, "SLAMEvent")
    .value("Relocalisation", SLAMEvent::Relocalisation)
    .value("LoopClosure", SLAMEvent::LoopClosure)
    .value("ImuInitialization", SLAMEvent::ImuInitialization);

  pybind11::enum_<PositioningMode>(coreModule, "PositioningMode")
    .value("Odometry_Only", PositioningMode::ODOMETRY_ONLY)
    .value("SLAM", PositioningMode::SLAM)
    .value("Multisession_Localisation", PositioningMode::MULTISESSION_LOCALISATION);

  pybind11::class_<Range2D>(coreModule, "Range2D")
    .def(py::init<>())
    .def(py::init<Range2D::ValueT, Range2D::ValueT>())
    .def("min", &Range2D::min)
    .def("max", &Range2D::max);
}

} // namespace slamcore::python
