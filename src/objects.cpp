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

#include "slamcore/objects/error_code.hpp"
#include "slamcore/objects/fixed_measurement_point.hpp"
#include "slamcore/objects/frame_sync.hpp"
#include "slamcore/objects/image.hpp"
#include "slamcore/objects/imu_list.hpp"
#include "slamcore/objects/imu_sensor_data.hpp"
#include "slamcore/objects/map2d.hpp"
#include "slamcore/objects/map_channel2d.hpp"
#include "slamcore/objects/matrix.hpp"
#include "slamcore/objects/meta_data.hpp"
#include "slamcore/objects/multi_frame.hpp"
#include "slamcore/objects/point_cloud.hpp"
#include "slamcore/objects/pose.hpp"
#include "slamcore/objects/pose_list.hpp"
#include "slamcore/objects/slam_event_list.hpp"
#include "slamcore/objects/slam_status.hpp"
#include "slamcore/objects/sparse_map.hpp"
#include "slamcore/objects/static_pose.hpp"
#include "slamcore/objects/task_status.hpp"
#include "slamcore/objects/tracking_status_list.hpp"
#include "slamcore/objects/velocity.hpp"
#include "slamcore/semantic/panoptic_bounding_box_3d.hpp"
#include "slamcore/semantic/panoptic_bounding_box_3d_list.hpp"
#include "slamcore/semantic/panoptic_segmentation_result.hpp"
#include "slamcore/types/basic.hpp"
#include "slamcore/types/slam_event.hpp"
#include "slamcore/types/tracking_status.hpp"

#include <Eigen/Core>

#include <matrix_converter.hpp>
#include <pybind11/chrono.h>
#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <optional>
#include <variant>

namespace py = pybind11;

namespace slamcore::python
{

namespace detail
{

std::string toNumpyFormat(const ImageFormat format)
{
  switch (format)
  {
    case (ImageFormat::Mono_8):
    {
      return py::format_descriptor<ImageFormatTraits<ImageFormat::Mono_8>::ChannelType>::format();
    }
    case (ImageFormat::Mono_16):
    {
      return py::format_descriptor<ImageFormatTraits<ImageFormat::Mono_16>::ChannelType>::format();
    }
    case (ImageFormat::Mono_F):
    {
      return py::format_descriptor<ImageFormatTraits<ImageFormat::Mono_F>::ChannelType>::format();
    }
    case (ImageFormat::Mono_32):
    {
      return py::format_descriptor<ImageFormatTraits<ImageFormat::Mono_32>::ChannelType>::format();
    }
    case (ImageFormat::Mono_64):
    {
      return py::format_descriptor<ImageFormatTraits<ImageFormat::Mono_64>::ChannelType>::format();
    }
    case (ImageFormat::Custom):
    case (ImageFormat::RGB_8):
    case (ImageFormat::RGBA_8):
    case (ImageFormat::RGB_F):
    case (ImageFormat::RGBA_F):
    case (ImageFormat::JPEG):
    case (ImageFormat::Count):
    {
      break; // only currently support single channel formats
    }
  }

  throw std::runtime_error("unknown image format");
}

std::variant<int, double, std::string> getMetadataValue(const MetaDataInterface& obj)
{
  const auto checkErrorCode = [](const std::error_code& ec)
  {
    if (ec)
    {
      throw slamcore::slam_exception(ec);
    }
  };

  switch (obj.getValueType())
  {
    case (MetaDataInterface::ValueType::Integer):
    {
      int value;
      checkErrorCode(obj.getValue(value));
      return {value};
    }
    case (MetaDataInterface::ValueType::Floating):
    {
      double value;
      checkErrorCode(obj.getValue(value));
      return {value};
    }
    case (MetaDataInterface::ValueType::String):
    {
      std::string value;
      checkErrorCode(obj.getValue(value));
      return {value};
    }
    case (MetaDataInterface::ValueType::Count):
    {
      break;
    }
  }

  throw std::runtime_error("unknown metadata value type");
}

template <typename ClockT>
py::class_<MeasurementPoint<ClockT>, std::shared_ptr<MeasurementPoint<ClockT>>>
makeMeasurementPoint(py::module& module, const std::string& prefix)
{
  return py::class_<MeasurementPoint<ClockT>,
                    FixedMeasurementPoint,
                    std::shared_ptr<MeasurementPoint<ClockT>>>(module,
                                                               (prefix + "MeasurementPoint").c_str())
    .def_property_readonly("hw_timestamp", &MeasurementPoint<ClockT>::getHWTimestamp)
    .def_property_readonly("acquisition_timestamp", &MeasurementPoint<ClockT>::getAcquisitionTimestamp)
    .def_property_readonly("source_acquisition_timestamp",
                           &MeasurementPoint<ClockT>::getSourceAcquisitionTimestamp);
}

template <typename Class, typename... Others>
py::class_<Class, Others...>& addCovarianceProperty(py::class_<Class, Others...>& pyClass)
{
  return pyClass.def_property_readonly(
    "covariance",
    [](const Class& obj) -> std::optional<const Matrix<double, 6, 6>*>
    {
      if (obj.haveCovarianceMatrix())
      {
        return {&obj.getCovariance()};
      }

      return std::nullopt;
    });
}

template <typename ClockT>
py::class_<PoseInterface<ClockT>, MeasurementPoint<ClockT>, std::shared_ptr<PoseInterface<ClockT>>>
makePose(py::module& module, const std::string& prefix)
{
  auto pose =
    py::class_<PoseInterface<ClockT>, MeasurementPoint<ClockT>, std::shared_ptr<PoseInterface<ClockT>>>(
      module, (prefix + "Pose").c_str(), py::multiple_inheritance())
      .def_property_readonly("rotation", &PoseInterface<ClockT>::getRotation)
      .def_property_readonly("translation", &PoseInterface<ClockT>::getTranslation)
      .def_property_readonly("child_reference_frame", &PoseInterface<ClockT>::getChildReferenceFrame);

  return addCovarianceProperty(pose);
}

template <typename Class, typename... Others>
py::class_<Class, Others...>& addSequenceMethods(py::class_<Class, Others...>& pyClass)
{
  return pyClass.def("__len__", &Class::size)
    .def(
      "__getitem__",
      [](Class& obj, size_t idx) -> decltype(auto) { return obj[idx]; },
      py::return_value_policy::reference_internal)
    .def(
      "__iter__", [](Class& obj) { return py::make_iterator(obj); }, py::keep_alive<0, 1>());
}

} // namespace detail

void objects(pybind11::module& module)
{
  auto coreModule = module.def_submodule("core");

  py::class_<ErrorCodeInterface, std::shared_ptr<ErrorCodeInterface>>(coreModule, "ErrorCode")
    .def_property_readonly("value", &ErrorCodeInterface::getValue);

  py::class_<FrameSyncInterface, std::shared_ptr<FrameSyncInterface>>(coreModule, "FrameSync");

  py::class_<FixedMeasurementPoint, std::shared_ptr<FixedMeasurementPoint>>(coreModule,
                                                                            "FixedMeasurementPoint")
    .def_property_readonly("sensor_id", &FixedMeasurementPoint::getSensorID)
    .def_property_readonly("reference_frame", &FixedMeasurementPoint::getReferenceFrame);

  detail::makeMeasurementPoint<camera_clock>(coreModule, "Camera");
  detail::makeMeasurementPoint<odometry_clock>(coreModule, "Odometry");
  detail::makeMeasurementPoint<imu_clock>(coreModule, "IMU");

  auto staticPose =
    py::class_<StaticPoseInterface, FixedMeasurementPoint, std::shared_ptr<StaticPoseInterface>>(
      coreModule, "StaticPose", py::multiple_inheritance())
      .def_property_readonly("rotation", &StaticPoseInterface::getRotation)
      .def_property_readonly("translation", &StaticPoseInterface::getTranslation)
      .def_property_readonly("child_reference_frame", &StaticPoseInterface::getChildReferenceFrame);
  detail::addCovarianceProperty(staticPose);

  detail::makePose<camera_clock>(coreModule, "Camera");
  detail::makePose<odometry_clock>(coreModule, "Odometry");

  auto poseList =
    py::class_<PoseListInterface<camera_clock>, std::shared_ptr<PoseListInterface<camera_clock>>>(
      coreModule, "PoseList");
  detail::addSequenceMethods(poseList);

  py::class_<IMUSensorDataInterface, MeasurementPoint<imu_clock>, std::shared_ptr<IMUSensorDataInterface>>(
    coreModule, "IMUSensorData", py::multiple_inheritance())
    .def_property_readonly("temperature", &IMUSensorDataInterface::getTemperature)
    .def_property_readonly("measurement",
                           &IMUSensorDataInterface::getMeasurement,
                           py::return_value_policy::reference_internal);

  auto imuList = py::class_<IMUListInterface, std::shared_ptr<IMUListInterface>>(coreModule,
                                                                                 "IMUList");
  detail::addSequenceMethods(imuList);

  auto velocity = py::class_<VelocityInterface<camera_clock>,
                             MeasurementPoint<camera_clock>,
                             std::shared_ptr<VelocityInterface<camera_clock>>>(
                    coreModule, "Velocity", py::multiple_inheritance())
                    .def_property_readonly("linear",
                                           &VelocityInterface<camera_clock>::getLinear,
                                           py::return_value_policy::reference_internal)
                    .def_property_readonly("angular",
                                           &VelocityInterface<camera_clock>::getAngular,
                                           py::return_value_policy::reference_internal);
  detail::addCovarianceProperty(velocity);

  py::class_<ImageInterface, MeasurementPoint<camera_clock>, std::shared_ptr<ImageInterface>>(
    coreModule, "Image", py::buffer_protocol(), py::multiple_inheritance())
    .def_property_readonly("width", &ImageInterface::getWidth)
    .def_property_readonly("height", &ImageInterface::getHeight)
    .def_property_readonly("pitch", &ImageInterface::getPitch, "in bytes")
    .def_buffer(
      [](ImageInterface& image) -> py::buffer_info
      {
        const auto bytesPerPixel = image.getBytesPerPixel();
        return py::buffer_info(
          const_cast<uint8_t*>(image.getData()),    /* Pointer to buffer */
          bytesPerPixel,                            /* Size of one scalar */
          detail::toNumpyFormat(image.getFormat()), /* Python struct-style format descriptor */
          2,                                        /* Number of dimensions */
          {image.getHeight(), image.getWidth()},    /* Buffer dimensions */
          {image.getPitch(), bytesPerPixel},        /* Strides (in bytes) for each index */
          true                                      /* readonly */
        );
      });

  py::class_<FrameInterface, std::shared_ptr<FrameInterface>>(coreModule, "Frame")
    .def("image", &FrameInterface::image, py::return_value_policy::reference_internal)
    .def("image_roi", &FrameInterface::imageROI);

  auto multiFrame =
    py::class_<MultiFrameInterface, std::shared_ptr<MultiFrameInterface>>(coreModule, "MultiFrame")
      .def_property_readonly("id", &MultiFrameInterface::id)
      .def("is_keyframe", &MultiFrameInterface::isKeyFrame);
  detail::addSequenceMethods(multiFrame);

  py::class_<LandmarkInterface, std::shared_ptr<LandmarkInterface>>(coreModule, "Landmark")
    .def_property_readonly("id", &LandmarkInterface::getID)
    .def_property_readonly("position", &LandmarkInterface::getPosition)
    .def_property_readonly("is_global", &LandmarkInterface::isGlobal);

  auto sparseMap =
    py::class_<SparseMapInterface, MeasurementPoint<camera_clock>, std::shared_ptr<SparseMapInterface>>(
      coreModule, "SparseMap", py::multiple_inheritance());

  py::enum_<SparseMapInterface::MapType>(sparseMap, "MapType")
    .value("Active", SparseMapInterface::MapType::Active)
    .value("Global", SparseMapInterface::MapType::Global);

  sparseMap.def_property_readonly("type", &SparseMapInterface::getMapType);
  detail::addSequenceMethods(sparseMap);

  py::enum_<MetaDataID>(coreModule, "MetaDataId")
    .value("NumFeatures", MetaDataID::NumFeatures)
    .value("TrackedFeatures", MetaDataID::TrackedFeatures)
    .value("DistanceTravelled", MetaDataID::DistanceTravelled)
    .value("ProcessedFrameRate", MetaDataID::ProcessedFrameRate)
    .value("TotalDroppedFrames", MetaDataID::TotalDroppedFrames);

  py::class_<MetaDataInterface, std::shared_ptr<MetaDataInterface>>(coreModule, "MetaData")
    .def_property_readonly("id", &MetaDataInterface::getID)
    .def_property_readonly("value", &detail::getMetadataValue)
    .def_property_readonly("index", &MetaDataInterface::getIndex);

  py::enum_<TaskType>(coreModule, "TaskType").value("SaveSession", TaskType::SaveSession);

  auto taskStatusObj =
    py::class_<TaskStatusInterface, std::shared_ptr<TaskStatusInterface>>(coreModule, "TaskStatus");

  py::enum_<TaskStatusInterface::TaskState>(taskStatusObj, "State")
    .value("Idle", TaskStatusInterface::TaskState::Idle)
    .value("Progress", TaskStatusInterface::TaskState::Progress)
    .value("Success", TaskStatusInterface::TaskState::Success)
    .value("Cancelled", TaskStatusInterface::TaskState::Cancelled)
    .value("Error", TaskStatusInterface::TaskState::Error);

  taskStatusObj.def_property_readonly("type", &TaskStatusInterface::getType)
    .def_property_readonly("id", &TaskStatusInterface::getID)
    .def_property_readonly("state", &TaskStatusInterface::getState)
    .def("get_error", &TaskStatusInterface::getError);

  py::enum_<MapChannelType>(coreModule, "MapChannelType")
    .value("Occupancy_Probability", MapChannelType::OccupancyProbability)
    .value("Height", MapChannelType::Height)
    .value("Height_Variance", MapChannelType::HeightVariance);

  py::class_<MapChannel2DInterface>(coreModule, "MapChannel2D", py::buffer_protocol())
    .def_property_readonly("type", &MapChannel2DInterface::getType)
    .def_property_readonly("format", &MapChannel2DInterface::getFormat)
    .def_property_readonly("width", &MapChannel2DInterface::getWidth)
    .def_property_readonly("height", &MapChannel2DInterface::getHeight)
    .def_property_readonly("pitch", &MapChannel2DInterface::getPitch, "in bytes")
    .def_property_readonly("min_value", &MapChannel2DInterface::getValueMin)
    .def_property_readonly("max_value", &MapChannel2DInterface::getValueMax)
    .def_buffer(
      [](MapChannel2DInterface& map) -> py::buffer_info
      {
        const auto bytesPerPixel = map.getBytesPerPixel();
        return py::buffer_info(
          const_cast<uint8_t*>(map.getData()),    /* Pointer to buffer */
          bytesPerPixel,                          /* Size of one scalar */
          detail::toNumpyFormat(map.getFormat()), /* Python struct-style format descriptor */
          2,                                      /* Number of dimensions */
          {map.getHeight(), map.getWidth()},      /* Buffer dimensions */
          {map.getPitch(), bytesPerPixel},        /* Strides (in bytes) for each index */
          true                                    /* readonly */
        );
      });

  py::class_<Map2DInterface, MeasurementPoint<camera_clock>, std::shared_ptr<Map2DInterface>>(
    coreModule, "Map2D", py::multiple_inheritance())
    .def("has_channel",
         py::overload_cast<MapChannelType>(&Map2DInterface::hasChannel, py::const_),
         py::arg("channel"))
    .def("has_channel",
         py::overload_cast<MapChannelType, size_t>(&Map2DInterface::hasChannel, py::const_),
         py::arg("channel"),
         py::arg("group"))
    .def("get_channel",
         py::overload_cast<MapChannelType>(&Map2DInterface::getChannel, py::const_),
         py::arg("channel"),
         py::return_value_policy::reference_internal)
    .def("get_channel",
         py::overload_cast<MapChannelType, size_t>(&Map2DInterface::getChannel, py::const_),
         py::arg("channel"),
         py::arg("group"),
         py::return_value_policy::reference_internal)
    .def_property_readonly("num_groups", &Map2DInterface::numGroups)
    .def_property_readonly("cell_size", &Map2DInterface::getCellSize)
    .def_property_readonly("width", &Map2DInterface::getWidth)
    .def_property_readonly("height", &Map2DInterface::getHeight)
    .def_property_readonly("min_bounds", &Map2DInterface::getBoundsMin)
    .def_property_readonly("map_id", &Map2DInterface::getMapId);

  auto pointCloud =
    py::class_<PointCloudInterface, MeasurementPoint<camera_clock>, std::shared_ptr<PointCloudInterface>>(
      coreModule, "PointCloud", py::buffer_protocol(), py::multiple_inheritance());

  pointCloud.def("__len__", &PointCloudInterface::size)
    .def(
      "__getitem__",
      [](const PointCloudInterface& obj, int index) { return obj.positions() + index; },
      py::return_value_policy::reference_internal)
    .def(
      "__iter__",
      [](PointCloudInterface& obj)
      { return py::make_iterator(obj.positions(), obj.positions() + obj.size()); },
      py::keep_alive<0, 1>())
    .def_buffer(
      [](PointCloudInterface& cloud)
      {
        return py::buffer_info(
          const_cast<PointCloudInterface::Point*>(cloud.positions()), /* Pointer to buffer */
          sizeof(PointCloudInterface::Point),                         /* Size of one scalar */
          py::format_descriptor<PointCloudInterface::Point>::format(), /* Python struct-style format descriptor */
          1,                                                           /* Number of dimensions */
          {cloud.size()},                                              /* Buffer dimensions */
          {sizeof(PointCloudInterface::Point)}, /* Strides (in bytes) for each index */
          true                                  /* readonly */
        );
      });

  auto trackingStatusList =
    py::class_<TrackingStatusListInterface, std::shared_ptr<TrackingStatusListInterface>>(
      coreModule, "TrackingStatusList");
  detail::addSequenceMethods(trackingStatusList);

  auto slamEventList = py::class_<SLAMEventListInterface, std::shared_ptr<SLAMEventListInterface>>(
    coreModule, "SLAMEventList");
  detail::addSequenceMethods(slamEventList);

  py::class_<SLAMStatusInterface, MeasurementPoint<camera_clock>, std::shared_ptr<SLAMStatusInterface>>(
    coreModule, "SLAMStatus", py::multiple_inheritance())
    .def_property_readonly("tracking_status", &SLAMStatusInterface::getTrackingStatus)
    .def_property_readonly("events",
                           &SLAMStatusInterface::getEvents,
                           py::return_value_policy::reference_internal);

  py::enum_<PredefinedSemanticLabel>(coreModule, "PredefinedSemanticLabel")
    .value("Person", PredefinedSemanticLabel::Person)
    .value("Background", PredefinedSemanticLabel::Background)
    .value("Unknown", PredefinedSemanticLabel::Unknown);

  py::class_<SemanticLabel, std::shared_ptr<SemanticLabel>>(coreModule, "SemanticLabel")
    .def(py::init<PredefinedSemanticLabel>())
    .def(py::init<std::string>())
    .def_property_readonly("name", &SemanticLabel::name)
    .def(py::self == py::self)
    .def(py::self != py::self)
    .def(py::self < py::self);

  py::class_<PanopticLabel, std::shared_ptr<PanopticLabel>>(coreModule, "PanopticLabel")
    .def(py::init<SemanticLabel, bool>())
    .def_readwrite("label", &PanopticLabel::label)
    .def_readwrite("isThing", &PanopticLabel::isThing);

  py::class_<PanopticBoundingBox3D, MeasurementPoint<camera_clock>, std::shared_ptr<PanopticBoundingBox3D>>(
    coreModule, "PanopticBoundingBox3D")
    .def(py::init<>())
    .def_property_readonly("bbox", &PanopticBoundingBox3D::getBBox)
    .def_property_readonly("label", &PanopticBoundingBox3D::getLabel)
    .def_property_readonly("instance", &PanopticBoundingBox3D::getInstance);

  auto panopticBoundingBox3DList =
    py::class_<PanopticBoundingBox3DList, std::shared_ptr<PanopticBoundingBox3DList>>(
      coreModule, "PanopticBoundingBox3DList");
  detail::addSequenceMethods(panopticBoundingBox3DList);

  py::class_<PanopticSegmentationResultInterface, std::shared_ptr<PanopticSegmentationResultInterface>>(
    coreModule, "PanopticSegmentationResult")
    .def_property_readonly("pixel_labels", &PanopticSegmentationResultInterface::getPixelLabelIds)
    .def_property_readonly("pixel_instances",
                           &PanopticSegmentationResultInterface::getPixelInstanceIds);
}

} // namespace slamcore::python
