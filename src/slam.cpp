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

#include "slamcore/slam/slam_create.hpp"
#include "slamcore/slam/system_configuration.hpp"
#include "slamcore/subsystems/height_mapping.hpp"
#include "slamcore/subsystems/optimised_trajectory.hpp"
#include "slamcore/subsystems/sensors_info.hpp"

#include <pybind11/chrono.h>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>

#include <mutex>

namespace py = pybind11;

PYBIND11_MAKE_OPAQUE(std::vector<slamcore::DataSource>); // to get direct access w/o copy

namespace slamcore::python
{

namespace detail
{

void slamcoreInit(LogSeverity severity, std::function<void(const LogMessageInterface*)> pyCallback)
{
  auto cppCallback = [pyCallback](const LogMessageInterface& message)
  {
    pyCallback(&message);
  };

  ::slamcore::slamcoreInit(severity, cppCallback);
}

std::unique_ptr<SLAMSystemCallbackInterface> createSLAMSystem(const v0::SystemConfiguration& cfg)
{
  return ::slamcore::createSLAMSystem(cfg);
}

std::variant<std::string, PositioningMode, bool, SessionIDT>
getProperty(const PropertiesInterface<Property>& obj, Property property, size_t idx)
{
  switch (property)
  {
    case (Property::ModelName):
    case (Property::SerialNumber):
    case (Property::SensorInfo):
    case (Property::FirmwareVersion):
    case (Property::FirmwareBuildTime):
    case (Property::FirmwareBuildVersion):
    case (Property::FirmwareBuildType):
    case (Property::ConfigPresetName):
    {
      return obj.getProperty<std::string>(property, idx);
    }
    case (Property::PositioningMode):
    {
      return obj.getProperty<PositioningMode>(property, idx);
    }
    case (Property::FeatureMap2DEnabled):
    case (Property::FeatureCoverage2DEnabled):
    case (Property::FeatureKinematicsEnabled):
    case (Property::LowLatencyMode):
    {
      return obj.getProperty<bool>(property, idx);
    }
    case (Property::SessionID):
    {
      return obj.getProperty<SessionIDT>(property, idx);
    }
    case (Property::Count):
    {
      break;
    }
  }

  throw std::runtime_error("unknown property");
}

void setProperty(PropertiesInterface<Property>& obj,
                 Property property,
                 const std::variant<std::string, PositioningMode, bool>& variant,
                 size_t idx)
{
  std::visit([&](const auto& value) { obj.setProperty(property, value, idx); }, variant);
}

} // namespace detail

void slam(py::module& module)
{
  auto coreModule = module.def_submodule("core");
  auto slamModule = module.def_submodule("slam");

  py::enum_<Stream>(coreModule, "Stream")
    .value("Pose", Stream::Pose)
    .value("Video", Stream::Video)
    .value("IMU", Stream::IMU)
    .value("ActiveMap", Stream::ActiveMap)
    .value("Velocity", Stream::Velocity)
    .value("MetaData", Stream::MetaData)
    .value("FrameSync", Stream::FrameSync)
    .value("ErrorCode", Stream::ErrorCode)
    .value("LocalPointCloud", Stream::LocalPointCloud);

  py::enum_<Property>(coreModule, "Property")
    .value("ModelName", Property::ModelName)
    .value("SerialNumber", Property::SerialNumber)
    .value("SensorInfo", Property::SensorInfo)
    .value("FirmwareVersion", Property::FirmwareVersion)
    .value("FirmwareBuildTime", Property::FirmwareBuildTime)
    .value("FirmwareBuildVersion", Property::FirmwareBuildVersion)
    .value("FirmwareBuildType", Property::FirmwareBuildType)
    .value("PositioningMode", Property::PositioningMode)
    .value("ConfigPresetName", Property::ConfigPresetName)
    .value("FeatureMap2DEnabled", Property::FeatureMap2DEnabled)
    .value("FeatureCoverage2DEnabled", Property::FeatureCoverage2DEnabled)
    .value("FeatureKinematicsEnabled", Property::FeatureKinematicsEnabled)
    .value("LowLatencyMode", Property::LowLatencyMode)
    .value("SessionID", Property::SessionID);

  py::class_<PropertiesInterface<Property>>(slamModule, "PropertiesInterface")
    .def("is_property_supported",
         &PropertiesInterface<Property>::isPropertySupported,
         py::arg("property"),
         "Check if we support a particular property")
    .def("is_property_read_only",
         &PropertiesInterface<Property>::isPropertyReadOnly,
         py::arg("property"),
         py::arg("index") = 0)
    .def("get_property",
         &detail::getProperty,
         py::arg("property"),
         py::arg("index") = 0,
         "Get the value of a property, with a optional index for array-like "
         "properties")
    .def("set_property",
         &detail::setProperty,
         py::arg("property"),
         py::arg("value"),
         py::arg("index") = 0,
         "Set the value of a property, with a optional index for array-like "
         "properties");

  py::class_<SLAMCoreInterface>(slamModule, "SLAMCoreInterface")
    .def("open", &SLAMCoreInterface::open, py::arg("name") = nullptr)
    .def("open_with_session",
         &SLAMCoreInterface::openWithSession,
         py::arg("path"),
         py::arg("name") = nullptr)
    .def("close", &SLAMCoreInterface::close)
    .def("is_open", &SLAMCoreInterface::isOpen)
    .def("start", &SLAMCoreInterface::start)
    .def("stop", &SLAMCoreInterface::stop)
    .def("is_running", &SLAMCoreInterface::isRunning)
    .def("is_stream_enabled", &SLAMCoreInterface::isStreamEnabled, py::arg("stream"))
    .def("set_stream_enabled",
         &SLAMCoreInterface::setStreamEnabled,
         py::arg("stream"),
         py::arg("value"));

  py::class_<SLAMSubsystemAccessInterface>(slamModule, "SLAMSubsystemAccessInterface")
    .def(
      "is_subsystem_supported",
      [](SLAMSubsystemAccessInterface& system, const py::handle& type)
      {
        if (type.is(py::type::handle_of<SensorsInfoInterface>()))
        {
          return system.isSubsystemSupported<SensorsInfoInterface>();
        }
        else if (type.is(py::type::handle_of<HeightMappingSubsystemInterface>()))
        {
          return system.isSubsystemSupported<HeightMappingSubsystemInterface>();
        }
        else if (type.is(py::type::handle_of<OptimisedTrajectorySubsystemInterface>()))
        {
          return system.isSubsystemSupported<OptimisedTrajectorySubsystemInterface>();
        }

        throw std::runtime_error("type argument was not recognised!");
      },
      py::arg("type"))
    .def(
      "get_sensor_info_subsystem",
      [](SLAMSubsystemAccessInterface& system)
      { return system.getSubsystem<SensorsInfoInterface>(); },
      py::return_value_policy::reference_internal)
    .def(
      "get_mapping_subsystem",
      [](SLAMSubsystemAccessInterface& system)
      { return system.getSubsystem<HeightMappingSubsystemInterface>(); },
      py::return_value_policy::reference_internal)
    .def(
      "get_trajectory_subsystem",
      [](SLAMSubsystemAccessInterface& system)
      { return system.getSubsystem<OptimisedTrajectorySubsystemInterface>(); },
      py::return_value_policy::reference_internal);

  py::class_<SLAMAsyncTasksInterface>(slamModule, "SLAMAsyncTasksInterface")
    .def("launch_async_task",
         &SLAMAsyncTasksInterface::launchAsyncTask,
         py::arg("type"),
         py::arg("params"))
    .def("cancel_async_task", &SLAMAsyncTasksInterface::cancelAsyncTask, py::arg("type"), py::arg("id"))
    .def("get_task_status", &SLAMAsyncTasksInterface::getTaskStatus, py::arg("type"), py::arg("id"));

  py::class_<SLAMSystemCallbackInterface,
             SLAMCoreInterface,
             SLAMSubsystemAccessInterface,
             SLAMAsyncTasksInterface,
             PropertiesInterface<Property>>(coreModule, "SLAMSystem")
    .def(
      "spin_once",
      [](SLAMSystemCallbackInterface& obj) { return obj.spinOnce(); },
      "spin once indefinitely",
      py::call_guard<py::gil_scoped_release>())
    .def(
      "spin_once",
      [](SLAMSystemCallbackInterface& obj, std::chrono::nanoseconds timeout)
      { return obj.spinOnce(timeout); },
      py::arg("timeout"),
      "spin once with timeout",
      py::call_guard<py::gil_scoped_release>())
    .def(
      "spin_some",
      [](SLAMSystemCallbackInterface& obj) { return obj.spinSome(); },
      "spin all events since last call",
      py::call_guard<py::gil_scoped_release>())
    .def(
      "spin_some",
      [](SLAMSystemCallbackInterface& obj, std::chrono::nanoseconds duration)
      { return obj.spinSome(duration); },
      py::arg("duration"),
      "spin events since last call until duration reached",
      py::call_guard<py::gil_scoped_release>())
    .def("register_pose_callback",
         &SLAMSystemCallbackInterface::registerCallback<Stream::Pose>,
         py::arg("callback").none(false))
    .def("register_video_callback",
         &SLAMSystemCallbackInterface::registerCallback<Stream::Video>,
         py::arg("callback").none(false))
    .def("register_imu_callback",
         &SLAMSystemCallbackInterface::registerCallback<Stream::IMU>,
         py::arg("callback").none(false))
    .def("register_active_map_callback",
         &SLAMSystemCallbackInterface::registerCallback<Stream::ActiveMap>,
         py::arg("callback").none(false))
    .def("register_velocity_callback",
         &SLAMSystemCallbackInterface::registerCallback<Stream::Velocity>,
         py::arg("callback").none(false))
    .def("register_meta_data_callback",
         &SLAMSystemCallbackInterface::registerCallback<Stream::MetaData>,
         py::arg("callback").none(false))
    .def("register_frame_sync_callback",
         &SLAMSystemCallbackInterface::registerCallback<Stream::FrameSync>,
         py::arg("callback").none(false))
    .def("register_error_code_callback",
         &SLAMSystemCallbackInterface::registerCallback<Stream::ErrorCode>,
         py::arg("callback").none(false))
    .def("register_local_point_cloud_callback",
         &SLAMSystemCallbackInterface::registerCallback<Stream::LocalPointCloud>,
         py::arg("callback").none(false));

  py::enum_<DataSource>(coreModule, "DataSource")
    .value("Dataset", DataSource::Dataset)
    .value("RealSense", DataSource::RealSense)
    .value("OakD", DataSource::OakD)
    .value("AutoDetect", DataSource::AutoDetect);

  py::bind_vector<std::vector<DataSource>>(coreModule, "VectorDataSource");
  
  v0::SystemConfiguration defaultConfigValues;
  py::class_<v0::SystemConfiguration>(coreModule, "SystemConfiguration")
    .def(py::init<std::vector<DataSource>,   // Sources
                  std::string,               // DatasetPath
                  double,                    // DatasetTimeScale
                  int,                       // DeviceID
                  std::string,               // ConfigFilePath
                  bool,                      // DisableSLAM
                  PositioningMode,           // Mode
                  std::string,               // LoadSessionFilePath
                  bool,                      // GenerateMap
                  std::map<SensorIDT, bool>, // SensorEnableMap
                  std::string,               // DatasetWritePath
                  std::string>(),            // ClientAppVersion
         py::kw_only(),                      //
         py::arg("sources") = defaultConfigValues.Sources,
         py::arg("dataset_path") = defaultConfigValues.DatasetPath,
         py::arg("dataset_time_scale") = defaultConfigValues.DatasetTimeScale,
         py::arg("device_id") = defaultConfigValues.DeviceID,
         py::arg("config_file_path") = defaultConfigValues.ConfigFilePath,
         py::arg("disable_SLAM") = defaultConfigValues.DisableSLAM,
         py::arg("mode") = defaultConfigValues.Mode,
         py::arg("load_session_file_path") = defaultConfigValues.LoadSessionFilePath,
         py::arg("generate_map") = defaultConfigValues.GenerateMap,
         py::arg("sensor_enable_map") = defaultConfigValues.SensorEnableMap,
         py::arg("dataset_write_path") = defaultConfigValues.DatasetWritePath,
         py::arg("client_app_version") = defaultConfigValues.ClientAppVersion)
    .def_readwrite("sources", &v0::SystemConfiguration::Sources, "Input data sources for SLAM")
    .def_readwrite("dataset_path",
                   &v0::SystemConfiguration::DatasetPath,
                   "Path to a dataset if DataSource::Dataset is set")
    .def_readwrite("dataset_time_scale",
                   &v0::SystemConfiguration::DatasetTimeScale,
                   "Time scale for dataset reader, for real-time processing set to 1")
    .def_readwrite("device_id", &v0::SystemConfiguration::DeviceID, "Device index for a camera device")
    .def_readwrite("config_file_path",
                   &v0::SystemConfiguration::ConfigFilePath,
                   "Path to a preset configuration file (optional)")
    .def_readwrite("disable_SLAM",
                   &v0::SystemConfiguration::DisableSLAM,
                   "Disable SLAM, just pass the data from the camera")
    .def_readwrite("mode", &v0::SystemConfiguration::Mode, "Positioning mode to use")
    .def_readwrite("load_session_file_path",
                   &v0::SystemConfiguration::LoadSessionFilePath,
                   "Session file to load (path)")
    .def_readwrite("generate_map",
                   &v0::SystemConfiguration::GenerateMap,
                   "Generate height map in the session file")
    .def_readwrite("sensor_enable_map",
                   &v0::SystemConfiguration::SensorEnableMap,
                   "Sensors to enable (e.g. depth, color, wheel odometry)")
    .def_readwrite("dataset_write_path",
                   &v0::SystemConfiguration::DatasetWritePath,
                   "Path to write dataset (if any)")
    .def_readwrite("client_app_version",
                   &v0::SystemConfiguration::ClientAppVersion,
                   "Version string of the client app");

  coreModule.def("get_version", &getVersion);
  coreModule.def("get_build_version", &getBuildVersion);
  coreModule.def("get_build_type", &getBuildType);

  coreModule.def("init",
                 &detail::slamcoreInit,
                 py::arg("severity") = LogSeverity::Warning,
                 py::arg("callback") = nullptr);
  coreModule.def("deinit", &slamcoreDeinit);

  coreModule.def("create_slam_system", &detail::createSLAMSystem, py::arg("configuration"));
}

} // namespace slamcore::python
