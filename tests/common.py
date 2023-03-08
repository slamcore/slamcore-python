# ******************************************************************************
#
#  Slamcore Confidential
#  ---------------------
#
#  Slamcore Limited
#  All Rights Reserved.
#  (C) Copyright 2023
#
#  NOTICE:
#
#  All information contained herein is, and remains the property of Slamcore
#  Limited and its suppliers, if any. The intellectual and technical concepts
#  contained herein are proprietary to Slamcore Limited and its suppliers and
#  may be covered by patents in process, and are protected by trade secret or
#  copyright law. Dissemination of this information or reproduction of this
#  material is strictly forbidden unless prior written permission is obtained
#  from Slamcore Limited.
#
# ******************************************************************************

import logging
import numpy
import pytest
import slamcore


def log_callback(msg: slamcore.logging.LogMessage):
    """
    To be given to the Slamcore library to log messages.
    """
    severity_conversion = {
        slamcore.logging.LogSeverity.Info: logging.INFO,
        slamcore.logging.LogSeverity.Warning: logging.WARNING,
        slamcore.logging.LogSeverity.Error: logging.ERROR,
        slamcore.logging.LogSeverity.Fatal: logging.FATAL,
    }

    logging.log(severity_conversion[msg.severity], msg.message)


@pytest.fixture
def slamcore_init():
    """
    Initializes and de-initializes the Slamcore library.
    """
    slamcore.init(severity=slamcore.logging.LogSeverity.Info, callback=log_callback)
    yield
    slamcore.deinit()


def create_dataset_system_config(dataset, config):
    """
    Helper function to create a system config to read from a dataset with the
    given config path.
    """
    cfg = slamcore.SystemConfiguration()
    cfg.sources.append(slamcore.DataSource.Dataset)
    cfg.dataset_path = dataset
    if config is not None:
        cfg.config_file_path = config
    return cfg


class EndOfDataToken:
    """
    Token that can be used to flag a dataset's end of data. It can be thought
    as a mutable boolean.
    """

    def __init__(self):
        self.__end_of_data = False

    def set_true(self):
        self.__end_of_data = True

    def __bool__(self):
        return self.__end_of_data

    def __repr__(self):
        return f"{self.__class__.__name__}({self.__end_of_data})"


def error_callback(eod: EndOfDataToken, error: slamcore.ErrorCode):
    """
    Callback on error. Fires end of data token if needed.
    """
    eod_int = int(slamcore.errors.SystemErrorCode.ErrorValues.end_of_data)
    if error.value.value == eod_int:
        eod.set_true()


def check_timestamps(measurement_point):
    """
    Exercises all the methods that provide timestamps.
    """
    assert measurement_point.hw_timestamp.total_seconds() > 0
    assert measurement_point.acquisition_timestamp.total_seconds() > 0
    assert measurement_point.source_acquisition_timestamp.total_seconds() > 0


def check_measurement_point(
    measurement_point, expected_sensor_id, expected_reference_frame
):
    """
    Exercises methods and properties of MeasurementPoint.
    """
    assert measurement_point.sensor_id == expected_sensor_id
    assert measurement_point.reference_frame.name() == expected_reference_frame.name()
    assert (
        measurement_point.reference_frame.category()
        == expected_reference_frame.category()
    )
    assert measurement_point.reference_frame.index() == expected_reference_frame.index()
    check_timestamps(measurement_point)


def check_pose(pose):
    """
    Exercises methods and properties of Pose.
    """
    expected_sensor_id = (slamcore.SensorType.SLAM, 0)
    expected_reference_frame = slamcore.ReferenceFrame(
        slamcore.ReferenceFrameCategory.World, 0
    )
    check_measurement_point(pose, expected_sensor_id, expected_reference_frame)
    assert isinstance(pose.rotation, numpy.ndarray)
    assert pose.rotation.size == 4
    assert isinstance(pose.translation, numpy.ndarray)
    assert pose.translation.size == 3
    assert pose.child_reference_frame == slamcore.ReferenceFrame(
        slamcore.ReferenceFrameCategory.Body, 0
    )
    assert pose.covariance is None


def check_multiframe(multiframe: slamcore.MultiFrame):
    """
    Exercises methods and properties of MultiFrame, Frame and Image.
    """
    assert multiframe.id.value() > -1
    assert isinstance(multiframe.is_keyframe(), bool)
    for frame in multiframe:
        assert frame.image_roi() is not None
        image = frame.image()
        sensor_type, sensor_index = image.sensor_id
        assert image.sensor_id in (
            (slamcore.SensorType.Depth, 0),
            (slamcore.SensorType.Infrared, 0),
            (slamcore.SensorType.Infrared, 1),
        )
        assert image.reference_frame in (
            slamcore.ReferenceFrame(slamcore.ReferenceFrameCategory.Camera, 1),
            slamcore.ReferenceFrame(slamcore.ReferenceFrameCategory.Camera, 2),
        )
        check_timestamps(image)
        assert image.width == 848
        assert image.height == 480
        assert image.pitch > 0


def check_IMU_list(IMU_list: slamcore.IMUList):
    """
    Exercises methods and properties of IMUList and IMUSensorData.
    """
    for IMU_sensor_data in IMU_list:
        assert IMU_sensor_data.sensor_id in (
            (slamcore.SensorType.Accelerometer, 0),
            (slamcore.SensorType.Gyroscope, 0),
        )
        assert IMU_sensor_data.reference_frame == slamcore.ReferenceFrame(
            slamcore.ReferenceFrameCategory.IMU, 0
        )
        check_timestamps(IMU_sensor_data)
        assert IMU_sensor_data.temperature is not None
        assert isinstance(IMU_sensor_data.measurement, numpy.ndarray)
        assert IMU_sensor_data.measurement.size == 3


def check_sparse_map(sparse_map: slamcore.SparseMap):
    """
    Exercises methods and properties of SparseMap and Landmark.
    """
    expected_sensor_id = (slamcore.SensorType.SLAM, 0)
    expected_reference_frame = slamcore.ReferenceFrame(
        slamcore.ReferenceFrameCategory.World, 0
    )
    check_measurement_point(sparse_map, expected_sensor_id, expected_reference_frame)
    assert sparse_map.type == slamcore.SparseMap.MapType.Active
    for landmark in sparse_map:
        assert landmark.id.value() > -1
        assert not landmark.is_global
        assert isinstance(landmark.position, numpy.ndarray)
        assert landmark.position.size == 3


def check_velocity(velocity: slamcore.Velocity):
    """
    Exercises methods and properties of Velocity.
    """
    expected_sensor_id = (slamcore.SensorType.SLAM, 0)
    expected_reference_frame = slamcore.ReferenceFrame(
        slamcore.ReferenceFrameCategory.Body, 0
    )
    check_measurement_point(velocity, expected_sensor_id, expected_reference_frame)
    assert isinstance(velocity.linear, numpy.ndarray)
    assert velocity.linear.size == 3
    assert isinstance(velocity.angular, numpy.ndarray)
    assert velocity.angular.size == 3


def check_meta_data(meta_data: slamcore.MetaData):
    """
    Exercises methods and properties of MetaData.
    """
    meta_data.id
    meta_data.value
    meta_data.index


def check_local_point_cloud(local_point_cloud: slamcore.PointCloud):
    """
    Exercises methods and properties of PointCloud and Point.
    """
    expected_sensor_id = (slamcore.SensorType.Depth, 0)
    expected_reference_frame = slamcore.ReferenceFrame(
        slamcore.ReferenceFrameCategory.Camera, 1
    )
    check_measurement_point(
        local_point_cloud, expected_sensor_id, expected_reference_frame
    )
    for point in local_point_cloud:
        assert isinstance(point, numpy.ndarray)
        assert point.size == 3


def check_trajectory_subsystem(
    trajectory_subsystem: slamcore.subsystems.OptimisedTrajectorySubsystem,
):
    """
    Exercises iteration over trajectory (PoseList) and Pose.
    """
    for pose in trajectory_subsystem.trajectory:
        check_pose(pose)


def check_mapping_subsystem(
    mapping_subsystem: slamcore.subsystems.HeightMappingSubsystem,
):
    """
    Exercises methods and properties of Map2D and MapChannel2D.
    """
    map2D = mapping_subsystem.get()
    assert map2D.num_groups > 0
    assert map2D.cell_size > 0
    assert map2D.width > 0
    assert map2D.height > 0
    assert isinstance(map2D.min_bounds, numpy.ndarray)
    assert map2D.min_bounds.size == 2
    assert map2D.map_id == 0
    for channel_type, format_ in (
        (slamcore.MapChannelType.Height, slamcore.ImageFormat.Mono_F),
        (slamcore.MapChannelType.Height_Variance, slamcore.ImageFormat.Mono_F),
        (slamcore.MapChannelType.Occupancy_Probability, slamcore.ImageFormat.Mono_8),
    ):
        assert map2D.has_channel(channel_type)
        channel = map2D.get_channel(channel_type)
        assert channel.type == channel_type
        assert channel.format == format_
        assert channel.width == map2D.width
        assert channel.height == map2D.height
        assert channel.pitch > 0
        assert channel.min_value <= channel.max_value
        buffer = numpy.array(channel)
        assert isinstance(buffer, numpy.ndarray)
