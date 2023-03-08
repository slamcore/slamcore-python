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

import datetime
import functools
import logging
import pytest
import slamcore
import typing
from common import (
    EndOfDataToken,
    check_mapping_subsystem,
    check_trajectory_subsystem,
    create_dataset_system_config,
    error_callback,
    slamcore_init,
)


def test_sensors_info_subsystem(dataset: str, config: str, slamcore_init) -> None:
    """
    Test the Sensors Info subsystem and some of the date we can get from it.

    :param dataset: Path to the dataset to use
    :param config: Path to the config to use
    :param slamcore_init: Fixture to properly setup and teardown the Slamcore library
    """
    cfg = create_dataset_system_config(dataset, config)
    slam_system = slamcore.create_slam_system(cfg)

    assert slam_system is not None

    slam_system.open()

    assert slam_system.is_subsystem_supported(slamcore.subsystems.SensorsInfo)
    sensors_info_subsystem = slam_system.get_sensor_info_subsystem()

    # The sensor is the Intel RealSense D435i, coming from UCF.

    assert len(sensors_info_subsystem.camera_list) == 3
    depth_sensor, ir0_sensor, ir1_sensor = sensors_info_subsystem.camera_list

    # Depth sensor
    sensor_type, sensor_index = depth_sensor
    assert sensor_type == slamcore.core.SensorType.Depth
    assert sensor_index == 0
    image_format = sensors_info_subsystem.image_format(depth_sensor)
    assert image_format == slamcore.ImageFormat.Mono_F
    distortion_model = sensors_info_subsystem.factory_distortion_model(depth_sensor)
    assert distortion_model == "Brown Conrady"

    # Infrared0 sensor
    sensor_type, sensor_index = ir0_sensor
    assert sensor_type == slamcore.core.SensorType.Infrared
    assert sensor_index == 0
    image_format = sensors_info_subsystem.image_format(ir0_sensor)
    assert image_format == slamcore.ImageFormat.Mono_8
    distortion_model = sensors_info_subsystem.factory_distortion_model(ir0_sensor)
    assert distortion_model == "Brown Conrady"

    # Infrared1 sensor
    sensor_type, sensor_index = ir1_sensor
    assert sensor_type == slamcore.core.SensorType.Infrared
    assert sensor_index == 1
    image_format = sensors_info_subsystem.image_format(ir1_sensor)
    assert image_format == slamcore.ImageFormat.Mono_8
    distortion_model = sensors_info_subsystem.factory_distortion_model(ir1_sensor)
    assert distortion_model == "Brown Conrady"

    slam_system.close()


@pytest.mark.parametrize(
    "subsystem_type, get_subsystem_function_name, test_func",
    [
        (
            slamcore.subsystems.HeightMappingSubsystem,
            "get_mapping_subsystem",
            check_mapping_subsystem,
        ),
        (
            slamcore.subsystems.OptimisedTrajectorySubsystem,
            "get_trajectory_subsystem",
            check_trajectory_subsystem,
        ),
    ],
)
def test_subsystem(
    dataset: str,
    config: str,
    slamcore_init,
    subsystem_type: slamcore.subsystems,
    get_subsystem_function_name: str,
    test_func: typing.Callable,
) -> None:
    """
    Parametrized generic test to validate subsystems. It tests that we can fetch and
    get sensible data from them.

    :param dataset: Path to the dataset to use
    :param config: Path to the config to use
    :param slamcore_init: Fixture to properly setup and teardown the Slamcore library
    :param subsystem_type: The type of the subsystem to test
    :param get_subsystem_function_name: The specific function to be called to
        get the subsystem
    :param test_func: The function to be called to perform the test
    """
    cfg = create_dataset_system_config(dataset, config)
    cfg.generate_map = True
    slam_system = slamcore.create_slam_system(cfg)

    assert slam_system is not None

    slam_system.open()

    assert slam_system.is_subsystem_supported(subsystem_type)
    subsystem = getattr(slam_system, get_subsystem_function_name)()

    end_of_data = EndOfDataToken()
    slam_system.set_stream_enabled(slamcore.Stream.ErrorCode, True)
    slam_system.register_error_code_callback(
        functools.partial(error_callback, end_of_data)
    )

    # We need at least one stream enabled
    slam_system.set_stream_enabled(slamcore.Stream.Pose, True)
    slam_system.register_pose_callback(lambda _: None)

    slam_system.start()

    fetched_at_least_once = False

    while not end_of_data:
        while slam_system.spin_once(datetime.timedelta(milliseconds=100)):
            if subsystem.fetch(datetime.timedelta(0.1)):
                test_func(subsystem)
                fetched_at_least_once = True
            continue
        else:
            logging.warning("timeout")

    assert fetched_at_least_once

    slam_system.stop()

    slam_system.close()
