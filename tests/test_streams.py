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
    check_IMU_list,
    check_local_point_cloud,
    check_meta_data,
    check_multiframe,
    check_pose,
    check_smooth_pose,
    check_slam_status,
    check_sparse_map,
    check_velocity,
    create_dataset_system_config,
    error_callback,
    slamcore_init,
)


@pytest.mark.parametrize(
    "stream_type, register_function_name, test_func",
    [
        (slamcore.Stream.Pose, "register_pose_callback", check_pose),
        (slamcore.Stream.SmoothPose, "register_smooth_pose_callback", check_smooth_pose),
        (slamcore.Stream.SLAMStatus, "register_slam_status_callback", check_slam_status),
        (slamcore.Stream.Video, "register_video_callback", check_multiframe),
        (slamcore.Stream.IMU, "register_imu_callback", check_IMU_list),
        (
            slamcore.Stream.ActiveMap,
            "register_active_map_callback",
            check_sparse_map,
        ),
        (
            slamcore.Stream.Velocity,
            "register_velocity_callback",
            check_velocity,
        ),
        (slamcore.Stream.MetaData, "register_meta_data_callback", check_meta_data),
        (
            slamcore.Stream.LocalPointCloud,
            "register_local_point_cloud_callback",
            check_local_point_cloud,
        ),
    ],
)
def test_stream(
    dataset: str,
    config: str,
    slamcore_init,
    stream_type: slamcore.Stream,
    register_function_name: str,
    test_func: typing.Callable,
) -> None:
    """
    Parametrized test to verify SLAM is publishing to all the streams.

    :param dataset: Path to the dataset to use
    :param config: Path to the config to use
    :param stream_type: Type of the stream to test
    :param register_function_name: Name of the function to use to register the
        callback
    :param test_func: This is the function that will perform the test on the
        received object(s) from the stream
    """
    cfg = create_dataset_system_config(dataset, config)
    slam_system = slamcore.create_slam_system(cfg)

    assert slam_system is not None

    slam_system.open()

    end_of_data = EndOfDataToken()
    slam_system.set_stream_enabled(slamcore.Stream.ErrorCode, True)
    slam_system.register_error_code_callback(
        functools.partial(error_callback, end_of_data)
    )

    # Enable and register to stream to test
    slam_system.set_stream_enabled(stream_type, True)
    getattr(slam_system, register_function_name)(test_func)

    slam_system.start()

    while not end_of_data:
        while slam_system.spin(datetime.timedelta(milliseconds=100)):
            continue
        else:
            logging.warning("timeout")

    slam_system.stop()

    slam_system.close()
