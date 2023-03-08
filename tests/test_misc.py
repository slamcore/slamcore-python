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
import pathlib
import pytest
import slamcore
import time
from common import (
    EndOfDataToken,
    create_dataset_system_config,
    error_callback,
    slamcore_init,
)


def __process(slam_system: slamcore.SLAMSystem) -> None:
    """
    Encapsulates running SLAM.

    :param slam_system: SLAM system
    """
    end_of_data = EndOfDataToken()
    slam_system.set_stream_enabled(slamcore.Stream.ErrorCode, True)
    slam_system.register_error_code_callback(
        functools.partial(error_callback, end_of_data)
    )

    slam_system.start()

    while not end_of_data:
        while slam_system.spin_once(datetime.timedelta(milliseconds=100)):
            continue
        else:
            logging.warning("timeout")

    slam_system.stop()


def test_reset(dataset: str, config: str, slamcore_init) -> None:
    """
    Parametrized test to verify SLAM is publishing to all the streams.

    :param dataset: Path to the dataset to use
    :param config: Path to the config to use
    :param slamcore_init: Fixture to properly setup and teardown the Slamcore library
    """
    cfg = create_dataset_system_config(dataset, config)
    slam_system = slamcore.create_slam_system(cfg)

    assert slam_system is not None

    slam_system.open()
    __process(slam_system)
    slam_system.close()

    # Reset by calling `del`, which will delete the underlying C++ object
    del slam_system

    slam_system = slamcore.create_slam_system(cfg)

    assert slam_system is not None

    slam_system.open()
    __process(slam_system)
    slam_system.close()


def test_save_session(dataset: str, config: str, slamcore_init) -> None:
    """
    Test a session can be saved.

    :param dataset: Path to the dataset to use
    :param config: Path to the config to use
    :param slamcore_init: Fixture to properly setup and teardown the Slamcore library
    """
    cfg = create_dataset_system_config(dataset, config)
    slam_system = slamcore.create_slam_system(cfg)

    assert slam_system is not None

    slam_system.open()
    __process(slam_system)

    task_id = slam_system.launch_async_task(
        slamcore.core.TaskType.SaveSession, {"filename": "/tmp/foo.session"}
    )

    timeout = 120  # 2 min
    start_time = datetime.datetime.now()
    status = slam_system.get_task_status(slamcore.core.TaskType.SaveSession, task_id)
    while (
        status.state
        not in (slamcore.TaskStatus.State.Success, slamcore.TaskStatus.State.Error)
        and (datetime.datetime.now() - start_time).total_seconds() < timeout
    ):
        status = slam_system.get_task_status(
            slamcore.core.TaskType.SaveSession, task_id
        )
        time.sleep(1)

    assert status.state == slamcore.TaskStatus.State.Success

    slam_system.close()


def test_load_session(dataset: str, config: str, slamcore_init) -> None:
    """
    Test a session can be loaded.

    :param dataset: Path to the dataset to use
    :param config: Path to the config to use
    :param slamcore_init: Fixture to properly setup and teardown the Slamcore library
    """
    cfg = create_dataset_system_config(dataset, config)
    cfg.load_session_file_path = "/tmp/foo.session"
    slam_system = slamcore.create_slam_system(cfg)

    assert slam_system is not None

    slam_system.open()
    __process(slam_system)
    slam_system.close()


@pytest.mark.parametrize(
    "data_source", [slamcore.DataSource.RealSense, slamcore.DataSource.OakD]
)
def test_config_with_data_source(
    dataset: str, config: str, slamcore_init, data_source: slamcore.DataSource
) -> None:
    """
    Test we can create system configurations with the different data sources.

    :param dataset: Path to the dataset to use
    :param config: Path to the config to use
    :param slamcore_init: Fixture to properly setup and teardown the Slamcore library
    :param data_source: Type of data source to create the system configuration
    """
    cfg = slamcore.SystemConfiguration()
    cfg.sources.append(data_source)
    if config is not None:
        cfg.config_file_path = config


@pytest.mark.parametrize("disable_SLAM", [False, True])
def test_passthrough(
    dataset: str,
    config: str,
    slamcore_init,
    disable_SLAM: bool,
    tmp_path: pathlib.Path,
) -> None:
    """
    Test a passthrough dataset can be written.

    :param dataset: Path to the dataset to use
    :param config: Path to the config to use
    :param slamcore_init: Fixture to properly setup and teardown the Slamcore library
    :param disable_SLAM: Whether to disable SLAM or not
    """
    cfg = create_dataset_system_config(dataset, config)
    cfg.dataset_write_path = str(tmp_path / "dataset")
    cfg.disable_SLAM = disable_SLAM
    slam_system = slamcore.create_slam_system(cfg)

    assert slam_system is not None

    slam_system.open()
    __process(slam_system)
    slam_system.close()

    dataset_write_path = pathlib.Path(cfg.dataset_write_path)
    assert (dataset_write_path / "ir0").is_dir()
    assert (dataset_write_path / "ir1").is_dir()
    assert (dataset_write_path / "depth0").is_dir()
    assert (dataset_write_path / "imu0").is_dir()
    assert (dataset_write_path / "capture_info.json").is_file()
