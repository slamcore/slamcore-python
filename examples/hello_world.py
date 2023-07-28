#!/usr/bin/env python3

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

import argparse
import datetime
import logging
import numpy as np
import slamcore
from pathlib import Path


def pose_callback(pose: slamcore.CameraPose):
    print(f"Received: Pose\n\t {pose.translation}")


def smooth_pose_callback(pose: slamcore.CameraPose):
    print(f"Received: SmoothPose\n\t {pose.translation}")


def slam_status_callback(status: slamcore.SLAMStatus):
    print(f"Received: SLAMStatus\n\t {status.tracking_status}")


def video_callback(multi_frame: slamcore.MultiFrame):
    kfStr = "KF" if multi_frame.is_keyframe else "NON-KF"
    print(f"Received: MultiFrame\n\t {len(multi_frame)} {kfStr}")
    for frame in multi_frame:
        image = frame.image()
        print(
            f"\t\t{image.width} x {image.height} at {image.hw_timestamp.total_seconds()} [s]"
        )


def imu_callback(imu_list: slamcore.IMUList):
    print("Received: IMUList")
    for measurement in imu_list:
        print(f"\tTimestamp: {measurement.hw_timestamp.total_seconds()} [s]")
        print(f"\t{measurement.sensor_id[0].name}: {measurement.measurement}")


def velocity_callback(velocity: slamcore.Velocity):
    print("Received: Velocity")
    print(f"\tLinear: {velocity.linear}\n\tAngular: {velocity.angular}")


def active_map_callback(active_map: slamcore.SparseMap):
    print("Received: SparseMap")
    print(f"\tLandmarks: {len(active_map)}")


def meta_data_callback(metadata: slamcore.MetaData):
    print(f"Received: MetaData")
    print(f"\t{metadata.id.name}: {metadata.value}")


def framesync_callback(_: slamcore.FrameSync):
    print("Received: FrameSync")


def local_point_cloud_callback(point_cloud: slamcore.PointCloud):
    print("Received: LocalPointCloud")
    print(f"\tNumber of Points: {len(point_cloud)}")

def panoptic_bounding_box_callback(bbox_list: slamcore.PanopticBoundingBox3DList):
    print("Received: PanopticBoundingBox3DList")
    for bbox in bbox_list:
        print(f"\tLabel: {bbox.label.name}")
        print(f"\tInstance: {bbox.instance}")
        print(f"\tSize: {bbox.bbox.size}")
        print(f"\tLocation: {bbox.bbox.center_pose.translation}")

def panoptic_segmentation_result_callback(segmentation: slamcore.PanopticSegmentationResult):
    print("Received: PanopticSegmentationResult")
    print(f"\tLabels: {segmentation.pixel_labels.width} x {segmentation.pixel_labels.height}")
    print(f"\tInstances: {segmentation.pixel_instances.width} x {segmentation.pixel_instances.height}")


def log_callback(msg: slamcore.logging.LogMessage):
    severity_conversion = {
        slamcore.logging.LogSeverity.Info: logging.INFO,
        slamcore.logging.LogSeverity.Warning: logging.WARNING,
        slamcore.logging.LogSeverity.Error: logging.ERROR,
        slamcore.logging.LogSeverity.Fatal: logging.FATAL,
    }

    logging.log(severity_conversion[msg.severity], msg.message)


def main(dataset: Path, config: Path = None, enable_panoptic_plugin: bool = False) -> None:
    slamcore.init(severity=slamcore.logging.LogSeverity.Info, callback=log_callback)

    try:
        cfg = slamcore.SystemConfiguration()
        if dataset is not None:
            cfg.sources.append(slamcore.DataSource.Dataset)
            cfg.dataset_path = str(dataset)
        else:
            cfg.sources.append(slamcore.DataSource.AutoDetect)
            if enable_panoptic_plugin:
                cfg.sensor_enable_map = {
                    (slamcore.SensorType.Visible, 0) : True,
                    (slamcore.SensorType.Depth, 0)   : True,
                }

        if config is not None:
            cfg.config_file_path = str(config)

        slam_system = slamcore.create_slam_system(cfg)

        if slam_system is None:
            logging.error("Failed to create SLAM system!")
            exit(-1)

        slam_system.open()

        finished = False

        logging.info(
            f"Client Version: {slamcore.get_version()} / {slamcore.get_build_version()} / {slamcore.get_build_type()}"
        )
        logging.info(
            f"SLAM Version: {slam_system.get_property(slamcore.Property.FirmwareVersion)} / {slam_system.get_property(slamcore.Property.FirmwareBuildVersion)} / {slam_system.get_property(slamcore.Property.FirmwareBuildType)}"
        )

        if slam_system.is_subsystem_supported(slamcore.subsystems.SensorsInfo):
            sensors_info_subsystem = slam_system.get_sensor_info_subsystem()
            for camera_sensor in sensors_info_subsystem.camera_list:
                print(f"Camera: {camera_sensor}")
                print(
                    f"\tSensor size: {sensors_info_subsystem.sensor_size(camera_sensor)}"
                )
                print(
                    f"\tImage format: {sensors_info_subsystem.image_format(camera_sensor)}"
                )

        if enable_panoptic_plugin:
            subsystem = slam_system.get_panoptic_segmentation_subsystem()
            subsystem.register_default_plugin()
            label_mapping = subsystem.get_runner().label_mapping
            for instance, panoptic_label in label_mapping.items():
                type_str = 'Thing' if panoptic_label.isThing else 'Stuff'
                print(f"{instance}: {panoptic_label.label.name} ({type_str})")

        # Uncomment the following if you want to use low latency mode!
        # slam_system.set_property(slamcore.Property.LowLatencyMode, True)

        slam_system.set_stream_enabled(slamcore.Stream.Pose, True)
        slam_system.set_stream_enabled(slamcore.Stream.SmoothPose, True)
        slam_system.set_stream_enabled(slamcore.Stream.SLAMStatus, True)
        slam_system.set_stream_enabled(slamcore.Stream.Video, True)
        slam_system.set_stream_enabled(slamcore.Stream.IMU, True)
        slam_system.set_stream_enabled(slamcore.Stream.ActiveMap, True)
        slam_system.set_stream_enabled(slamcore.Stream.Velocity, True)
        slam_system.set_stream_enabled(slamcore.Stream.MetaData, True)
        slam_system.set_stream_enabled(slamcore.Stream.LocalPointCloud, True)
        slam_system.set_stream_enabled(slamcore.Stream.ErrorCode, True)
        slam_system.set_stream_enabled(slamcore.Stream.PanopticSegmentationResult, True)
        slam_system.set_stream_enabled(slamcore.Stream.PanopticBoundingBox3D, True)

        slam_system.register_pose_callback(pose_callback)
        slam_system.register_smooth_pose_callback(smooth_pose_callback)
        slam_system.register_slam_status_callback(slam_status_callback)
        slam_system.register_video_callback(video_callback)
        slam_system.register_imu_callback(imu_callback)
        slam_system.register_active_map_callback(active_map_callback)
        slam_system.register_velocity_callback(velocity_callback)
        slam_system.register_meta_data_callback(meta_data_callback)
        slam_system.register_local_point_cloud_callback(local_point_cloud_callback)
        slam_system.register_panoptic_segmentation_result_callback(panoptic_segmentation_result_callback)
        slam_system.register_panoptic_bounding_box_callback(panoptic_bounding_box_callback)

        def error_callback(error: slamcore.ErrorCode):
            eod_int = int(slamcore.errors.SystemErrorCode.ErrorValues.end_of_data)
            if error.value.value == eod_int:
                nonlocal finished
                finished = True

        slam_system.register_error_code_callback(error_callback)

        slam_system.start()

        while not finished:
            try:
                while slam_system.spin(datetime.timedelta(milliseconds=100)):
                    continue
                else:
                    logging.warning("timeout")
            except KeyboardInterrupt:
                break

        slam_system.stop()

        slam_system.close()

        print("We're Done Here.")

    except Exception as ex:
        logging.error(f"Exception caught: {ex}")

    slamcore.deinit()


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)

    parser = argparse.ArgumentParser(
        description="Example python script",
    )

    parser.add_argument("dataset", type=Path, nargs='?')
    parser.add_argument("--config", type=Path, required=False)
    parser.add_argument("--enable-panoptic-plugin", action="store_true", default=False)

    args = parser.parse_args()

    main(args.dataset, args.config, args.enable_panoptic_plugin)
