#!/usr/bin/env python3
"""
Export data from euroc_rosbag2 to a rosbag2 file.

Usage:
    export.py [-h] --output OUTPUT_ROSBAG2_PATH --input INPUT_PATH_TO_EUROC_MAV_DATA
"""

import argparse
from pathlib import Path

from rclpy.serialization import serialize_message
import rosbag2_py
from tf2_msgs.msg import TFMessage

import euroc_rosbag2.camera_handler as cam_h
import euroc_rosbag2.gt_handler as gt_h
import euroc_rosbag2.imu_handler as imu_h
import euroc_rosbag2.position_handler as pos_h
from euroc_rosbag2.utils import get_frame_transform_msg


def write_to(in_path: Path, output_path: Path):
    writer = rosbag2_py.SequentialWriter()

    writer.open(
        rosbag2_py.StorageOptions(uri=str(output_path), storage_id="mcap"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )

    cam0_dir = in_path / "mav0/cam0/data/"
    cam0_config_file = in_path / "mav0/cam0/sensor.yaml"
    cam1_dir = in_path / "mav0/cam1/data/"
    cam1_config_file = in_path / "mav0/cam1/sensor.yaml"
    imu0_file = in_path / "mav0/imu0/data.csv"
    imu0_config_file = in_path / "mav0/imu0/sensor.yaml"
    pos_file = in_path / "mav0/leica0/data.csv"
    pos_config_file = in_path / "mav0/leica0/sensor.yaml"
    gt_file = in_path / "mav0/state_groundtruth_estimate0/data.csv"
    gt_config_file = in_path / "mav0/state_groundtruth_estimate0/sensor.yaml"

    gt_frame = "imu0"

    data_maps = {
        "cam0_image": {"topic": "cam0/image_mono", "frame_id": "cam0"},
        "cam0_info": {"topic": "cam0/camera_info", "frame_id": "cam0"},
        "cam1_image": {"topic": "cam1/image_mono", "frame_id": "cam1"},
        "cam1_info": {"topic": "cam1/camera_info", "frame_id": "cam1"},
        "imu0": {"topic": "imu0/imu", "frame_id": "imu0"},
        "leica0": {"topic": "leica0/pose", "frame_id": "leica0"},
        "gt_pose": {"topic": "gt/pose", "frame_id": gt_frame},
        "gt_vel": {"topic": "gt/vel", "frame_id": gt_frame},
        "gt_imu_bias": {"topic": "gt/imu_bias", "frame_id": gt_frame},
    }

    cam_h.create_image_topic(writer, 0, data_maps["cam0_image"]["topic"])
    cam_h.create_info_topic(writer, 1, data_maps["cam0_info"]["topic"])

    cam_h.create_image_topic(writer, 2, data_maps["cam1_image"]["topic"])
    cam_h.create_info_topic(writer, 3, data_maps["cam1_info"]["topic"])

    imu_h.create_topic(writer, 4, data_maps["imu0"]["topic"])
    pos_h.create_topic(writer, 5, data_maps["leica0"]["topic"])
    gt_h.create_pose_topic(writer, 6, data_maps["gt_pose"]["topic"])
    gt_h.create_twist_topic(writer, 7, data_maps["gt_vel"]["topic"])
    gt_h.create_imu_bias_topic(writer, 8, data_maps["gt_imu_bias"]["topic"])

    writer.create_topic(
        rosbag2_py.TopicMetadata(
            id=9,
            name="/tf_static",
            type="tf2_msgs/msg/TFMessage",
            serialization_format="cdr",
        )
    )

    static_tf = TFMessage()

    static_tf.transforms = [
        get_frame_transform_msg(
            imu0_config_file, "base_link", data_maps["imu0"]["frame_id"]
        ),
        get_frame_transform_msg(
            cam0_config_file, "base_link", data_maps["cam0_image"]["frame_id"]
        ),
        get_frame_transform_msg(
            cam1_config_file, "base_link", data_maps["cam1_image"]["frame_id"]
        ),
        get_frame_transform_msg(
            pos_config_file, "base_link", data_maps["leica0"]["frame_id"]
        ),
        get_frame_transform_msg(gt_config_file, "map", "base_link"),
    ]

    data_generators = [
        cam_h.msg_generator(
            cam0_dir,
            cam0_config_file,
            data_maps["cam0_image"]["frame_id"],
            data_maps["cam0_image"]["topic"],
            data_maps["cam0_info"]["topic"],
        ),
        cam_h.msg_generator(
            cam1_dir,
            cam1_config_file,
            data_maps["cam1_image"]["frame_id"],
            data_maps["cam1_image"]["topic"],
            data_maps["cam1_info"]["topic"],
        ),
        imu_h.msg_generator(
            imu0_file,
            data_maps["imu0"]["frame_id"],
            data_maps["imu0"]["topic"],
        ),
        pos_h.msg_generator(
            pos_file, data_maps["leica0"]["frame_id"], data_maps["leica0"]["topic"]
        ),
        gt_h.msg_generator(
            gt_file,
            gt_frame,
            data_maps["gt_pose"]["topic"],
            data_maps["gt_vel"]["topic"],
            data_maps["gt_imu_bias"]["topic"],
        ),
    ]

    tf_published = False

    print("Writing data...")
    for i, gen in enumerate(data_generators):
        print(f"Writing data from generator {i}...")
        for topic, msg, timestamp in gen:
            if not tf_published:
                writer.write("/tf_static", serialize_message(static_tf), timestamp)
                tf_published = True
            writer.write(topic, serialize_message(msg), timestamp)

    del writer


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "-o", "--output", required=True, help="output directory to create and write to"
    )
    parser.add_argument("-i", "--input", required=True, help="input ros2 bag file path")

    args = parser.parse_args()
    in_path = Path(args.input).expanduser().resolve()
    out_path = Path(args.output).expanduser().resolve() / (in_path.stem + "_rosbag2")
    out_path.parent.mkdir(parents=True, exist_ok=True)
    print(f"Writing to {out_path}...")
    write_to(in_path, out_path)


if __name__ == "__main__":
    main()
