import argparse

import rosbag2_py
import image_handler as ih
from pathlib import Path
import cv2

# API: https://docs.ros.org/en/iron/p/rosbag2_compression/generated/classrosbag2__compression_1_1SequentialCompressionWriter.html


def write_to(in_path: Path, output_path: Path):
    compression_options = rosbag2_py.CompressionOptions()
    compression_options.compression_format = "zstd"
    compression_options.compression_mode = rosbag2_py.CompressionMode.FILE
    # compression_options.compression_queue_size = 8
    # compression_options.compression_threads = 8
    # writer = rosbag2_py.SequentialWriter()
    writer = rosbag2_py.SequentialCompressionWriter(compression_options)

    writer.open(
        rosbag2_py.StorageOptions(uri=str(output_path), storage_id="mcap"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )

    cam0_topic = "/mav0/cam0/image_mono"
    ih.create_topic(writer, id=0, topic=cam0_topic)

    cam0_dir = in_path / "mav0/cam0/data/"

    for img_path in cam0_dir.iterdir():
        if img_path.suffix == ".png":
            timestamp = int(img_path.stem)
            img = cv2.imread(str(img_path), cv2.IMREAD_GRAYSCALE)
            ih.write_message(writer, "cam0", cam0_topic, img, timestamp)

    del writer


def main():
    # parser = argparse.ArgumentParser(description=__doc__)
    # parser.add_argument("output", help="output directory to create and write to")

    # args = parser.parse_args()
    # write_to(args.output)
    in_path = Path("~/data/vslam/asl_eth/MH_01_easy/").expanduser().resolve()
    out_path = (
        Path("~/data/vslam/asl_eth/exports/MH_01_easy_ros2bag").expanduser().resolve()
    )
    out_path.parent.mkdir(parents=True, exist_ok=False)

    write_to(in_path, out_path)


if __name__ == "__main__":
    main()
