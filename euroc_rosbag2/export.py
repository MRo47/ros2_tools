import rosbag2_py
import image_handler as img_h
import imu_handler as imu_h
from pathlib import Path
from rclpy.serialization import serialize_message


def write_to(in_path: Path, output_path: Path):
    writer = rosbag2_py.SequentialWriter()

    writer.open(
        rosbag2_py.StorageOptions(uri=str(output_path), storage_id="mcap"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )

    cam0_dir = in_path / "mav0/cam0/data/"
    cam1_dir = in_path / "mav0/cam1/data/"
    imu0_file = in_path / "mav0/imu0/data.csv"

    data_maps = {
        "cam0": {"topic": "/mav0/cam0/image_mono", "frame_id": "cam0"},
        "cam1": {"topic": "/mav0/cam1/image_mono", "frame_id": "cam1"},
        "imu0": {"topic": "/mav0/imu0/imu", "frame_id": "imu0"},
    }

    img_h.create_topic(writer, 0, data_maps["cam0"]["topic"])
    img_h.create_topic(writer, 1, data_maps["cam1"]["topic"])
    imu_h.create_topic(writer, 2, data_maps["imu0"]["topic"])

    data_generators = [
        img_h.msg_generator(
            cam0_dir, data_maps["cam0"]["frame_id"], data_maps["cam0"]["topic"]
        ),
        img_h.msg_generator(
            cam1_dir, data_maps["cam1"]["frame_id"], data_maps["cam1"]["topic"]
        ),
        imu_h.msg_generator(
            imu0_file, data_maps["imu0"]["frame_id"], data_maps["imu0"]["topic"]
        ),
    ]

    for gen in data_generators:
        for topic, msg, timestamp in gen:
            writer.write(topic, serialize_message(msg), timestamp)

    del writer


def main():
    # parser = argparse.ArgumentParser(description=__doc__)
    # parser.add_argument("output", help="output directory to create and write to")

    # args = parser.parse_args()
    # write_to(args.output)
    in_path = Path("~/data/vslam/asl_eth/MH_01_easy/").expanduser().resolve()
    out_path = Path("~/data/vslam/asl_eth/exports/").expanduser().resolve()
    out_path = out_path / (in_path.stem + "_rosbag2")
    out_path.parent.mkdir(parents=True, exist_ok=True)

    write_to(in_path, out_path)


if __name__ == "__main__":
    main()
