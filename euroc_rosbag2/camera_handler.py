import rosbag2_py
from cv_bridge import CvBridge
from std_msgs.msg import Header
from utils import to_ros_timestamp
from pathlib import Path
import cv2
from sensor_msgs.msg import CameraInfo
import yaml


def create_image_topic(
    writer: rosbag2_py.SequentialWriter,
    id: int,
    topic: str,
    serialization_format="cdr",
):
    writer.create_topic(
        rosbag2_py.TopicMetadata(
            id=id,
            name=topic,
            type="sensor_msgs/msg/Image",
            serialization_format=serialization_format,
        )
    )


def create_info_topic(
    writer: rosbag2_py.SequentialWriter,
    id: int,
    topic: str,
    serialization_format="cdr",
):
    writer.create_topic(
        rosbag2_py.TopicMetadata(
            id=id,
            name=topic,
            type="sensor_msgs/msg/CameraInfo",
            serialization_format=serialization_format,
        )
    )


def msg_generator(
    cam_dir: Path,
    cam_calib_file_path: Path,
    frame_id: str,
    image_topic: str,
    info_topic: str,
    img_suffixes=(".png", ".jpg"),
):
    """
    Generator that yields ROS2 messages for images and their corresponding CameraInfo.

    Args:
        cam_dir: Path to the directory containing the images.
        cam_calib_file_path: Path to the calibration YAML file for the camera.
        frame_id: Frame ID for the camera.
        image_topic: Topic to which the image messages should be published.
        info_topic: Topic to which the CameraInfo messages should be published.
        img_suffixes: List of allowed suffixes for the image files.

    Yields:
        Tuple of topic name, ROS2 message, and timestamp.
        The timestamp is in nanoseconds as an integer.
    """
    with open(cam_calib_file_path, "r") as f:
        camera_config = yaml.safe_load(f)

    # Create a CameraInfo message
    camera_info = CameraInfo()
    camera_info.header.frame_id = frame_id
    camera_info.height = camera_config["resolution"][1]
    camera_info.width = camera_config["resolution"][0]
    camera_info.distortion_model = camera_config["distortion_model"]
    camera_info.d = camera_config["distortion_coefficients"]
    intrinsics = camera_config["intrinsics"]
    camera_info.k = [
        intrinsics[0],
        0,
        intrinsics[2],
        0,
        intrinsics[1],
        intrinsics[3],
        0,
        0,
        1,
    ]
    camera_info.r = [
        1.0,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
        0.0,
        0.0,
        1.0,
    ]

    for img_path in cam_dir.iterdir():
        if img_path.suffix in img_suffixes:
            header = Header()
            header.frame_id = frame_id
            timestamp = int(img_path.stem)
            header.stamp = to_ros_timestamp(timestamp)
            img = cv2.imread(str(img_path), cv2.IMREAD_GRAYSCALE)
            msg = CvBridge().cv2_to_imgmsg(img, header=header, encoding="passthrough")
            msg.height, msg.width = img.shape
            camera_info.header.stamp = header.stamp
            yield image_topic, msg, timestamp
            yield info_topic, camera_info, timestamp
        else:
            raise ValueError(
                f"Image found with different suffix than ones in {img_suffixes}"
            )
