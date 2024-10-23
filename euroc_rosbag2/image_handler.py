import rosbag2_py
from cv_bridge import CvBridge
from std_msgs.msg import Header
from utils import to_ros_timestamp
from pathlib import Path
import cv2


def create_topic(
    writer: rosbag2_py.SequentialWriter,
    id: int,
    topic: str,
    topic_type="sensor_msgs/msg/Image",
    serialization_format="cdr",
):
    writer.create_topic(
        rosbag2_py.TopicMetadata(
            id=id,
            name=topic,
            type=topic_type,
            serialization_format=serialization_format,
        )
    )


def msg_generator(
    cam_dir: Path, frame_id: str, topic: str, img_suffixes=(".png", ".jpg")
):
    for img_path in cam_dir.iterdir():
        if img_path.suffix in img_suffixes:
            header = Header()
            header.frame_id = frame_id
            timestamp = int(img_path.stem)
            header.stamp = to_ros_timestamp(timestamp)
            img = cv2.imread(str(img_path), cv2.IMREAD_GRAYSCALE)
            msg = CvBridge().cv2_to_imgmsg(img, header=header, encoding="passthrough")
            msg.height, msg.width = img.shape
            yield topic, msg, timestamp
        else:
            raise ValueError(
                f"Image found with different suffix than ones in {img_suffixes}"
            )
