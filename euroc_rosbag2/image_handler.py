import rosbag2_py
from rclpy.serialization import serialize_message
import numpy as np
from cv_bridge import CvBridge
from rclpy.time import Time
from std_msgs.msg import Header


def create_topic(
    writer, topic: str, topic_type="sensor_msgs/msg/Image", serialization_format="cdr"
):
    writer.create_topic(
        rosbag2_py.TopicMetadata(
            name=topic, type=topic_type, serialization_format=serialization_format
        )
    )


def write_message(
    writer: rosbag2_py.SequentialCompressionWriter,
    frame_id: str,
    topic: str,
    image: np.array,
    timestamp: int,
):
    header = Header()
    header.frame_id = frame_id
    ts_s = int(timestamp // 1e9)
    ts_ns = int(timestamp % 1e9)
    header.stamp = Time(ts_s, ts_ns)
    msg = CvBridge().cv2_to_imgmsg(image, header=header, encoding="passthrough")
    msg.height, msg.width = image.shape
    writer.write(topic, serialize_message(msg), timestamp)
