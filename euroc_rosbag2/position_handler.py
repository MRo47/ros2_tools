import numpy as np
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header
import rosbag2_py
from utils import to_ros_timestamp


def create_topic(
    writer: rosbag2_py.SequentialWriter,
    id: int,
    topic: str,
    topic_type="geometry_msgs/msg/PointStamped",
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


def msg_generator(csv_file_path, frame_id: str, topic: str):
    # Load CSV data using numpy, skipping the header row
    data = np.loadtxt(csv_file_path, delimiter=",", skiprows=1)

    for row in data:
        point_msg = PointStamped()

        point_msg.header = Header()
        timestamp_ns = int(row[0])
        point_msg.header.stamp = to_ros_timestamp(timestamp_ns)
        point_msg.header.frame_id = frame_id

        # Positions (in meters), columns 1, 2, 3
        point_msg.point.x = row[1]
        point_msg.point.y = row[2]
        point_msg.point.z = row[3]

        yield topic, point_msg, timestamp_ns
