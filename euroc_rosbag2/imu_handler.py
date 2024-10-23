import numpy as np
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Header
import rosbag2_py
from utils import to_ros_timestamp


def create_topic(
    writer: rosbag2_py.SequentialWriter,
    id: int,
    topic: str,
    topic_type="sensor_msgs/msg/Imu",
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
        imu_msg = Imu()

        imu_msg.header = Header()
        timestamp_ns = int(row[0])
        imu_msg.header.stamp = to_ros_timestamp(timestamp_ns)
        imu_msg.header.frame_id = frame_id

        # Angular velocities (in rad/s), columns 1, 2, 3
        imu_msg.angular_velocity.x = row[1]
        imu_msg.angular_velocity.y = row[2]
        imu_msg.angular_velocity.z = row[3]

        # Linear accelerations (in m/s^2), columns 4, 5, 6
        imu_msg.linear_acceleration.x = row[4]
        imu_msg.linear_acceleration.y = row[5]
        imu_msg.linear_acceleration.z = row[6]

        # Assuming no orientation data (Quaternion identity)
        imu_msg.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        yield topic, imu_msg, timestamp_ns
