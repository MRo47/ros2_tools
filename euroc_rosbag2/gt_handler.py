import numpy as np
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
import rosbag2_py
from utils import to_ros_timestamp


def create_pose_topic(
    writer: rosbag2_py.SequentialWriter,
    id: int,
    topic: str,
    topic_type="geometry_msgs/msg/PoseStamped",
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


def create_twist_topic(
    writer: rosbag2_py.SequentialWriter,
    id: int,
    topic: str,
    topic_type="geometry_msgs/msg/TwistStamped",
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


def create_imu_bias_topic(
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


def msg_generator(
    csv_file_path,
    frame_id: str,
    pose_topic: str,
    velocity_topic: str,
    imu_bias_topic: str,
):
    # Load CSV data using numpy, skipping the header row
    data = np.loadtxt(csv_file_path, delimiter=",", skiprows=1)

    for row in data:
        timestamp_ns = int(row[0])  # First column is the timestamp in nanoseconds

        # Create and populate the PoseStamped message for position and orientation
        pose_msg = PoseStamped()
        pose_msg.header = Header()
        pose_msg.header.stamp = to_ros_timestamp(timestamp_ns)
        pose_msg.header.frame_id = frame_id

        # Position (columns 1, 2, 3)
        pose_msg.pose.position.x = row[1]
        pose_msg.pose.position.y = row[2]
        pose_msg.pose.position.z = row[3]

        # Orientation (columns 4, 5, 6, 7)
        pose_msg.pose.orientation.w = row[4]
        pose_msg.pose.orientation.x = row[5]
        pose_msg.pose.orientation.y = row[6]
        pose_msg.pose.orientation.z = row[7]

        # Create and populate the TwistStamped message for velocity
        velocity_msg = TwistStamped()
        velocity_msg.header = Header()
        velocity_msg.header.stamp = to_ros_timestamp(timestamp_ns)
        velocity_msg.header.frame_id = frame_id

        # Linear velocity (columns 8, 9, 10)
        velocity_msg.twist.linear.x = row[8]
        velocity_msg.twist.linear.y = row[9]
        velocity_msg.twist.linear.z = row[10]

        # Create and populate the IMU message for biases
        imu_msg = Imu()
        imu_msg.header = Header()
        imu_msg.header.stamp = to_ros_timestamp(timestamp_ns)
        imu_msg.header.frame_id = frame_id

        # Gyroscope biases (columns 11, 12, 13)
        imu_msg.angular_velocity.x = row[11]
        imu_msg.angular_velocity.y = row[12]
        imu_msg.angular_velocity.z = row[13]

        # Accelerometer biases (columns 14, 15, 16)
        imu_msg.linear_acceleration.x = row[14]
        imu_msg.linear_acceleration.y = row[15]
        imu_msg.linear_acceleration.z = row[16]

        # Yield messages with respective topics and timestamps
        yield pose_topic, pose_msg, timestamp_ns
        yield velocity_topic, velocity_msg, timestamp_ns
        yield imu_bias_topic, imu_msg, timestamp_ns
