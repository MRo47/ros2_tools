import numpy as np
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Header
import rosbag2_py
from utils import to_ros_timestamp
import yaml


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


def msg_generator(csv_file_path, calib_file_path, frame_id: str, topic: str):
    # Load IMU calibration data
    with open(calib_file_path, "r") as f:
        calib_data = yaml.safe_load(f)

    freq = calib_data["rate_hz"]

    angular_velocity_cov_value = calib_data["gyroscope_noise_density"] ** 2 * freq
    linear_acceleration_cov_value = (
        calib_data["accelerometer_noise_density"] ** 2 * freq
    )

    angular_velocity_cov = [
        angular_velocity_cov_value,
        0,
        0,
        0,
        angular_velocity_cov_value,
        0,
        0,
        0,
        angular_velocity_cov_value,
    ]

    linear_acceleration_cov = [
        linear_acceleration_cov_value,
        0,
        0,
        0,
        linear_acceleration_cov_value,
        0,
        0,
        0,
        linear_acceleration_cov_value,
    ]

    orientation_cov = [1e-2, 0, 0, 0, 1e-2, 0, 0, 0, 1e-2]  # unknown

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

        imu_msg.angular_velocity_covariance = angular_velocity_cov

        # Linear accelerations (in m/s^2), columns 4, 5, 6
        imu_msg.linear_acceleration.x = row[4]
        imu_msg.linear_acceleration.y = row[5]
        imu_msg.linear_acceleration.z = row[6]

        imu_msg.linear_acceleration_covariance = linear_acceleration_cov

        # Assuming no orientation data (Quaternion identity)
        imu_msg.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        imu_msg.orientation_covariance = orientation_cov

        yield topic, imu_msg, timestamp_ns
