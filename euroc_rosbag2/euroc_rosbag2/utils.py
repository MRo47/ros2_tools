import builtin_interfaces.msg
from geometry_msgs.msg import TransformStamped
import numpy as np
from rclpy.time import Time
import tf_transformations
import yaml


def to_ros_timestamp(timestamp_ns: int):
    return builtin_interfaces.msg.Time(
        sec=int(timestamp_ns // 1e9), nanosec=int(timestamp_ns % 1e9)
    )


# Function to create TransformStamped from matrix
def create_transform_msg(
    parent_frame, child_frame, matrix: np.array, timestamp=Time(seconds=0).to_msg()
) -> tuple[TransformStamped, int]:
    """
    Create a TransformStamped message from a transformation matrix.

    Args:
        parent_frame (str): The name of the parent coordinate frame.
        child_frame (str): The name of the child coordinate frame.
        matrix (np.array): A 4x4 transformation matrix representing
                            the translation and rotation.
        timestamp (builtin_interfaces.msg.Time, optional): The timestamp
                                for the transformation. Defaults to the
                                epoch time conversion.

    Returns:
        TransformStamped: The resulting TransformStamped message.
    """
    transform_msg = TransformStamped()
    transform_msg.header.stamp = timestamp
    transform_msg.header.frame_id = parent_frame
    transform_msg.child_frame_id = child_frame

    quat = tf_transformations.quaternion_from_matrix(matrix)

    # Populate translation and rotation
    transform_msg.transform.translation.x = matrix[0, 3]
    transform_msg.transform.translation.y = matrix[1, 3]
    transform_msg.transform.translation.z = matrix[2, 3]
    transform_msg.transform.rotation.x = quat[0]
    transform_msg.transform.rotation.y = quat[1]
    transform_msg.transform.rotation.z = quat[2]
    transform_msg.transform.rotation.w = quat[3]

    return transform_msg


def get_frame_transform_msg(
    calib_file_path: str,
    parent_frame: str,
    child_frame: str,
    timestamp=Time(seconds=0).to_msg(),
):
    with open(calib_file_path, "r") as f:
        calib_data = yaml.safe_load(f)

    tf_mat = np.array(calib_data["T_BS"]["data"]).reshape((4, 4))

    return create_transform_msg(parent_frame, child_frame, tf_mat, timestamp=timestamp)
