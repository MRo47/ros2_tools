import builtin_interfaces.msg


def to_ros_timestamp(timestamp_ns: int):
    return builtin_interfaces.msg.Time(
        sec=int(timestamp_ns // 1e9), nanosec=int(timestamp_ns % 1e9)
    )
