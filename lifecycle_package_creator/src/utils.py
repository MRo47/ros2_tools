def to_snake_case(value):
    return ''.join(['_' + c.lower() if c.isupper() else c for c in value]).lstrip('_')


def to_upper_case(value):
    # Insert underscores before each uppercase letter (except the first one) and convert to uppercase
    return ''.join(['_' + c if c.isupper() and i != 0 else c for i, c in enumerate(value)]).upper()


def to_header(msg_type):
    elems = msg_type.split('::')
    elems[-1] = to_snake_case(elems[-1])
    elems[-1] += '.hpp'
    header_str = "/".join(elems)
    return f'#include "{header_str}"'


def format_subscribers(subs: dict):
    sub_list = []
    for name, msg_type in subs.items():
        sub_list.append(
            f"rclcpp::Subscription<{msg_type}>::SharedPtr {name}_sub_;")
    return '\n'.join(sub_list)


def format_publishers(subs: dict):
    pub_list = []
    for name, msg_type in subs.items():
        pub_list.append(
            f"rclcpp_lifecycle::LifecyclePublisher<{msg_type}>::SharedPtr {name}_pub_;")
    return '\n'.join(pub_list)
