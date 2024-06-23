import re


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


def to_human_readable(s):
    # Replace underscores and hyphens with spaces
    s = s.replace('_', ' ').replace('-', ' ')
    # Split camel case and join with spaces
    s = re.sub('([a-z])([A-Z])', r'\1 \2', s)
    # Capitalize each word
    s = s.title()
    return s


def get_package_name(msg_type):
    return msg_type.split('::')[0]


def get_unique_msg_types(publishers, subscribers):
    unique_msg_types = set()

    for msg_type in publishers.values():
        unique_msg_types.add(msg_type)

    for msg_type in subscribers.values():
        unique_msg_types.add(msg_type)

    return list(unique_msg_types)


def get_unique_packages(publishers, subscribers):
    unique_pkg_types = set()
    for msg_type in publishers.values():
        unique_pkg_types.add(get_package_name(msg_type))

    for msg_type in subscribers.values():
        unique_pkg_types.add(get_package_name(msg_type))

    return list(unique_pkg_types)
