from jinja2 import Environment, FileSystemLoader

from utils import *


def generate_node_source(template_dir: str, hpp_template: str, cpp_template: str, main_template: str,
                         node_name: str, namespace: str, publishers: dict, subscribers: dict):
    env = Environment(loader=FileSystemLoader(template_dir), trim_blocks=True)

    env.filters['snake_case'] = to_snake_case
    env.filters['upper_case'] = to_upper_case
    env.filters['header'] = to_header
    env.globals['get_unique_msg_types'] = get_unique_msg_types

    hpp_template = env.get_template(hpp_template)
    hpp_file = hpp_template.render(node_name=node_name, namespace=namespace,
                                   publishers=publishers, subscribers=subscribers)

    cpp_template = env.get_template(cpp_template)
    cpp_file = cpp_template.render(node_name=node_name, namespace=namespace,
                                   publishers=publishers, subscribers=subscribers)

    main_template = env.get_template(main_template)
    main_file = main_template.render(node_name=node_name, namespace=namespace)
    print(hpp_file)
    print(cpp_file)
    print(main_file)


if __name__ == "__main__":
    template_dir = "/home/myron/athena/ros2_tools/lifecycle_package_creator/templates"
    cpp_template = "node_cpp.template"
    hpp_template = "node_hpp.template"
    main_template = "node_main.template"
    subscribers = {
        "roi": "sensor_msgs::msg::RegionOfInterest",
        "image": "sensor_msgs::msg::Image"
    }

    publishers = {
        "roi": "sensor_msgs::msg::RegionOfInterest",
        "image": "sensor_msgs::msg::Image"
    }

    generate_node_source(template_dir, hpp_template, cpp_template, main_template, "CodeMaker",
                         "codega", publishers, subscribers)
