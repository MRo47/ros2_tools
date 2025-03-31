from jinja2 import Environment, FileSystemLoader

from .utils import *


def generate_source(template_dir: str, hpp_template: str, cpp_template: str, main_template: str,
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
    return hpp_file, cpp_file, main_file


if __name__ == "__main__":
    from pathlib import Path

    template_dir = Path(__file__).resolve().parent/"templates"
    cpp_template = "node_cpp.j2"
    hpp_template = "node_hpp.j2"
    main_template = "node_main.j2"
    subscribers = {
        "roi": "sensor_msgs::msg::RegionOfInterest",
        "image": "sensor_msgs::msg::Image"
    }

    publishers = {
        "roi": "sensor_msgs::msg::RegionOfInterest",
        "image": "sensor_msgs::msg::Image",
        "num": "std_msgs::msg::Float32"
    }

    hpp_file, cpp_file, main_file = generate_source(template_dir, hpp_template, cpp_template,
                                                    main_template, "ObjectSegmenter",
                                                    "perception", publishers, subscribers)

    print("########## HPP FILE ##########\n\n", hpp_file)
    print("\n\n########## CPP FILE ##########\n\n", cpp_file)
    print("\n\n########## MAIN FILE ##########\n\n", main_file)
