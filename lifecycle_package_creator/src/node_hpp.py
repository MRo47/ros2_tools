from jinja2 import Environment, FileSystemLoader

from utils import *


def create_hpp(template_dir, template_name, node_name, namespace, publishers, subscribers):
    env = Environment(loader=FileSystemLoader(template_dir),
                      trim_blocks=False, lstrip_blocks=False)

    env.filters['snake_case'] = to_snake_case
    env.filters['upper_case'] = to_upper_case
    env.filters['format_subscribers'] = format_subscribers
    env.filters['format_publishers'] = format_publishers
    template = env.get_template(template_name)

    output = template.render(node_name=node_name, namespace=namespace,
                             publishers=publishers, subscribers=subscribers)
    print(output)


if __name__ == "__main__":
    template_dir = "/home/myron/athena/ros2_tools/lifecycle_package_creator/templates"
    template_name = "node_hpp.template"
    subscribers = {
        "roi": "sensor_msgs::msg::RegionOfInterest",
        "image": "sensor_msgs::msg::Image"
    }

    publishers = {
        "roi": "sensor_msgs::msg::RegionOfInterest",
        "image": "sensor_msgs::msg::Image"
    }

    create_hpp(template_dir, template_name, "CodeMaker",
               "codega", publishers, subscribers)
