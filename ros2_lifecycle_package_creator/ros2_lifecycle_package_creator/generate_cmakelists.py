from jinja2 import Environment, FileSystemLoader

from .utils import *


def generate_cmakelists(template_dir: str, cmakelists_template: str,
                        node_name: str, namespace: str, publishers: dict, subscribers: dict):
    env = Environment(loader=FileSystemLoader(template_dir), trim_blocks=True)

    env.filters['snake_case'] = to_snake_case
    env.filters['package_name'] = get_package_name
    env.globals['get_unique_pkgs'] = get_unique_packages

    cmakelists_template = env.get_template(cmakelists_template)
    return cmakelists_template.render(
        node_name=node_name, namespace=namespace, subscribers=subscribers, publishers=publishers)


if __name__ == "__main__":
    from pathlib import Path

    template_dir = Path(__file__).resolve().parent/"templates"
    cmakelists_template = "cmakelists.j2"
    subscribers = {
        "roi": "sensor_msgs::msg::RegionOfInterest",
        "image": "sensor_msgs::msg::Image"
    }

    publishers = {
        "roi": "sensor_msgs::msg::RegionOfInterest",
        "image": "sensor_msgs::msg::Image",
        "num_objects": "std_msgs::msg::Int32"
    }

    print(generate_cmakelists(template_dir, cmakelists_template,
                              "ObjectSegmenter", "perception", publishers, subscribers))
