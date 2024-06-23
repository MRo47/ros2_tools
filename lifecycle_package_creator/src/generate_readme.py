from jinja2 import Environment, FileSystemLoader

from utils import *


def generate_readme(template_dir: str, readme_template: str, node_name: str,
                    namespace: str, publishers: dict, subscribers: dict):
    env = Environment(loader=FileSystemLoader(template_dir), trim_blocks=True)

    env.filters['snake_case'] = to_snake_case
    env.filters['human_readable'] = to_human_readable
    env.filters['package_name'] = get_package_name
    env.globals['get_unique_pkgs'] = get_unique_packages

    readme_template = env.get_template(readme_template)
    readme_file = readme_template.render(namespace=namespace,
                                         node_name=node_name, subscribers=subscribers, publishers=publishers)

    print(readme_file)


if __name__ == "__main__":
    template_dir = "/home/myron/athena/ros2_tools/lifecycle_package_creator/templates"
    readme_template = "readme.template"
    subscribers = {
        "roi": "sensor_msgs::msg::RegionOfInterest",
        "image": "sensor_msgs::msg::Image"
    }

    publishers = {
        "roi": "sensor_msgs::msg::RegionOfInterest",
        "image": "sensor_msgs::msg::Image",
        "num": "std_msgs::msg::Float32"
    }

    generate_readme(template_dir, readme_template,
                    "CodeMaker", "codega", publishers, subscribers)
