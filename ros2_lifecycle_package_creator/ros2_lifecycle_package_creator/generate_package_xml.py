from jinja2 import Environment, FileSystemLoader

from .utils import *


def generate_package_xml(template_dir: str, package_xml_template: str,
                         node_name: str, publishers: dict, subscribers: dict, author_name: str,
                         author_email: str, license_name: str, description: str):
    env = Environment(loader=FileSystemLoader(template_dir), trim_blocks=True)

    env.filters['snake_case'] = to_snake_case
    env.filters['package_name'] = get_package_name
    env.globals['get_unique_pkgs'] = get_unique_packages

    package_xml_template = env.get_template(package_xml_template)
    return package_xml_template.render(
        node_name=node_name, subscribers=subscribers, publishers=publishers,
        author_name=author_name, author_email=author_email, license_name=license_name,
        description=description)


if __name__ == "__main__":
    from pathlib import Path

    template_dir = Path(__file__).resolve().parent/"templates"
    package_xml_template = "package_xml.j2"
    subscribers = {
        "roi": "sensor_msgs::msg::RegionOfInterest",
        "image": "sensor_msgs::msg::Image"
    }

    publishers = {
        "roi": "sensor_msgs::msg::RegionOfInterest",
        "image": "sensor_msgs::msg::Image",
        "num": "std_msgs::msg::Float32"
    }

    print(generate_package_xml(template_dir, package_xml_template,
                               "ObjectSegmenter", publishers, subscribers, "Kitty Fugues",
                               "kittyfugues14@robots.com",
                               "Apache-2.0", "detect objects in images"))
