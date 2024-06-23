from jinja2 import Environment, FileSystemLoader

from utils import *


def generate_package_xml(template_dir: str, package_xml_template: str,
                         node_name: str, publishers: dict, subscribers: dict, author_name: str,
                         author_email: str, license_name: str):
    env = Environment(loader=FileSystemLoader(template_dir), trim_blocks=True)

    env.filters['snake_case'] = to_snake_case
    env.filters['package_name'] = get_package_name

    package_xml_template = env.get_template(package_xml_template)
    package_xml_file = package_xml_template.render(
        node_name=node_name, subscribers=subscribers, publishers=publishers,
        author_name=author_name, author_email=author_email, license_name=license_name)

    print(package_xml_file)


if __name__ == "__main__":
    template_dir = "/home/myron/athena/ros2_tools/lifecycle_package_creator/templates"
    package_xml_template = "package_xml.template"
    subscribers = {
        "roi": "sensor_msgs::msg::RegionOfInterest",
        "image": "sensor_msgs::msg::Image"
    }

    publishers = {
        "roi": "sensor_msgs::msg::RegionOfInterest",
        "image": "sensor_msgs::msg::Image",
        "num": "std_msgs::msg::Float32"
    }

    generate_package_xml(template_dir, package_xml_template,
                         "CodeMaker", publishers, subscribers, "Myron Rodrigues",
                         "rodriguesmyron47@gmail.com",
                         "Apache-2.0")
