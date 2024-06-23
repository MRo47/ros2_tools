from jinja2 import Environment, FileSystemLoader

from utils import *


def generate_launch(template_dir: str, launch_template: str,
                    node_name: str, namespace: str):
    env = Environment(loader=FileSystemLoader(template_dir), trim_blocks=True)

    env.filters['snake_case'] = to_snake_case

    launch_template = env.get_template(launch_template)
    return launch_template.render(
        node_name=node_name, namespace=namespace)


if __name__ == "__main__":
    template_dir = "/home/myron/athena/ros2_tools/lifecycle_package_creator/templates"
    launch_template = "node_launch.template"

    print(generate_launch(template_dir, launch_template, "CodeMaker",
                          "codega"))
