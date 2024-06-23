from generate_cmakelists import generate_cmakelists
from generate_launch import generate_launch
from generate_package_xml import generate_package_xml
from generate_readme import generate_readme
from generate_source import generate_source
from utils import *

import yaml
from pathlib import Path


def yaml_to_dict(file_path):
    path = Path(file_path)

    # Check if the path exists
    if not path.exists():
        raise FileNotFoundError(f"The file {file_path} does not exist.")

    try:
        with path.open('r') as file:
            data = yaml.safe_load(file)
        return data
    except yaml.YAMLError as e:
        raise ValueError(f"Error parsing YAML file: {e}")
    except Exception as e:
        raise RuntimeError(f"An error occurred while reading the file: {e}")


def create_dir(dir_path):
    path = Path(dir_path)

    try:
        path.mkdir(parents=True, exist_ok=False)
        print(f"Directory {dir_path} created successfully.")
        return path
    except FileExistsError:
        if not any(path.iterdir()):  # dir empty
            return path
        else:
            raise FileExistsError(
                f"The directory {dir_path} already exists and is not empty")
    except Exception as e:
        raise RuntimeError(
            f"An error occurred while creating the directory: {e}")


def write_to_file(file_path, formatted_string):
    path = Path(file_path)

    # Open the file in write mode and write the formatted string to it
    with path.open('w') as f:
        f.write(formatted_string)
    print(f"created {str(file_path)}")


def generate_package(config_file: Path, template_dir: Path, target_dir: Path):
    config = yaml_to_dict(config_file)

    target_dir = create_dir(target_dir)

    # create dir structure and files
    cmakelists = target_dir/"CMakeLists.txt"
    readme = target_dir/"README.md"
    package_xml = target_dir/"package.xml"
    launch = target_dir/"launch" / \
        (to_snake_case(config["node_name"])+".launch.py")
    node_hpp = target_dir/"include" / \
        config["namespace"]/(to_snake_case(config["node_name"])+".hpp")
    node_cpp = target_dir/"src"/(to_snake_case(config["node_name"])+".cpp")
    node_main = target_dir/"src" / \
        (to_snake_case(config["node_name"])+"_node.cpp")

    # generate file and add data
    write_to_file(cmakelists,
                  generate_cmakelists(template_dir, "cmakelists.template",
                                      config["node_name"], config["namespace"],
                                      config["publishers"], config["subscribers"]))

    write_to_file(readme,
                  generate_readme(template_dir, "readme.template",
                                  config["node_name"], config["namespace"],
                                  config["publishers"], config["subscribers"],
                                  config["description"]))

    write_to_file(package_xml,
                  generate_package_xml(template_dir, "package_xml.template",
                                       config["node_name"], config["publishers"],
                                       config["subscribers"], config["author"],
                                       config["email"], config["license"],
                                       config["description"]))

    write_to_file(launch,
                  generate_launch(template_dir, "node_launch.template",
                                  config["node_name"], config["namespace"]))

    hpp_text, cpp_text, main_text = generate_source(
        template_dir, "node_hpp.template", "node_cpp.template", "node_main.template",
        config["node_name"], config["namespace"], config["publishers"], config["subscribers"]
    )

    write_to_file(node_hpp, hpp_text)
    write_to_file(node_cpp, cpp_text)
    write_to_file(node_main, main_text)


def main():
    config_file = Path(
        "/home/myron/athena/ros2_tools/lifecycle_package_creator/test_config/object_segmenter.yaml")
    target_dir = Path(
        "/home/myron/athena/ros2_tools/lifecycle_package_creator/sandbox/test_pkg")
    template_dir = Path(
        "/home/myron/athena/ros2_tools/lifecycle_package_creator/templates")
    generate_package(config_file, template_dir, target_dir)


if __name__ == "__main__":
    main()
