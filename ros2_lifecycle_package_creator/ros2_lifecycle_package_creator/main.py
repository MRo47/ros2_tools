from pathlib import Path
import argparse
import shutil
from .package_generator import generate_package


def main():
    parser = argparse.ArgumentParser(
        description="Create a ROS2 lifecycle package from a yaml config file, "
        "see ../README.md or ../example")
    parser.add_argument(
        "-c", "--config_file", help="The path to the example configuration file (YAML format)")
    parser.add_argument(
        "-t", "--target_dir", help="The path to the target directory for creating the new package"
        " (target_dir will be created if not present, or it should be empty)")
    args = parser.parse_args()

    if args.config_file is None or args.target_dir is None:
        parser.print_help()
        parser.error("Both config_file and target_dir are required.")

    package_dir = Path(__file__).resolve().parent
    template_dir = package_dir/"templates"
    generate_package(args.config_file, template_dir, args.target_dir)
    package_files = package_dir/"package_files"
    shutil.copy(package_files/".clang-format", args.target_dir)
    print(f"copied .clang-format to {args.target_dir}")


if __name__ == "__main__":
    main()
