# ros2_tools
Tools and tips for ROS2

## ros2 lifecycle package creator

Simplify lifecycle node package creation in ROS2 by writing templates for files using standard naming and publisher subscriber data.
This package works best with standard ROS2 messages and if other packages follow the naming conventions like in ROS2.

### Demo

```bash
python src/package_generator.py --config example/object_segmenter.yaml --target_dir example_pkg
```

### What this package generates
- `node.cpp` file
- `node.hpp` file
- `main.cpp` executable file
- `node.launch.py` file with activated node
- `package.xml` file with required subscriber and publisher dependencies
- `CMakeLists.txt` file with required subscriber and publisher dependencies
- `README.md` with provided subscriber and publisher data
- adds `.clang-format` file as provided in repo [here](https://github.com/ament/ament_lint/blob/rolling/ament_clang_format/ament_clang_format/configuration/.clang-format)


### Setup

#### config file

```yaml
author: authors name
email: authors_email
license: license_name
node_name: NodeName
namespace: some_namespace
description: some description
subscribers:
  topic1: type1
  topic2: type2
publishers:
  topic3: type3
  topic4: type4
```

susbcribers and publishers will have handles that will have names `topic1_sub_` or `topic3_pub_` respectively. 
Topics will be named exactly as they are specified. eg: `topic1`

#### target dir

The package files will be put in `target_dir` ensure its empty or doesnt exist (will be created)

### TODO
- add license file
- add license header
- add parameter helpers