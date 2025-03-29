# ROS2 lifecycle package creator

Simplify lifecycle package creation in ROS2 by generating boilerplate files.

This package works best with standard ROS2 messages and for custom interfaces follow the naming conventions like in ROS2 official packages.

## Demo

```bash
python src/main.py --config example/object_segmenter.yaml --target_dir example_pkg
```

## What this package generates
- `node.cpp`
- `node.hpp`
- `main.cpp` executable
- `node.launch.py` with activated node
- `package.xml` with required subscriber and publisher dependencies
- `CMakeLists.txt` with required subscriber and publisher dependencies
- `README.md` with provided subscriber and publisher data
- adds `.clang-format` file as provided in repo [here](https://github.com/ament/ament_lint/blob/rolling/ament_clang_format/ament_clang_format/configuration/.clang-format)


## Setup

### config_file

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

Susbcribers and publishers will have handles that will have names `topic1_sub_` or `topic3_pub_` respectively. 
Topics will be named exactly as they are specified. eg: `topic1`

### target_dir

The package files will be put in `target_dir` ensure its empty or doesnt exist (will be created)

## TODO
- add license file
- add license header
- add parameter helpers

