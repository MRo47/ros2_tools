# EuRoC MAV Dataset to rosbag2 exporter

export data arranged in [EuRoC MAV Dataset](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets) to rosbag2

## Usage

### Export to rosbag2

```bash
python3 export.py
```

### Play the bag

```bash
ros2 bag play MH_01_easy_rosbag2/ --qos-profile-overrides-path override_policy.yaml
```

the qos override_policy.yaml is in [configs](configs/qos/override_policy.yaml)

### Visualise in rviz2

```bash
rviz2 -d configs/rviz/bag_view_config.rviz
```
