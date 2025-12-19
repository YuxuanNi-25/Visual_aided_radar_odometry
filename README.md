# Visual_aided_radar_odometry (ROS 2)

A minimal ROS 2 pipeline to:
1) play pre-exported 4D radar frames (.bin + radar.csv) into a PointCloud2 topic
2) run a radar odometry node
3) visualize in RViz

> This repo currently contains 3 ROS 2 Python packages under `src/`:
- `dataset_player`: publish radar frames as `sensor_msgs/PointCloud2`
- `radar_odometry`: consume pointcloud and publish odometry/TF (WIP/simple)
- `radar_rviz`: RViz config + demo launch

## Requirements
- ROS 2 (tested on Humble, recommended)
- Python deps: numpy

## Dataset format
`dataset_player` expects a folder like:
- `radar/`
  - `radar.csv`   (index: timestamp + seq + filename)
  - `000000.bin`, `000001.bin`, ...

## Build (colcon)
```bash
cd ~/colcon_varo
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
