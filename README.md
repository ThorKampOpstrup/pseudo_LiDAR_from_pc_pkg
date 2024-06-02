# Pseudo-Lidar-from-Stereo-Point-Cloud
This is a ROS2 package that merges stereo point cloud with a confidence mapped point cloud,
## Installation
1. Clone this repository into your ROS2 workspace
<!-- white git clone command -->
```bash
git clone <ssh>
```

2. Build the package using colcon build
```bash
colcon build --packages-select pseudo_lidar_from_pc
```
3. Source the workspace using source install/setup.bash
```bash
source install/setup.bash
```

## Running the package
1. Run the package using ros2 run pseudo_lidar_from_pc pseudo_lidar_from_pc
```bash
source <ros path>
source install/setup.bash
ros2 run pseudo_lidar_from_pc pseudo_lidar_from_pc
```

## Default behavior and changing topics
The default behavior of this package will work out of the box with a ZED2i stereo camera and a confidence map published by the ZED2i SDK. The default topics are:
```bash
            PC_TOPIC "/zed/zed_node/point_cloud/cloud_registered"
CONFIDENCE_MAP_TOPIC "/zed/zed_node/confidence/confidence_map"
  PSEUDO_LIDAR_TOPIC "/pseudo_LiDAR"
```

Changing the topic names of the confidence map and stereo pair point cloud can be done in [src/pseudo_LIDAR_from_pc.cpp](src/pseudo_LIDAR_from_pc.cpp)

The resulting confidence mapped point cloud can be sen below.

![Confidence Mapped Point Cloud selfie holding up 2 pieces of paper, one crumbled and on flat](images/selfie.png)
![Confidence Mapped Point Cloud of building](images/building.png)

The confidence mapped point cloud aims to imitate a LIDAR point cloud by assigning a confidence value the intensity channel of each point.

The confidence map and stereo pair point cloud are assumed to be published simultaneously.
