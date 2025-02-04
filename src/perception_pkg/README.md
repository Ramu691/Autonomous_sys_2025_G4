perception_pkg
========

This package is responsible for the perception of the environment. It is the first step in the pipeline of the
autonomous system. The perception package is responsible for the following tasks:

- Undistorting the camera images
- Generating point cloud from depth image
- Generating global map from point cloud as an occupancy grid
- Publishing the global map via the octomap server onto /octomap_full and /octomap_binary topics

Launch the perception package with the following command:

```bash
roslaunch perception_pkg perception.launch
```