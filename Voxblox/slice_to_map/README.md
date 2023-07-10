# slice_to_map

This package is designed to convert the Voxblox_node topic "tsdf_slice" to a 2D grid occupancy map.

In the Voxblox node, the pointcloud information "tsdf_slice" can be used to plot the 2D grid occupancy map. The output map can be used for planning.

## input: tsdf_slice (sensor_msg/Pointcloud2)


![tsdf_env](https://user-images.githubusercontent.com/89951560/185733246-6954a44a-38e5-4a7e-aadd-baa893356020.jpg)
*voxblox_tsdf pointcloud in environment*

![tsdf](https://user-images.githubusercontent.com/89951560/185733247-33ac5bbf-c90f-401e-b9b0-27e35db50cbf.jpg)
*voxblox_tsdf pointcloud*

## output: occupancy map (nav_msgs/OccupancyGrid)


![gridmap](https://user-images.githubusercontent.com/89951560/185733249-f9c2cc77-2b08-492b-9c5c-d1d2f4a7a539.jpg)
*converted occupancy map*

## Notes
latest version with numpy inbuild function for speeding up

*Author: Shusen Lin, Zhexu Li*

*Contact: shl090@ucsd.edu, zhl411@ucsd.edu*

