# erl_car_mapping

- [Octomap](#octomap)
- [Voxblox](#voxblox)

## Octomap

The following figures show demos of the Octomap mapping in a basement environment
<i>
<p align = "left">
<img src = "https://user-images.githubusercontent.com/89951560/185774672-7cb2cf24-25b2-4fac-92e7-9a3ab8b7e958.jpg" width="480" height="300>
</p>
<p align = "left">
<p>
Fig.1 - Octomap grid in basement environment
</p>
<p align = "left">
<img src = "https://user-images.githubusercontent.com/89951560/185774676-05f52bb6-5557-44c5-be5b-5db66e0b553c.jpg" width="480" height="300>
</p>
<p align = "left">
<p>
Fig.2 - Octomap grid
</p>
<p align = "left">
<img src = "https://user-images.githubusercontent.com/89951560/185774679-ed133dad-53bf-48c3-9203-3fedf854d596.jpg" width="480" height="300>
</p>
<p align = "left">
<p>
Fig.3 - Octomap occupancy gridmap
</p>
</i>

## Voxblox
The following figures show demos of the Voxblox mapping in a basement environment
<i>
<p align = "left">
<img src = "https://user-images.githubusercontent.com/89951560/185774308-91960b33-053f-45ea-ad97-f0b27ca52e86.jpg" width="450" height="300>
</p>
<p align = "left">
<p>
Fig.1 - Voxblox tsdf slice in basement environment
</p>
<p align = "left">
<img src = "https://user-images.githubusercontent.com/89951560/185774636-7e397c1b-6f3d-4726-b9dd-d6254abd7f65.jpg" width="450" height="300>
</p>
<p align = "left">
<p>
Fig.2 - Voxblox tsdf slice 
</p>
</i>

### slice_to_map

slice_to_map package can convert the Voxblox_slice pointcloud to a 2D occupancy gridmap.
<i>
<p align = "left">
<img src = "https://user-images.githubusercontent.com/89951560/185774644-210efeb5-503e-4ffa-baa3-3ae060273303.jpg" width="450" height="300>
</p>
<p align = "left">
<p>
Fig.3 - Voxblox occupancy gridmap
</p>
</i>

## How to launch
```xml
<?xml version="1.0" encoding="ISO-8859-15"?>
<launch>
<!-- choose mapping -->
  <arg name="use_octomap" default="true"/>
  <arg name="use_voxblox" default="false"/>
  <arg name="map_resolution" value="0.05" />

  <!-- Launch rviz for octomap -->
  <group if="$(arg use_octomap)">
     <node pkg="octomap_server" type="octomap_server_node" name="octomap_server">
	<!-- ruduce the resolution for speed up -->
        <param name="resolution" value="$(arg map_resolution)" />

        <!-- fixed map frame (set to 'map' if SLAM or localization running!) -->
        <param name="frame_id" type="string" value="racecar01/map" />

        <!-- maximum range to integrate (speedup!) -->
        <param name="sensor_model/max_range" value="6.0" />
        
	<!-- Distance threshold for points (in z direction) to be segmented to the ground plane 0.04 in default -->  
        <param name="~occupancy_min_z" value="0.0" />
        	
        <!-- data source to integrate (PointCloud2) -->
        <remap from="cloud_in" to="racecar01/realsense/depth/color/points" />
        <remap from="projected_map" to="racecar01/map" />
     </node>
  </group>


  <!-- Launch rviz for voxblox-->
  <group if="$(arg use_voxblox)">

     <!-- Launch the Voxblox node for mapping -->
     <arg name="voxel_size" default="$(arg map_resolution)" />
     <arg name="voxels_per_side" default="16" />

     <node name="voxblox_node" pkg="voxblox_ros" type="tsdf_server" output="screen" args="-alsologtostderr" clear_params="true">
      <remap from="pointcloud" to="racecar01/realsense/depth/color/points"/>
      <param name="method" value="fast" />
      <param name="tsdf_voxel_size" value="$(arg voxel_size)" />
      <param name="tsdf_voxels_per_side" value="$(arg voxels_per_side)" />
      <param name="clear_sphere_for_planning" value="true" />
      <param name="slice_level" value="0.0" />
      <param name="publish_pointclouds" value="true" />
      <param name="publish_slices" value="true" />
      <param name="publish_tsdf_map" value="false" />
      <param name="publish_tsdf_info" value="true" />
      <param name="publish_traversable" value="false" />
      <param name="use_tf_transforms" value="true" />
      <param name="update_mesh_every_n_sec" value="2.0" />
      <param name="world_frame" value="racecar01/map" />
     </node>
     
     <!-- Launch the Voxblox slice gridmapp convertor node -->
     <node name="voxblox_grid_mapping" pkg="slice_to_map" type="slice_to_map.py" output="screen">
      <remap from="pc_input"          to="voxblox_node/tsdf_slice"/>
      <remap from="mapout"            to="racecar01/map"/>
      <param name="map_resolution" value="$(arg map_resolution)"/> <!-- should match with the voxel_size -->
      <param name="map_frame"      value="racecar01/map"/> 
    </node>	  
  </group>

</launch>
```
