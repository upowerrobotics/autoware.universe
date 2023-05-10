# compare_map_segmentation

## Purpose

The `compare_map_segmentation` is a node that filters the ground points from the input pointcloud by using map info (e.g. pcd, elevation map or split map pointcloud from map_loader interface).

## Inner-workings / Algorithms

### Compare Elevation Map Filter

Compare the z of the input points with the value of elevation_map. The height difference is calculated by the binary integration of neighboring cells. Remove points whose height difference is below the `height_diff_thresh`.

<p align="center">
  <img src="./media/compare_elevation_map.png" width="1000">
</p>

### Distance Based Compare Map Filter

WIP

### Voxel Based Approximate Compare Map Filter

WIP

### Voxel Based Compare Map Filter

The filter loads the map pointcloud (static loading whole map at once at beginning or dynamic loading during vehicle moving) and utilizes VoxelGrid to downsample map pointcloud.

For each point of input pointcloud, the filter use `getCentroidIndexAt` combine with `getGridCoordinates` function from VoxelGrid class to check if the downsampled map point existing surrounding input points. Remove the input point which has downsampled map point in voxels containing or being close to the point.

### Voxel Distance based Compare Map Filter

WIP

## Inputs / Outputs

### Compare Elevation Map Filter

#### Input

| Name                    | Type                            | Description      |
| ----------------------- | ------------------------------- | ---------------- |
| `~/input/points`        | `sensor_msgs::msg::PointCloud2` | reference points |
| `~/input/elevation_map` | `grid_map::msg::GridMap`        | elevation map    |

#### Output

| Name              | Type                            | Description     |
| ----------------- | ------------------------------- | --------------- |
| `~/output/points` | `sensor_msgs::msg::PointCloud2` | filtered points |

#### Parameters

| Name                 | Type   | Description                                                                     | Default value |
| :------------------- | :----- | :------------------------------------------------------------------------------ | :------------ |
| `map_layer_name`     | string | elevation map layer name                                                        | elevation     |
| `map_frame`          | float  | frame_id of the map that is temporarily used before elevation_map is subscribed | map           |
| `height_diff_thresh` | float  | Remove points whose height difference is below this value [m]                   | 0.15          |

### Other Filters

#### Input

| Name                     | Type                                            | Description                                            |
| ------------------------ | ----------------------------------------------- | ------------------------------------------------------ |
| `~/input/points`         | `sensor_msgs::msg::PointCloud2`                 | reference points                                       |
| `~/input/map`            | `sensor_msgs::msg::PointCloud2`                 | map (in case static map loading)                       |
| `~/pose_with_covariance` | `geometry_msgs::msg::PoseWithCovarianceStamped` | current ego-vehicle pose (in case dynamic map loading) |

#### Output

| Name              | Type                            | Description     |
| ----------------- | ------------------------------- | --------------- |
| `~/output/points` | `sensor_msgs::msg::PointCloud2` | filtered points |

#### Parameters

| Name                            | Type  | Description                                                                                  | Default value |
| :------------------------------ | :---- | :------------------------------------------------------------------------------------------- | :------------ |
| `use_dynamic_map_loading`       | bool  | map loading mode selection, `true` for dynamic map loading, `false` for static map loading   | true          |
| `distance_threshold`            | float | VoxelGrid's leaf_size also the threshold to check distance between input point and map point | 0.5           |
| `map_update_distance_threshold` | float | Threshold of vehicle movement distance when map update is necessary [m]                      | 10.0          |
| `map_loader_radius`             | float | Radius of map need to be loaded (in dynamic map loading) [m]                                 | 150.0         |
| `timer_interval_ms`             | int   | time interval of timer to check if update the map is necessary (in dynamic map loading) [ms] | 100           |

## Assumptions / Known limits

## (Optional) Error detection and handling

## (Optional) Performance characterization

## (Optional) References/External links

## (Optional) Future extensions / Unimplemented parts
