include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  -- Cartographer's 2D origin is at the IMU. A ground-map bridge projects its
  -- occupancy grid and anchors the public map frame at base_footprint height.
  map_frame = "cartographer_map",
  tracking_frame = "imu_link",
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,  -- EKF owns odom -> base_footprint.
  publish_frame_projected_to_2d = true,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 0.02,
  trajectory_publish_period_sec = 0.1,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
  publish_to_tf = true,
  publish_tracked_pose = true,
}

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 2
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.min_range = 0.08
TRAJECTORY_BUILDER_2D.max_range = 30.
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 90
POSE_GRAPH.optimize_every_n_nodes = 90

return options
