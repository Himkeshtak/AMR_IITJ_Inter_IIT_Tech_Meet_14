-- Cartographer configuration for SCUTTLE robot (2D SLAM)

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu_link",        -- Change to "base_link" if no IMU
  published_frame = "base_link",
  odom_frame = "odom",

  provide_odom_frame = false,         -- Since robot already publishes odom → base_link
  publish_frame_projected_to_2d = true,

  use_odometry = true,                -- Set false if no /odom available
  use_nav_sat = false,
  use_landmarks = false,

  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,

  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.5,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
}

-- Use 2D SLAM
MAP_BUILDER.use_trajectory_builder_2d = true

-- LIDAR parameters (adjust max_range for your sensor)
TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 12.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0

-- Use IMU if available
TRAJECTORY_BUILDER_2D.use_imu_data = true

-- Scan Matching (improves accuracy but uses more CPU)
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true

-- Motion filter (ignore tiny movements)
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)

-- Loop closure & optimization
POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 20

return options
