-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = false,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}
-- Original defaults
TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1

MAP_BUILDER.use_trajectory_builder_3d = true
MAP_BUILDER.num_background_threads = 4

POSE_GRAPH.optimization_problem.huber_scale = 5e2
POSE_GRAPH.optimize_every_n_nodes = 320
POSE_GRAPH.constraint_builder.sampling_ratio = 0.03
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 500
POSE_GRAPH.constraint_builder.min_score = 0.62
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.66

-- Modifications
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.translation_weight = 5. 

TRAJECTORY_BUILDER_3D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_3D.imu_gravity_time_constant = .1

-- No point of trying to SLAM over the points on your cart.
TRAJECTORY_BUILDER_3D.min_range = 1.0
TRAJECTORY_BUILDER_3D.max_range = 50

-- These were just my first guess: use more points for SLAMing and adapt a bit for the ranges that are bigger for cars.
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.max_length = 5.
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.min_num_points = 250.
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.max_length = 8.
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.min_num_points = 400.

-- The submaps felt pretty big - since the car moves faster, we want them to be
-- slightly smaller. You are also slamming at 10cm - which might be aggressive
-- for cars and for the quality of the laser. I increased 'high_resolution',
-- you might need to increase 'low_resolution'. Increasing the
-- '*num_iterations' in the various optimization problems also trades
-- performance/quality.
TRAJECTORY_BUILDER_3D.submaps.high_resolution = .25
TRAJECTORY_BUILDER_3D.submaps.low_resolution = .60
TRAJECTORY_BUILDER_3D.submaps.num_range_data = 270
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 50


-- Trying loop closing too often will cost CPU and not buy you a lot. There is
-- little point in trying more than once per submap.
MAP_BUILDER.pose_graph.optimize_every_n_nodes = 100
MAP_BUILDER.pose_graph.constraint_builder.sampling_ratio = 0.03
MAP_BUILDER.pose_graph.optimization_problem.ceres_solver_options.max_num_iterations = 200

MAP_BUILDER.pose_graph.constraint_builder.min_score = 0.5

-- Crazy search window to force loop closure to work. All other changes are probably not needed.
--MAP_BUILDER.sparse_pose_graph.constraint_builder.max_constraint_distance = 250.
--MAP_BUILDER.sparse_pose_graph.constraint_builder.fast_correlative_scan_matcher_3d.linear_xy_search_window = 250.
--MAP_BUILDER.sparse_pose_graph.constraint_builder.fast_correlative_scan_matcher_3d.linear_z_search_window = 30.
--MAP_BUILDER.sparse_pose_graph.constraint_builder.fast_correlative_scan_matcher_3d.angular_search_window = math.rad(60.)
--MAP_BUILDER.sparse_pose_graph.constraint_builder.ceres_scan_matcher_3d.ceres_solver_options.max_num_iterations = 50

return options
