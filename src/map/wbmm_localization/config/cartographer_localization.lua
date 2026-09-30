include "cartographer_mapping.lua"

-- The loaded map stays frozen; only a bounded live trajectory is retained.
TRAJECTORY_BUILDER.pure_localization_trimmer = { max_submaps_to_keep = 3 }
-- Constraints only update map -> odom after global optimization. With a
-- stationary robot, 20 nodes and the default 5 s motion filter delay that
-- update for roughly 100 s. Keep localization responsive while stationary.
POSE_GRAPH.optimize_every_n_nodes = 1
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 1.
-- Initial values for arbitrary-start global matching; tune on real recordings.
POSE_GRAPH.global_sampling_ratio = 0.05
POSE_GRAPH.global_constraint_search_after_n_seconds = 5.
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.65

return options
