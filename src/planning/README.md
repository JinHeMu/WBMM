# Planning packages

- `wbmm_planner/` is the ROS-free planning library. `src/search/` implements
  base and whole-body search; `src/optimization/` builds and optimizes the
  trajectory; `src/whole_body_planner.cpp` composes them. Public headers follow
  the same layout under `include/wbmm_planner/`.
- `wbmm_planner_ros/` owns the ROS node and the `WholeBodyGoal` and
  `WholeBodyTrajectory` messages in `msg/`. Their ROS type names are
  `wbmm_planner_ros/msg/WholeBodyGoal` and
  `wbmm_planner_ros/msg/WholeBodyTrajectory`.

The C++ algorithm namespaces `wbmm::search` and `wbmm::traj_opt` remain in use;
only their former standalone packages and include paths were consolidated.
The collision/search integration test now belongs to `wbmm_planner`, so
`wbmm_collision` has no dependency on the planner.
