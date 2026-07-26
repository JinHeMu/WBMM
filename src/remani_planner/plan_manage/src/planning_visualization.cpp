#include <plan_manage/planning_visualization.h>

using std::cout;
using std::endl;
namespace remani_planner
{
  PlanningVisualization::PlanningVisualization(rclcpp::Node::SharedPtr nh){
    node_ = nh;
    // PlanningVisualization is constructed before GridMap declares its
    // parameters. Read the launch/YAML override without declaring it here,
    // otherwise GridMap::initMap() would see a duplicate declaration.
    if (node_->has_parameter("grid_map.frame_id")) {
      frame_id_ = node_->get_parameter("grid_map.frame_id").as_string();
    } else {
      const auto &overrides =
          node_->get_node_parameters_interface()->get_parameter_overrides();
      const auto frame_it = overrides.find("grid_map.frame_id");
      if (frame_it != overrides.end()) {
        frame_id_ = frame_it->second.get<std::string>();
      }
    }

    // 这些 topic 与 plan_manage/launch/*.rviz 中的 Display 一一对应。
    // Marker 使用普通 Volatile QoS；RViz 启动后需要等待下一次发布才能显示。
    goal_point_pub = nh->create_publisher<visualization_msgs::msg::Marker>("goal_point", 2);
    global_traj_pub = nh->create_publisher<visualization_msgs::msg::Marker>("global_traj", 2);
    init_ctrl_pts_pub = nh->create_publisher<visualization_msgs::msg::Marker>("init_ctrl_pts", 2);
    optmizing_traj_pub = nh->create_publisher<visualization_msgs::msg::Marker>("optmizing_traj", 2);
    init_waypoints_pub = nh->create_publisher<visualization_msgs::msg::Marker>("init_waypoints", 2);
    optimal_ctrl_pts_pub = nh->create_publisher<visualization_msgs::msg::Marker>("optimal_ctrl_pts", 2);
    optimal_waypoints_pub = nh->create_publisher<visualization_msgs::msg::Marker>("optimal_waypoints", 2);
    failed_list_pub = nh->create_publisher<visualization_msgs::msg::Marker>("failed_list", 2);
    a_star_list_pub = nh->create_publisher<visualization_msgs::msg::Marker>("a_star_list", 20);
    init_list_debug_pub = nh->create_publisher<visualization_msgs::msg::Marker>("init_debug_list",2);
    
    intermediate_pt0_pub = nh->create_publisher<visualization_msgs::msg::Marker>("pt0_dur_opt", 10);
    intermediate_grad0_pub = nh->create_publisher<visualization_msgs::msg::MarkerArray>("grad0_dur_opt", 10);
    intermediate_pt1_pub = nh->create_publisher<visualization_msgs::msg::Marker>("pt1_dur_opt", 10);
    intermediate_grad1_pub = nh->create_publisher<visualization_msgs::msg::MarkerArray>("grad1_dur_opt", 10);
    intermediate_grad_smoo_pub = nh->create_publisher<visualization_msgs::msg::MarkerArray>("smoo_grad_dur_opt", 10);
    intermediate_grad_dist_pub = nh->create_publisher<visualization_msgs::msg::MarkerArray>("dist_grad_dur_opt", 10);
    intermediate_grad_feas_pub = nh->create_publisher<visualization_msgs::msg::MarkerArray>("feas_grad_dur_opt", 10);
    
    // 后续若启用性能记录，可用该时间作为可视化/规划的相对时间基准。
    t_init = node_->now();
  }

  // // real ids used: {id, id+1000}
  void PlanningVisualization::displayMarkerList(const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr &pub, const vector<Eigen::Vector3d> &list, double scale,
                                                Eigen::Vector4d color, int id, bool show_sphere /* = true */ )
  {
    // 一个点列表可编码为 SPHERE_LIST（离散点）或 LINE_STRIP（连续路径）。
    // id+1000 给同一 topic 下的线条预留独立 ID，避免覆盖其它 Marker。
    visualization_msgs::msg::Marker sphere, line_strip;
    sphere.header.frame_id = line_strip.header.frame_id = frame_id_;
    sphere.header.stamp = line_strip.header.stamp = node_->now();
    sphere.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
    sphere.action = line_strip.action = visualization_msgs::msg::Marker::ADD;
    sphere.id = id;
    line_strip.id = id + 1000;

    sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
    sphere.color.r = line_strip.color.r = color(0);
    sphere.color.g = line_strip.color.g = color(1);
    sphere.color.b = line_strip.color.b = color(2);
    sphere.color.a = line_strip.color.a = color(3) > 1e-5 ? color(3) : 1.0;
    sphere.scale.x = scale;
    sphere.scale.y = scale;
    sphere.scale.z = scale;
    line_strip.scale.x = scale / 2;
    geometry_msgs::msg::Point pt;
    for (int i = 0; i < int(list.size()); i++)
    {
      pt.x = list[i](0);
      pt.y = list[i](1);
      pt.z = list[i](2);
      if (show_sphere) sphere.points.push_back(pt);
      else line_strip.points.push_back(pt);
    }
    if (show_sphere) pub->publish(sphere);
    else pub->publish(line_strip);
  }

  // real ids used: {id, id+1}
  void PlanningVisualization::generatePathDisplayArray(visualization_msgs::msg::MarkerArray &array,
                                                       const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id)
  {
    // MarkerArray 版本使用相邻 ID（id/id+1），适合一次性发布点和连线。
    visualization_msgs::msg::Marker sphere, line_strip;
    sphere.header.frame_id = line_strip.header.frame_id = frame_id_;
    sphere.header.stamp = line_strip.header.stamp = node_->now();
    sphere.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
    sphere.action = line_strip.action = visualization_msgs::msg::Marker::ADD;
    sphere.id = id;
    line_strip.id = id + 1;

    sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
    sphere.color.r = line_strip.color.r = color(0);
    sphere.color.g = line_strip.color.g = color(1);
    sphere.color.b = line_strip.color.b = color(2);
    sphere.color.a = line_strip.color.a = color(3) > 1e-5 ? color(3) : 1.0;
    sphere.scale.x = scale;
    sphere.scale.y = scale;
    sphere.scale.z = scale;
    line_strip.scale.x = scale / 3;
    geometry_msgs::msg::Point pt;
    for (int i = 0; i < int(list.size()); i++)
    {
      pt.x = list[i](0);
      pt.y = list[i](1);
      pt.z = list[i](2);
      sphere.points.push_back(pt);
      line_strip.points.push_back(pt);
    }
    array.markers.push_back(sphere);
    array.markers.push_back(line_strip);
  }

  // real ids used: {1000*id ~ (arrow nums)+1000*id}
  void PlanningVisualization::generateArrowDisplayArray(visualization_msgs::msg::MarkerArray &array,
                                                        const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id)
  {
    // list 按 [start0,end0,start1,end1,...] 组织，每两点生成一个箭头。
    // 调用方应保证 list.size() 为偶数；多余的最后一个点会被忽略。
    visualization_msgs::msg::Marker arrow;
    arrow.header.frame_id = frame_id_;
    arrow.header.stamp = node_->now();
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.action = visualization_msgs::msg::Marker::ADD;

    // geometry_msgs::Point start, end;
    // arrow.points

    arrow.color.r = color(0);
    arrow.color.g = color(1);
    arrow.color.b = color(2);
    arrow.color.a = color(3) > 1e-5 ? color(3) : 1.0;
    arrow.scale.x = scale;
    arrow.scale.y = 2 * scale;
    arrow.scale.z = 2 * scale;

    geometry_msgs::msg::Point start, end;
    for (int i = 0; i < int(list.size() / 2); i++)
    {
      // arrow.color.r = color(0) / (1+i);
      // arrow.color.g = color(1) / (1+i);
      // arrow.color.b = color(2) / (1+i);

      start.x = list[2 * i](0);
      start.y = list[2 * i](1);
      start.z = list[2 * i](2);
      end.x = list[2 * i + 1](0);
      end.y = list[2 * i + 1](1);
      end.z = list[2 * i + 1](2);
      arrow.points.clear();
      arrow.points.push_back(start);
      arrow.points.push_back(end);
      arrow.id = i + id * 1000;

      array.markers.push_back(arrow);
    }
  }

  void PlanningVisualization::displayGoalPoint(Eigen::Vector2d goal_point, Eigen::Vector4d color, const double scale, int id)
  {
    visualization_msgs::msg::Marker sphere;
    sphere.header.frame_id = frame_id_;
    sphere.header.stamp = node_->now();
    sphere.type = visualization_msgs::msg::Marker::SPHERE;
    sphere.action = visualization_msgs::msg::Marker::ADD;
    sphere.id = id;

    sphere.pose.orientation.w = 1.0;
    sphere.color.r = color(0);
    sphere.color.g = color(1);
    sphere.color.b = color(2);
    sphere.color.a = color(3);
    sphere.scale.x = scale;
    sphere.scale.y = scale;
    sphere.scale.z = scale;
    sphere.pose.position.x = goal_point(0);
    sphere.pose.position.y = goal_point(1);
    sphere.pose.position.z = 0.0;

    goal_point_pub->publish(sphere);
  }

  void PlanningVisualization::displayGlobalTraj(vector<Eigen::Vector2d> init_pts, const double scale, int id)
  {

    if (global_traj_pub->get_subscription_count() == 0)
    {
      return;
    }

    vector<Eigen::Vector3d> init_pts_3d;
    for(unsigned int i = 0; i < init_pts.size(); ++i){
      init_pts_3d.push_back(Eigen::Vector3d(init_pts[i](0), init_pts[i](1), 0.0));
    }

    Eigen::Vector4d color(0, 0, 1, 1);
    displayMarkerList(global_traj_pub, init_pts_3d, scale, color, id);
  }

  void PlanningVisualization::displayOptimizingTraj(vector<vector<Eigen::Vector2d>> trajs, const double scale)
  {

    if (optmizing_traj_pub->get_subscription_count() == 0)
    {
      return;
    }

    // 优化过程中轨迹段数量会变化。先用透明 LINE_STRIP 覆盖旧 ID，
    // 再发布当前结果，避免 RViz 保留上一轮多出来的轨迹段。
    static int last_nums = 0;

    for ( int id=0; id<last_nums; id++ )
    {
      Eigen::Vector4d color(0, 0, 0, 0);
      vector<Eigen::Vector3d> blank;
      displayMarkerList(optmizing_traj_pub, blank, scale, color, id, false);
      rclcpp::sleep_for(std::chrono::milliseconds(1));
    }
    last_nums = 0;

    for ( int id=0; id<(int)trajs.size(); id++ )
    {
      vector<Eigen::Vector3d> init_pts_3d;
      for(unsigned int i = 0; i < trajs[id].size(); ++i){
        init_pts_3d.push_back(Eigen::Vector3d(trajs[id][i](0), trajs[id][i](1), 0.0));
      }
      Eigen::Vector4d color(0, 0, 1, 0.7);
      displayMarkerList(optmizing_traj_pub, init_pts_3d, scale, color, id, false);
      rclcpp::sleep_for(std::chrono::milliseconds(1));
      last_nums++;
    }

  }

  void PlanningVisualization::displayInitCtrlPts(vector<Eigen::Vector2d> init_pts, const double scale, int id)
  {
    if (init_ctrl_pts_pub->get_subscription_count() == 0)
    {
      return;
    }

    vector<Eigen::Vector3d> pts_3d;
    for(unsigned int i = 0; i < init_pts.size(); ++i){
      pts_3d.push_back(Eigen::Vector3d(init_pts[i](0), init_pts[i](1), 0.0));
    }

    Eigen::Vector4d color(0, 0, 1, 1);
    displayMarkerList(init_ctrl_pts_pub, pts_3d, scale, color, id);
  }

  void PlanningVisualization::displayInitWaypoints(vector<Eigen::Vector2d> pts, const double scale, int id)
  {
    if (init_waypoints_pub->get_subscription_count() == 0)
    {
      return;
    }

    vector<Eigen::Vector3d> pts_3d;
    for(unsigned int i = 0; i < pts.size(); ++i){
      pts_3d.push_back(Eigen::Vector3d(pts[i](0), pts[i](1), 0.0));
    }

    Eigen::Vector4d color(0, 0, 0, 1);
    displayMarkerList(init_waypoints_pub, pts_3d, scale, color, id, true);
  }

  void PlanningVisualization::displayOptWaypoints(vector<Eigen::Vector2d> pts, const double scale, int id)
  {
    if (optimal_waypoints_pub->get_subscription_count() == 0)
    {
      return;
    }
    
    if(id == 0){
      visualization_msgs::msg::Marker MarkerDelete;
      MarkerDelete.action = visualization_msgs::msg::Marker::DELETEALL;
      optimal_waypoints_pub->publish(MarkerDelete);
    }

    vector<Eigen::Vector3d> opt_pts_3d;
    for(unsigned int i = 0; i < pts.size(); ++i){
      opt_pts_3d.push_back(Eigen::Vector3d(pts[i](0), pts[i](1), 0.0));
    }

    Eigen::Vector4d color(0, 1, 1, 1);
    displayMarkerList(optimal_waypoints_pub, opt_pts_3d, scale, color, id);
  }

  void PlanningVisualization::displayInitPathListDebug(vector<Eigen::Vector2d> init_pts, const double scale, int id)
  {

    if (init_list_debug_pub->get_subscription_count() == 0)
    {
      return;
    }

    vector<Eigen::Vector3d> init_pts_3d;
    for(unsigned int i = 0; i < init_pts.size(); ++i){
      init_pts_3d.push_back(Eigen::Vector3d(init_pts[i](0), init_pts[i](1), 0.0));
    }

    Eigen::Vector4d color(1, 1, 0, 1);
    displayMarkerList(init_list_debug_pub, init_pts_3d, scale, color, id);
  }

  void PlanningVisualization::displayOptimalCtrlPts(std::vector<Eigen::MatrixXd> optimal_pts_list, int id)
  {

    if (optimal_ctrl_pts_pub->get_subscription_count() == 0)
    {
      return;
    }

    if(id == 0){
      visualization_msgs::msg::Marker MarkerDelete;
      MarkerDelete.action = visualization_msgs::msg::Marker::DELETEALL;
      optimal_ctrl_pts_pub->publish(MarkerDelete);
    }

    // 每个矩阵的一列是一个控制点；多个 singul 段被展平成一个 Marker。
    vector<Eigen::Vector3d> list;
    for(unsigned int j = 0; j < optimal_pts_list.size(); ++j){
      Eigen::MatrixXd optimal_pts = optimal_pts_list[j];
      for (int i = 0; i < (int)optimal_pts.cols(); i++)
      {
        Eigen::Vector3d pt;
        pt.setZero();
        pt.head(2) = optimal_pts.col(i).transpose().head(2);
        list.push_back(pt);
      }
    }
    
    Eigen::Vector4d color(1.0, 0.0, 0, 0.6);
    displayMarkerList(optimal_ctrl_pts_pub, list, 0.08, color, id);
  }

  void PlanningVisualization::displayFailedList(Eigen::MatrixXd failed_pts, int id)
  {

    if (failed_list_pub->get_subscription_count() == 0)
    {
      return;
    }

    if(id == 0){
      visualization_msgs::msg::Marker MarkerDelete;
      MarkerDelete.action = visualization_msgs::msg::Marker::DELETEALL;
      failed_list_pub->publish(MarkerDelete);
    }

    vector<Eigen::Vector3d> list;
    Eigen::Vector3d pt;
    pt.setZero();
    for (int i = 0; i < failed_pts.cols(); i++)
    {
      pt.head(2) = failed_pts.col(i).head(2);
      list.push_back(pt);
    }
    Eigen::Vector4d color(0.3, 0, 0, 1);
    displayMarkerList(failed_list_pub, list, 0.05, color, id);
  }

  void PlanningVisualization::displayAStarList(std::vector<std::vector<Eigen::Vector2d>> a_star_paths, int id /* = Eigen::Vector4d(0.5,0.5,0,1)*/)
  {

    if (a_star_list_pub->get_subscription_count() == 0)
    {
      return;
    }

    int i = 0;
    vector<Eigen::Vector3d> list;

    Eigen::Vector4d color = Eigen::Vector4d(0.5 + ((double)rand() / RAND_MAX / 2), 0.5 + ((double)rand() / RAND_MAX / 2), 0, 1); // make the A star pathes different every time.
    double scale = 0.05 + (double)rand() / RAND_MAX / 10;

    for (auto block : a_star_paths)
    {
      
      for (auto pt : block)
      {
        list.push_back(Eigen::Vector3d(pt(0), pt(1), 0.1));
      }
      //Eigen::Vector4d color(0.5,0.5,0,1);
      
      i++;
    }
    displayMarkerList(a_star_list_pub, list, scale, color, id); // real ids used: [ id ~ id+a_star_paths.size() ]
  }

  void PlanningVisualization::displayArrowList(const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr &pub, const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id)
  {
    visualization_msgs::msg::MarkerArray array;
    // 发布空数组不能删除 RViz 中已有 Marker，因此真正的清理依赖稳定的
    // namespace/id；这里先发布空数组，再发布当前箭头集合。
    pub->publish(array);

    generateArrowDisplayArray(array, list, scale, color, id);

    pub->publish(array);
  }

  void PlanningVisualization::displayIntermediatePt(std::string type, Eigen::MatrixXd &pts, int id, Eigen::Vector4d color)
  {
    std::vector<Eigen::Vector3d> pts_;
    pts_.reserve(pts.cols());
    for ( int i=0; i<pts.cols(); i++ )
    {
      pts_.emplace_back(pts.col(i));
    }

    if ( !type.compare("0") )
    {
      displayMarkerList(intermediate_pt0_pub, pts_, 0.1, color, id);
    }
    else if ( !type.compare("1") )
    {
      displayMarkerList(intermediate_pt1_pub, pts_, 0.1, color, id);
    }
  }

  void PlanningVisualization::displayIntermediateGrad(std::string type, Eigen::MatrixXd &pts, Eigen::MatrixXd &grad, int id, Eigen::Vector4d color)
  {
    if ( pts.cols() != grad.cols() )
    {
      RCLCPP_ERROR(node_->get_logger(), "pts.cols() != grad.cols()");
      return;
    }
    std::vector<Eigen::Vector3d> arrow_;
    arrow_.reserve(pts.cols()*2);
    if ( !type.compare("swarm") )
    {
      for ( int i=0; i<pts.cols(); i++ )
      {
        arrow_.emplace_back(pts.col(i));
        arrow_.emplace_back(grad.col(i));
      }
    }
    else
    {
      for ( int i=0; i<pts.cols(); i++ )
      {
        arrow_.emplace_back(pts.col(i));
        arrow_.emplace_back(pts.col(i)+grad.col(i));
      }
    }
    

    if ( !type.compare("grad0") )
    {
      displayArrowList(intermediate_grad0_pub, arrow_, 0.05, color, id);
    }
    else if ( !type.compare("grad1") )
    {
      displayArrowList(intermediate_grad1_pub, arrow_, 0.05, color, id);
    }
    else if ( !type.compare("dist") )
    {
      displayArrowList(intermediate_grad_dist_pub, arrow_, 0.05, color, id);
    }
    else if ( !type.compare("smoo") )
    {
      displayArrowList(intermediate_grad_smoo_pub, arrow_, 0.05, color, id);
    }
    else if ( !type.compare("feas") )
    {
      displayArrowList(intermediate_grad_feas_pub, arrow_, 0.05, color, id);
    }
    
  }

  // PlanningVisualization::
} // namespace remani_planner
