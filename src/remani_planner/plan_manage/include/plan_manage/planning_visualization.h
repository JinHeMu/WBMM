#ifndef _PLANNING_VISUALIZATION_H_
#define _PLANNING_VISUALIZATION_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <stdlib.h>
#include <nav_msgs/msg/odometry.hpp>
#include <fstream>
#include "traj_utils/plan_container.hpp"
using std::vector;
namespace remani_planner
{
  class PlanningVisualization
  {
  private:
    rclcpp::Node::SharedPtr node_;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_point_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr global_traj_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr init_ctrl_pts_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr optmizing_traj_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr init_waypoints_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr optimal_ctrl_pts_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr optimal_waypoints_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr failed_list_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr a_star_list_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr guide_vector_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr init_list_debug_pub;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr intermediate_pt0_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr intermediate_pt1_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr intermediate_grad0_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr intermediate_grad1_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr intermediate_grad_smoo_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr intermediate_grad_dist_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr intermediate_grad_feas_pub;

    bool start_visual_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr mm_car_odom_sub_;

    rclcpp::TimerBase::SharedPtr benchmark_recorder;

    std::ofstream odom_csv;
    rclcpp::Time t_init;
    rclcpp::Time t_record;

  public:

    PlanningVisualization(/* args */) {}

    PlanningVisualization(rclcpp::Node::SharedPtr nh);

    typedef std::shared_ptr<PlanningVisualization> Ptr;

    void displayMarkerList(const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr &pub, const vector<Eigen::Vector3d> &list, double scale,
                           Eigen::Vector4d color, int id,  bool show_sphere = true);
    void generatePathDisplayArray(visualization_msgs::msg::MarkerArray &array,
                                  const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id);
    void generateArrowDisplayArray(visualization_msgs::msg::MarkerArray &array,
                                   const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id);
    void displayGoalPoint(Eigen::Vector2d goal_point, Eigen::Vector4d color, const double scale, int id);
    void displayGlobalTraj(vector<Eigen::Vector2d> global_pts, const double scale, int id);
    void displayInitCtrlPts(vector<Eigen::Vector2d> init_pts, const double scale, int id);
    void displayInitWaypoints(vector<Eigen::Vector2d> pts, const double scale, int id);
    void displayOptimizingTraj(vector<vector<Eigen::Vector2d>> init_trajs, const double scale);
    void displayOptimalCtrlPts(std::vector<Eigen::MatrixXd> optimal_pts, int id);
    void displayOptWaypoints(vector<Eigen::Vector2d> pts, const double scale, int id);
    void displayFailedList(Eigen::MatrixXd failed_pts, int id);
    void displayAStarList(std::vector<std::vector<Eigen::Vector2d>> a_star_paths, int id);
    void displayArrowList(const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr &pub, const vector<Eigen::Vector3d> &list, double scale, Eigen::Vector4d color, int id);
    void displayInitPathListDebug(vector<Eigen::Vector2d> init_pts, const double scale, int id);

    void displayIntermediatePt(std::string type, Eigen::MatrixXd &pts, int id, Eigen::Vector4d color);
    void displayIntermediateGrad(std::string type, Eigen::MatrixXd &pts, Eigen::MatrixXd &grad, int id, Eigen::Vector4d color);
  };
} // namespace remani_planner
#endif
