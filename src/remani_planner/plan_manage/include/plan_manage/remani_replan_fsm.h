#ifndef _REBO_REPLAN_FSM_H_
#define _REBO_REPLAN_FSM_H_

#include <fstream>
#include <array>
#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <optimizer/poly_traj_optimizer.hpp>
#include <plan_env/grid_map.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <traj_utils/msg/data_disp.hpp>
#include <plan_manage/planner_manager.h>
#include <plan_manage/planning_visualization.h>
#include <quadrotor_msgs/msg/polynomial_traj.hpp>
#include <traj_utils/msg/assignment.hpp>
#include <traj_utils/msg/whole_body_goal.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <iostream>
#include <iomanip>
#include <ctime>
#include <chrono>
#include <sstream>
#include <fstream>
#include <functional>
#include <stdexcept>
using std::vector;

namespace remani_planner
{

  class REMANIReplanFSM
  {

  private:
    /* ---------- flag ---------- */
    enum FSM_EXEC_STATE
    {
      INIT,
      WAIT_TARGET,
      GEN_NEW_TRAJ,
      REPLAN_TRAJ,
      EXEC_TRAJ,
      TASK_EXEC,
      // WAIT_GRIPPER,
      EMERGENCY_STOP,
    };
    enum TARGET_TYPE
    {
      MANUAL_TARGET = 1,
      PRESET_TARGET = 2
    };

    /* planning utils */
    MMPlannerManager::Ptr planner_manager_;
    PlanningVisualization::Ptr visualization_;
    traj_utils::msg::DataDisp data_disp_;

    /* parameters */
    int target_type_; // 1 mannual select, 2 hard code
    int wpt_id_;
    double no_replan_thresh_, replan_thresh_;
    std::vector<Eigen::VectorXd, Eigen::aligned_allocator<Eigen::VectorXd>> waypoints_;
    std::vector<double> waypoints_yaw_;
    std::vector<bool> waypoint_gripper_close_;
    bool gripper_flag_;
    int waypoint_num_;
    double planning_horizen_;
    double emergency_time_;
    bool enable_fail_safe_;
    int last_end_id_;
    double replan_trajectory_time_;
    int replan_fail_time_;
    double time_for_gripper_;
    bool global_plan_;
    bool tracking_error_replan_enabled_;
    bool planning_enabled_;
    double tracking_error_position_threshold_;
    double tracking_error_yaw_threshold_;
    double tracking_error_joint_threshold_;
    double tracking_error_persistence_;
    double tracking_error_min_interval_;
    double tracking_error_grace_period_;
    double tracking_goal_position_tolerance_;
    double tracking_goal_yaw_tolerance_;
    double tracking_goal_joint_tolerance_;
    double tracking_error_since_sec_;
    double last_tracking_replan_sec_;

    int mobile_base_dim_, manipulator_dim_, traj_dim_;
    double mobile_base_non_singul_vel_;
    bool odom_twist_in_body_frame_;
    std::vector<std::string> manipulator_joint_names_;
    std::string planning_frame_;

    /* planning data */
    bool have_trigger_, have_target_, have_odom_, have_joint_state_, have_new_target_, have_recv_pre_agent_, have_local_traj_;
    FSM_EXEC_STATE exec_state_;
    int continously_called_times_{0};

    Eigen::VectorXd mm_state_pos_, mm_state_vel_, mm_state_acc_, init_state_;
    bool gripper_state_, rcv_gripper_state_;
    int mm_car_singul_;
    Eigen::Quaterniond mm_car_orient_;
    double mm_car_yaw_, mm_car_yaw_rate_;

    Eigen::VectorXd start_pos_, start_vel_, start_acc_, start_jer_;
    int start_singul_;
    double start_yaw_, end_yaw_;
    Eigen::VectorXd end_pt_;
    Eigen::VectorXd local_target_pt_, local_target_vel_, local_target_acc_;
    int local_target_singul_;

    bool flag_escape_emergency_;
    bool flag_relan_astar_;
    bool try_plan_after_emergency_;

    /* ROS utils */
    rclcpp::Node::SharedPtr node_;
    rclcpp::TimerBase::SharedPtr exec_timer_, safety_timer_, model_vis_timer_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr nav2_goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gripper_state_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr trigger_sub_;
    rclcpp::Subscription<traj_utils::msg::Assignment>::SharedPtr assignment_sub_;
    rclcpp::Subscription<traj_utils::msg::WholeBodyGoal>::SharedPtr whole_body_goal_sub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr replan_pub_, new_pub_;
    rclcpp::Publisher<quadrotor_msgs::msg::PolynomialTraj>::SharedPtr poly_traj_pub_;
    rclcpp::Publisher<traj_utils::msg::DataDisp>::SharedPtr data_disp_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gripper_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr map_state_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reached_pub_, start_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr model_vis_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fsm_state_pub_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr planning_enable_service_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr task_execution_service_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Time t_last_Astar_;

    int map_state_;

    std::vector<double> init_time_list_;
    std::vector<double> opt_time_list_;
    std::vector<double> total_time_list_;

    /* helper functions */
    bool callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj);
    bool callEmergencyStop(Eigen::VectorXd stop_pos, double stop_yaw, const int singul);
    bool planFromGlobalTraj(const int trial_times = 1);
    bool planFromLocalTraj(bool flag_use_poly_init);

    void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
    std::pair<int, REMANIReplanFSM::FSM_EXEC_STATE> timesOfConsecutiveStateCalls();
    void printFSMExecState();

    /* ROS2 callback functions (no TimerEvent parameter) */
    void execFSMCallback();
    void checkCollisionCallback();
    bool planNextWaypoint(const Eigen::VectorXd next_wp, const double nect_yaw);
    void waypointCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void wholeBodyGoalCallback(const traj_utils::msg::WholeBodyGoal::SharedPtr msg);
    void mmCarOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void mmManiOdomCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void gripperCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void publishRobotModel();
    void sendPolyTrajROSMsg();
    void setPlanningEnabled(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response);
    void setTaskExecution(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response);
    bool frontEndPathSearching();
    bool checkCollision();

    /** Helper: get yaw from quaternion */
    static double getYawFromQuaternion(const geometry_msgs::msg::Quaternion &q);

  public:
    REMANIReplanFSM(/* args */)
    {
    }
    ~REMANIReplanFSM();

    void init(rclcpp::Node::SharedPtr node);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace remani_planner

#endif
