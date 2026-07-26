/**
 * @file remani_replan_fsm.cpp
 * @brief REMANI-Planner 有限状态机 (FSM) 实现
 *
 * ============================================================
 *  核心职责: 以 100Hz 频率调度全局规划→执行→重规划→紧急停止
 * ============================================================
 *
 * FSM 状态流转:
 *   INIT ──(有里程计)──→ WAIT_TARGET ──(收到目标)──→ GEN_NEW_TRAJ
 *     ↑                                              │
 *     │                                              ▼ success
 *     │                                          EXEC_TRAJ
 *     │                                         ↙    │    ↘
 *     │                                  碰撞/重规划   │  到达终点
 *     │                                      │        │
 *     │                                      ▼        ▼
 *     │                                  REPLAN_TRAJ  WAIT_TARGET
 *     │                                      │
 *     │                              ┌───────┴────────┐
 *     │                              ▼                ▼
 *     │                           success         fail×20
 *     │                              │                │
 *     │                              └──→ EXEC_TRAJ   │
 *     │                                               ▼
 *     │     ┌────────────────────────────────── WAIT_TARGET (放弃)
 *     │     │
 *     │     │  碰撞即将发生 & 重规划失败
 *     │     ▼
 *     │  EMERGENCY_STOP ──(静止后)──→ GEN_NEW_TRAJ (重新规划)
 *     └───────────────────────────────────────┘
 *
 * @note 这是一个 ROS1 节点，由 10ms 定时器驱动轮询
 * @see 对应的头文件: remani_replan_fsm.h
 * @see PlannerManager (展开实际规划逻辑): planner_manager.h
 */

#include <plan_manage/remani_replan_fsm.h>

namespace remani_planner
{
  template<typename T>
  static T declareAndGet(const rclcpp::Node::SharedPtr &node,
                         const std::string &name, const T &default_value)
  {
    if (!node->has_parameter(name)) {
      node->declare_parameter<T>(name, default_value);
    }
    return node->get_parameter(name).get_value<T>();
  }

  REMANIReplanFSM::~REMANIReplanFSM() {}

  // ============================================================================
  // 初始化: 加载参数 + 创建子模块 + 注册 ROS 回调
  // ============================================================================
  void REMANIReplanFSM::init(rclcpp::Node::SharedPtr node)
  {
    node_ = node;

    /* ---------- FSM 状态初始化 ---------- */
    exec_state_ = FSM_EXEC_STATE::INIT;
    have_target_ = false;
    have_odom_ = false;
    have_joint_state_ = false;
    have_recv_pre_agent_ = false;
    flag_escape_emergency_ = true;
    try_plan_after_emergency_ = false;
    flag_relan_astar_ = false;
    have_local_traj_ = false;
    replan_fail_time_ = 0;

    /* ---------- FSM 参数 ---------- */
    target_type_ = declareAndGet<int>(node_, "fsm.target_type", -1);
    replan_thresh_ = declareAndGet<double>(node_, "fsm.thresh_replan_time", -1.0);
    no_replan_thresh_ = declareAndGet<double>(node_, "fsm.thresh_no_replan_meter", -1.0);
    planning_horizen_ = declareAndGet<double>(node_, "fsm.planning_horizon", -1.0);
    emergency_time_ = declareAndGet<double>(node_, "fsm.emergency_time", 1.0);
    enable_fail_safe_ = declareAndGet<bool>(node_, "fsm.fail_safe", true);
    replan_trajectory_time_ = declareAndGet<double>(node_, "fsm.replan_trajectory_time", 0.0);
    time_for_gripper_ = declareAndGet<double>(node_, "fsm.time_for_gripper", -1.0);
    global_plan_ = declareAndGet<bool>(node_, "fsm.global_plan", false);
    if(global_plan_) planning_horizen_ = 1.0e3;

    /* ---------- 机器人自由度参数 ---------- */
    mobile_base_dim_ = declareAndGet<int>(node_, "mm.mobile_base_dof", -1);
    manipulator_dim_ = declareAndGet<int>(node_, "mm.manipulator_dof", -1);
    mobile_base_non_singul_vel_ =
        declareAndGet<double>(node_, "mm.mobile_base_non_singul_vel", -1.0);
    odom_twist_in_body_frame_ =
        declareAndGet<bool>(node_, "mm.odom_twist_in_body_frame", true);
    manipulator_joint_names_ =
        declareAndGet<std::vector<std::string>>(
            node_, "mm.urdf_joint_names",
            std::vector<std::string>{"joint_1", "joint_2", "joint_3",
                                     "joint_4", "joint_5", "joint_6"});
    if (static_cast<int>(manipulator_joint_names_.size()) != manipulator_dim_)
    {
      throw std::runtime_error(
          "mm.urdf_joint_names size must match mm.manipulator_dof");
    }

    traj_dim_ = mobile_base_dim_ + manipulator_dim_;

    /* ---------- 状态向量初始化 ---------- */
    mm_state_pos_ = Eigen::VectorXd::Zero(traj_dim_);
    mm_state_vel_ = Eigen::VectorXd::Zero(traj_dim_);
    mm_state_acc_ = Eigen::VectorXd::Zero(traj_dim_);

    gripper_flag_ = true;

    /* ---------- 规划起始状态 ---------- */
    start_pos_.resize(traj_dim_);
    start_vel_.resize(traj_dim_);
    start_acc_.resize(traj_dim_);
    start_jer_.resize(traj_dim_);

    /* ---------- 预设航点加载 (PRESET_TARGET 模式) ---------- */
    waypoint_num_ = declareAndGet<int>(node_, "fsm.waypoint_num", -1);

    waypoints_.clear();
    waypoints_yaw_.clear();
    Eigen::VectorXd wp = Eigen::VectorXd::Zero(traj_dim_);
    double yaw_temp;
    bool gripper_close;
    for (int i = 0; i < waypoint_num_; i++){
      const auto waypoint_name = "fsm.waypoint" + std::to_string(i);
      yaw_temp = declareAndGet<double>(node_, waypoint_name + "_yaw", -1.0);
      waypoints_yaw_.push_back(yaw_temp * M_PI / 180.0);

      gripper_close = declareAndGet<bool>(node_, waypoint_name + "_gripper_close", true);
      waypoint_gripper_close_.push_back(gripper_close);

      std::vector<double> waypoints_temp =
          declareAndGet<std::vector<double>>(node_, waypoint_name, std::vector<double>{});
      for(unsigned int j = 0; j < waypoints_temp.size(); j++){
        wp(j) = waypoints_temp[j];
        if((int)j >= mobile_base_dim_) wp(j) = wp(j) * M_PI / 180.0;
      }
      waypoints_.push_back(wp);
    }

    /* ---------- 性能统计容器 ---------- */
    init_time_list_.clear();
    opt_time_list_.clear();
    total_time_list_.clear();

    rcv_gripper_state_ = false;
    gripper_state_ = false;
    map_state_ = 0;

    /* ============================================================
     *  核心子模块创建
     * ============================================================ */

    // 可视化模块: RViz 显示轨迹、航点、碰撞球等
    visualization_.reset(new PlanningVisualization(node_));
    // 规划管理器: 封装前端搜索 + 后端优化的完整规划管线
    planner_manager_.reset(new MMPlannerManager);
    planner_manager_->initPlanModules(node_, visualization_);

    /* ---------- ROS2 定时器 (100Hz) ---------- */

    // 主 FSM 调度定时器 (10ms): 状态机轮询
    exec_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&REMANIReplanFSM::execFSMCallback, this));
    // 碰撞检测定时器 (10ms): 独立于 FSM 执行碰撞检查
    safety_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&REMANIReplanFSM::checkCollisionCallback, this));

    /* ---------- ROS2 订阅者 ---------- */

    // 里程计: 移动基底位姿 (x, y, yaw, vx, vy)
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "odom_world", 1,
        std::bind(&REMANIReplanFSM::mmCarOdomCallback, this, std::placeholders::_1));
    // 关节状态: 机械臂各关节位置/速度/力矩
    joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "joint_state", 1,
        std::bind(&REMANIReplanFSM::mmManiOdomCallback, this, std::placeholders::_1));
    // 夹爪状态反馈
    gripper_state_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "gripper_state", 1,
        std::bind(&REMANIReplanFSM::gripperCallback, this, std::placeholders::_1));

    /* ---------- ROS2 发布者 ---------- */

    // 多项式轨迹发布: 发给下游控制器执行
    poly_traj_pub_ = node_->create_publisher<quadrotor_msgs::msg::PolynomialTraj>("planning/trajectory", 10);
    // 数据显示: 调试信息
    data_disp_pub_ = node_->create_publisher<traj_utils::msg::DataDisp>("planning/data_display", 100);

    // 夹爪控制指令
    gripper_cmd_pub_ = node_->create_publisher<std_msgs::msg::Bool>("gripper_cmd", 100);
    // 地图状态
    map_state_pub_ = node_->create_publisher<std_msgs::msg::Int32>("/map_generator/map_state", 100);

    // 规划开始/结束信号
    start_pub_ = node_->create_publisher<std_msgs::msg::Bool>("planning/start", 1);
    reached_pub_ = node_->create_publisher<std_msgs::msg::Bool>("planning/finish", 1);

    // 当前机器人网格模型，供两个示例的 RViz2 配置显示。
    // transient_local 让稍后启动的 RViz2 也能立即收到最近一帧。
    model_vis_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/model_vis/vis_mm", rclcpp::QoS(1).reliable().transient_local());
    model_vis_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&REMANIReplanFSM::publishRobotModel, this));

    // 2D Nav Goal 目标点订阅 (RViz 点击)
    waypoint_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/move_base_simple/goal", 1,
        std::bind(&REMANIReplanFSM::waypointCallback, this, std::placeholders::_1));
    // RViz2's SetGoal tool defaults to /goal_pose in the OCS2 MuJoCo RViz
    // configuration. Keep the ROS1-compatible topic above as well so both
    // existing RViz configurations drive the same manual-goal pipeline.
    nav2_goal_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 1,
        std::bind(&REMANIReplanFSM::waypointCallback, this, std::placeholders::_1));
  }

  // ============================================================================
  // FSM 主回调 (100Hz)
  // 每个周期执行的调度逻辑, 通过 switch-case 驱动状态流转
  // ============================================================================
  void REMANIReplanFSM::execFSMCallback()
  {
    exec_timer_->cancel(); // 防止重入: 执行期间暂停定时器

    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 100){
      fsm_num = 0;
      // printFSMExecState();  // 可选的每秒状态打印
    }

    switch (exec_state_){

    // ====================================================================
    // 状态: INIT — 初始化, 等待里程计数据
    // 入口条件: 启动后立即进入
    // 出口条件: 收到里程计 → WAIT_TARGET
    // ====================================================================
    case INIT:
    {
      if (!have_odom_){
        goto force_return; // 无里程计则跳过本轮
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    // ====================================================================
    // 状态: WAIT_TARGET — 等待用户/系统下发目标点
    // 入口条件: 已获取里程计
    // 出口条件: 收到目标 → GEN_NEW_TRAJ
    // ====================================================================
    case WAIT_TARGET:
    {
      if (!have_target_)
        goto force_return; // 无目标则跳过
      else{
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    // ====================================================================
    // 状态: GEN_NEW_TRAJ — 生成全新全局轨迹
    // 入口条件: 收到新目标
    // 逻辑: 尝试 planFromGlobalTraj() 最多10次
    // 出口条件:
    //   ├─ 成功 → EXEC_TRAJ
    //   └─ 失败 → GEN_NEW_TRAJ (重试)
    // ====================================================================
    case GEN_NEW_TRAJ:
    {
      if(try_plan_after_emergency_){
        // 紧急停止后的状态打印 (调试用)
        std::cout << "emergency stop mm pos: " << mm_state_pos_.transpose() << std::endl;
        std::cout << "emergency stop mm vel: " << mm_state_vel_.transpose() << std::endl;
        std::cout << "emergency stop mm acc: " << mm_state_acc_.transpose() << std::endl;
        std::cout << "emergency stop mm yaw: " << mm_car_yaw_ << std::endl;
      }
      have_local_traj_ = false;
      bool success = planFromGlobalTraj(10); // 最多尝试10次
      if (success){
        changeFSMExecState(EXEC_TRAJ, "FSM");
        flag_escape_emergency_ = true;     // 恢复紧急停止能力
        try_plan_after_emergency_ = false;
      }
      else
      {
        // 规划失败 → 重试 (不放弃, 不退回到 WAIT_TARGET)
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    // ====================================================================
    // 状态: REPLAN_TRAJ — 局部重规划 (在已有轨迹基础上调整)
    // 入口条件: EXEC_TRAJ 中检测到需要重规划
    // 逻辑: planFromLocalTraj() 从当前轨迹采样起始状态重新规划
    // 出口条件:
    //   ├─ 成功 → EXEC_TRAJ
    //   ├─ 失败×20 → WAIT_TARGET (放弃任务)
    //   └─ 失败(≤20次) → REPLAN_TRAJ (重试)
    // ====================================================================
    case REPLAN_TRAJ:
    {
      if(planFromLocalTraj(flag_relan_astar_)){
        replan_fail_time_ = 0;
        flag_relan_astar_ = false;
        if((node_->now() - t_last_Astar_ ).seconds() > 1.0){
          std::cout << "cal front end next time" << std::endl;
          flag_relan_astar_ = true;  // 每1秒启用一次前端搜索
          t_last_Astar_ = node_->now();
        }
        changeFSMExecState(EXEC_TRAJ, "FSM");
      }
      else{
        replan_fail_time_++;
        flag_relan_astar_ = true;     // 失败时启用前端搜索
        t_last_Astar_ = node_->now();
        if(replan_fail_time_ >= 20){  // 连续失败20次则放弃
          replan_fail_time_ = 0;
          RCLCPP_ERROR(node_->get_logger(),"[FSM]:REPLAN fail over 20 times!!!");
          changeFSMExecState(WAIT_TARGET, "FSM");
        }
        else{
          changeFSMExecState(REPLAN_TRAJ, "FSM"); // 重试
        }
      }
      break;
    }

    // ====================================================================
    // 状态: EXEC_TRAJ — 执行轨迹, 同时监控是否需要重规划
    // 入口条件: 轨迹生成成功
    // 监控逻辑:
    //   1. 预设航点模式: 到达航点后自动切换到下一航点/夹爪操作
    //   2. 到达终点: 发布完成信号, 回到 WAIT_TARGET
    //   3. 超出重规划时间阈值: 触发 REPLAN_TRAJ
    // ====================================================================
    case EXEC_TRAJ:
    {
      /* determine if need to replan */
      SingulTrajData *info = &planner_manager_->traj_container_.singul_traj_data;
      double t_cur = node_->now().seconds() - info->start_time; // 轨迹执行到的相对时间
      bool need_to_plan_next = ((t_cur - info->duration) > time_for_gripper_);
      bool need_to_gripper = (t_cur > info->duration + 0.01);
      t_cur = min(info->duration, t_cur); // 钳位到轨迹时长内

      Eigen::VectorXd pos = info->getPos(t_cur);                  // 当前跟踪位置
      bool touch_the_goal = ((local_target_pt_ - end_pt_).norm() < 1e-2);  // 局部目标≈终点?
      bool close_to_no_replan_thresh = ((end_pt_ - pos).head(2).norm() < no_replan_thresh_); // 离终点足够近?

      // --- 预设航点模式 ---
      if((target_type_ == TARGET_TYPE::PRESET_TARGET) && close_to_no_replan_thresh){
        // 还有后续航点: 规划下一段
        if((wpt_id_ < waypoint_num_ - 1) && need_to_plan_next){
          ++wpt_id_;
          planNextWaypoint(waypoints_[wpt_id_], waypoints_yaw_[wpt_id_]);
          gripper_flag_ = true;
        // 到达航点且需要夹爪操作
        }else if(need_to_gripper && gripper_flag_){
          ++map_state_;
          std_msgs::msg::Bool gripper_cmd;
          gripper_cmd.data = waypoint_gripper_close_[wpt_id_]; // true: 夹紧; false: 松开
          gripper_cmd_pub_->publish(gripper_cmd);

          std::string gripper_cmd_str = waypoint_gripper_close_[wpt_id_] ? "close gripper" : "open gripper";
          RCLCPP_INFO(node_->get_logger(),gripper_cmd_str.c_str());

          std_msgs::msg::Int32 map_state;
          map_state.data = map_state_;
          gripper_flag_ = false;
        }
      // --- 到达终点: 任务完成 ---
      }else if(t_cur > info->duration - 1e-2 && touch_the_goal){
        if(target_type_ != TARGET_TYPE::PRESET_TARGET && wpt_id_ >= waypoint_num_ - 1){
          have_target_ = false;
          have_trigger_ = false;
          std::cout << "reach goal\n";
          changeFSMExecState(WAIT_TARGET, "FSM");

          std_msgs::msg::Bool msg;
          msg.data = true;
          reached_pub_->publish(msg);  // 通知外部: 导航完成
          goto force_return;
        }
      // --- 需要重规划: 超过时间阈值且未接近终点 ---
      }else if(!close_to_no_replan_thresh && t_cur > replan_thresh_ && (!global_plan_)){
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    // ====================================================================
    // 状态: EMERGENCY_STOP — 紧急停止
    // 触发条件: checkCollisionCallback 检测到碰撞即将发生
    // 动作: 发布停止轨迹 (轨迹ID_ABORT)
    // 出口条件:
    //   ├─ 机器人静止(速度<0.1) → GEN_NEW_TRAJ
    //   └─ 等待静止中 → 保持 EMERGENCY_STOP
    // ====================================================================
    case EMERGENCY_STOP:
    {
      if(flag_escape_emergency_){
        callEmergencyStop(mm_state_pos_, mm_car_yaw_, mm_car_singul_); // 首次进入: 执行急停
      }
      else{
        if(enable_fail_safe_ && mm_state_vel_.head(2).norm() < 0.1){
          try_plan_after_emergency_ = true;  // 标记需要重规划
          have_local_traj_ = false;
          changeFSMExecState(GEN_NEW_TRAJ, "FSM"); // 静止后重新规划
        }
      }
      flag_escape_emergency_ = false;  // 防止重复调用急停
      break;
    }
    }

    /* 发布调试数据 */
    data_disp_.header.stamp = node_->now();
    data_disp_pub_->publish(data_disp_);

  force_return:;  // goto 跳转目标: 跳过本轮剩余逻辑
    exec_timer_->reset();  // 重启定时器
  }

  // ============================================================================
  // 碰撞检测回调 (100Hz, 独立于 FSM 主循环)
  // 沿着当前轨迹采样, 检测未来碰撞:
  //   - 若碰撞剩余时间 < emergency_time_ → 直接 EMERGENCY_STOP
  //   - 若碰撞剩余时间 ≥ emergency_time_ → 尝试重规划
  // ============================================================================
  void REMANIReplanFSM::checkCollisionCallback(){
    SingulTrajData *info = &planner_manager_->traj_container_.singul_traj_data;
    auto map = planner_manager_->grid_map_;

    if (exec_state_ == WAIT_TARGET || info->traj_id <= 0)
      return;

    /* ---------- 深度丢失检测 ---------- */
    if (map->getOdomDepthTimeout()){
      RCLCPP_ERROR(node_->get_logger(),"Depth Lost! EMERGENCY_STOP");
      enable_fail_safe_ = false;          // 深度丢失时不启用故障安全
      changeFSMExecState(EMERGENCY_STOP, "SAFETY");
    }

    /* ---------- 沿轨迹采样碰撞检测 ---------- */
    constexpr double time_step = 0.01;    // 10ms 采样步长
    double t_cur = node_->now().seconds() - info->start_time;
    Eigen::VectorXd p_cur = info->getPos(t_cur);
    double t_1_2 = info->duration * 1 / 2; // 轨迹前1/2分界点
    double t_2_3 = info->duration * 2 / 3; // 轨迹后2/3分界点
    double t_temp;
    bool occ = false;
    int coll_type;  // 碰撞类型: 0=car-obs, 1=mani-obs, 2=car-mani, 3=mani-mani

    for (double t = t_cur; t < info->duration; t += time_step){
      // 优化: 如果当前时间 < t_1_2, 只检查到 t_2_3 为止
      // (更远的轨迹段不确定性大, 且控制器有能力处理)
      if (t_cur < t_1_2 && t >= t_2_3)
        break;

      if (planner_manager_->ploy_traj_opt_->checkCollision(*info, t, coll_type)){
        if(coll_type == 0){
          RCLCPP_WARN(node_->get_logger(),"car collision at relative time %f!", t / info->duration);
        }else if (coll_type == 1){
          RCLCPP_WARN(node_->get_logger(),"mani collision at relative time %f!", t / info->duration);
        }else if (coll_type == 2){
          RCLCPP_WARN(node_->get_logger(),"car-mani collision at relative time %f!", t / info->duration);
        }else if (coll_type == 3){
          RCLCPP_WARN(node_->get_logger(),"mani-mani collision at relative time %f!", t / info->duration);
        }
        t_temp = t;
        occ = true;
        break;
      }
    }

    if (occ){
      /* 碰撞处理策略 */
      RCLCPP_INFO(node_->get_logger(),"Try to replan a safe trajectory");
      // 策略1: 先尝试轻量级重规划 (不使用A*)
      if (planFromLocalTraj(false)){
        RCLCPP_INFO(node_->get_logger(),"Plan success when detect collision.");
        changeFSMExecState(EXEC_TRAJ, "SAFETY");
        return;
      }else{
        // 策略2: 如果碰撞即将在 emergency_time_ 内发生 → 紧急停止
        if (t_temp - t_cur < emergency_time_){
          RCLCPP_WARN(node_->get_logger(),"Emergency stop! time=%f", t_temp - t_cur);
          changeFSMExecState(EMERGENCY_STOP, "SAFETY");
        }else{
          // 策略3: 碰撞时间充裕, 使用 A* 重规划
          RCLCPP_WARN(node_->get_logger(),"current traj in collision, replan.");
          if(planFromLocalTraj(true))
          {
            RCLCPP_INFO(node_->get_logger(),"Plan success when detect collision.");
            changeFSMExecState(EXEC_TRAJ, "SAFETY");
            return;
          }
          changeFSMExecState(REPLAN_TRAJ, "SAFETY");
        }
        return;
      }
    }
  }

  // ============================================================================
  // 规划下一航点轨迹 (用于 PRESET_TARGET 模式)
  //
  // @param next_wp  下一个航点 [x, y, q1, ..., qN]
  // @param next_yaw 下一个航点的偏航角
  // @return true 规划成功
  //
  // 流程:
  //   1. 调用 planner_manager_->planGlobalTrajWaypoints() 生成全局轨迹
  //   2. 更新 end_pt_ 为新的航点
  //   3. 显示轨迹到 RViz
  //   4. 如果当前 FSM 不在 WAIT_TARGET, 强制触发 GEN_NEW_TRAJ
  // ============================================================================
  bool REMANIReplanFSM::planNextWaypoint(const Eigen::VectorXd next_wp, const double next_yaw)
  {
    std::vector<Eigen::VectorXd> one_pt_wps;
    one_pt_wps.push_back(next_wp);
    bool success = planner_manager_->planGlobalTrajWaypoints(
        mm_state_pos_, mm_car_yaw_,
        Eigen::VectorXd::Zero(traj_dim_), Eigen::VectorXd::Zero(traj_dim_),
        one_pt_wps, next_yaw,
        Eigen::VectorXd::Zero(traj_dim_), Eigen::VectorXd::Zero(traj_dim_));

    if (success)
    {
      end_pt_ = next_wp;
      end_yaw_ = next_yaw;
      have_local_traj_ = false;
      start_singul_ = 0;

      /*** 显示全局轨迹到 RViz ***/
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->traj_container_.global_traj.duration / step_size_t);
      vector<Eigen::Vector2d> global_traj(i_end);
      for (int i = 0; i < i_end; i++){
        global_traj[i] = planner_manager_->traj_container_.global_traj.traj.getPos(i * step_size_t).head(mobile_base_dim_);
      }

      have_target_ = true;
      have_new_target_ = true;

      /*** 强制 FSM 进入 GEN_NEW_TRAJ ***/
      if (exec_state_ != WAIT_TARGET)
      {
        while (exec_state_ != EXEC_TRAJ)
        {
          rclcpp::spin_some(node_);;
          std::this_thread::sleep_for(std::chrono::milliseconds(1));;
        }
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      }

      visualization_->displayGoalPoint(end_pt_.head(2), Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      visualization_->displayGlobalTraj(global_traj, 0.05, 0);
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(),"Unable to generate global trajectory!");
    }

    return success;
  }

  // ============================================================================
  // 2D Nav Goal 目标点回调 (RViz 点击 / 外部发送)
  //
  // 根据 target_type_ 区分处理:
  //   PRESET_TARGET: 触发预设航点序列 (忽略消息内容)
  //   MANUAL_TARGET: 从消息中解析 (x, y, yaw) 作为目标
  // ============================================================================
  void REMANIReplanFSM::waypointCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
    if(!have_odom_ || !have_joint_state_){
      RCLCPP_WARN(node_->get_logger(), "Ignoring goal until odometry and joint state are ready");
      return;
    }

    if (target_type_ == TARGET_TYPE::PRESET_TARGET){
      have_trigger_ = true;
      RCLCPP_INFO(node_->get_logger(), "Triggered preset waypoint sequence");

      std_msgs::msg::Bool flag_msg;
      flag_msg.data = true;
      planner_manager_->global_start_time_ = node_->now();
      planner_manager_->start_flag_ = true;
      start_pub_->publish(flag_msg);   // 通知规划开始
      wpt_id_ = 0;
      planNextWaypoint(waypoints_[wpt_id_], waypoints_yaw_[wpt_id_]);
      return;
    }

    // z < -0.1 的点击忽略 (过滤地面以下点击)
    if(msg->pose.position.z < -0.1)
      return;

    init_state_ = mm_state_pos_;
    // A 2D goal only changes the mobile-base pose. Keep the current arm
    // configuration instead of silently commanding all joints to zero.
    end_pt_ = mm_state_pos_;

    if(target_type_ == TARGET_TYPE::MANUAL_TARGET){
      end_pt_(0) = msg->pose.position.x;
      end_pt_(1) = msg->pose.position.y;
      end_yaw_ = tf2::getYaw(msg->pose.orientation);
      RCLCPP_INFO(
          node_->get_logger(), "Received manual 2D goal: x=%.3f, y=%.3f, yaw=%.3f rad",
          end_pt_(0), end_pt_(1), end_yaw_);
    }else{
      RCLCPP_ERROR(node_->get_logger(),"wrong target type: %d", target_type_);
      return;
    }

    planNextWaypoint(end_pt_, end_yaw_);
  }

  // ============================================================================
  // 移动基底里程计回调
  // 更新: 位置 (x, y), 偏航角, 速度, 前进/后退标志
  //
  // 奇异速度处理:
  //   当线速度 < mobile_base_non_singul_vel_ (近似静止/换向) 时,
  //   将速度方向强制设为朝向当前 yaw 方向, 避免数值不稳定
  // ============================================================================
  void REMANIReplanFSM::mmCarOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    mm_state_pos_(0) = msg->pose.pose.position.x;
    mm_state_pos_(1) = msg->pose.pose.position.y;
    mm_car_yaw_ = tf2::getYaw(msg->pose.pose.orientation);

    // 四元数保存 (用于可视化/坐标变换)
    mm_car_orient_.w() = msg->pose.pose.orientation.w;
    mm_car_orient_.x() = msg->pose.pose.orientation.x;
    mm_car_orient_.y() = msg->pose.pose.orientation.y;
    mm_car_orient_.z() = msg->pose.pose.orientation.z;

    if (odom_twist_in_body_frame_)
    {
      const double body_vx = msg->twist.twist.linear.x;
      const double body_vy = msg->twist.twist.linear.y;
      mm_state_vel_(0) =
          std::cos(mm_car_yaw_) * body_vx -
          std::sin(mm_car_yaw_) * body_vy;
      mm_state_vel_(1) =
          std::sin(mm_car_yaw_) * body_vx +
          std::cos(mm_car_yaw_) * body_vy;
    }
    else
    {
      mm_state_vel_(0) = msg->twist.twist.linear.x;
      mm_state_vel_(1) = msg->twist.twist.linear.y;
    }

    // 判断前进/后退状态 (singul = singular direction)
    if(mm_state_vel_.head(2).norm() < mobile_base_non_singul_vel_){
      // 速度接近于零 → 视为"奇异点" (即将换向或静止)
      // 强制将速度方向指向当前朝向, 保持数值稳定
      mm_state_vel_(0) = mobile_base_non_singul_vel_ * cos(mm_car_yaw_);
      mm_state_vel_(1) = mobile_base_non_singul_vel_ * sin(mm_car_yaw_);
      mm_car_singul_ = 0;  // 0: 奇异区 (换向过渡)
    }else{
      // 正常运动: 判断前进(1)还是后退(-1)
      Eigen::Vector2d car_head(cos(mm_car_yaw_), sin(mm_car_yaw_));
      mm_car_singul_ = car_head.dot(mm_state_vel_.head(2)) >= 0 ? 1 : -1;
    }

    mm_car_yaw_rate_ = msg->twist.twist.angular.z;
    have_odom_ = true;
  }

  // ============================================================================
  // 机械臂关节状态回调
  //
  // 从 sensor_msgs/JointState 中提取:
  //   position → mm_state_pos_ (用于规划起始状态)
  //   velocity → mm_state_vel_
  //   effort   → mm_state_acc_ (这里将力矩作为加速度近似)
  // ============================================================================
  void REMANIReplanFSM::mmManiOdomCallback(const sensor_msgs::msg::JointState::SharedPtr msg){
    if(msg->position.size() < static_cast<size_t>(manipulator_dim_)){
      RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 2000,
          "JointState has %zu positions, but %d are required",
          msg->position.size(), manipulator_dim_);
      return;
    }
    for(int i = 0; i < manipulator_dim_; ++i){
      size_t source_index = static_cast<size_t>(i);
      if (!msg->name.empty())
      {
        const auto it = std::find(
            msg->name.begin(), msg->name.end(),
            manipulator_joint_names_[static_cast<size_t>(i)]);
        if (it == msg->name.end())
        {
          RCLCPP_WARN_THROTTLE(
              node_->get_logger(), *node_->get_clock(), 2000,
              "JointState is missing required joint '%s'",
              manipulator_joint_names_[static_cast<size_t>(i)].c_str());
          return;
        }
        source_index =
            static_cast<size_t>(std::distance(msg->name.begin(), it));
      }
      mm_state_pos_(mobile_base_dim_ + i) = msg->position[source_index];
      mm_state_vel_(mobile_base_dim_ + i) =
          msg->velocity.size() > source_index ? msg->velocity[source_index] : 0.0;
      mm_state_acc_(mobile_base_dim_ + i) =
          msg->effort.size() > source_index ? msg->effort[source_index] : 0.0;
    }
    have_joint_state_ = true;
  }

  void REMANIReplanFSM::publishRobotModel(){
    // 这是“当前实测状态”的 RViz 管线，与规划轨迹显示相互独立：
    // 里程计/关节状态 -> MMConfig 正运动学 -> Mesh MarkerArray -> RViz。
    // 使用 idx=0 和固定 namespace，使每一帧覆盖上一帧的模型。
    if(!have_odom_ || !have_joint_state_ || planner_manager_ == nullptr
       || planner_manager_->mm_config_ == nullptr)
      return;

    const Eigen::Vector3d car_state(mm_state_pos_(0), mm_state_pos_(1), mm_car_yaw_);
    const Eigen::VectorXd joint_state =
        mm_state_pos_.segment(mobile_base_dim_, manipulator_dim_);
    visualization_msgs::msg::MarkerArray marker_array;
    planner_manager_->mm_config_->getMMMarkerArray(
        marker_array, "vis_mm_odom", 0, 1.0,
        car_state, joint_state, gripper_state_);
    model_vis_pub_->publish(marker_array);

    RCLCPP_INFO_ONCE(
        node_->get_logger(),
        "Publishing current robot model on /model_vis/vis_mm (%zu markers)",
        marker_array.markers.size());
  }

  // ============================================================================
  // 夹爪状态回调
  //
  // 当夹爪状态改变时, 通知 MMConfig 更新碰撞模型 (夹爪开/闭时外形不同)
  // ============================================================================
  void REMANIReplanFSM::gripperCallback(const std_msgs::msg::Bool::SharedPtr msg){
    if(gripper_state_ != msg->data || (!rcv_gripper_state_)){
      rcv_gripper_state_ = true;
      gripper_state_ = msg->data;
      planner_manager_->mm_config_->setGripperPoint(gripper_state_); // 更新碰撞模型
    }
  }

  // ============================================================================
  // FSM 状态切换
  //
  // @param new_state 目标状态
  // @param pos_call  调用位置标识 (用于日志, 如 "FSM", "SAFETY", "TRIG")
  //
  // 维护 continously_called_times_: 同一状态连续被调用的次数
  //   - 用于 GEN_NEW_TRAJ 判断是否使用随机初值 (避免局部最优)
  // ============================================================================
  void REMANIReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call){
    if (new_state == exec_state_)
      continously_called_times_++;
    else
      continously_called_times_ = 1;

    static string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
  }

  // ============================================================================
  // 打印 FSM 当前状态 (调试用, 默认关闭)
  // ============================================================================
  void REMANIReplanFSM::printFSMExecState(){
    static string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};
    static int last_printed_state = -1, dot_nums = 0;

    if (exec_state_ != last_printed_state)
      dot_nums = 0;
    else
      dot_nums++;

    cout << "\r[FSM]: state: " + state_str[int(exec_state_)];

    last_printed_state = exec_state_;

    // 显示当前阻塞原因
    if (!have_odom_)
      cout << ", waiting for odom";
    if (!have_target_)
      cout << ", waiting for target";
    if (!have_trigger_)
      cout << ", waiting for trigger";
    if (planner_manager_->pp_.drone_id >= 1 && !have_recv_pre_agent_)
      cout << ", haven't receive traj from previous drone";

    cout << string(dot_nums, '.') << endl;
    fflush(stdout);
  }

  // ============================================================================
  // 获取当前状态连续调用次数
  //
  // @return pair<连续调用次数, 当前状态>
  //
  // 用途: planFromGlobalTraj() 据此决定是否使用随机初值
  //   首次调用 → 使用 MINCO 平滑初值 (更快)
  //   连续失败 → 使用随机初值 (避免局部最优)
  // ============================================================================
  std::pair<int, REMANIReplanFSM::FSM_EXEC_STATE> REMANIReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continously_called_times_, exec_state_);
  }

  // ============================================================================
  // 发布多项式轨迹到 ROS 消息
  //
  // 将 REMANI 内部的 SingulTrajData 转换为 quadrotor_msgs::msg::PolynomialTraj
  // 每条消息包含一段基底运动方向(singul)对应的多项式轨迹
  // ============================================================================
  void REMANIReplanFSM::sendPolyTrajROSMsg(){
    auto data = &planner_manager_->traj_container_.singul_traj_data;

    for(unsigned int i = 0; i < data->singul_traj.size(); ++i){
      quadrotor_msgs::msg::PolynomialTraj msg;
      msg.trajectory_id = data->singul_traj[i].traj_id;  // 轨迹ID
      msg.header.stamp = rclcpp::Time(static_cast<int64_t>(data->start_time * 1e9));     // 起始时间戳
      msg.action = msg.ACTION_ADD;                        // ADD: 添加新轨迹
      msg.singul = data->singul_traj[i].singul;           // 前进方向: 1=前进, -1=后退

      int piece_num = data->singul_traj[i].traj.getPieceNum();
      for (int j = 0; j < piece_num; ++j)
      {
        quadrotor_msgs::msg::PolynomialMatrix piece;
        piece.num_dim = data->singul_traj[i].traj.getPiece(j).getDim();    // 维度
        piece.num_order = data->singul_traj[i].traj.getPiece(j).getDegree(); // 多项式阶数
        piece.duration = data->singul_traj[i].traj.getPiece(j).getDuration(); // 该段时长
        auto cMat = data->singul_traj[i].traj.getPiece(j).getCoeffMat();  // 系数矩阵
        piece.data.assign(cMat.data(), cMat.data() + cMat.rows()*cMat.cols());
        msg.trajectory.emplace_back(piece);
      }
      poly_traj_pub_->publish(msg);
    }
  }

  // ============================================================================
  // 从全局规划生成轨迹 (GEN_NEW_TRAJ 入口)
  //
  // @param trial_times 最大尝试次数 (默认10)
  // @return true 规划成功
  //
  // 流程:
  //   1. 从里程计获取当前机器人状态 (位置/速度/加速度/yaw/singul)
  //   2. 判断是否连续调用: 连续失败则启用随机初值 (跳出局部最优)
  //   3. 调用 callReboundReplan() 执行完整的前端+后端规划
  // ============================================================================
  bool REMANIReplanFSM::planFromGlobalTraj(const int trial_times /*= 1*/){
    start_pos_ = mm_state_pos_;
    start_vel_ = mm_state_vel_;
    start_acc_.setZero();
    start_jer_.setZero();
    start_yaw_ = mm_car_yaw_;
    start_singul_ = mm_car_singul_;
    bool flag_random_poly_init;
    // 第一次调用使用 MINCO 初值 (fast), 连续失败则随机初值 (exploration)
    if(timesOfConsecutiveStateCalls().first == 1) flag_random_poly_init = false;
    else flag_random_poly_init = true;
    for(int i = 0; i < trial_times; i++){
      if(callReboundReplan(true, flag_random_poly_init)){
        return true;
      }
    }
    return false;
  }

  // ============================================================================
  // 从局部重规划生成轨迹 (REPLAN_TRAJ 入口)
  //
  // @param flag_use_poly_init 是否使用当前多项式作为初值
  // @return true 规划成功
  //
  // 流程:
  //   1. 从当前执行轨迹的当前位置采样起始状态
  //   2. 调用 callReboundReplan()
  //   3. 若失败, 再尝试一次全随机初值
  // ============================================================================
  bool REMANIReplanFSM::planFromLocalTraj(bool flag_use_poly_init){
    SingulTrajData *info = &planner_manager_->traj_container_.singul_traj_data;
    double t_cur = node_->now().seconds() - info->start_time + replan_trajectory_time_;
    t_cur = min(info->duration, t_cur);

    // 从当前执行的轨迹上获取起始状态 (热启动)
    start_pos_     = info->getPos(t_cur);
    start_vel_    = info->getVel(t_cur);
    start_acc_    = info->getAcc(t_cur);
    start_jer_   = info->getJer(t_cur);
    start_singul_ = info->getSingul(t_cur);
    if(start_vel_.norm() >= mobile_base_non_singul_vel_)
      start_yaw_ = atan2(start_singul_ * start_vel_(1), start_singul_ * start_vel_(0));
    else
      start_yaw_ = mm_car_yaw_;

    bool success = callReboundReplan(flag_use_poly_init, false);
    if (!success){
      // 一次失败 → 使用完全随机初值再试
      for (int i = 0; i < 1; i++){
        success = callReboundReplan(true, true);
        if (success)
          break;
      }
      if (!success)
        return false;
    }
    return true;
  }

  // ============================================================================
  // 核心规划调用: 前端搜索 + 后端优化 (关键函数)
  //
  // @param flag_use_poly_init    是否使用多项式初值 (true=MINCO, false=A*)
  // @param flag_randomPolyTraj   是否使用随机轨迹初值
  // @return true 规划成功
  //
  // 步骤:
  //   1. getLocalTarget():   从全局轨迹上选取局部目标点
  //   2. 构造 desired_start: 热启动状态 (使用已有轨迹或当前状态)
  //   3. reboundReplan():    planner_manager 执行完整规划管线
  //   4. 成功后: 发布轨迹 + 可视化
  // ============================================================================
  bool REMANIReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj){
    /* ---------- Step 1: 获取局部目标 ---------- */
    bool reach_horizon;
    planner_manager_->getLocalTarget(
        planning_horizen_, start_pos_, start_yaw_, end_pt_, end_yaw_,
        local_target_pt_, local_target_vel_, local_target_acc_, reach_horizon);
    bool local_target_gripper;
    if(reach_horizon){
      local_target_gripper = gripper_state_;
    }else{
      local_target_gripper = waypoint_gripper_close_[wpt_id_];
    }
    local_target_acc_.setZero();
    double local_target_yaw = atan2(local_target_vel_(1), local_target_vel_(0));
    local_target_vel_.setZero();
    local_target_vel_.head(2) = mobile_base_non_singul_vel_ * Eigen::Vector2d(cos(local_target_yaw), sin(local_target_yaw));

    /* ---------- Step 2: 构造起始状态 (含热启动) ---------- */
    Eigen::VectorXd desired_start_pt, desired_start_vel, desired_start_acc, desired_start_jerk;
    int desired_start_singul;
    double desired_start_yaw;
    double desired_start_time, start_time_dura;

    if(have_local_traj_)
    {
      // 已有轨迹 → 从中采样热启动状态 (replan_trajectory_time_ 后的状态)
      desired_start_time = node_->now().seconds() + replan_trajectory_time_;
      start_time_dura = desired_start_time - planner_manager_->traj_container_.singul_traj_data.start_time;
      start_time_dura = min(start_time_dura, planner_manager_->traj_container_.singul_traj_data.duration);

      desired_start_pt = planner_manager_->traj_container_.singul_traj_data.getPos(start_time_dura);
      desired_start_vel = planner_manager_->traj_container_.singul_traj_data.getVel(start_time_dura);
      if(desired_start_vel.head(2).norm() < mobile_base_non_singul_vel_){
        desired_start_vel(0) = start_singul_ * mobile_base_non_singul_vel_ * cos(start_yaw_);
        desired_start_vel(1) = start_singul_ * mobile_base_non_singul_vel_ * sin(start_yaw_);
      }
      desired_start_singul = planner_manager_->traj_container_.singul_traj_data.getSingul(start_time_dura);
      desired_start_acc = planner_manager_->traj_container_.singul_traj_data.getAcc(start_time_dura);
      desired_start_jerk = planner_manager_->traj_container_.singul_traj_data.getJer(start_time_dura);
      desired_start_yaw = atan2(desired_start_singul * desired_start_vel(1), desired_start_singul * desired_start_vel(0));
    }else{
      // 无轨迹 → 使用当前机器人状态
      desired_start_time = node_->now().seconds();
      desired_start_pt = start_pos_;
      desired_start_vel = start_vel_;
      if(desired_start_vel.head(2).norm() < mobile_base_non_singul_vel_){
        desired_start_vel(0) = start_singul_ * mobile_base_non_singul_vel_ * cos(start_yaw_);
        desired_start_vel(1) = start_singul_ * mobile_base_non_singul_vel_ * sin(start_yaw_);
      }
      desired_start_acc = start_acc_;
      desired_start_jerk = start_jer_;
      desired_start_yaw = start_yaw_;
      desired_start_singul = start_singul_;
    }

    double init_time, opt_time;

    /* ---------- Step 3: 调用规划管理器 (核心管线) ---------- */
    bool plan_success = planner_manager_->reboundReplan(
        desired_start_pt, desired_start_vel, desired_start_acc, desired_start_jerk,
        desired_start_yaw, desired_start_singul, gripper_state_,
        desired_start_time,
        local_target_pt_, local_target_vel_, local_target_acc_, local_target_yaw, local_target_gripper,
        (have_new_target_ || flag_use_poly_init),  // 是否使用多项式初值
        flag_randomPolyTraj,                        // 是否随机轨迹
        have_local_traj_,                           // 是否有局部轨迹 (热启动)
        init_time, opt_time);
    have_new_target_ = false;

    /* ---------- Step 4: 发布 + 可视化 ---------- */
    if (plan_success){
      init_time_list_.push_back(init_time);    // 记录前端搜索耗时
      opt_time_list_.push_back(opt_time);      // 记录后端优化耗时
      total_time_list_.push_back(init_time + opt_time);
      sendPolyTrajROSMsg();                     // 发布轨迹给控制器
      have_local_traj_ = true;

      // 显示局部轨迹
      int i_end = floor(planner_manager_->traj_container_.singul_traj_data.duration / 0.02);
      std::vector<Eigen::Vector2d> local_path_list;
      Eigen::Vector2d local_traj_pt;
      for(int i = 0; i < i_end; ++i){
        local_traj_pt = planner_manager_->traj_container_.singul_traj_data.getPos(i * 0.02).head(2);
        local_path_list.push_back(local_traj_pt);
      }
      visualization_->displayGlobalTraj(local_path_list, 0.05, 0);
      planner_manager_->ploy_traj_opt_->displayBackEndMesh(
          planner_manager_->traj_container_.singul_traj_data, false, gripper_state_);
    }

    return plan_success;
  }

  // ============================================================================
  // 紧急停止
  //
  // @param stop_pos  停止位置 (当前机器人位置)
  // @param stop_yaw  停止朝向
  // @param singul    停止时前进/后退状态
  //
  // 发布:
  //   1. 零速轨迹给控制器
  //   2. ACTION_ABORT 信号通知控制器取消当前轨迹
  // ============================================================================
  bool REMANIReplanFSM::callEmergencyStop(Eigen::VectorXd stop_pos, double stop_yaw, const int singul){
    std::cout << "\033[31mcall EmergencyStop\033[0m" << std::endl;
    planner_manager_->EmergencyStop(stop_pos, stop_yaw, singul);
    quadrotor_msgs::msg::PolynomialTraj msg;
    msg.action = quadrotor_msgs::msg::PolynomialTraj::ACTION_ABORT;  // 终止当前轨迹
    poly_traj_pub_->publish(msg);

    return true;
  }

} // namespace remani_planner
